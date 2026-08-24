/**
 * @file  ndt_i2cb.c
 * @brief I2CB slave protocol — implementation.
 * @see   ndt_i2cb.h
 */

#include <stddef.h>
#include "driverlib.h"
#include "device.h"
#include "ndt_config.h"
#include "ndt_i2cb.h"
#include "ndt_sm.h"

/* ── Commands ────────────────────────────────────────────────────────────── */
#define CMD_TRIGGER      (0x01u)
#define CMD_SET_BURST    (0x02u)
#define CMD_SEL_STATUS   (0x03u)
#define CMD_SEL_WAVE     (0x04u)
#define CMD_SEL_TAPS     (0x05u)
#define CMD_STORE_SAVE   (0x06u)
#define CMD_STORE_LOAD   (0x07u)
#define CMD_SEL_HDR      (0x08u)
#define CMD_MAX          (0x08u)

/* Parameter bytes expected after each command byte. Indexed lookup keeps the
 * RX path free of a compare chain. */
static const uint16_t k_paramLen[CMD_MAX + 1u] = {
    0u,  /* 0x00 unused    */
    0u,  /* 0x01 trigger   */
    1u,  /* 0x02 burst n   */
    0u,  /* 0x03 status    */
    1u,  /* 0x04 wave ch   */
    0u,  /* 0x05 taps      */
    2u,  /* 0x06 save id   */
    2u,  /* 0x07 load id   */
    0u   /* 0x08 header    */
};

/* ── TX descriptor: prepared on stream select, consumed one byte at a time ── */
typedef struct {
    const volatile uint16_t *src;
    uint16_t                 bytes;  /**< stream length in bytes            */
    bool                     wide;   /**< true: uint16 LE per element       */
} tx_desc_t;

static ndt_i2cb_streams_t s_str;
static volatile tx_desc_t s_tx;
static volatile uint16_t  s_txIdx   = 0u;
static volatile bool      s_txLock  = false;
static volatile uint16_t  s_badCmd  = 0u;

/* RX parser state */
static volatile uint16_t  s_cmd     = 0u;
static volatile uint16_t  s_need    = 0u;
static volatile uint16_t  s_got     = 0u;
static volatile uint16_t  s_par[2]  = {0u, 0u};

/* ── setup ───────────────────────────────────────────────────────────────── */

static void select_status(void)
{
    s_tx.src   = s_str.status;
    s_tx.bytes = 2u;
    s_tx.wide  = true;
    s_txIdx    = 0u;
}

void ndt_i2cb_init(const ndt_i2cb_streams_t *streams)
{
    if (streams != NULL) { s_str = *streams; }

    s_txIdx  = 0u;
    s_txLock = false;
    s_cmd    = 0u;
    s_need   = 0u;
    s_got    = 0u;
    s_badCmd = 0u;

    select_status();                       /* sane default before first cmd */
}

void ndt_i2cb_lock_tx(bool locked)   { s_txLock = locked; }
uint16_t ndt_i2cb_bad_commands(void) { return s_badCmd; }

/* ── command application (ISR context, deliberately tiny) ────────────────── */

static void apply_command(uint16_t cmd, uint16_t p0, uint16_t p1)
{
    switch (cmd)
    {
        /* Stream selection must happen here: a master may issue the read
         * immediately after the write, with no main-loop turn in between.
         * Cost is three stores. */
        case CMD_SEL_STATUS:
            select_status();
            break;

        case CMD_SEL_TAPS:
            s_tx.src   = s_str.taps;
            s_tx.bytes = (uint16_t)(s_str.tap_count * 2u);
            s_tx.wide  = true;
            s_txIdx    = 0u;
            break;

        case CMD_SEL_WAVE:
        {
            uint16_t ch = (uint16_t)(p0 & 0x07u);
            if (ch >= s_str.wave_channels) { ch = 0u; }
            s_tx.src   = &s_str.wave[(uint32_t)ch * s_str.wave_samples];
            s_tx.bytes = (uint16_t)(s_str.wave_samples * 2u);
            s_tx.wide  = true;
            s_txIdx    = 0u;
            break;
        }

        case CMD_SEL_HDR:
            s_tx.src   = s_str.hdr_stream;
            s_tx.bytes = s_str.hdr_len;
            s_tx.wide  = false;
            s_txIdx    = 0u;
            break;

        /* Everything with a consequence becomes an event. Clamping, state
         * transitions and EEPROM work all happen in main-loop context. */
        case CMD_TRIGGER:
            (void)ndt_sm_post(NDT_EV_TRIGGER, 0u);
            break;

        case CMD_SET_BURST:
            (void)ndt_sm_post(NDT_EV_SET_BURST, p0);
            break;

        case CMD_STORE_SAVE:
            (void)ndt_sm_post(NDT_EV_STORE_SAVE,
                              (uint16_t)(p0 | (uint16_t)(p1 << 8)));
            break;

        case CMD_STORE_LOAD:
            (void)ndt_sm_post(NDT_EV_STORE_LOAD,
                              (uint16_t)(p0 | (uint16_t)(p1 << 8)));
            break;

        default:
            s_badCmd++;
            break;
    }
}

/* ── ISR ─────────────────────────────────────────────────────────────────── */
/*
 * Registered by SysConfig as INT_myI2CB. The source is drained in a loop:
 * I2C_getInterruptSource() reports one event per call, so a coincident event
 * used to wait a full interrupt entry/exit before being served.
 */
__interrupt void INT_myI2CB_ISR(void)
{
    uint32_t src;

    while ((src = I2C_getInterruptSource(I2CB_BASE)) != I2C_INTSRC_NONE)
    {
        switch (src)
        {
            case I2C_INTSRC_ADDR_TARGET:      /* new transaction opened */
                s_txIdx = 0u;
                s_need  = 0u;
                s_got   = 0u;
                break;

            case I2C_INTSRC_RX_DATA_RDY:
            {
                uint16_t d = (uint16_t)(I2C_getData(I2CB_BASE) & 0x00FFu);

                if (s_need == 0u)
                {
                    /* Command byte. */
                    if (d > CMD_MAX || d == 0u) {
                        s_badCmd++;
                    } else if (k_paramLen[d] == 0u) {
                        apply_command(d, 0u, 0u);
                    } else {
                        s_cmd  = d;
                        s_need = k_paramLen[d];
                        s_got  = 0u;
                    }
                }
                else
                {
                    /* Parameter byte. */
                    s_par[s_got] = d;
                    s_got++;
                    if (s_got >= s_need) {
                        s_need = 0u;
                        apply_command(s_cmd, s_par[0], s_par[1]);
                    }
                }
                break;
            }

            case I2C_INTSRC_TX_DATA_RDY:
            {
                uint16_t idx = s_txIdx;
                uint16_t len = s_tx.bytes;
                uint16_t b;

                if (s_txLock || s_tx.src == NULL || len == 0u) {
                    b = 0x00FFu;                    /* buffer busy / unset */
                } else if (s_tx.wide) {
                    uint16_t w = s_tx.src[idx >> 1u];
                    b = (idx & 1u) ? (uint16_t)((w >> 8u) & 0x00FFu)
                                   : (uint16_t)( w        & 0x00FFu);
                } else {
                    b = (uint16_t)(s_tx.src[idx] & 0x00FFu);
                }

                I2C_putData(I2CB_BASE, b);

                idx++;
                s_txIdx = (idx >= len) ? 0u : idx;  /* wrap on overrun */
                break;
            }

            case I2C_INTSRC_STOP_CONDITION:
                s_txIdx = 0u;
                s_need  = 0u;
                break;

            case I2C_INTSRC_NO_ACK:
            case I2C_INTSRC_ARB_LOST:
                /* Master aborted or lost the bus: drop any partial command
                 * so the next transaction starts clean. */
                s_need  = 0u;
                s_got   = 0u;
                s_txIdx = 0u;
                I2C_clearStatus(I2CB_BASE, I2C_STS_NO_ACK | I2C_STS_ARB_LOST);
                break;

            default:
                break;
        }
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}
