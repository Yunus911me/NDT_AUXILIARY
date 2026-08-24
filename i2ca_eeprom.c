/**
 * @file  i2ca_eeprom.c
 * @brief m24m01e_io_t transport bound to F280025 I2CA (polled master mode).
 *
 * Assumes Board_init() configured I2CA as controller @ 400 kHz, FIFO off.
 * All functions are blocking with iteration-budget timeouts (never hang).
 *
 * C28x note: uint8_t is a 16-bit container on C2000 — every byte moved to
 * the I2C data register is masked with 0xFF, and every byte read back is
 * masked before storing.
 */

#include "driverlib.h"
#include "device.h"
#include "m24m01e.h"
#include "i2ca_eeprom.h"

/* Rough poll-loop iteration budgets (each iteration is a few SYSCLK cycles).
 * At 400 kHz one byte is ~22.5 µs; these allow multi-ms of slack. */
#define I2CA_TIMEOUT_BYTE   200000UL
#define I2CA_TIMEOUT_BUS    400000UL

static bool wait_flag_set(uint32_t base, uint16_t stsBit, uint32_t budget)
{
    while (budget--) {
        if ((I2C_getStatus(base) & stsBit) != 0U) return true;
    }
    return false;
}

static bool wait_stop_done(uint32_t base, uint32_t budget)
{
    while (budget--) {
        if (I2C_getStopConditionStatus(base) == false) return true;
    }
    return false;
}

static bool wait_bus_free(uint32_t base, uint32_t budget)
{
    while (budget--) {
        if ((I2C_getStatus(base) & I2C_STS_BUS_BUSY) == 0U) return true;
    }
    return false;
}

/* Abort helper: force STOP, clear sticky status. */
static int i2ca_abort(uint32_t base, int rc)
{
    I2C_sendStopCondition(base);
    (void)wait_stop_done(base, I2CA_TIMEOUT_BUS);
    I2C_clearStatus(base, I2C_STS_NACK_SENT | I2C_STS_NO_ACK |
                          I2C_STS_ARB_LOST  | I2C_STS_REG_ACCESS_RDY);
    return rc;
}

/* ── write: START, addr+W, data[0..len-1], STOP ─────────────────────────────
 * len==0 → address-only probe (START, addr+W, STOP) for ACK polling.       */
int i2ca_ep_write(void *ctx, uint8_t addr7, const uint8_t *data, uint16_t len)
{
    (void)ctx;
    const uint32_t base = I2CA_BASE;
    uint16_t i;

    if (!wait_bus_free(base, I2CA_TIMEOUT_BUS)) return -1;

    I2C_setTargetAddress(base, (uint16_t)addr7);
    I2C_setConfig(base, I2C_CONTROLLER_SEND_MODE);
    I2C_setDataCount(base, len);          /* 0 is legal: addr phase only     */
    I2C_clearStatus(base, I2C_STS_NO_ACK | I2C_STS_ARB_LOST);

    I2C_sendStartCondition(base);
    I2C_sendStopCondition(base);          /* STOP auto-queues after count    */

    for (i = 0U; i < len; i++) {
        if (!wait_flag_set(base, I2C_STS_TX_DATA_RDY, I2CA_TIMEOUT_BYTE))
            return i2ca_abort(base, -1);
        if ((I2C_getStatus(base) & I2C_STS_NO_ACK) != 0U)
            return i2ca_abort(base, M24M01E_ERR_NACK);
        I2C_putData(base, (uint16_t)(data[i] & 0x00FFU));   /* C28x mask     */
    }

    if (!wait_stop_done(base, I2CA_TIMEOUT_BUS))
        return i2ca_abort(base, -1);

    if ((I2C_getStatus(base) & I2C_STS_NO_ACK) != 0U) {
        I2C_clearStatus(base, I2C_STS_NO_ACK);
        return M24M01E_ERR_NACK;          /* addr NACK (probe) or data NACK  */
    }
    return 0;
}

/* ── write_read: START,addr+W,wdata..., RESTART,addr+R, read..., STOP ────── */
int i2ca_ep_write_read(void *ctx, uint8_t addr7,
                       const uint8_t *wdata, uint16_t wlen,
                       uint8_t *rdata, uint16_t rlen)
{
    (void)ctx;
    const uint32_t base = I2CA_BASE;
    uint16_t i;

    if (!wait_bus_free(base, I2CA_TIMEOUT_BUS)) return -1;

    /* Phase 1: address bytes, NO stop (repeated start follows). */
    I2C_setTargetAddress(base, (uint16_t)addr7);
    I2C_setConfig(base, I2C_CONTROLLER_SEND_MODE);
    I2C_setDataCount(base, wlen);
    I2C_clearStatus(base, I2C_STS_NO_ACK | I2C_STS_ARB_LOST);
    I2C_sendStartCondition(base);

    for (i = 0U; i < wlen; i++) {
        if (!wait_flag_set(base, I2C_STS_TX_DATA_RDY, I2CA_TIMEOUT_BYTE))
            return i2ca_abort(base, -1);
        if ((I2C_getStatus(base) & I2C_STS_NO_ACK) != 0U)
            return i2ca_abort(base, M24M01E_ERR_NACK);
        I2C_putData(base, (uint16_t)(wdata[i] & 0x00FFU));
    }

    /* Wait until the last address byte has actually gone out (ARDY). */
    if (!wait_flag_set(base, I2C_STS_REG_ACCESS_RDY, I2CA_TIMEOUT_BYTE))
        return i2ca_abort(base, -1);
    if ((I2C_getStatus(base) & I2C_STS_NO_ACK) != 0U)
        return i2ca_abort(base, M24M01E_ERR_NACK);

    /* Phase 2: repeated START in receive mode, STOP after rlen bytes. */
    I2C_setConfig(base, I2C_CONTROLLER_RECEIVE_MODE);
    I2C_setDataCount(base, rlen);
    I2C_sendStartCondition(base);
    I2C_sendStopCondition(base);

    for (i = 0U; i < rlen; i++) {
        if (!wait_flag_set(base, I2C_STS_RX_DATA_RDY, I2CA_TIMEOUT_BYTE))
            return i2ca_abort(base, -1);
        rdata[i] = (uint8_t)(I2C_getData(base) & 0x00FFU);
    }

    if (!wait_stop_done(base, I2CA_TIMEOUT_BUS))
        return i2ca_abort(base, -1);
    return 0;
}

/* ── delay callback ──────────────────────────────────────────────────────── */
void i2ca_ep_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    while (ms--) { DEVICE_DELAY_US(1000U); }
}
