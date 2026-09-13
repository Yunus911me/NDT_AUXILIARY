/**
 * @file  ndt_sm.c
 * @brief NDT application state machine — implementation.
 * @see   ndt_sm.h
 *
 * Event queue concurrency
 * -----------------------
 * A power-of-two ring with one producer (interrupt context, moves `head`)
 * and one consumer (main loop, moves `tail`). Each index is written by
 * exactly one context and only read by the other, so no critical section is
 * needed on either side — provided the single-producer rule holds. That is
 * why the periodic trigger and the thermal check below are handled inline
 * instead of being posted as events.
 */

#include <stddef.h>
#include "ndt_sm.h"

#define QDEPTH  NDT_EVENT_QUEUE_DEPTH   /* QUEUE DEPTH */

#define QMASK   (QDEPTH - 1u)   /* QUEUE MASK */ 

#if (QDEPTH & QMASK) != 0u
#error "NDT_EVENT_QUEUE_DEPTH must be a power of two"
#endif

typedef struct {
    uint16_t id;
    uint16_t arg;
} sm_event_t;

static sm_event_t       s_queue[QDEPTH];
static volatile uint16_t s_head    = 0u;   /* producer (ISR) */
static volatile uint16_t s_tail    = 0u;   /* consumer (main loop) */
static volatile uint16_t s_dropped = 0u;

static ndt_sm_hooks_t   s_hk;
static ndt_state_t      s_state       = NDT_ST_IDLE;
static uint32_t         s_lastTrigMs  = 0u;
static uint32_t         s_lastBlinkMs = 0u;

/* ── queue ───────────────────────────────────────────────────────────────── */

static bool q_pop(sm_event_t *out)
{
    uint16_t tail = s_tail;

    if (tail == s_head) { return false; }
    *out   = s_queue[tail];
    s_tail = (uint16_t)((tail + 1u) & QMASK);
    return true;
}

bool ndt_sm_post(ndt_event_id_t id, uint16_t arg)
{
    uint16_t head = s_head;
    uint16_t next = (uint16_t)((head + 1u) & QMASK);

    if (next == s_tail) {                 /* full — drop, never block */
        s_dropped++;
        return false;
    }
    s_queue[head].id  = (uint16_t)id;
    s_queue[head].arg = arg;
    s_head            = next;             /* publish last */
    return true;
}

/* ── helpers ─────────────────────────────────────────────────────────────── */

static uint32_t now_ms(void)
{
    return (s_hk.millis != NULL) ? s_hk.millis() : 0u;
}

/* Wrap-safe elapsed comparison. */
static bool elapsed(uint32_t since, uint32_t period)
{
    return (uint32_t)(now_ms() - since) >= period;
}

/* A trigger from either source. Only ever called from IDLE — commands are
 * consumed there and the periodic check runs there — so there is no "busy"
 * case to arbitrate; a command that arrives mid-scan simply waits in the
 * queue. The one real policy question is what a periodic tick does after a
 * scan overran its period. */
static void accept_trigger(bool periodic)
{
    s_state = NDT_ST_FIRE;

#if (NDT_TRIG_SKIP_MISSED == 0)
    if (periodic) {
        s_lastTrigMs += NDT_TRIG_PERIOD_MS;   /* catch up on missed ticks */
        return;
    }
#else
    (void)periodic;
#endif
    s_lastTrigMs = now_ms();
}

/* ── init ────────────────────────────────────────────────────────────────── */

void ndt_sm_init(const ndt_sm_hooks_t *hooks)
{
    if (hooks != NULL) { s_hk = *hooks; }

    s_head        = 0u;
    s_tail        = 0u;
    s_dropped     = 0u;
    s_state       = NDT_ST_IDLE;
    s_lastTrigMs  = now_ms();
    s_lastBlinkMs = s_lastTrigMs;
}

/* ── idle handling: faults, timers, then queued commands ─────────────────── */

static void step_idle(void)
{
    sm_event_t ev;

    /* 1. Thermal flag has priority over anything queued. Handled inline, not
     *    posted, to keep the queue single-producer. */
    if ((s_hk.thermal_fault != NULL) && s_hk.thermal_fault()) {
        if (s_hk.on_thermal_enter != NULL) { s_hk.on_thermal_enter(); }
        s_state       = NDT_ST_THERMAL_FAULT;
        s_lastBlinkMs = now_ms();
        return;
    }

    /* 2. Periodic trigger source. */
#if (NDT_TRIGGER_SOURCE == NDT_TRIG_PERIODIC) || \
    (NDT_TRIGGER_SOURCE == NDT_TRIG_BOTH)
    if (elapsed(s_lastTrigMs, NDT_TRIG_PERIOD_MS)) {
        accept_trigger(true);
        return;
    }
#endif

    /* 3. Commands from the I2CB slave. Events are consumed only in IDLE, so
     *    a command issued mid-scan waits in the ring instead of being lost;
     *    it is dropped only on queue overflow. */
    if (!q_pop(&ev)) { return; }

    switch ((ndt_event_id_t)ev.id)
    {
        case NDT_EV_TRIGGER:
#if (NDT_TRIGGER_SOURCE == NDT_TRIG_PERIODIC)
            /* Commanded firing disabled in this build — ignore. */
#else
            accept_trigger(false);
#endif
            break;

        case NDT_EV_STORE_SAVE:
            s_state = NDT_ST_STORE_OP;
            if (s_hk.store_save != NULL) { s_hk.store_save(ev.arg); }
            s_state = NDT_ST_IDLE;
            break;

        case NDT_EV_STORE_LOAD:
            s_state = NDT_ST_STORE_OP;
            if (s_hk.store_load != NULL) { s_hk.store_load(ev.arg); }
            s_state = NDT_ST_IDLE;
            break;

        case NDT_EV_SET_BURST:
            if (s_hk.set_burst_cycles != NULL) {
                s_hk.set_burst_cycles(ev.arg);
            }
            break;

        case NDT_EV_CLEAR_FAULT:
        default:
            break;
    }
}

/* ── step ────────────────────────────────────────────────────────────────── */

void ndt_sm_step(void)
{
    switch (s_state)
    {
        case NDT_ST_IDLE:
            step_idle();
            break;

        case NDT_ST_FIRE:
            if ((s_hk.supplies_ok != NULL) && s_hk.supplies_ok()) {
                if (s_hk.fire_and_capture != NULL) {
                    s_hk.fire_and_capture();      /* blocking, ~0.9 ms */
                }
                s_state = NDT_ST_DATA_READY;
            } else {
                if (s_hk.on_voltage_fault != NULL) { s_hk.on_voltage_fault(); }
                s_state = NDT_ST_VOLTAGE_FAULT;
            }
            break;

        case NDT_ST_DATA_READY:
            if (s_hk.after_capture != NULL) { s_hk.after_capture(); }
            s_state = NDT_ST_IDLE;
            break;

        case NDT_ST_VOLTAGE_FAULT:
            /* Not latched: the master retries by issuing another trigger. */
            s_state = NDT_ST_IDLE;
            break;

        case NDT_ST_THERMAL_FAULT:

            if (elapsed(s_lastBlinkMs, NDT_THERMAL_BLINK_MS)) {
                s_lastBlinkMs = now_ms();
                if (s_hk.thermal_blink != NULL) { s_hk.thermal_blink(); }
            }
#if (NDT_THERMAL_LATCHED == 0)
            if ((s_hk.thermal_fault == NULL) || !s_hk.thermal_fault()) {
                s_state = NDT_ST_IDLE;
            }
#endif
            break;

        case NDT_ST_STORE_OP:
        default:
            s_state = NDT_ST_IDLE;
            break;
    }
}

/* ── accessors ───────────────────────────────────────────────────────────── */

ndt_state_t ndt_sm_state(void)  { return s_state; }
bool ndt_sm_is_idle(void)       { return s_state == NDT_ST_IDLE; }
uint16_t ndt_sm_dropped(void)   { return s_dropped; }
