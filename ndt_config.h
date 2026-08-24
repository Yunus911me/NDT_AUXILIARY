/**
 * @file  ndt_config.h
 * @brief Single place for every application-level policy knob.
 *
 * Nothing in this file touches registers. Hardware timing constants stay in
 * main.c (they are pin/clock facts); everything that is a *decision* lives
 * here, so re-targeting the board's behaviour is a rebuild, not a rewrite.
 */

#ifndef NDT_CONFIG_H
#define NDT_CONFIG_H

/* ═══════════════════════════════════════════════════════════════════════════
 * Record geometry
 *   SAMPLES * CHANNELS * 2 must be a whole multiple of the EEPROM page size
 *   (256 B) — ndt_store.h enforces this with a compile-time check.
 * ═══════════════════════════════════════════════════════════════════════════ */
#define NDT_ASCAN_SAMPLES        (512u)   /* samples per channel             */
#define NDT_ASCAN_CHANNELS       (8u)     /* transducer channels             */

/* ═══════════════════════════════════════════════════════════════════════════
 * Excitation
 * ═══════════════════════════════════════════════════════════════════════════ */
#define NDT_BURST_CYCLES_MIN     (5u)
#define NDT_BURST_CYCLES_MAX     (10u)
#define NDT_BURST_CYCLES_DEFAULT (5u)

/* ═══════════════════════════════════════════════════════════════════════════
 * Trigger policy — how an A-scan gets started
 *
 *   NDT_TRIG_I2C       only I2CB command 0x01 fires a scan (as designed)
 *   NDT_TRIG_PERIODIC  a free-running timer fires every NDT_TRIG_PERIOD_MS;
 *                      command 0x01 is ignored
 *   NDT_TRIG_BOTH      periodic firing, and 0x01 fires early on demand
 *
 * Both sources post the same NDT_EV_TRIGGER event, so ndt_sm.c does not need
 * to know which one is active.
 * ═══════════════════════════════════════════════════════════════════════════ */
#define NDT_TRIG_I2C             (0)
#define NDT_TRIG_PERIODIC        (1)
#define NDT_TRIG_BOTH            (2)

#define NDT_TRIGGER_SOURCE       NDT_TRIG_I2C
#define NDT_TRIG_PERIOD_MS       (1000u)

/* Periodic source only: what to do when a scan overruns the period, so one
 * or more ticks were missed.
 *   1 = skip them; the next scan starts one full period from now
 *   0 = catch up; fire again immediately until the schedule is met         */
#define NDT_TRIG_SKIP_MISSED     (1)

/* Note on commanded triggers: I2CB commands are consumed only while the
 * machine is idle, so a command arriving mid-scan waits in the event queue
 * rather than being lost. It is dropped only if the queue is full — see
 * ndt_sm_dropped(). This differs from the pre-refactor firmware, which
 * ignored 0x01/0x06/0x07 outright unless the board was idle.              */

/* ═══════════════════════════════════════════════════════════════════════════
 * Fault policy
 * ═══════════════════════════════════════════════════════════════════════════ */
/* 1 = thermal fault is latched until power cycle (current behaviour).
 * 0 = the machine returns to IDLE once THP releases.                       */
#define NDT_THERMAL_LATCHED      (1)

/* Blink period of the fault LED while in the thermal state, in ms.         */
#define NDT_THERMAL_BLINK_MS     (100u)

/* ═══════════════════════════════════════════════════════════════════════════
 * I2CB slave interface
 * ═══════════════════════════════════════════════════════════════════════════ */
#define NDT_I2CB_ADDRESS         (0x21u)

/* Depth of the ISR → main-loop event queue. Power of two. Eight covers any
 * plausible burst of commands between two ndt_sm_step() calls.            */
#define NDT_EVENT_QUEUE_DEPTH    (8u)

#endif /* NDT_CONFIG_H */
