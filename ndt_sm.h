/**
 * @file  ndt_sm.h
 * @brief NDT application state machine — policy, separated from hardware.
 *
 * Ownership rules that make this safe:
 *
 *   - The state variable lives inside ndt_sm.c and is written by exactly one
 *     context: whoever calls ndt_sm_step() (the main loop). No ISR touches it.
 *   - Interrupts influence the machine only by posting events into a
 *     lock-free ring: ndt_sm_post() is the ISR-side entry point and is the
 *     ONLY producer. The step function is the only consumer.
 *   - Everything that touches a register is reached through ndt_sm_hooks_t,
 *     supplied by main.c. The state machine therefore compiles and reasons
 *     without driverlib.
 *
 * Consequence for reconfiguration: an I2CB command and a periodic timer both
 * post NDT_EV_TRIGGER, and the machine cannot tell them apart. Switching
 * between commanded and periodic firing is a change in ndt_config.h only.
 */

#ifndef NDT_SM_H
#define NDT_SM_H

#include <stdint.h>
#include <stdbool.h>
#include "ndt_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── States ──────────────────────────────────────────────────────────────── */
typedef enum {
    NDT_ST_IDLE = 0,       /**< Waiting for an event                         */
    NDT_ST_FIRE,           /**< Rails checked, burst + capture running       */
    NDT_ST_DATA_READY,     /**< Record complete; housekeeping                */
    NDT_ST_STORE_OP,       /**< EEPROM save/load running                     */
    NDT_ST_THERMAL_FAULT,  /**< MAX14808 over-temperature                    */
    NDT_ST_VOLTAGE_FAULT   /**< Supply rail(s) out of range                  */
} ndt_state_t;

/* ── Events ──────────────────────────────────────────────────────────────── */
typedef enum {
    NDT_EV_TRIGGER = 0,    /**< Fire an A-scan (arg unused)                  */
    NDT_EV_STORE_SAVE,     /**< Save current record   (arg = record ID)      */
    NDT_EV_STORE_LOAD,     /**< Load a record by ID   (arg = record ID)      */
    NDT_EV_SET_BURST,      /**< Change burst length   (arg = cycles, raw)    */
    NDT_EV_CLEAR_FAULT     /**< Attempt to leave a latched fault state       */
} ndt_event_id_t;

/* ── Hooks: everything the machine needs from the board ──────────────────── */
typedef struct {
    /** Free-running millisecond counter. Must be monotonic; wrap is handled. */
    uint32_t (*millis)(void);

    /** True while the pulser reports over-temperature (THP low). */
    bool (*thermal_fault)(void);

    /** Sample the supply taps; return true when every rail is in range.
     *  Implementation is expected to update the board status word itself. */
    bool (*supplies_ok)(void);

    /** Blocking: HV on, tone-burst, receive mode, full record capture. */
    void (*fire_and_capture)(void);

    /** Post-capture housekeeping: restore pulser mode, HV off, mark valid. */
    void (*after_capture)(void);

    /** Rails were bad: HV off, flag the fault. */
    void (*on_voltage_fault)(void);

    /** Entering the thermal state: HV off, TX disable, latch the flag. */
    void (*on_thermal_enter)(void);

    /** Called at NDT_THERMAL_BLINK_MS intervals while in the thermal state. */
    void (*thermal_blink)(void);

    /** Blocking EEPROM save / load of the current record buffer. */
    void (*store_save)(uint16_t id);
    void (*store_load)(uint16_t id);

    /** Apply a new burst length; the hook is responsible for clamping. */
    void (*set_burst_cycles)(uint16_t cycles);
} ndt_sm_hooks_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/**
 * @brief Bind hooks and reset the machine to NDT_ST_IDLE.
 * @note  Call before interrupts are enabled: the event queue must be empty
 *        before the first producer can run.
 */
void ndt_sm_init(const ndt_sm_hooks_t *hooks);

/**
 * @brief Post an event. ISR-safe, non-blocking, never spins.
 * @return false if the queue was full and the event was dropped.
 * @warning Single-producer only. Post from interrupt context; the main loop
 *          must not post, or the lock-free ring loses its guarantee.
 */
bool ndt_sm_post(ndt_event_id_t id, uint16_t arg);

/** @brief Run one iteration. Call from the main loop, unconditionally. */
void ndt_sm_step(void);

/** @brief Current state, for status reporting. */
ndt_state_t ndt_sm_state(void);

/** @brief True when the machine is idle and can accept work immediately. */
bool ndt_sm_is_idle(void);

/** @brief Count of events dropped because the queue was full (diagnostic). */
uint16_t ndt_sm_dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* NDT_SM_H */
