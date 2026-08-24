/**
 * @file  ndt_i2cb.h
 * @brief I2CB slave command/stream interface to the master MCU.
 *        SDA = GPIO2 (chip pin 61), SCL = GPIO3 (chip pin 60), address 0x21.
 *
 * The peripheral itself is configured by SysConfig (Board_init); this module
 * owns only the protocol.
 *
 * ── Protocol ──────────────────────────────────────────────────────────────
 *   WRITE [0x01]            trigger A-scan
 *   WRITE [0x02][n]         set burst cycles (clamped by the state machine)
 *   WRITE [0x03]            select STATUS stream    -> READ 2 bytes
 *   WRITE [0x04][ch]        select WAVEFORM stream  -> READ samples*2 bytes
 *   WRITE [0x05]            select TAPS stream      -> READ taps*2 bytes
 *   WRITE [0x06][idL][idH]  save current record under ID
 *   WRITE [0x07][idL][idH]  load record by ID
 *   WRITE [0x08]            select HEADER stream    -> READ 16 bytes
 *   READ                    bytes of the selected stream, wrapping at length
 *
 * ── ISR discipline ────────────────────────────────────────────────────────
 * The ISR does three things and nothing else:
 *   1. RX  : accumulate at most two parameter bytes, then either retarget the
 *            TX descriptor (three stores) or push one event onto the state
 *            machine's queue. No clamping, no EEPROM work, no state changes.
 *   2. TX  : one indexed fetch through a descriptor prepared when the stream
 *            was selected — constant time per byte, instead of the previous
 *            per-byte switch that re-derived source and length 1024 times per
 *            waveform read.
 *   3. STOP: reset the byte pointers.
 * Interpretation of the commands happens in main-loop context.
 */

#ifndef NDT_I2CB_H
#define NDT_I2CB_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Read-only views the slave may serve. All buffers are owned by the caller
 * and must outlive the interface (statics in main.c).
 */
typedef struct {
    const volatile uint16_t *status;      /**< 1 word, served little-endian  */
    const volatile uint16_t *taps;        /**< tap_count words, LE           */
    uint16_t                 tap_count;
    const volatile uint16_t *wave;        /**< channels x samples, ch-major  */
    uint16_t                 wave_samples;
    uint16_t                 wave_channels;
    const volatile uint16_t *hdr_stream;  /**< hdr_len byte values           */
    uint16_t                 hdr_len;
} ndt_i2cb_streams_t;

/**
 * @brief Bind the served buffers and reset the protocol state.
 * @note  Call before Interrupt_enableGlobal().
 */
void ndt_i2cb_init(const ndt_i2cb_streams_t *streams);

/**
 * @brief Freeze the read side while a buffer is being rewritten.
 *
 * While locked the slave answers 0xFF instead of buffer contents, so a master
 * read that overlaps a capture or an EEPROM load cannot return a torn record.
 * Cheaper than double-buffering, which would cost another 8 KB of the 24 KB
 * SRAM. Safe to call from the main loop; the ISR only reads the flag.
 */
void ndt_i2cb_lock_tx(bool locked);

/** @brief Count of unknown command bytes seen (diagnostic). */
uint16_t ndt_i2cb_bad_commands(void);

#ifdef __cplusplus
}
#endif

#endif /* NDT_I2CB_H */
