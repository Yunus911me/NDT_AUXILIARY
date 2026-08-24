/**
 * @file  ndt_store.h
 * @brief Slotted A-scan record store for the NDT auxiliary board.
 *
 * Layer position:
 *
 *     ndt_store.c   <- record format: magic, slots, sequence, checksum
 *          |            (this file — application knowledge)
 *     m24m01e.c     <- chip: 17-bit addressing, 256-byte pages, ACK polling
 *          |
 *     i2ca_eeprom.c <- MCU: F280025 I2CA register access
 *
 * The store knows nothing about I2C, and the chip driver knows nothing about
 * A-scans. Swapping the memory part touches only the middle layer; changing
 * the record format touches only this one.
 *
 * Slot geometry (bytes in the EEPROM array):
 *
 *   ┌ slot k, base = k * NDT_STORE_SLOT_BYTES ────────────────────────────┐
 *   │ page 0     : header (magic, id, seq, time axis, checksum)          │
 *   │ page 1..N  : sample data, little-endian uint16, channel-major      │
 *   └─────────────────────────────────────────────────────────────────────┘
 *
 * Defaults: 8 ch x 512 samples -> 8192 data bytes -> 32 data pages
 *           -> slot = 33 pages = 8448 B -> 15 slots in a 128 KB part.
 *
 * Not internally locked: call from one context only (the main loop).
 */

#ifndef NDT_STORE_H
#define NDT_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include "ndt_config.h"
#include "m24m01e.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Derived geometry ────────────────────────────────────────────────────── */
#define NDT_STORE_PAGE_SIZE   M24M01E_PAGE_SIZE
#define NDT_STORE_CAPACITY    M24M01E_SIZE_BYTES

#define NDT_STORE_DATA_BYTES  (NDT_ASCAN_SAMPLES * NDT_ASCAN_CHANNELS * 2u)
#define NDT_STORE_DATA_PAGES  (NDT_STORE_DATA_BYTES / NDT_STORE_PAGE_SIZE)
#define NDT_STORE_SLOT_BYTES  ((1u + NDT_STORE_DATA_PAGES) * NDT_STORE_PAGE_SIZE)
#define NDT_STORE_SLOT_COUNT_RAW (NDT_STORE_CAPACITY / NDT_STORE_SLOT_BYTES)
#define NDT_STORE_SLOT_COUNT  ((uint16_t)NDT_STORE_SLOT_COUNT_RAW)

/* Record data must tile the page grid exactly, or a page write would wrap. */
#if (NDT_STORE_DATA_BYTES % NDT_STORE_PAGE_SIZE) != 0
#error "NDT_ASCAN_SAMPLES * NDT_ASCAN_CHANNELS * 2 must be a multiple of 256"
#endif
#if NDT_STORE_SLOT_COUNT_RAW == 0
#error "Record geometry too large: not one slot fits in the EEPROM"
#endif

/* Serialised header stream length for I2CB command 0x08. */
#define NDT_STORE_HDR_STREAM_LEN (16u)

/* ── Status ──────────────────────────────────────────────────────────────── */
typedef enum {
    NDT_STORE_OK = 0,
    NDT_STORE_ERR_PARAM,      /**< NULL pointer or geometry mismatch        */
    NDT_STORE_ERR_RANGE,      /**< Slot index out of range                  */
    NDT_STORE_ERR_IO,         /**< Underlying EEPROM driver reported a fault*/
    NDT_STORE_ERR_NOT_FOUND,  /**< No record carries the requested ID       */
    NDT_STORE_ERR_CORRUPT     /**< Checksum mismatch on load                */
} ndt_store_status_t;

/* ── Record header ───────────────────────────────────────────────────────── */
/**
 * Time axis of sample i on every channel:
 *     t(i) = start_delay_ns + (uint32_t)i * sample_period_ns
 * with t = 0 at the start of the excitation burst.
 */
typedef struct {
    uint16_t id;               /**< Caller-chosen record ID                  */
    uint32_t seq;              /**< Monotonic save counter (store-managed)   */
    uint16_t burst_cycles;     /**< Excitation tone-burst cycles             */
    uint32_t freq_hz;          /**< Excitation centre frequency              */
    uint16_t sample_period_ns; /**< Time between samples of one channel      */
    uint32_t start_delay_ns;   /**< Fire moment -> first sample              */
    uint16_t samples_per_ch;   /**< Must equal NDT_ASCAN_SAMPLES             */
    uint16_t channels;         /**< Must equal NDT_ASCAN_CHANNELS            */
    uint16_t status_flags;     /**< Board status word at capture time        */
} ndt_ascan_hdr_t;

/* ── Handle ──────────────────────────────────────────────────────────────── */
typedef struct {
    m24m01e_t *dev;            /**< Bound memory device (not owned)          */
} ndt_store_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/** Bind the store to an already-initialised, already-probed EEPROM handle. */
ndt_store_status_t ndt_store_init(ndt_store_t *st, m24m01e_t *dev);

/**
 * @brief Save one A-scan record.
 *
 * Slot selection: existing slot with the same ID (overwrite) -> first free
 * slot -> slot holding the lowest sequence number (evict oldest).
 * hdr->seq is assigned by the store (highest existing + 1) and written back.
 *
 * @param data Channel-major buffer of NDT_ASCAN_CHANNELS x NDT_ASCAN_SAMPLES
 *             uint16 values.
 * @note  Blocking, roughly 0.3 s for the default geometry at 400 kHz
 *        (33 pages, each ~6 ms transfer + up to 4 ms internal write cycle).
 */
ndt_store_status_t ndt_store_save(ndt_store_t *st,
                                  ndt_ascan_hdr_t *hdr,
                                  const uint16_t *data);

/**
 * @brief Load a record by ID.
 * @param data Destination buffer, or NULL for a header-only query.
 * @return NDT_STORE_ERR_CORRUPT still returns the data, so the caller may
 *         inspect it.
 */
ndt_store_status_t ndt_store_load(ndt_store_t *st, uint16_t id,
                                  ndt_ascan_hdr_t *hdr,
                                  uint16_t *data);

/** @brief Locate the slot holding @p id. */
ndt_store_status_t ndt_store_find(ndt_store_t *st, uint16_t id,
                                  uint16_t *slot);

/** @brief Read one slot's header, for directory listings. */
ndt_store_status_t ndt_store_slot_header(ndt_store_t *st, uint16_t slot,
                                         ndt_ascan_hdr_t *hdr);

/**
 * @brief Serialise a header into the 16-byte little-endian stream served by
 *        I2CB command 0x08: id(2) seq(4) burst(2) samples(2) period(2)
 *        start_delay(4). One byte value per array element (C28x).
 */
void ndt_store_hdr_to_stream(const ndt_ascan_hdr_t *hdr,
                             volatile uint16_t *out);

/** @brief Human-readable status text, for logging. */
const char *ndt_store_strerror(ndt_store_status_t s);

#ifdef __cplusplus
}
#endif

#endif /* NDT_STORE_H */
