/**
 * @file    m24m01e.h
 * @brief   Portable, MCU-agnostic driver for the STMicroelectronics M24M01E-F
 *          1-Mbit (128 KB) I2C serial EEPROM (e.g. M24M01E-FMN6TP).
 *
 * Reference: ST datasheet DS13858 Rev 4 (March 2026).
 *
 * Highlights of the part this driver targets:
 *   - 1 Mbit = 131072 bytes, addressed with 17 bits. The upper bit (A16) is
 *     carried inside the I2C device-select byte, NOT in the two address bytes.
 *   - 256-byte pages. A page write may span at most one page; writes past the
 *     page end wrap around to the start of the same page in hardware, so this
 *     driver splits multi-page writes.
 *   - Internal write cycle up to 4 ms (typ 3 ms), handled here by ACK polling.
 *   - Device-select high nibble: 1010b for the memory array,
 *     1011b for the DTI / CDA / SWP registers and the identification page.
 *   - Chip-enable address bits C2,C1 (0..3) select one of up to four parts on
 *     the same bus, matching the CDA register. Default from factory is 0.
 *
 * ── NEW in this revision: A-scan record storage layer ─────────────────────
 * A slotted record store for NDT A-scan waveform records:
 *   - Fixed geometry: M24M01E_ASCAN_CHANNELS channels of
 *     M24M01E_ASCAN_SAMPLES uint16 samples (channel-major buffer layout).
 *   - Each slot = 1 header page (256 B) + data pages, page-aligned so every
 *     internal write cycle carries a full page (fastest possible commit).
 *   - Records are addressed by a caller-chosen 16-bit ID. Saving with an
 *     existing ID overwrites that slot; otherwise a free slot is used;
 *     if none is free, the OLDEST record (lowest sequence number) is evicted.
 *   - Header stores everything needed to reconstruct the time axis:
 *       t(i) = start_delay_ns + i * sample_period_ns   (t = 0 at burst start)
 *   - A 16-bit additive checksum over the sample data detects corruption.
 *
 * Thread-safety: not internally locked. Serialize access to one device handle
 * from a single context.
 */

#ifndef M24M01E_H
#define M24M01E_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "inc/hw_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Device geometry                                                           */
/* ------------------------------------------------------------------------- */

#define M24M01E_SIZE_BYTES        (131072u)  /**< Total array: 1 Mbit = 128 KB */
#define M24M01E_PAGE_SIZE         (256u)     /**< Page (write) size in bytes   */
#define M24M01E_PAGE_COUNT        (M24M01E_SIZE_BYTES / M24M01E_PAGE_SIZE)
#define M24M01E_ADDR_MAX          (M24M01E_SIZE_BYTES - 1u)
#define M24M01E_ID_PAGE_SIZE      (256u)     /**< Identification page size     */
#define M24M01E_WRITE_TIME_MS_MAX (5u)       /**< tW max 4 ms + guard band     */

/* ------------------------------------------------------------------------- */
/* Return codes                                                              */
/* ------------------------------------------------------------------------- */

typedef enum {
    M24M01E_OK = 0,             /**< Success                                  */
    M24M01E_ERR_PARAM,          /**< NULL pointer / bad argument              */
    M24M01E_ERR_RANGE,          /**< Address + length exceeds a boundary      */
    M24M01E_ERR_IO,             /**< Transport reported a bus error           */
    M24M01E_ERR_NACK,           /**< Device did not acknowledge               */
    M24M01E_ERR_WRITE_PROTECT,  /**< Target write-protected (SWP/WC/lock/DAL) */
    M24M01E_ERR_TIMEOUT,        /**< Write cycle did not complete in time     */
    M24M01E_ERR_UNSUPPORTED,    /**< Optional feature needs an io callback    */
    M24M01E_ERR_NOT_FOUND,      /**< A-scan: no record with the requested ID  */
    M24M01E_ERR_CORRUPT         /**< A-scan: checksum mismatch on load        */
} m24m01e_status_t;

/* ------------------------------------------------------------------------- */
/* Register bit definitions (see datasheet section 4)                        */
/* ------------------------------------------------------------------------- */

/* Device Type Identifier (DTI) register - read only, factory value 0xB1 */
#define M24M01E_DTI_DTIL          (1u << 0)  /**< Lock bit (always 1)          */
#define M24M01E_DTI_ID_MASK       (0xF0u)    /**< b7..b4 = 1011b               */
#define M24M01E_DTI_ID_VALUE      (0xB0u)    /**< Expected device type id      */

/* Configurable Device Address (CDA) register */
#define M24M01E_CDA_DAL           (1u << 0)  /**< Device Address Lock          */
#define M24M01E_CDA_C1            (1u << 2)  /**< Chip-enable bit C1           */
#define M24M01E_CDA_C2            (1u << 3)  /**< Chip-enable bit C2           */

/* Software Write Protection (SWP) register */
#define M24M01E_SWP_WPL           (1u << 0)  /**< Write-Protect Lock           */
#define M24M01E_SWP_BP0           (1u << 1)  /**< Block-Protect 0              */
#define M24M01E_SWP_BP1           (1u << 2)  /**< Block-Protect 1              */
#define M24M01E_SWP_WPA           (1u << 3)  /**< Write-Protect Activation     */

/** Size of the protected region when SWP write-protection is active. */
typedef enum {
    M24M01E_SWP_BLOCK_UPPER_QUARTER = 0, /**< (BP1,BP0)=00: upper 1/4 protected */
    M24M01E_SWP_BLOCK_UPPER_HALF    = 1, /**< (BP1,BP0)=01: upper 1/2 protected */
    M24M01E_SWP_BLOCK_UPPER_3Q      = 2, /**< (BP1,BP0)=10: upper 3/4 protected */
    M24M01E_SWP_BLOCK_WHOLE         = 3  /**< (BP1,BP0)=11: whole array         */
} m24m01e_swp_block_t;

/* ------------------------------------------------------------------------- */
/* Transport (I2C) interface                                                 */
/* ------------------------------------------------------------------------- */
/*
 * Address convention: `addr7` is a 7-bit right-aligned I2C address. If your
 * HAL wants an 8-bit address, shift left by 1 inside the callback.
 *
 * IMPORTANT for TI C28x targets: uint8_t is a 16-bit container on C2000.
 * Only bits 7:0 of every uint8_t are meaningful; callbacks must mask with
 * 0xFF when moving data to/from the physical bus.
 *
 * Return 0 on success (all bytes ACKed). Return non-zero on any bus error or
 * NACK. If you can distinguish a clean address/data NACK from a physical bus
 * error, return M24M01E_ERR_NACK for the former so the driver can report
 * write-protection precisely; otherwise return any non-zero.
 */
typedef struct {
    /**
     * Write `len` bytes to `addr7` (START, addr+W, data..., STOP).
     * When len == 0 / data == NULL this MUST perform an address-only probe
     * (START, addr+W, STOP) used for write-cycle ACK polling.
     */
    int (*write)(void *ctx, uint8_t addr7, const uint8_t *data, uint16_t len);

    /**
     * Combined write-then-read for random reads:
     * START, addr+W, wdata..., REPEATED-START, addr+R, read rdata..., STOP.
     */
    int (*write_read)(void *ctx, uint8_t addr7,
                      const uint8_t *wdata, uint16_t wlen,
                      uint8_t *rdata, uint16_t rlen);

    /**
     * Optional: blocking delay in milliseconds. May be NULL. Used only as a
     * fallback pause between ACK-poll attempts; ACK polling works without it.
     */
    void (*delay_ms)(void *ctx, uint32_t ms);

    /**
     * Optional: send (START, addr+W, wdata..., NO STOP) then abort with a
     * REPEATED START + STOP, reporting whether the LAST byte was ACKed.
     * Needed only for the non-destructive ID-page lock-status query. May be
     * NULL (that query then returns M24M01E_ERR_UNSUPPORTED). Return 0 if the
     * last byte was ACKed, non-zero if it was NACKed.
     */
    int (*probe_no_stop)(void *ctx, uint8_t addr7,
                         const uint8_t *wdata, uint16_t wlen);

    /** Opaque pointer passed back to every callback. */
    void *ctx;
} m24m01e_io_t; 

/* ------------------------------------------------------------------------- */
/* Device handle / configuration                                             */
/* ------------------------------------------------------------------------- */

typedef struct {
    m24m01e_io_t io;        /**< Transport callbacks (copied into handle)     */
    uint8_t      ce;        /**< Chip-enable address (C2,C1) as value 0..3    */
    uint16_t     poll_tries;/**< Max ACK-poll attempts per write (0 => 32)
                                Controls timeout sensitivity                  */
} m24m01e_t;

/* ------------------------------------------------------------------------- */
/* Initialization                                                            */
/* ------------------------------------------------------------------------- */

m24m01e_status_t m24m01e_init(m24m01e_t *dev, const m24m01e_io_t *io, uint8_t ce);
m24m01e_status_t m24m01e_probe(m24m01e_t *dev);
m24m01e_status_t m24m01e_wait_ready(m24m01e_t *dev);

/* ------------------------------------------------------------------------- */
/* Memory array access                                                       */
/* ------------------------------------------------------------------------- */

m24m01e_status_t m24m01e_read(m24m01e_t *dev, uint32_t addr,
                              uint8_t *buf, size_t len);
m24m01e_status_t m24m01e_write(m24m01e_t *dev, uint32_t addr,
                               const uint8_t *buf, size_t len);
m24m01e_status_t m24m01e_read_byte(m24m01e_t *dev, uint32_t addr, uint8_t *val);
m24m01e_status_t m24m01e_write_byte(m24m01e_t *dev, uint32_t addr, uint8_t val);

/* ------------------------------------------------------------------------- */
/* Identification page (256 bytes, separately lockable)                      */
/* ------------------------------------------------------------------------- */

m24m01e_status_t m24m01e_id_page_read(m24m01e_t *dev, uint8_t offset,
                                      uint8_t *buf, size_t len);
m24m01e_status_t m24m01e_id_page_write(m24m01e_t *dev, uint8_t offset,
                                       const uint8_t *buf, size_t len);
m24m01e_status_t m24m01e_id_page_lock(m24m01e_t *dev);
m24m01e_status_t m24m01e_id_page_is_locked(m24m01e_t *dev, bool *locked);

/* ------------------------------------------------------------------------- */
/* Feature registers                                                         */
/* ------------------------------------------------------------------------- */

m24m01e_status_t m24m01e_read_dti(m24m01e_t *dev, uint8_t *val);
m24m01e_status_t m24m01e_read_cda(m24m01e_t *dev, uint8_t *val);
m24m01e_status_t m24m01e_write_cda(m24m01e_t *dev, uint8_t ce, bool lock);
m24m01e_status_t m24m01e_read_swp(m24m01e_t *dev, uint8_t *val);
m24m01e_status_t m24m01e_write_swp(m24m01e_t *dev, bool enable,
                                   m24m01e_swp_block_t block, bool lock);

/** @brief Human-readable string for a status code (for logging). */
const char *m24m01e_strerror(m24m01e_status_t s);

/* ═══════════════════════════════════════════════════════════════════════════
 * A-scan record storage layer
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Slot geometry (all sizes in EEPROM *bytes*):
 *
 *   ┌ slot 0 ─────────────────────────────────────────────┐
 *   │ page 0        : header (magic, ID, seq, time info)  │
 *   │ pages 1..N    : sample data, little-endian uint16,  │
 *   │                 channel-major (ch0 samples, ch1 …)  │
 *   └──────────────────────────────────────────────────────┘
 *   slot k starts at byte address k * M24M01E_ASCAN_SLOT_BYTES.
 *
 * Defaults: 8 channels × 512 samples → 8192 data bytes → 32 data pages
 * → slot = 33 pages = 8448 bytes → 15 slots in a 128 KB part.
 * Override M24M01E_ASCAN_SAMPLES / _CHANNELS at compile time if needed;
 * SAMPLES × CHANNELS × 2 must stay a multiple of the 256-byte page size.
 */

#ifndef M24M01E_ASCAN_SAMPLES
#define M24M01E_ASCAN_SAMPLES     (512u)   /**< samples per channel           */
#endif
#ifndef M24M01E_ASCAN_CHANNELS
#define M24M01E_ASCAN_CHANNELS    (8u)     /**< transducer channels           */
#endif

#define M24M01E_ASCAN_DATA_BYTES  (M24M01E_ASCAN_SAMPLES * \
                                   M24M01E_ASCAN_CHANNELS * 2u)
#define M24M01E_ASCAN_DATA_PAGES  (M24M01E_ASCAN_DATA_BYTES / M24M01E_PAGE_SIZE)
#define M24M01E_ASCAN_SLOT_BYTES  ((1u + M24M01E_ASCAN_DATA_PAGES) * \
                                   M24M01E_PAGE_SIZE)
#define M24M01E_ASCAN_SLOT_COUNT  ((uint16_t)(M24M01E_SIZE_BYTES / \
                                              M24M01E_ASCAN_SLOT_BYTES))

#define M24M01E_ASCAN_MAGIC0      (0x4Eu)  /* 'N' */
#define M24M01E_ASCAN_MAGIC1      (0x44u)  /* 'D' */
#define M24M01E_ASCAN_VERSION     (0x01u)

/**
 * A-scan record header. The time axis of sample i on every channel is:
 *     t(i) = start_delay_ns + (uint32_t)i * sample_period_ns
 * with t = 0 defined as the start of the excitation burst (fire moment).
 */
typedef struct {
    uint16_t id;               /**< Caller-chosen record ID                   */
    uint32_t seq;              /**< Monotonic save counter (driver-managed)   */
    uint16_t burst_cycles;     /**< Excitation tone-burst cycles (5..10)      */
    uint32_t freq_hz;          /**< Excitation centre frequency (150000)      */
    uint16_t sample_period_ns; /**< Time between samples of one channel       */
    uint32_t start_delay_ns;   /**< Fire moment → first sample                */
    uint16_t samples_per_ch;   /**< Must equal M24M01E_ASCAN_SAMPLES          */
    uint16_t channels;         /**< Must equal M24M01E_ASCAN_CHANNELS         */
    uint16_t status_flags;     /**< Board status word at capture time         */
} m24m01e_ascan_hdr_t;

/**
 * @brief  Save one A-scan record.
 *         Slot selection: existing slot with the same ID (overwrite) →
 *         first free slot → slot holding the lowest sequence number (evict
 *         oldest). hdr->seq is assigned by the driver (max existing + 1).
 * @param  hdr   Header to store; id/geometry/time fields filled by caller,
 *               seq is written back by the driver.
 * @param  data  Channel-major sample buffer,
 *               M24M01E_ASCAN_CHANNELS × M24M01E_ASCAN_SAMPLES uint16 values
 *               (only bits 11:0 are meaningful for 12-bit ADC data; all 16
 *               bits are stored).
 * @return M24M01E_OK, or the first transport / write error encountered.
 * @note   Blocking: a full default-geometry record commits in roughly 0.3 s
 *         (33 pages × ~6 ms transfer + ~4 ms tW each) at 400 kHz.
 */
m24m01e_status_t m24m01e_ascan_save(m24m01e_t *dev,
                                    m24m01e_ascan_hdr_t *hdr,
                                    const uint16_t *data);

/**
 * @brief  Locate the record with the given ID.
 * @param  slot  Receives the slot index (0-based) on success.
 * @return M24M01E_OK or M24M01E_ERR_NOT_FOUND.
 */
m24m01e_status_t m24m01e_ascan_find(m24m01e_t *dev, uint16_t id,
                                    uint16_t *slot);

/**
 * @brief  Load a record by ID: header and (optionally) sample data.
 * @param  hdr   Filled with the stored header. Required.
 * @param  data  Channel-major destination buffer of
 *               M24M01E_ASCAN_CHANNELS × M24M01E_ASCAN_SAMPLES uint16 values,
 *               or NULL to read the header only.
 * @return M24M01E_OK, M24M01E_ERR_NOT_FOUND, M24M01E_ERR_CORRUPT (checksum
 *         mismatch — data is still returned so the caller may inspect it),
 *         or a transport error.
 */
m24m01e_status_t m24m01e_ascan_load(m24m01e_t *dev, uint16_t id,
                                    m24m01e_ascan_hdr_t *hdr,
                                    uint16_t *data);

/**
 * @brief  Read the header of a specific slot (for directory listings).
 * @return M24M01E_OK, M24M01E_ERR_NOT_FOUND if the slot holds no valid
 *         record, M24M01E_ERR_RANGE for a bad slot index.
 */
m24m01e_status_t m24m01e_ascan_slot_header(m24m01e_t *dev, uint16_t slot,
                                           m24m01e_ascan_hdr_t *hdr);

#ifdef __cplusplus
}
#endif

#endif /* M24M01E_H */