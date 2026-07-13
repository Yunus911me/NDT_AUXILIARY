/**
 * @file    m24m01e.c
 * @brief   Implementation of the M24M01E-F 1-Mbit I2C EEPROM driver,
 *          plus the slotted A-scan record storage layer.
 * @see     m24m01e.h  (ST datasheet DS13858 Rev 4)
 */

#include "m24m01e.h"

/* ------------------------------------------------------------------------- */
/* Device-select code construction (datasheet Table 9)                       */
/*                                                                           */
/*   Memory : 1 0 1 0 | C2 C1 A16 | RW   -> 7-bit base 0x50 | (ce<<1) | A16  */
/*   Feature: 1 0 1 1 | C2 C1 X   | RW   -> 7-bit base 0x58 | (ce<<1)        */
/* ------------------------------------------------------------------------- */

#define DSEL_MEM_BASE      (0x50u)  /* 1010 000b */
#define DSEL_FEATURE_BASE  (0x58u)  /* 1011 000b */

/* First-address-byte high-3-bit selectors for feature access (Table 10). */
#define FA_DTI     (0xE0u) // Device Type Identifier
#define FA_CDA     (0xC0u) // Configurable device address
#define FA_SWP     (0xA0u) // Software write protection 
#define FA_ID      (0x00u) // Identification page 
#define FA_ID_LOCK (0x60u) // Identification page lock 

#define DEFAULT_POLL_TRIES (32u)


/* ------------------------------------------------------------------------- */
/* Helper Functions                                                          */
/* ------------------------------------------------------------------------- */

// Builds the 7-bit device select code for memory access from a given address
static inline uint8_t mem_dsel7(const m24m01e_t *dev, uint32_t addr) 
{
    uint8_t a16 = (uint8_t)((addr >> 16) & 0x1u);
    return (uint8_t)(DSEL_MEM_BASE | ((dev->ce & 0x3u) << 1) | a16);
}

// Builds the 7-bit device select code for feature register access
static inline uint8_t feature_dsel7(const m24m01e_t *dev)
{
    return (uint8_t)(DSEL_FEATURE_BASE | ((dev->ce & 0x3u) << 1));
}

// Converts low-level I/O return codes to standardized M24M01E status codes
static inline m24m01e_status_t map_io(int rc)
{
    if (rc == 0)                return M24M01E_OK;
    if (rc == M24M01E_ERR_NACK) return M24M01E_ERR_NACK;
    return M24M01E_ERR_IO;
}

/* ------------------------------------------------------------------------- */
/* Init / probe / ready                                                      */
/* ------------------------------------------------------------------------- */

// Initializes the device context with I/O interface and chip enable address
m24m01e_status_t m24m01e_init(m24m01e_t *dev, const m24m01e_io_t *io, uint8_t ce)
{
    // io : local copy of the hardware interface map, ce : Chip Enable Address, poll_tries : 
    if (dev == NULL || io == NULL || io->write == NULL || io->write_read == NULL)
        return M24M01E_ERR_PARAM;
    if (ce > 3u)
        return M24M01E_ERR_PARAM;

    dev->io         = *io;
    dev->ce         = ce;
    dev->poll_tries = DEFAULT_POLL_TRIES;
    return M24M01E_OK;
}

// Probes the device by sending a start condition and checking for acknowledgment
m24m01e_status_t m24m01e_probe(m24m01e_t *dev)
{
    if (dev == NULL) return M24M01E_ERR_PARAM;
    int rc = dev->io.write(dev->io.ctx, mem_dsel7(dev, 0u), NULL, 0u);
    return map_io(rc);
}

// Polls the device until it becomes ready (internal write cycle completes)
m24m01e_status_t m24m01e_wait_ready(m24m01e_t *dev)
{
    if (dev == NULL) return M24M01E_ERR_PARAM;

    uint16_t tries = dev->poll_tries ? dev->poll_tries : DEFAULT_POLL_TRIES;
    uint8_t  addr7 = mem_dsel7(dev, 0u);

    for (uint16_t i = 0; i < tries; ++i) {
        if (dev->io.write(dev->io.ctx, addr7, NULL, 0u) == 0)
            return M24M01E_OK;
        if (dev->io.delay_ms != NULL)
            dev->io.delay_ms(dev->io.ctx, 1u);
    }
    return M24M01E_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------------- */
/* Memory read                                                               */
/* ------------------------------------------------------------------------- */

// Reads a block of data from the M24M01E memory starting at the specified address
m24m01e_status_t m24m01e_read(m24m01e_t *dev, uint32_t addr,
                              uint8_t *buf, size_t len)
{
    if (dev == NULL || buf == NULL)                  return M24M01E_ERR_PARAM;
    if (len == 0u)                                   return M24M01E_OK;
    if (addr > M24M01E_ADDR_MAX)                     return M24M01E_ERR_RANGE;
    if ((uint32_t)(addr + len) > M24M01E_SIZE_BYTES) return M24M01E_ERR_RANGE;

    /* A16 lives in the device-select byte, so restart at the 64 KB edge. */
    // Read in chunks that don't cross 64KB page boundaries (where A16 would change)
    while (len > 0u) {
        uint32_t half_end = ((addr >> 16) + 1u) << 16;  // Next 64KB boundary
        uint32_t chunk    = half_end - addr;            // Bytes until boundary
        if (chunk > len) chunk = (uint32_t)len;

        // Send 16-bit memory address (most significant byte first)
        uint8_t addr_bytes[2] = {
            (uint8_t)((addr >> 8) & 0xFFu),
            (uint8_t)(addr & 0xFFu)
        };

        // Perform the I/O operation: send address then read data
        int rc = dev->io.write_read(dev->io.ctx, mem_dsel7(dev, addr),
                                    addr_bytes, 2u, buf, (uint16_t)chunk);
        m24m01e_status_t st = map_io(rc);
        if (st != M24M01E_OK) return st;

        // Advance pointers and decrement remaining length
        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    return M24M01E_OK;
}

// Convenience wrapper to read a single byte from memory
m24m01e_status_t m24m01e_read_byte(m24m01e_t *dev, uint32_t addr, uint8_t *val)
{
    return m24m01e_read(dev, addr, val, 1u);
}

/* ------------------------------------------------------------------------- */
/* Memory write (page-aware)                                                 */
/* ------------------------------------------------------------------------- */

// Writes a chunk of data to a single memory page (handles page alignment internally)
static m24m01e_status_t write_page_chunk(m24m01e_t *dev, uint32_t addr,
                                         const uint8_t *buf, uint16_t len)
{
    // Build the write frame: 2-byte address + data payload
    uint8_t frame[2 + M24M01E_PAGE_SIZE];
    frame[0] = (uint8_t)((addr >> 8) & 0xFFu);
    frame[1] = (uint8_t)(addr & 0xFFu);
    for (uint16_t i = 0; i < len; ++i)
        frame[2 + i] = buf[i];

    // Send the complete frame (address + data) to the device
    int rc = dev->io.write(dev->io.ctx, mem_dsel7(dev, addr),
                           frame, (uint16_t)(len + 2u));
    if (rc != 0) {
        // NACK typically means write-protect is enabled on the device
        return (rc == M24M01E_ERR_NACK) ? M24M01E_ERR_WRITE_PROTECT
                                        : M24M01E_ERR_IO;
    }
    // Wait for the internal write cycle to complete
    return m24m01e_wait_ready(dev);
}

// Writes a block of data to memory, automatically handling page boundaries
m24m01e_status_t m24m01e_write(m24m01e_t *dev, uint32_t addr,
                               const uint8_t *buf, size_t len)
{
    if (dev == NULL || buf == NULL)                  return M24M01E_ERR_PARAM;
    if (len == 0u)                                   return M24M01E_OK;
    if (addr > M24M01E_ADDR_MAX)                     return M24M01E_ERR_RANGE;
    if ((uint32_t)(addr + len) > M24M01E_SIZE_BYTES) return M24M01E_ERR_RANGE;

    // Write in chunks that don't cross page boundaries (max PAGE_SIZE bytes per write)
    while (len > 0u) {
        uint32_t page_room = M24M01E_PAGE_SIZE - (addr % M24M01E_PAGE_SIZE);
        uint32_t chunk     = (page_room < len) ? page_room : (uint32_t)len;

        m24m01e_status_t st = write_page_chunk(dev, addr, buf, (uint16_t)chunk);
        if (st != M24M01E_OK) return st;

        // Advance pointers and decrement remaining length
        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    return M24M01E_OK;
}

// Convenience wrapper to write a single byte to memory
m24m01e_status_t m24m01e_write_byte(m24m01e_t *dev, uint32_t addr, uint8_t val)
{
    return m24m01e_write(dev, addr, &val, 1u);
}

/* ------------------------------------------------------------------------- */
/* Identification page                                                       */
/* ------------------------------------------------------------------------- */

// Reads data from the Identification Page (separate 256-byte memory area)
m24m01e_status_t m24m01e_id_page_read(m24m01e_t *dev, uint8_t offset,
                                      uint8_t *buf, size_t len)
{
    if (dev == NULL || buf == NULL)                    return M24M01E_ERR_PARAM;
    if (len == 0u)                                     return M24M01E_OK;
    if ((uint32_t)offset + len > M24M01E_ID_PAGE_SIZE) return M24M01E_ERR_RANGE;

    // Send Feature Address (FA_ID) + offset to read from identification page
    uint8_t addr_bytes[2] = { FA_ID, offset };
    int rc = dev->io.write_read(dev->io.ctx, feature_dsel7(dev),
                                addr_bytes, 2u, buf, (uint16_t)len);
    return map_io(rc);
}

// Writes data to the Identification Page (can be locked permanently)
m24m01e_status_t m24m01e_id_page_write(m24m01e_t *dev, uint8_t offset,
                                       const uint8_t *buf, size_t len)
{
    if (dev == NULL || buf == NULL)                    return M24M01E_ERR_PARAM;
    if (len == 0u)                                     return M24M01E_OK;
    if ((uint32_t)offset + len > M24M01E_ID_PAGE_SIZE) return M24M01E_ERR_RANGE;

    // Build frame: Feature Address + offset + data payload
    uint8_t frame[2 + M24M01E_ID_PAGE_SIZE];
    frame[0] = FA_ID;
    frame[1] = offset;
    for (size_t i = 0; i < len; ++i)
        frame[2 + i] = buf[i];

    int rc = dev->io.write(dev->io.ctx, feature_dsel7(dev),
                           frame, (uint16_t)(len + 2u));
    if (rc != 0) {
        return (rc == M24M01E_ERR_NACK) ? M24M01E_ERR_WRITE_PROTECT
                                        : M24M01E_ERR_IO;
    }
    return m24m01e_wait_ready(dev);
}

// Permanently locks the Identification Page (write-protects it forever)
m24m01e_status_t m24m01e_id_page_lock(m24m01e_t *dev)
{
    if (dev == NULL) return M24M01E_ERR_PARAM;

    // Send lock command: FA_ID_LOCK + dummy byte + lock value (0x02)
    uint8_t frame[3] = { FA_ID_LOCK, 0x00u, 0x02u };
    int rc = dev->io.write(dev->io.ctx, feature_dsel7(dev), frame, 3u);
    if (rc != 0) {
        return (rc == M24M01E_ERR_NACK) ? M24M01E_ERR_WRITE_PROTECT
                                        : M24M01E_ERR_IO;
    }
    return m24m01e_wait_ready(dev);
}

// Checks whether the Identification Page has been permanently locked
m24m01e_status_t m24m01e_id_page_is_locked(m24m01e_t *dev, bool *locked)
{
    if (dev == NULL || locked == NULL) return M24M01E_ERR_PARAM;
    if (dev->io.probe_no_stop == NULL) return M24M01E_ERR_UNSUPPORTED;

    // Send probe command to check lock status without generating a STOP condition
    uint8_t frame[3] = { FA_ID_LOCK, 0x00u, 0x00u };
    int rc = dev->io.probe_no_stop(dev->io.ctx, feature_dsel7(dev), frame, 3u);
    *locked = (rc != 0);  // Non-zero response indicates the page is locked
    return M24M01E_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * A-scan record storage layer
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Header page byte layout (little-endian; unlisted bytes are don't-care):
 *   [ 0] magic0 'N'      [ 1] magic1 'D'      [ 2] version
 *   [ 4..5 ] id          [ 6..9 ] seq         [10..11] burst_cycles
 *   [12..15] freq_hz     [16..17] sample_period_ns
 *   [18..21] start_delay_ns                   [22..23] samples_per_ch
 *   [24..25] channels    [26..27] status_flags
 *   [28..29] checksum (uint16 additive sum of all sample words)
 */

#define ASCAN_HDR_RAW_LEN   (30u)

/* C28x-safe LE pack/unpack helpers (mask every byte to 8 bits). */
// Pack 16-bit value into little-endian byte array
static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

// Pack 32-bit value into little-endian byte array
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8)  & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

// Unpack 16-bit value from little-endian byte array
static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((p[0] & 0xFFu) | ((uint16_t)(p[1] & 0xFFu) << 8));
}

// Unpack 32-bit value from little-endian byte array
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)(p[0] & 0xFFu)
         | ((uint32_t)(p[1] & 0xFFu) << 8)
         | ((uint32_t)(p[2] & 0xFFu) << 16)
         | ((uint32_t)(p[3] & 0xFFu) << 24);
}

// Calculate starting address of a slot in memory
static inline uint32_t slot_addr(uint16_t slot)
{
    return (uint32_t)slot * M24M01E_ASCAN_SLOT_BYTES;
}

// Compute additive checksum over all sample data words
static uint16_t ascan_checksum(const uint16_t *data)
{
    uint32_t sum = 0u;
    uint32_t n   = (uint32_t)M24M01E_ASCAN_SAMPLES * M24M01E_ASCAN_CHANNELS;
    uint32_t i;
    for (i = 0u; i < n; ++i) sum += data[i];
    return (uint16_t)(sum & 0xFFFFu);
}

/* Read + parse a slot header. Returns OK (valid), NOT_FOUND (blank/invalid
 * magic), or a transport error. checksum output is optional. */
// Reads and parses the header from a specific slot
static m24m01e_status_t ascan_read_hdr(m24m01e_t *dev, uint16_t slot,
                                       m24m01e_ascan_hdr_t *hdr,
                                       uint16_t *checksum)
{
    uint8_t raw[ASCAN_HDR_RAW_LEN];
    m24m01e_status_t st = m24m01e_read(dev, slot_addr(slot),
                                       raw, ASCAN_HDR_RAW_LEN);
    if (st != M24M01E_OK) return st;

    // Validate magic bytes and version
    if ((raw[0] & 0xFFu) != M24M01E_ASCAN_MAGIC0 ||
        (raw[1] & 0xFFu) != M24M01E_ASCAN_MAGIC1 ||
        (raw[2] & 0xFFu) != M24M01E_ASCAN_VERSION)
        return M24M01E_ERR_NOT_FOUND;

    // Parse header fields from raw bytes if requested
    if (hdr != NULL) {
        hdr->id               = get16(&raw[4]);
        hdr->seq              = get32(&raw[6]);
        hdr->burst_cycles     = get16(&raw[10]);
        hdr->freq_hz          = get32(&raw[12]);
        hdr->sample_period_ns = get16(&raw[16]);
        hdr->start_delay_ns   = get32(&raw[18]);
        hdr->samples_per_ch   = get16(&raw[22]);
        hdr->channels         = get16(&raw[24]);
        hdr->status_flags     = get16(&raw[26]);
    }
    if (checksum != NULL) *checksum = get16(&raw[28]);
    return M24M01E_OK;
}

// Retrieves the header information for a specific slot
m24m01e_status_t m24m01e_ascan_slot_header(m24m01e_t *dev, uint16_t slot,
                                           m24m01e_ascan_hdr_t *hdr)
{
    if (dev == NULL || hdr == NULL)         return M24M01E_ERR_PARAM;
    if (slot >= M24M01E_ASCAN_SLOT_COUNT)   return M24M01E_ERR_RANGE;
    return ascan_read_hdr(dev, slot, hdr, NULL);
}

// Finds the slot containing a specific A-scan record ID
m24m01e_status_t m24m01e_ascan_find(m24m01e_t *dev, uint16_t id,
                                    uint16_t *slot)
{
    if (dev == NULL || slot == NULL) return M24M01E_ERR_PARAM;

    m24m01e_ascan_hdr_t h;
    uint16_t s;
    // Scan all slots looking for matching ID
    for (s = 0u; s < M24M01E_ASCAN_SLOT_COUNT; ++s) {
        m24m01e_status_t st = ascan_read_hdr(dev, s, &h, NULL);
        if (st == M24M01E_OK && h.id == id) {
            *slot = s;
            return M24M01E_OK;
        }
        if (st != M24M01E_OK && st != M24M01E_ERR_NOT_FOUND)
            return st;                     /* real transport error */
    }
    return M24M01E_ERR_NOT_FOUND;
}

// Saves A-scan data with header, handling slot allocation and eviction
m24m01e_status_t m24m01e_ascan_save(m24m01e_t *dev,
                                    m24m01e_ascan_hdr_t *hdr,
                                    const uint16_t *data)
{
    if (dev == NULL || hdr == NULL || data == NULL) return M24M01E_ERR_PARAM;
    if (hdr->samples_per_ch != M24M01E_ASCAN_SAMPLES ||
        hdr->channels       != M24M01E_ASCAN_CHANNELS)
        return M24M01E_ERR_PARAM;

    /* ── Pass 1: scan the directory for target slot + next sequence no. ── */
    // Find target slot (existing match, free slot, or oldest for eviction)
    int32_t  match = -1, freeslot = -1, oldest = 0;
    uint32_t max_seq = 0u, min_seq = 0xFFFFFFFFu;
    uint16_t s;

    for (s = 0u; s < M24M01E_ASCAN_SLOT_COUNT; ++s) {
        m24m01e_ascan_hdr_t h;
        m24m01e_status_t st = ascan_read_hdr(dev, s, &h, NULL);

        if (st == M24M01E_ERR_NOT_FOUND) {
            if (freeslot < 0) freeslot = (int32_t)s;
            continue;
        }
        if (st != M24M01E_OK) return st;

        // Track sequence numbers for aging/eviction
        if (h.seq > max_seq) max_seq = h.seq;
        if (h.seq < min_seq) { min_seq = h.seq; oldest = (int32_t)s; }
        if (h.id == hdr->id && match < 0) match = (int32_t)s;
    }

    // Select target slot: existing match > free slot > oldest (evict)
    uint16_t target = (match >= 0)    ? (uint16_t)match
                    : (freeslot >= 0) ? (uint16_t)freeslot
                                      : (uint16_t)oldest;   /* evict oldest */
    hdr->seq = max_seq + 1u;

    /* ── Header page ─────────────────────────────────────────────────────── */
    // Build header raw bytes with checksum
    uint8_t raw[ASCAN_HDR_RAW_LEN];
    uint16_t i;
    for (i = 0u; i < ASCAN_HDR_RAW_LEN; ++i) raw[i] = 0u;

    raw[0] = M24M01E_ASCAN_MAGIC0;
    raw[1] = M24M01E_ASCAN_MAGIC1;
    raw[2] = M24M01E_ASCAN_VERSION;
    put16(&raw[4],  hdr->id);
    put32(&raw[6],  hdr->seq);
    put16(&raw[10], hdr->burst_cycles);
    put32(&raw[12], hdr->freq_hz);
    put16(&raw[16], hdr->sample_period_ns);
    put32(&raw[18], hdr->start_delay_ns);
    put16(&raw[22], hdr->samples_per_ch);
    put16(&raw[24], hdr->channels);
    put16(&raw[26], hdr->status_flags);
    put16(&raw[28], ascan_checksum(data));

    // Write header to device
    m24m01e_status_t st = m24m01e_write(dev, slot_addr(target),
                                        raw, ASCAN_HDR_RAW_LEN);
    if (st != M24M01E_OK) return st;

    /* ── Data pages: 128 sample words → one 256-byte page each ───────────── */
    // Write sample data in page-sized chunks (128 words per page)
    {
        uint8_t  stage[M24M01E_PAGE_SIZE];
        uint32_t addr = slot_addr(target) + M24M01E_PAGE_SIZE;
        uint32_t w    = 0u;                         /* word index into data  */
        uint16_t page, k;

        for (page = 0u; page < M24M01E_ASCAN_DATA_PAGES; ++page) {
            // Pack 16-bit words into bytes
            for (k = 0u; k < (M24M01E_PAGE_SIZE / 2u); ++k, ++w) {
                stage[2u * k]      = (uint8_t)(data[w] & 0xFFu);
                stage[2u * k + 1u] = (uint8_t)((data[w] >> 8) & 0xFFu);
            }
            st = m24m01e_write(dev, addr, stage, M24M01E_PAGE_SIZE);
            if (st != M24M01E_OK) return st;
            addr += M24M01E_PAGE_SIZE;
        }
    }
    return M24M01E_OK;
}

// Loads A-scan data by ID, returning header and sample data
m24m01e_status_t m24m01e_ascan_load(m24m01e_t *dev, uint16_t id,
                                    m24m01e_ascan_hdr_t *hdr,
                                    uint16_t *data)
{
    if (dev == NULL || hdr == NULL) return M24M01E_ERR_PARAM;

    // Find the slot containing this ID
    uint16_t slot;
    m24m01e_status_t st = m24m01e_ascan_find(dev, id, &slot);
    if (st != M24M01E_OK) return st;

    // Read and parse the header
    uint16_t stored_ck;
    st = ascan_read_hdr(dev, slot, hdr, &stored_ck);
    if (st != M24M01E_OK) return st;

    if (data == NULL) return M24M01E_OK;   /* header-only query */

    /* Data pages back into words, one page per iteration. */
    // Read data pages and unpack bytes back into 16-bit words
    {
        uint8_t  stage[M24M01E_PAGE_SIZE];
        uint32_t addr = slot_addr(slot) + M24M01E_PAGE_SIZE;
        uint32_t w    = 0u;
        uint16_t page, k;

        for (page = 0u; page < M24M01E_ASCAN_DATA_PAGES; ++page) {
            st = m24m01e_read(dev, addr, stage, M24M01E_PAGE_SIZE);
            if (st != M24M01E_OK) return st;
            // Unpack bytes to 16-bit words
            for (k = 0u; k < (M24M01E_PAGE_SIZE / 2u); ++k, ++w) {
                data[w] = (uint16_t)((stage[2u * k] & 0xFFu) |
                          ((uint16_t)(stage[2u * k + 1u] & 0xFFu) << 8));
            }
            addr += M24M01E_PAGE_SIZE;
        }
    }

    // Verify data integrity with checksum
    return (ascan_checksum(data) == stored_ck) ? M24M01E_OK
                                               : M24M01E_ERR_CORRUPT;
}

/* ------------------------------------------------------------------------- */
/* Misc                                                                      */
/* ------------------------------------------------------------------------- */

// Converts a status code to a human-readable error message string
const char *m24m01e_strerror(m24m01e_status_t s)
{
    switch (s) {
    case M24M01E_OK:                return "ok";
    case M24M01E_ERR_PARAM:         return "invalid parameter";
    case M24M01E_ERR_RANGE:         return "address/length out of range";
    case M24M01E_ERR_IO:            return "i2c bus error";
    case M24M01E_ERR_NACK:          return "no acknowledge";
    case M24M01E_ERR_WRITE_PROTECT: return "write protected";
    case M24M01E_ERR_TIMEOUT:       return "write-cycle timeout";
    case M24M01E_ERR_UNSUPPORTED:   return "unsupported (missing io callback)";
    case M24M01E_ERR_NOT_FOUND:     return "record not found";
    case M24M01E_ERR_CORRUPT:       return "record checksum mismatch";
    default:                        return "unknown";
    }
}