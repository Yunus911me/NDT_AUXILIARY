/**
 * @file  ndt_store.c
 * @brief Slotted A-scan record store — implementation.
 * @see   ndt_store.h
 *
 * Header page byte layout (little-endian; unlisted bytes are don't-care):
 *   [ 0] magic0 'N'      [ 1] magic1 'D'      [ 2] version
 *   [ 4.. 5] id          [ 6.. 9] seq         [10..11] burst_cycles
 *   [12..15] freq_hz     [16..17] sample_period_ns
 *   [18..21] start_delay_ns                   [22..23] samples_per_ch
 *   [24..25] channels    [26..27] status_flags
 *   [28..29] checksum (uint16 additive sum over all sample words)
 *
 * C28x note: uint8_t is a 16-bit container; every byte is masked with 0xFF
 * on the way in and on the way out.
 */

#include <stddef.h>
#include "ndt_store.h"

#define HDR_RAW_LEN     (30u)
#define STORE_MAGIC0    (0x4Eu)   /* 'N' */
#define STORE_MAGIC1    (0x44u)   /* 'D' */
#define STORE_VERSION   (0x01u)

/* ── little-endian pack / unpack ─────────────────────────────────────────── */

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)( v        & 0xFFu);
    p[1] = (uint8_t)((v >> 8)  & 0xFFu);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)( v        & 0xFFu);
    p[1] = (uint8_t)((v >> 8)  & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((p[0] & 0xFFu) | ((uint16_t)(p[1] & 0xFFu) << 8));
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)(p[0] & 0xFFu)
         | ((uint32_t)(p[1] & 0xFFu) << 8)
         | ((uint32_t)(p[2] & 0xFFu) << 16)
         | ((uint32_t)(p[3] & 0xFFu) << 24);
}

/* ── helpers ─────────────────────────────────────────────────────────────── */

/* Map a chip-driver status onto a store status. Everything the store cannot
 * act on differently collapses to ERR_IO; the caller logs the detail via the
 * driver's own strerror if it wants it. */
static ndt_store_status_t map_dev(m24m01e_status_t s)
{
    switch (s) {
    case M24M01E_OK:            return NDT_STORE_OK;
    case M24M01E_ERR_PARAM:     return NDT_STORE_ERR_PARAM;
    case M24M01E_ERR_RANGE:     return NDT_STORE_ERR_RANGE;
    default:                    return NDT_STORE_ERR_IO;
    }
}

static uint32_t slot_addr(uint16_t slot)
{
    return (uint32_t)slot * NDT_STORE_SLOT_BYTES;
}

static uint16_t ascan_checksum(const uint16_t *data)
{
    uint32_t sum = 0u;
    uint32_t n   = (uint32_t)NDT_ASCAN_SAMPLES * NDT_ASCAN_CHANNELS;
    uint32_t i;

    for (i = 0u; i < n; ++i) { sum += data[i]; }
    return (uint16_t)(sum & 0xFFFFu);
}

/* Read + parse one slot header.
 * Returns OK (valid record), ERR_NOT_FOUND (blank / bad magic), or an IO
 * status. Both outputs are optional. */
static ndt_store_status_t read_hdr(ndt_store_t *st, uint16_t slot,
                                   ndt_ascan_hdr_t *hdr, uint16_t *checksum)
{
    uint8_t raw[HDR_RAW_LEN];
    ndt_store_status_t rc = map_dev(m24m01e_read(st->dev, slot_addr(slot),
                                                 raw, HDR_RAW_LEN));
    if (rc != NDT_STORE_OK) { return rc; }

    if ((raw[0] & 0xFFu) != STORE_MAGIC0 ||
        (raw[1] & 0xFFu) != STORE_MAGIC1 ||
        (raw[2] & 0xFFu) != STORE_VERSION)
    {
        return NDT_STORE_ERR_NOT_FOUND;
    }

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
    if (checksum != NULL) { *checksum = get16(&raw[28]); }

    return NDT_STORE_OK;
}

/* ── API ─────────────────────────────────────────────────────────────────── */

ndt_store_status_t ndt_store_init(ndt_store_t *st, m24m01e_t *dev)
{
    if (st == NULL || dev == NULL) { return NDT_STORE_ERR_PARAM; }
    st->dev = dev;
    return NDT_STORE_OK;
}

ndt_store_status_t ndt_store_slot_header(ndt_store_t *st, uint16_t slot,
                                         ndt_ascan_hdr_t *hdr)
{
    if (st == NULL || st->dev == NULL || hdr == NULL) {
        return NDT_STORE_ERR_PARAM;
    }
    if (slot >= NDT_STORE_SLOT_COUNT) { return NDT_STORE_ERR_RANGE; }
    return read_hdr(st, slot, hdr, NULL);
}

ndt_store_status_t ndt_store_find(ndt_store_t *st, uint16_t id, uint16_t *slot)
{
    ndt_ascan_hdr_t h;
    uint16_t s;

    if (st == NULL || st->dev == NULL || slot == NULL) {
        return NDT_STORE_ERR_PARAM;
    }

    for (s = 0u; s < NDT_STORE_SLOT_COUNT; ++s) {
        ndt_store_status_t rc = read_hdr(st, s, &h, NULL);

        if (rc == NDT_STORE_OK && h.id == id) {
            *slot = s;
            return NDT_STORE_OK;
        }
        if (rc != NDT_STORE_OK && rc != NDT_STORE_ERR_NOT_FOUND) {
            return rc;                       /* real bus error — give up */
        }
    }
    return NDT_STORE_ERR_NOT_FOUND;
}

ndt_store_status_t ndt_store_save(ndt_store_t *st,
                                  ndt_ascan_hdr_t *hdr,
                                  const uint16_t *data)
{
    int32_t  match = -1, freeslot = -1, oldest = 0;
    uint32_t max_seq = 0u, min_seq = 0xFFFFFFFFu;
    uint16_t s, target;
    uint8_t  raw[HDR_RAW_LEN];
    uint16_t i;
    ndt_store_status_t rc;

    if (st == NULL || st->dev == NULL || hdr == NULL || data == NULL) {
        return NDT_STORE_ERR_PARAM;
    }
    if (hdr->samples_per_ch != NDT_ASCAN_SAMPLES ||
        hdr->channels       != NDT_ASCAN_CHANNELS)
    {
        return NDT_STORE_ERR_PARAM;
    }

    /* ── Pass 1: directory scan — target slot and next sequence number ──── */
    for (s = 0u; s < NDT_STORE_SLOT_COUNT; ++s) {
        ndt_ascan_hdr_t h;
        rc = read_hdr(st, s, &h, NULL);

        if (rc == NDT_STORE_ERR_NOT_FOUND) {
            if (freeslot < 0) { freeslot = (int32_t)s; }
            continue;
        }
        if (rc != NDT_STORE_OK) { return rc; }

        if (h.seq > max_seq) { max_seq = h.seq; }
        if (h.seq < min_seq) { min_seq = h.seq; oldest = (int32_t)s; }
        if (h.id == hdr->id && match < 0) { match = (int32_t)s; }
    }

    target = (match    >= 0) ? (uint16_t)match
           : (freeslot >= 0) ? (uint16_t)freeslot
                             : (uint16_t)oldest;      /* evict oldest */
    hdr->seq = max_seq + 1u;

    /* ── Header page ──────────────────────────────────────────────────────── */
    for (i = 0u; i < HDR_RAW_LEN; ++i) { raw[i] = 0u; }

    raw[0] = STORE_MAGIC0;
    raw[1] = STORE_MAGIC1;
    raw[2] = STORE_VERSION;
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

    rc = map_dev(m24m01e_write(st->dev, slot_addr(target), raw, HDR_RAW_LEN));
    if (rc != NDT_STORE_OK) { return rc; }

    /* ── Data pages: 128 sample words fill one 256-byte page ──────────────── */
    {
        uint8_t  stage[NDT_STORE_PAGE_SIZE];
        uint32_t addr = slot_addr(target) + NDT_STORE_PAGE_SIZE;
        uint32_t w    = 0u;
        uint16_t page, k;

        for (page = 0u; page < NDT_STORE_DATA_PAGES; ++page) {
            for (k = 0u; k < (NDT_STORE_PAGE_SIZE / 2u); ++k, ++w) {
                stage[2u * k]      = (uint8_t)( data[w]        & 0xFFu);
                stage[2u * k + 1u] = (uint8_t)((data[w] >> 8)  & 0xFFu);
            }
            rc = map_dev(m24m01e_write(st->dev, addr, stage,
                                       NDT_STORE_PAGE_SIZE));
            if (rc != NDT_STORE_OK) { return rc; }
            addr += NDT_STORE_PAGE_SIZE;
        }
    }

    return NDT_STORE_OK;
}

ndt_store_status_t ndt_store_load(ndt_store_t *st, uint16_t id,
                                  ndt_ascan_hdr_t *hdr, uint16_t *data)
{
    uint16_t slot, stored_ck;
    ndt_store_status_t rc;

    if (st == NULL || st->dev == NULL || hdr == NULL) {
        return NDT_STORE_ERR_PARAM;
    }

    rc = ndt_store_find(st, id, &slot);
    if (rc != NDT_STORE_OK) { return rc; }

    rc = read_hdr(st, slot, hdr, &stored_ck);
    if (rc != NDT_STORE_OK) { return rc; }

    if (data == NULL) { return NDT_STORE_OK; }   /* header-only query */

    {
        uint8_t  stage[NDT_STORE_PAGE_SIZE];
        uint32_t addr = slot_addr(slot) + NDT_STORE_PAGE_SIZE;
        uint32_t w    = 0u;
        uint16_t page, k;

        for (page = 0u; page < NDT_STORE_DATA_PAGES; ++page) {
            rc = map_dev(m24m01e_read(st->dev, addr, stage,
                                      NDT_STORE_PAGE_SIZE));
            if (rc != NDT_STORE_OK) { return rc; }

            for (k = 0u; k < (NDT_STORE_PAGE_SIZE / 2u); ++k, ++w) {
                data[w] = (uint16_t)((stage[2u * k] & 0xFFu) |
                          ((uint16_t)(stage[2u * k + 1u] & 0xFFu) << 8));
            }
            addr += NDT_STORE_PAGE_SIZE;
        }
    }

    return (ascan_checksum(data) == stored_ck) ? NDT_STORE_OK
                                               : NDT_STORE_ERR_CORRUPT;
}

void ndt_store_hdr_to_stream(const ndt_ascan_hdr_t *hdr, volatile uint16_t *out)
{
    if (hdr == NULL || out == NULL) { return; }

    out[0]  = (uint16_t)( hdr->id                    & 0xFFu);
    out[1]  = (uint16_t)((hdr->id              >> 8) & 0xFFu);
    out[2]  = (uint16_t)( hdr->seq                   & 0xFFu);
    out[3]  = (uint16_t)((hdr->seq             >> 8) & 0xFFu);
    out[4]  = (uint16_t)((hdr->seq            >> 16) & 0xFFu);
    out[5]  = (uint16_t)((hdr->seq            >> 24) & 0xFFu);
    out[6]  = (uint16_t)( hdr->burst_cycles          & 0xFFu);
    out[7]  = (uint16_t)((hdr->burst_cycles    >> 8) & 0xFFu);
    out[8]  = (uint16_t)( hdr->samples_per_ch        & 0xFFu);
    out[9]  = (uint16_t)((hdr->samples_per_ch  >> 8) & 0xFFu);
    out[10] = (uint16_t)( hdr->sample_period_ns      & 0xFFu);
    out[11] = (uint16_t)((hdr->sample_period_ns>> 8) & 0xFFu);
    out[12] = (uint16_t)( hdr->start_delay_ns        & 0xFFu);
    out[13] = (uint16_t)((hdr->start_delay_ns  >> 8) & 0xFFu);
    out[14] = (uint16_t)((hdr->start_delay_ns >> 16) & 0xFFu);
    out[15] = (uint16_t)((hdr->start_delay_ns >> 24) & 0xFFu);
}

const char *ndt_store_strerror(ndt_store_status_t s)
{
    switch (s) {
    case NDT_STORE_OK:            return "ok";
    case NDT_STORE_ERR_PARAM:     return "invalid parameter";
    case NDT_STORE_ERR_RANGE:     return "slot out of range";
    case NDT_STORE_ERR_IO:        return "eeprom i/o error";
    case NDT_STORE_ERR_NOT_FOUND: return "record not found";
    case NDT_STORE_ERR_CORRUPT:   return "record checksum mismatch";
    default:                      return "unknown";
    }
}
