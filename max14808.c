/**
 * @file max14808.c
 * @brief Driver for MAX14808/MAX14809 Octal Three-Level / Quad Five-Level
 *        High-Voltage 2A Digital Pulsers with T/R Switch
 *
 * All timing requirements are derived from the datasheet (Rev 2, Jan 2014):
 *   - Output enable from shutdown  : 100 µs max  (tEN1)
 *   - Output disable to shutdown   : 10  µs max  (tDIS1)
 *   - T/R switch turn-on           : 1.2 µs max  (tONTRSW)
 *   - Recommended T/R switch pre-off before TX burst : 3 µs
 *   - Transparent-mode propagation delay : 18 ns (no SW wait needed)
 */
#include "max14808.h"

/* ---------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------*/

/**
 * @brief Write MODE0/MODE1 pins to set the operating mode.
 */
static void _write_mode_pins(max14808_dev_t *dev, max14808_mode_t mode)
{
    uint8_t m0 = (mode == MAX14808_MODE_OCTAL_3LEVEL || mode == MAX14808_MODE_TX_DISABLE) ? 1u : 0u;
    uint8_t m1 = (mode == MAX14808_MODE_QUAD_5LEVEL  || mode == MAX14808_MODE_TX_DISABLE) ? 1u : 0u;

    dev->gpio_write(dev->pins.mode0, m0);
    dev->gpio_write(dev->pins.mode1, m1);
}

/**
 * @brief Write CC0/CC1 pins to set the current drive.
 */
static void _write_cc_pins(max14808_dev_t *dev, max14808_current_t current)
{
    uint8_t cc0 = (current == MAX14808_CURRENT_1P5A || current == MAX14808_CURRENT_0P5A) ? 1u : 0u;
    uint8_t cc1 = (current == MAX14808_CURRENT_1A   || current == MAX14808_CURRENT_0P5A) ? 1u : 0u;

    dev->gpio_write(dev->pins.cc0, cc0);
    dev->gpio_write(dev->pins.cc1, cc1);
}

/**
 * @brief Drive DINP/DINN for a single channel (0-indexed).
 */
static void _write_channel(max14808_dev_t *dev, uint8_t ch_idx, uint8_t dinp, uint8_t dinn)
{
    if (dev->pins.dinp[ch_idx] != MAX14808_PIN_NC) {
        dev->gpio_write(dev->pins.dinp[ch_idx], dinp);
    }
    if (dev->pins.dinn[ch_idx] != MAX14808_PIN_NC) {
        dev->gpio_write(dev->pins.dinn[ch_idx], dinn);
    }
}

/**
 * @brief Decode a three-level state to DINP/DINN values.
 */
static void _decode_3level(max14808_3level_out_t state, uint8_t *dinp, uint8_t *dinn)
{
    switch (state) {
        case MAX14808_OUT_CLAMP_OFF:   *dinp = 0u; *dinn = 0u; break;
        case MAX14808_OUT_VNN:         *dinp = 0u; *dinn = 1u; break;
        case MAX14808_OUT_VPP:         *dinp = 1u; *dinn = 0u; break;
        case MAX14808_OUT_CLAMP_DAMP:  *dinp = 1u; *dinn = 1u; break;
        default:                       *dinp = 0u; *dinn = 0u; break;
    }
}

/**
 * @brief Decode a five-level state to DINNx, DINPx, DINNy values.
 *        x = channels 1–4, y = channels 5–8.
 */
static void _decode_5level(max14808_5level_out_t state,
                            uint8_t *dinnx, uint8_t *dinpx, uint8_t *dinny)
{
    switch (state) {
        case MAX14808_5L_HI_Z:          *dinnx=0; *dinpx=0; *dinny=0; break;
        case MAX14808_5L_CLAMP:         *dinnx=0; *dinpx=0; *dinny=1; break;
        case MAX14808_5L_VPPB:          *dinnx=0; *dinpx=1; *dinny=0; break;
        case MAX14808_5L_VNNB:          *dinnx=1; *dinpx=0; *dinny=0; break;
        case MAX14808_5L_VPPA:          *dinnx=0; *dinpx=1; *dinny=1; break;
        case MAX14808_5L_VNNA:          *dinnx=1; *dinpx=0; *dinny=1; break;
        case MAX14808_5L_CLAMP_DAMP_TR: *dinnx=1; *dinpx=1; *dinny=1; break;
        default:                        *dinnx=0; *dinpx=0; *dinny=0; break;
    }
}

/* ---------------------------------------------------------------------------
 * Public API implementation
 * -------------------------------------------------------------------------*/

max14808_err_t max14808_init(max14808_dev_t            *dev,
                             max14808_variant_t         variant,
                             const max14808_pins_t     *pins,
                             max14808_gpio_write_fn     gpio_write,
                             max14808_gpio_read_fn      gpio_read,
                             max14808_delay_us_fn       delay_us)
{
    if (!dev || !pins || !gpio_write || !gpio_read || !delay_us) {
        return MAX14808_ERR_NULL;
    }
    if (variant != MAX14808_VARIANT_14808 && variant != MAX14808_VARIANT_14809) {
        return MAX14808_ERR_PARAM;
    }

    dev->variant      = variant;
    dev->pins         = *pins;
    dev->gpio_write   = gpio_write;
    dev->gpio_read    = gpio_read;
    dev->delay_us     = delay_us;
    dev->current_mode = MAX14808_MODE_SHUTDOWN;
    dev->current_drive= MAX14808_CURRENT_2A;
    dev->sync_mode    = MAX14808_SYNC_TRANSPARENT;

    /* Drive all data inputs low */
     
    for (uint8_t i = 0u; i < 8u; i++) {
        _write_channel(dev, i, 0u, 0u);
    }

    /* Default: internal LDO enabled (LDO_EN = low) */
    if (dev->pins.ldo_en != MAX14808_PIN_NC) {
        dev->gpio_write(dev->pins.ldo_en, 0u);
    }

    /* Default current drive: 2 A */
    _write_cc_pins(dev, MAX14808_CURRENT_2A);

    /* SYNC = low → transparent mode */
    if (dev->pins.sync != MAX14808_PIN_NC) {
        dev->gpio_write(dev->pins.sync, 0u);
    }

    /* Place device in shutdown mode */
    _write_mode_pins(dev, MAX14808_MODE_SHUTDOWN);

    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_set_mode(max14808_dev_t *dev, max14808_mode_t mode)
{
    if (!dev) return MAX14808_ERR_NULL;

    _write_mode_pins(dev, mode);

    if (mode != MAX14808_MODE_SHUTDOWN &&
        dev->current_mode == MAX14808_MODE_SHUTDOWN)
    {
        /* tEN1: max 100 µs for outputs to be valid after leaving shutdown */
        dev->delay_us(100u);
    }
    else if (mode == MAX14808_MODE_SHUTDOWN) {
        /* tDIS1: max 10 µs disable time */
        dev->delay_us(10u);
    }

    dev->current_mode = mode;
    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_set_current(max14808_dev_t *dev, max14808_current_t current)
{
    if (!dev) return MAX14808_ERR_NULL;
    if (current > MAX14808_CURRENT_0P5A) return MAX14808_ERR_PARAM;

    _write_cc_pins(dev, current);
    dev->current_drive = current;
    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_set_sync_mode(max14808_dev_t *dev, max14808_sync_mode_t mode)
{
    if (!dev) return MAX14808_ERR_NULL;

    uint8_t sync_val = (mode != MAX14808_SYNC_TRANSPARENT) ? 1u : 0u;

    if (dev->pins.sync != MAX14808_PIN_NC) {
        dev->gpio_write(dev->pins.sync, sync_val);
    }

    dev->sync_mode = mode;

    if (mode != MAX14808_SYNC_TRANSPARENT) {
        /* tEN3: max 4 µs to enter sync mode */
        dev->delay_us(4u);
    } else {
        /* tDIS3: max 500 ns to leave sync mode – round up to 1 µs */
        dev->delay_us(1u);
    }

    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_set_channel_3level(max14808_dev_t       *dev,
                                            uint8_t               channel,
                                            max14808_3level_out_t state)
{
    if (!dev) return MAX14808_ERR_NULL;
    if (channel < MAX14808_CH_MIN || channel > MAX14808_CH_MAX) return MAX14808_ERR_PARAM;
    if (dev->current_mode != MAX14808_MODE_OCTAL_3LEVEL) return MAX14808_ERR_MODE;

    uint8_t dinp, dinn;
    _decode_3level(state, &dinp, &dinn);
    _write_channel(dev, (uint8_t)(channel - 1u), dinp, dinn);
    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_set_channel_5level(max14808_dev_t       *dev,
                                            uint8_t               pair,
                                            max14808_5level_out_t state)
{
    if (!dev) return MAX14808_ERR_NULL;
    if (pair < 1u || pair > 4u) return MAX14808_ERR_PARAM;
    if (dev->current_mode != MAX14808_MODE_QUAD_5LEVEL) return MAX14808_ERR_MODE;

    uint8_t dinnx, dinpx, dinny;
    _decode_5level(state, &dinnx, &dinpx, &dinny);

    /* x-channel index: pairs 1–4 map to channels 1–4 (indices 0–3) */
    uint8_t idx_x = (uint8_t)(pair - 1u);
    /* y-channel index: pairs 1–4 map to channels 5–8 (indices 4–7) */
    uint8_t idx_y = (uint8_t)(pair - 1u + 4u);

    _write_channel(dev, idx_x, dinpx, dinnx);

    /* DINNy drives the supply select; DINPy is don't-care (drive low) */
    if (dev->pins.dinn[idx_y] != MAX14808_PIN_NC) {
        dev->gpio_write(dev->pins.dinn[idx_y], dinny);
    }
    if (dev->pins.dinp[idx_y] != MAX14808_PIN_NC) {
        dev->gpio_write(dev->pins.dinp[idx_y], 0u);  /* don't-care → low */
    }

    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_set_all_channels_3level(max14808_dev_t             *dev,
                                                 const max14808_3level_out_t states[8])
{
    if (!dev || !states) return MAX14808_ERR_NULL;
    if (dev->current_mode != MAX14808_MODE_OCTAL_3LEVEL) return MAX14808_ERR_MODE;

    for (uint8_t i = 0u; i < 8u; i++) {
        uint8_t dinp, dinn;
        _decode_3level(states[i], &dinp, &dinn);
        _write_channel(dev, i, dinp, dinn);
    }
    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_set_internal_ldo(max14808_dev_t *dev, bool enable)
{
    if (!dev) return MAX14808_ERR_NULL;

    if (dev->pins.ldo_en != MAX14808_PIN_NC) {
        /* LDO_EN high = disable internal LDO; low = enable */
        dev->gpio_write(dev->pins.ldo_en, enable ? 0u : 1u);
    }
    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_read_thermal(max14808_dev_t *dev, bool *overtemp)
{
    if (!dev || !overtemp) return MAX14808_ERR_NULL;
    if (dev->pins.thp == MAX14808_PIN_NC) return MAX14808_ERR_PARAM;

    /* THP is open-drain, asserts (low) when Tj > +150 °C */
    *overtemp = (dev->gpio_read(dev->pins.thp) == 0u);
    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_enter_receive_mode(max14808_dev_t *dev, bool enable_tr)
{
    if (!dev) return MAX14808_ERR_NULL;

    /* Switch to TX-disable mode */
    max14808_err_t err = max14808_set_mode(dev, MAX14808_MODE_TX_DISABLE);
    if (err != MAX14808_OK) return err;

    if (enable_tr) {
        if (dev->variant == MAX14808_VARIANT_14809) {
            return MAX14808_ERR_VARIANT;
        }
        /* DINN=1, DINP=1 on all channels → damp on + T/R switch on */
        for (uint8_t i = 0u; i < 8u; i++) {
            _write_channel(dev, i, 1u, 1u);
        }
        /* tONTRSW max = 1.2 µs → wait 2 µs for all switches to close */
        dev->delay_us(2u);
    } else {
        /* DINN=0, DINP=0 → T/R switches off */
        for (uint8_t i = 0u; i < 8u; i++) {
            _write_channel(dev, i, 0u, 0u);
        }
    }

    return MAX14808_OK;
}

/* --------------------------------------------------------------------------*/

max14808_err_t max14808_pulse_burst(max14808_dev_t *dev,
                                    uint8_t         ch_mask,
                                    uint16_t        half_periods,
                                    uint32_t        half_period_us,
                                    bool            start_high)
{
    if (!dev)           return MAX14808_ERR_NULL;
    if (!half_periods)  return MAX14808_ERR_PARAM;
    if (dev->current_mode != MAX14808_MODE_OCTAL_3LEVEL) return MAX14808_ERR_MODE;

    /* Pre-condition: T/R switches should have been turned off ≥ 3 µs before
     * calling this function if MAX14808 T/R switches were in receive mode.
     * The caller is responsible for that timing. */

    bool high_phase = start_high;

    for (uint16_t hp = 0u; hp < half_periods; hp++) {
        max14808_3level_out_t state = high_phase ? MAX14808_OUT_VPP : MAX14808_OUT_VNN;

        for (uint8_t ch = 0u; ch < 8u; ch++) {
            if (ch_mask & (1u << ch)) {
                uint8_t dinp, dinn;
                _decode_3level(state, &dinp, &dinn);
                _write_channel(dev, ch, dinp, dinn);
            }
        }

        dev->delay_us(half_period_us);
        high_phase = !high_phase;
    }

    /* Apply clamp (DINN=0, DINP=0) to end the burst */
    for (uint8_t ch = 0u; ch < 8u; ch++) {
        if (ch_mask & (1u << ch)) {
            _write_channel(dev, ch, 0u, 0u);
        }
    }

    return MAX14808_OK;
}
