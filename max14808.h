/**
 * @file max14808.h
 * @brief Driver for MAX14808/MAX14809 Octal Three-Level / Quad Five-Level
 *        High-Voltage 2A Digital Pulsers with T/R Switch
 *
 * TI C28x changes vs original (marked *** C28x FIX ***):
 *  1. uint8_t defined via UINT8_MAX guard — TI C28x stdint.h omits uint8_t
 *     (no native 8-bit type on this arch); the guard works regardless of
 *     whether __TMS320C28XX__ is predefined.
 *  2. bool defined inline — <stdbool.h>/_Bool unreliable on C28x toolchain.
 *  3. max14808_err_t uses typedef int + #define — C28x enums are unsigned,
 *     so negative enum members trigger error #63.
 */

#ifndef MAX14808_H
#define MAX14808_H

#include <stdint.h>
#include "inc/hw_types.h"

// #ifndef UINT8_MAX
//   typedef unsigned char   uint8_t;   /* unsigned char = 16-bit on C28x      */
//   typedef signed   char   int8_t;
//   #define UINT8_MAX  0xFFU
//   #define INT8_MAX   0x7F
//   #define INT8_MIN   (-0x7F - 1)
// #endif

/* *** C28x FIX 2 *** --------------------------------------------------------
 * <stdbool.h> maps bool → _Bool.  _Bool is not reliably available on all
 * C28x CGT versions; typedef unsigned int is safe and binary-identical.     */
#ifndef __cplusplus
  #ifndef bool
    typedef unsigned int bool;
    #define true  (1U)
    #define false (0U)
  #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Device variant
 * -------------------------------------------------------------------------*/
typedef enum {
    MAX14808_VARIANT_14808 = 0,
    MAX14808_VARIANT_14809 = 1,
} max14808_variant_t;

/* ---------------------------------------------------------------------------
 * Operation modes  (MODE0 / MODE1 pins)
 * -------------------------------------------------------------------------*/
typedef enum {
    MAX14808_MODE_SHUTDOWN        = 0x00,
    MAX14808_MODE_OCTAL_3LEVEL    = 0x01,
    MAX14808_MODE_QUAD_5LEVEL     = 0x02,
    MAX14808_MODE_TX_DISABLE      = 0x03,
} max14808_mode_t;

/* ---------------------------------------------------------------------------
 * Current drive capability  (CC0 / CC1 pins)
 * -------------------------------------------------------------------------*/
typedef enum {
    MAX14808_CURRENT_2A   = 0x00,
    MAX14808_CURRENT_1P5A = 0x01,
    MAX14808_CURRENT_1A   = 0x02,
    MAX14808_CURRENT_0P5A = 0x03,
} max14808_current_t;

/* ---------------------------------------------------------------------------
 * Clock / sync mode
 * -------------------------------------------------------------------------*/
typedef enum {
    MAX14808_SYNC_TRANSPARENT    = 0,
    MAX14808_SYNC_CLOCKED_SE     = 1,
    MAX14808_SYNC_CLOCKED_DIFF   = 2,
} max14808_sync_mode_t;

/* ---------------------------------------------------------------------------
 * Three-level output state (octal mode, per channel)
 * -------------------------------------------------------------------------*/
typedef enum {
    MAX14808_OUT_CLAMP_OFF    = 0x00,
    MAX14808_OUT_VNN          = 0x01,
    MAX14808_OUT_VPP          = 0x02,
    MAX14808_OUT_CLAMP_DAMP   = 0x03,
} max14808_3level_out_t;

/* ---------------------------------------------------------------------------
 * Five-level output state (quad mode, per channel pair x/y)
 * -------------------------------------------------------------------------*/
typedef enum {
    MAX14808_5L_HI_Z          = 0,
    MAX14808_5L_CLAMP         = 1,
    MAX14808_5L_VPPB          = 2,
    MAX14808_5L_VNNB          = 3,
    MAX14808_5L_VPPA          = 4,
    MAX14808_5L_VNNA          = 5,
    MAX14808_5L_CLAMP_DAMP_TR = 6,
} max14808_5level_out_t;

/* ---------------------------------------------------------------------------
 * Channel index (1–8)
 * -------------------------------------------------------------------------*/
#define MAX14808_CH_MIN   1u
#define MAX14808_CH_MAX   8u

/* ---------------------------------------------------------------------------
 * GPIO HAL callbacks — signature matches TI DriverLib directly:
 *   gpio_write ← GPIO_writePin  (uint32_t pin, uint32_t val)
 *   gpio_read  ← GPIO_readPin   (uint32_t pin) → uint32_t
 * No wrapper functions needed; assign driverlib functions directly.
 * -------------------------------------------------------------------------*/
typedef void     (*max14808_gpio_write_fn)(uint32_t pin, uint32_t val);
typedef uint32_t (*max14808_gpio_read_fn )(uint32_t pin);
typedef void     (*max14808_delay_us_fn  )(uint32_t us);

/* ---------------------------------------------------------------------------
 * Pin map — set unused pins to MAX14808_PIN_NC
 * -------------------------------------------------------------------------*/
#define MAX14808_PIN_NC  UINT32_MAX

typedef struct {
    uint32_t mode0;
    uint32_t mode1;
    uint32_t cc0;
    uint32_t cc1;
    uint32_t sync;
    uint32_t clk;
    uint32_t ldo_en;
    uint32_t thp;
    uint32_t dinp[8];
    uint32_t dinn[8];
} max14808_pins_t;

/* ---------------------------------------------------------------------------
 * Device handle
 * -------------------------------------------------------------------------*/
typedef struct {
    max14808_variant_t      variant;
    max14808_pins_t         pins;
    max14808_gpio_write_fn  gpio_write;
    max14808_gpio_read_fn   gpio_read;
    max14808_delay_us_fn    delay_us;
    max14808_mode_t         current_mode;
    max14808_current_t      current_drive;
    max14808_sync_mode_t    sync_mode;
} max14808_dev_t;

/* ---------------------------------------------------------------------------
 * *** C28x FIX 3 *** — error codes
 * C28x enums are unsigned; -1..-4 as enum members triggers error #63.
 * typedef int + #define is ABI-identical and portable.
 * -------------------------------------------------------------------------*/
typedef int max14808_err_t;
#define MAX14808_OK           ( 0)
#define MAX14808_ERR_NULL     (-1)
#define MAX14808_ERR_PARAM    (-2)
#define MAX14808_ERR_MODE     (-3)
#define MAX14808_ERR_VARIANT  (-4)

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/
max14808_err_t max14808_init(max14808_dev_t            *dev,
                             max14808_variant_t         variant,
                             const max14808_pins_t     *pins,
                             max14808_gpio_write_fn     gpio_write,
                             max14808_gpio_read_fn      gpio_read,
                             max14808_delay_us_fn       delay_us);

max14808_err_t max14808_set_mode           (max14808_dev_t *, max14808_mode_t);
max14808_err_t max14808_set_current        (max14808_dev_t *, max14808_current_t);
max14808_err_t max14808_set_sync_mode      (max14808_dev_t *, max14808_sync_mode_t);
max14808_err_t max14808_set_channel_3level (max14808_dev_t *, uint8_t ch,
                                             max14808_3level_out_t);
max14808_err_t max14808_set_channel_5level (max14808_dev_t *, uint8_t pair,
                                             max14808_5level_out_t);
max14808_err_t max14808_set_all_channels_3level(max14808_dev_t *,
                                                const max14808_3level_out_t states[8]);
max14808_err_t max14808_set_internal_ldo   (max14808_dev_t *, bool enable);
max14808_err_t max14808_read_thermal       (max14808_dev_t *, bool *overtemp);
max14808_err_t max14808_enter_receive_mode (max14808_dev_t *, bool enable_tr);
max14808_err_t max14808_pulse_burst        (max14808_dev_t *, uint8_t ch_mask,
                                             uint16_t half_periods,
                                             uint32_t half_period_us,
                                             bool     start_high);

#ifdef __cplusplus
}
#endif

#endif /* MAX14808_H */
