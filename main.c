/**
 * @file main.c
 * @brief F280025 NDT Auxiliary Board — hardware layer and composition root
 *        (150 kHz guided-wave collar: tone-burst A-scan + EEPROM storage)
 *
 * ── What lives where after the refactor ───────────────────────────────────
 *   main.c        pin map, peripheral init, acquisition, buffers, and the
 *                 hook table that hands those capabilities to the state
 *                 machine. Everything here touches registers.
 *   ndt_sm.c      when things happen (states, events, trigger policy).
 *                 No driverlib.
 *   ndt_i2cb.c    the I2CB slave protocol and its ISR.
 *   ndt_store.c   A-scan record format on top of the EEPROM.
 *   ndt_config.h  every policy knob in one place.
 *
 * Layering below main.c:
 *   ndt_store.c -> m24m01e.c (chip) -> i2ca_eeprom.c (F280025 I2CA)
 *
 * ── System flow ───────────────────────────────────────────────────────────
 *  1. Board peripherals initialised  (SysConfig -> Board_init)
 *  2. ePWM1 configured (in C, no pins) as the ADC sample-rate pacer
 *  3. CPU Timer 0 free-running (10 ns/tick) for burst timing + timestamps
 *  4. CPU Timer 1 at 1 kHz provides the millisecond tick the state machine
 *     uses for periodic triggering and non-blocking fault blinking
 *  5. Analogue front-end powered via Analog_EN (GPIO17), AFE gain set HIGH
 *  6. MAX14808 initialised in octal three-level mode (MODE0=1, MODE1=0)
 *  7. M24M01E EEPROM on I2CA probed (WC = GPIO33 driven LOW), record store
 *     bound on top of it
 *  8. I2CB slave serves the master MCU; commands become state-machine events
 *  9. Supply rails checked (ADCA SOC4-8) BEFORE firing; scan blocked if bad
 * 10. 150 kHz bipolar tone-burst (5..10 cycles) fired on all 8 channels
 *     simultaneously; T/R switches close (~12 us dead time)
 * 11. ePWM1 paces ADCA+ADCC SOC0-3 every SAMPLE_PERIOD_EPWM_TICKS; a tight
 *     polling loop stores NDT_ASCAN_SAMPLES continuous samples per channel
 * 12. Time axis is implicit: t(i) = start_delay_ns + i * sample_period_ns
 *
 * ── I2C protocol ──────────────────────────────────────────────────────────
 *   See ndt_i2cb.h — the protocol table now lives with its implementation.
 *
 * ── Timing constants (SYSCLK = 100 MHz; EPWMCLK = SYSCLK/2 fixed) ────────
 *   Burst      : 150 kHz -> half-periods 333/334 SYSCLK (3.33/3.34 us)
 *   Sampling   : ePWM1 SOCA every 80 EPWMCLK ticks (20 ns each) = 1.6 us
 *                -> 625 kSps/channel
 *   Record     : 512 samples x 1.6 us = 819.2 us listen window
 *
 * ── RAM budget (F28002x has 24 KB SRAM total) ─────────────────────────────
 *   g_waveBuf  : 8 x 512 uint16 = 4096 words (8 KB) — dominant consumer.
 *   EEPROM save/load peaks ~0x250 words of stack (two 256-entry staging
 *   buffers nested) — keep the linker stack >= 0x400.
 *   NDT_captureRecord() is placed in .TI.ramfunc: the linker .cmd must map
 *   that section to RAM.
 *
 * ── ADC channel map (g_waveBuf row index) ────────────────────────────────
 *   [0] ADCA CH6   tlv1_out1      [4] ADCC CH6   tlv1_out2
 *   [1] ADCA CH3   tlv1_out3      [5] ADCC CH14  tlv2_out2
 *   [2] ADCA CH2   tlv1_out4      [6] ADCC CH11  tlv2_out3
 *   [3] ADCA CH9   tlv2_out1      [7] ADCC CH10  tlv2_out4
 *
 * ── Voltage-tap map (g_voltBuf index, ADCA SOC4-8) ────────────────────────
 *   [0] vpp_tap      ADCIN12  target 2.912 V   [3] 5v_pulse_tap ADCIN11 1.250 V
 *   [1] 12V_tap      ADCIN5   target 1.714 V   [4] 5v_vfd_tap   ADCIN0  1.250 V
 *   [2] 7v_vfd_tap   ADCIN1   target 1.750 V
 *
 * ── Pin assignments (schematic rev-A + F280025 80QFP pinout) ─────────────
 *   Analog_EN  = GPIO17 (pin 40)   MODE0 = GPIO39 (pin 56)
 *   Pulser_EN  = GPIO25 (pin 42)   MODE1 = GPIO42 (pin 57)
 *   THP        = GPIO13 (pin 35)   EEPROM_WC = GPIO33 (LOW = writes enabled)
 *   EEPROM I2C : I2CA — SDA GPIO26 (pin 43), SCL GPIO27 (pin 44)
 *   Master I2C : I2CB — SDA GPIO2  (pin 61), SCL GPIO3  (pin 60)
 */

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "max14808.h"
#include "m24m01e.h"
#include "i2ca_eeprom.h"
#include "ndt_config.h"
#include "ndt_store.h"
#include "ndt_sm.h"
#include "ndt_i2cb.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * GPIO pin IDs (F280025 GPIO indices; chip package pins in comments)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── MAX14808 mode / current / fault pins ───────────────────────────────── */
#define PIN_MODE0       39U   /* chip pin 56 — HIGH = octal three-level      */
#define PIN_MODE1       42U   /* chip pin 57 — LOW  = octal three-level      */
#define PIN_CC0         MAX14808_PIN_NC   /* hard-wired 2 A on PCB            */
#define PIN_CC1         MAX14808_PIN_NC
#define PIN_SYNC        MAX14808_PIN_NC
#define PIN_LDO_EN      MAX14808_PIN_NC
#define PIN_THP         13U   /* chip pin 35 — thermal fault, active-low     */

/* ── Board-level enables / status ───────────────────────────────────────── */
#define PIN_PULSER_EN   25U   /* chip pin 42 — HV supply enable              */
#define PIN_ANALOG_EN   17U   /* chip pin 40 — AD8334 + TLV9354 enable       */
#define PIN_MCU_LED     16U   /* chip pin 39                                 */
#define PIN_EEPROM_WC   33U   /* M24M01E write-control: LOW = writes enabled */

/* ── AFE gain-select pins (AD8334) ──────────────────────────────────────── */
#define PIN_AFE_GAIN_A1  31U
#define PIN_AFE_GAIN_A2  30U
#define PIN_AFE_GAIN_B1   4U
#define PIN_AFE_GAIN_B2   8U

/* ── Pulser DINP (A-side) and DINN (B-side) ─────────────────────────────── */
#define PIN_DINP_CH1     6U
#define PIN_DINP_CH2    15U
#define PIN_DINP_CH3    10U
#define PIN_DINP_CH4     5U
#define PIN_DINP_CH5     1U
#define PIN_DINP_CH6    40U
#define PIN_DINP_CH7    41U
#define PIN_DINP_CH8     7U

#define PIN_DINN_CH1    14U
#define PIN_DINN_CH2    34U
#define PIN_DINN_CH3     9U
#define PIN_DINN_CH4    45U
#define PIN_DINN_CH5     0U
#define PIN_DINN_CH6    23U
#define PIN_DINN_CH7    22U
#define PIN_DINN_CH8    44U

/* ═══════════════════════════════════════════════════════════════════════════
 * GPIO bitmasks for timing-critical simultaneous register writes
 * ═══════════════════════════════════════════════════════════════════════════ */
#define GPIOA_A_MASK    0x000084E2UL
#define GPIOA_B_MASK    0x00C04201UL
#define GPIOB_A_MASK    0x00000300UL
#define GPIOB_B_MASK    0x00003004UL
#define GPIOA_ALL_MASK  (GPIOA_A_MASK | GPIOA_B_MASK)
#define GPIOB_ALL_MASK  (GPIOB_A_MASK | GPIOB_B_MASK)

/* ═══════════════════════════════════════════════════════════════════════════
 * Excitation / acquisition timing (hardware facts; policy is in ndt_config.h)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define PZT_FREQ_HZ           150000UL

/* 150 kHz period = 666.67 SYSCLK. Alternating 333/334-tick half-periods give
 * a 667-tick cycle → 149.925 kHz (-0.05 % — negligible against PZT Q).      */
#define BURST_HALF_TICKS_A    333U
#define BURST_HALF_TICKS_B    334U

/* ePWM1 SOCA period. EPWMCLK is FIXED at SYSCLK/2 = 50 MHz → 20 ns/tick.
 * 80 ticks → 1.6 µs → 625 kSps per channel. Lower bound: the 4-SOC round
 * robin per ADC needs ≈1.45 µs.                                            */
#define EPWMCLK_TICK_NS            20U
#define SAMPLE_PERIOD_EPWM_TICKS   80U
#define SAMPLE_PERIOD_NS           (SAMPLE_PERIOD_EPWM_TICKS * EPWMCLK_TICK_NS)

#define WAVE_SAMPLES          NDT_ASCAN_SAMPLES
#define WAVE_CHANNELS         NDT_ASCAN_CHANNELS

#define HV_SETTLE_US          500U
#define CPUTIMER_NS_PER_TICK  10U      /* CPU Timer 0, prescaler 0, 100 MHz  */
#define MS_TICK_PERIOD        (DEVICE_SYSCLK_FREQ / 1000UL)  /* Timer 1      */

/* ═══════════════════════════════════════════════════════════════════════════
 * Voltage-tap monitoring
 * ═══════════════════════════════════════════════════════════════════════════ */
#define NUM_TAPS         5U

#define ADC_VREF_MV      3300U
#define ADC_FULL_COUNTS  4095U
#define MV_TO_COUNTS(mv) ((uint16_t)(((uint32_t)(mv) * ADC_FULL_COUNTS) / ADC_VREF_MV))
#define TAP_TOL_PCT      5U

/* ═══════════════════════════════════════════════════════════════════════════
 * Status-flag word (I2CB cmd 0x03)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define STATUS_VPP_OK        0x0001U
#define STATUS_12V_OK        0x0002U
#define STATUS_7V_VFD_OK     0x0004U
#define STATUS_5V_PULSE_OK   0x0008U
#define STATUS_5V_VFD_OK     0x0010U
#define STATUS_ALL_RAILS_OK  0x001FU
#define STATUS_DATA_VALID    0x0020U   /* g_waveBuf holds a valid A-scan      */
#define STATUS_THERMAL_FAULT 0x0040U
#define STATUS_VOLTAGE_FAULT 0x0080U
#define STATUS_EEPROM_BUSY   0x0100U   /* save/load in progress               */
#define STATUS_EEPROM_FAIL   0x0200U   /* last EEPROM operation failed        */

/* ═══════════════════════════════════════════════════════════════════════════
 * Shared state
 *
 * Concurrency note: every write below happens in main-loop context. The I2CB
 * ISR only reads these buffers (through the descriptor it was given), so the
 * read-modify-write on g_statusFlags has a single writer and needs no guard.
 * Keep it that way when adding features.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* A-scan waveform buffer, channel-major: g_waveBuf[ch][sample]. 8 KB. */
volatile uint16_t g_waveBuf[WAVE_CHANNELS][WAVE_SAMPLES];

volatile uint16_t g_voltBuf[NUM_TAPS] = {0};
volatile uint16_t g_statusFlags       = 0U;

/* Pre-serialised 16-byte header stream for I2CB cmd 0x08 (byte values). */
volatile uint16_t g_hdrStream[NDT_STORE_HDR_STREAM_LEN] = {0};

/* Header of the record currently in g_waveBuf (captured or loaded). */
static ndt_ascan_hdr_t g_lastHdr;

/* Configurable excitation burst length (I2CB cmd 0x02). */
static uint16_t g_burstCycles = NDT_BURST_CYCLES_DEFAULT;

/* Fire moment → first-sample delay of the current record, in ns. */
static uint32_t g_startDelayNs = 0U;

/* Millisecond tick, incremented by the CPU Timer 1 ISR. */
static volatile uint32_t g_msTicks = 0U;

static const uint16_t g_tapExpected[NUM_TAPS] = {
    MV_TO_COUNTS(2912U), MV_TO_COUNTS(1714U), MV_TO_COUNTS(1750U),
    MV_TO_COUNTS(1250U), MV_TO_COUNTS(1250U)
};
static const uint16_t g_tapOkBit[NUM_TAPS] = {
    STATUS_VPP_OK, STATUS_12V_OK, STATUS_7V_VFD_OK,
    STATUS_5V_PULSE_OK, STATUS_5V_VFD_OK
};
static const uint16_t g_tapSoc[NUM_TAPS] = {
    ADC_SOC_NUMBER4, ADC_SOC_NUMBER5, ADC_SOC_NUMBER6,
    ADC_SOC_NUMBER7, ADC_SOC_NUMBER8
};

/* ═══════════════════════════════════════════════════════════════════════════
 * Device handles
 * ═══════════════════════════════════════════════════════════════════════════ */
static max14808_dev_t g_pulser;
static m24m01e_t      g_eeprom;
static ndt_store_t    g_store;

/* ═══════════════════════════════════════════════════════════════════════════
 * MAX14808 platform callbacks
 * ═══════════════════════════════════════════════════════════════════════════ */

static void ndt_gpio_write(uint32_t pin, uint32_t val)
{
    GPIO_writePin(pin, (uint32_t)val);
}

static uint32_t ndt_gpio_read(uint32_t pin)
{
    return GPIO_readPin(pin);
}

static void ndt_delay_us(uint32_t us)
{
    DEVICE_DELAY_US(us);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Millisecond tick — CPU Timer 1 (INT13, not PIE-routed, so no ACK group)
 * ═══════════════════════════════════════════════════════════════════════════ */
__interrupt void ndt_timer1ISR(void)
{
    g_msTicks++;
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
}

/* 32-bit counter read on a 16-bit machine: retry until two reads agree, so a
 * tick landing between the halves cannot produce a torn value. */
static uint32_t ndt_millis(void)
{
    uint32_t a, b;
    do { a = g_msTicks; b = g_msTicks; } while (a != b);
    return a;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Initialisation
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ─────────────────────────────────────────────────────────────────────────
 * NDT_initPulser — octal three-level mode (MODE0=1, MODE1=0)
 * ───────────────────────────────────────────────────────────────────────── */
static void NDT_initPulser(void)
{
    const max14808_pins_t pins = {
        .mode0  = PIN_MODE0,
        .mode1  = PIN_MODE1,
        .cc0    = PIN_CC0,
        .cc1    = PIN_CC1,
        .sync   = PIN_SYNC,
        .ldo_en = PIN_LDO_EN,
        .thp    = PIN_THP,
        .dinp   = { PIN_DINP_CH1, PIN_DINP_CH2, PIN_DINP_CH3, PIN_DINP_CH4,
                    PIN_DINP_CH5, PIN_DINP_CH6, PIN_DINP_CH7, PIN_DINP_CH8 },
        .dinn   = { PIN_DINN_CH1, PIN_DINN_CH2, PIN_DINN_CH3, PIN_DINN_CH4,
                    PIN_DINN_CH5, PIN_DINN_CH6, PIN_DINN_CH7, PIN_DINN_CH8 }
    };

    max14808_err_t err = max14808_init(&g_pulser,
                                       MAX14808_VARIANT_14808,
                                       &pins,
                                       ndt_gpio_write,
                                       ndt_gpio_read,
                                       ndt_delay_us);
    if (err != MAX14808_OK) { ESTOP0; }

    max14808_set_current(&g_pulser, MAX14808_CURRENT_2A);

    err = max14808_set_mode(&g_pulser, MAX14808_MODE_OCTAL_3LEVEL);
    if (err != MAX14808_OK) { ESTOP0; }
}

/* ─────────────────────────────────────────────────────────────────────────
 * NDT_initTimers — CPU Timer 0 free-running (burst pacing, timestamps),
 *                  CPU Timer 1 at 1 kHz (millisecond tick),
 *                  ePWM1 as the ADC sample-rate pacer (frozen at boot).
 * ───────────────────────────────────────────────────────────────────────── */
static void NDT_initTimers(void)
{
    /* CPU Timer 0: 32-bit down-counter, SYSCLK rate → 10 ns/tick. */
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFFUL);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);

    /* CPU Timer 1: 1 ms periodic interrupt. The only work in its ISR is a
     * 32-bit increment — the state machine needs a time base for periodic
     * triggering and for blinking without blocking the loop. */
    CPUTimer_setPeriod(CPUTIMER1_BASE, MS_TICK_PERIOD - 1UL);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0U);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE,
                              CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    Interrupt_register(INT_TIMER1, &ndt_timer1ISR);
    Interrupt_enable(INT_TIMER1);
    CPUTimer_startTimer(CPUTIMER1_BASE);

    /* EPWMCLK is hard-fixed at SYSCLK/2 = 50 MHz. */
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);   /* TBCLK = EPWMCLK    */
    EPWM_setTimeBasePeriod(EPWM1_BASE, SAMPLE_PERIOD_EPWM_TICKS - 1U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);

    /* SOCA pulse on every counter-zero event → one ADC sweep per period.   */
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1U);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    /* Release the global time-base clock so the counter can run when we
     * switch it to COUNTER_MODE_UP during capture.                          */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

/* ─────────────────────────────────────────────────────────────────────────
 * NDT_initEeprom — WC pin low, transport bind, probe, record store bind.
 * A missing / failing EEPROM must not brick the scanner: on failure we set
 * STATUS_EEPROM_FAIL and carry on (scans still work, storage doesn't).
 * ───────────────────────────────────────────────────────────────────────── */
static void NDT_initEeprom(void)
{
    const m24m01e_io_t io = {
        .write         = i2ca_ep_write,
        .write_read    = i2ca_ep_write_read,
        .delay_ms      = i2ca_ep_delay_ms,
        .probe_no_stop = NULL,
        .ctx           = NULL
    };

    GPIO_writePin(PIN_EEPROM_WC, 0U);   /* writes enabled */

    if (m24m01e_init(&g_eeprom, &io, 0U) != M24M01E_OK ||
        m24m01e_probe(&g_eeprom)         != M24M01E_OK ||
        ndt_store_init(&g_store, &g_eeprom) != NDT_STORE_OK)
    {
        g_statusFlags |= STATUS_EEPROM_FAIL;
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 * NDT_initSlave — bind the buffers the I2CB slave is allowed to serve.
 * ───────────────────────────────────────────────────────────────────────── */
static void NDT_initSlave(void)
{
    const ndt_i2cb_streams_t streams = {
        .status        = &g_statusFlags,
        .taps          = g_voltBuf,
        .tap_count     = NUM_TAPS,
        .wave          = &g_waveBuf[0][0],
        .wave_samples  = WAVE_SAMPLES,
        .wave_channels = WAVE_CHANNELS,
        .hdr_stream    = g_hdrStream,
        .hdr_len       = NDT_STORE_HDR_STREAM_LEN
    };
    ndt_i2cb_init(&streams);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Acquisition
 * ═══════════════════════════════════════════════════════════════════════════ */

static bool tap_in_range(uint16_t meas, uint16_t expected)
{
    uint16_t margin = (uint16_t)(((uint32_t)expected * TAP_TOL_PCT) / 100U);
    return (meas >= (uint16_t)(expected - margin)) &&
           (meas <= (uint16_t)(expected + margin));
}

static uint16_t NDT_checkSupplies(void)
{
    uint16_t flags = 0U;
    uint16_t i;

    ADC_forceMultipleSOC(ADCA_BASE,
        ADC_FORCE_SOC4 | ADC_FORCE_SOC5 | ADC_FORCE_SOC6 |
        ADC_FORCE_SOC7 | ADC_FORCE_SOC8);

    while (ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2) == false) { }
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);

    for (i = 0U; i < NUM_TAPS; i++)
    {
        uint16_t meas = ADC_readResult(ADCARESULT_BASE, g_tapSoc[i]);
        g_voltBuf[i]  = meas;
        if (tap_in_range(meas, g_tapExpected[i]))
        {
            flags |= g_tapOkBit[i];
        }
    }
    return flags;
}

/* ─────────────────────────────────────────────────────────────────────────
 * burst_wait — drift-free busy wait on the free-running CPU Timer 0
 * (down-counter). *t_ref is advanced by exactly `ticks` each call, so
 * loop-overhead jitter never accumulates across the burst.
 * ───────────────────────────────────────────────────────────────────────── */
static inline void burst_wait(uint32_t *t_ref, uint32_t ticks)
{
    while ((uint32_t)(*t_ref - CPUTimer_getTimerCount(CPUTIMER0_BASE))
           < ticks) { }
    *t_ref -= ticks;
}

/* ─────────────────────────────────────────────────────────────────────────
 * NDT_burst — N-cycle 150 kHz bipolar tone-burst, all 8 channels
 * simultaneously via direct GPIO register writes. Each half-cycle swap is
 * clear-then-set (brief both-low gap → no P/N shoot-through).
 * ───────────────────────────────────────────────────────────────────────── */
static void NDT_burst(uint16_t cycles)
{
    uint32_t t_ref = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    uint16_t i;

    for (i = 0U; i < cycles; i++)
    {
        /* Positive half-cycle: DINP = 1, DINN = 0 */
        HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR) = GPIOA_B_MASK;
        HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIOB_B_MASK;
        HWREG(GPIODATA_BASE + GPIO_O_GPASET)   = GPIOA_A_MASK;
        HWREG(GPIODATA_BASE + GPIO_O_GPBSET)   = GPIOB_A_MASK;
        burst_wait(&t_ref, BURST_HALF_TICKS_A);

        /* Negative half-cycle: DINP = 0, DINN = 1 */
        HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR) = GPIOA_A_MASK;
        HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIOB_A_MASK;
        HWREG(GPIODATA_BASE + GPIO_O_GPASET)   = GPIOA_B_MASK;
        HWREG(GPIODATA_BASE + GPIO_O_GPBSET)   = GPIOB_B_MASK;
        burst_wait(&t_ref, BURST_HALF_TICKS_B);
    }

    /* All drive pins LOW — clamped / zero-volt state */
    HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR) = GPIOA_ALL_MASK;
    HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIOB_ALL_MASK;
}

/* ─────────────────────────────────────────────────────────────────────────
 * NDT_captureRecord — continuous fixed-rate capture of all 8 channels.
 *
 * ePWM1 SOCA triggers ADCA SOC0–3 and ADCC SOC0–3 every 1.6 µs. Each sweep
 * completes in ≈1.45 µs; this loop polls both INT1 flags (SOC3 EOC), stores
 * the 8 results, and repeats for WAVE_SAMPLES sweeps. Per-sweep CPU budget
 * is 160 cycles — the store body fits with margin at -O2, and the function
 * runs from RAM (.TI.ramfunc) so flash wait-states can't push it over.
 * ───────────────────────────────────────────────────────────────────────── */
#pragma CODE_SECTION(NDT_captureRecord, ".TI.ramfunc")
static void NDT_captureRecord(void)
{
    uint16_t n;

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

    /* Arm the pacer: counter to zero, then run. */
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);

    for (n = 0U; n < WAVE_SAMPLES; n++)
    {
        while (ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false) {}
        while (ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1) == false) {}
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

        g_waveBuf[0][n] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
        g_waveBuf[1][n] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
        g_waveBuf[2][n] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2);
        g_waveBuf[3][n] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3);
        g_waveBuf[4][n] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
        g_waveBuf[5][n] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
        g_waveBuf[6][n] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2);
        g_waveBuf[7][n] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3);
    }

    /* Freeze the pacer — no further conversions until the next A-scan.      */
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * State-machine hooks — the only surface ndt_sm.c sees of this board
 * ═══════════════════════════════════════════════════════════════════════════ */

static bool hook_thermal_fault(void)
{
    /* MAX14808 THP: open-drain, active-low over-temp flag. */
    return (GPIO_readPin(PIN_THP) == 0U);
}

static bool hook_supplies_ok(void)
{
    uint16_t rails;

    g_statusFlags &= ~(STATUS_DATA_VALID | STATUS_VOLTAGE_FAULT |
                       STATUS_ALL_RAILS_OK);

    rails = NDT_checkSupplies();
    g_statusFlags |= rails;

    return (rails & STATUS_ALL_RAILS_OK) == STATUS_ALL_RAILS_OK;
}

/* One complete A-scan:
 *   1. HV on, settle.
 *   2. t0 := now; N-cycle 150 kHz tone-burst.
 *   3. T/R switches to receive (~12 µs dead time in the driver).
 *   4. Measure actual t0 → capture-start delay (stored in the header).
 *   5. Continuous ePWM-paced capture of WAVE_SAMPLES per channel.
 * The read side is frozen throughout, so a master read cannot observe a
 * half-overwritten record. */
static void hook_fire_and_capture(void)
{
    uint32_t t_fire, t_cap;

    ndt_i2cb_lock_tx(true);

    GPIO_writePin(PIN_PULSER_EN, 1U);
    DEVICE_DELAY_US(HV_SETTLE_US);

    t_fire = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    NDT_burst(g_burstCycles);

    max14808_enter_receive_mode(&g_pulser, true);

    /* Down-counter: elapsed = old - new. */
    t_cap = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    g_startDelayNs = (uint32_t)(t_fire - t_cap) * CPUTIMER_NS_PER_TICK;

    NDT_captureRecord();

    g_lastHdr.id               = 0U;          /* assigned on save (0x06) */
    g_lastHdr.seq              = 0U;
    g_lastHdr.burst_cycles     = g_burstCycles;
    g_lastHdr.freq_hz          = PZT_FREQ_HZ;
    g_lastHdr.sample_period_ns = SAMPLE_PERIOD_NS;
    g_lastHdr.start_delay_ns   = g_startDelayNs;
    g_lastHdr.samples_per_ch   = WAVE_SAMPLES;
    g_lastHdr.channels         = WAVE_CHANNELS;
    g_lastHdr.status_flags     = g_statusFlags;
    ndt_store_hdr_to_stream(&g_lastHdr, g_hdrStream);

    ndt_i2cb_lock_tx(false);
}

static void hook_after_capture(void)
{
    /* Restore MAX14808 octal-3L ready state (resets DINP/DINN). */
    max14808_set_mode(&g_pulser, MAX14808_MODE_OCTAL_3LEVEL);

    /* HV off until the next scan command. */
    GPIO_writePin(PIN_PULSER_EN, 0U);

    g_statusFlags |= STATUS_DATA_VALID;
}

static void hook_on_voltage_fault(void)
{
    g_statusFlags |= STATUS_VOLTAGE_FAULT;
    GPIO_writePin(PIN_PULSER_EN, 0U);
}

static void hook_on_thermal_enter(void)
{
    g_statusFlags |= STATUS_THERMAL_FAULT;
    GPIO_writePin(PIN_PULSER_EN, 0U);
    max14808_set_mode(&g_pulser, MAX14808_MODE_TX_DISABLE);
}

static void hook_thermal_blink(void)
{
    GPIO_togglePin(PIN_MCU_LED);
}

static void hook_set_burst_cycles(uint16_t cycles)
{
    if (cycles < NDT_BURST_CYCLES_MIN) { cycles = NDT_BURST_CYCLES_MIN; }
    if (cycles > NDT_BURST_CYCLES_MAX) { cycles = NDT_BURST_CYCLES_MAX; }
    g_burstCycles = cycles;
}

static void hook_store_save(uint16_t id)
{
    ndt_store_status_t rc;

    g_statusFlags |=  STATUS_EEPROM_BUSY;
    g_statusFlags &= ~STATUS_EEPROM_FAIL;

    if ((g_statusFlags & STATUS_DATA_VALID) != 0U)
    {
        g_lastHdr.id = id;
        rc = ndt_store_save(&g_store, &g_lastHdr,
                            (const uint16_t *)&g_waveBuf[0][0]);
    }
    else
    {
        rc = NDT_STORE_ERR_PARAM;         /* nothing valid to save */
    }

    if (rc != NDT_STORE_OK) { g_statusFlags |= STATUS_EEPROM_FAIL; }

    ndt_store_hdr_to_stream(&g_lastHdr, g_hdrStream);   /* seq updated */
    g_statusFlags &= ~STATUS_EEPROM_BUSY;
}

static void hook_store_load(uint16_t id)
{
    ndt_store_status_t rc;

    g_statusFlags |=  STATUS_EEPROM_BUSY;
    g_statusFlags &= ~(STATUS_EEPROM_FAIL | STATUS_DATA_VALID);

    /* The load rewrites g_waveBuf page by page — freeze the read side. */
    ndt_i2cb_lock_tx(true);
    rc = ndt_store_load(&g_store, id, &g_lastHdr, (uint16_t *)&g_waveBuf[0][0]);
    ndt_i2cb_lock_tx(false);

    if (rc == NDT_STORE_OK)
    {
        g_statusFlags |= STATUS_DATA_VALID;
    }
    else
    {
        g_statusFlags |= STATUS_EEPROM_FAIL;
    }

    ndt_store_hdr_to_stream(&g_lastHdr, g_hdrStream);
    g_statusFlags &= ~STATUS_EEPROM_BUSY;
}

static const ndt_sm_hooks_t g_hooks = {
    .millis           = ndt_millis,
    .thermal_fault    = hook_thermal_fault,
    .supplies_ok      = hook_supplies_ok,
    .fire_and_capture = hook_fire_and_capture,
    .after_capture    = hook_after_capture,
    .on_voltage_fault = hook_on_voltage_fault,
    .on_thermal_enter = hook_on_thermal_enter,
    .thermal_blink    = hook_thermal_blink,
    .store_save       = hook_store_save,
    .store_load       = hook_store_load,
    .set_burst_cycles = hook_set_burst_cycles
};

/* ═══════════════════════════════════════════════════════════════════════════
 * main — composition root: bring the hardware up, wire the modules, run.
 * ═══════════════════════════════════════════════════════════════════════════ */
void main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();

    /* 1. Timers: CPU Timer 0 free-running, Timer 1 = 1 kHz, ePWM1 pacer. */
    NDT_initTimers();

    /* 2. Analogue front end + gain. */
    GPIO_writePin(PIN_ANALOG_EN, 1U);
    DEVICE_DELAY_US(500U);
    GPIO_writePin(PIN_AFE_GAIN_A1, 1U);
    GPIO_writePin(PIN_AFE_GAIN_A2, 1U);
    GPIO_writePin(PIN_AFE_GAIN_B1, 1U);
    GPIO_writePin(PIN_AFE_GAIN_B2, 1U);

    /* 3. MAX14808 pulser. */
    NDT_initPulser();

    /* 4. EEPROM + record store (non-fatal on failure). */
    NDT_initEeprom();

    /* 5. Wire the modules. Both must be ready before the first interrupt:
     *    the I2CB ISR posts into the state machine's queue. */
    ndt_sm_init(&g_hooks);
    NDT_initSlave();

    /* 6. Enable CPU interrupts. */
    Interrupt_enableGlobal();

    /* 7. Status LED on — initialisation complete. */
    GPIO_writePin(PIN_MCU_LED, 1U);

    for (;;)
    {
        ndt_sm_step();
    }
}
