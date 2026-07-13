/**
 * @file main.c
 * @brief F280025 NDT Auxiliary Board — main application
 *        (150 kHz guided-wave collar: tone-burst A-scan + EEPROM storage)
 *
 * ── System flow ───────────────────────────────────────────────────────────
 *  1. Board peripherals initialised  (SysConfig → Board_init)
 *  2. ePWM1 configured (in C, no pins) as the ADC sample-rate pacer
 *  3. CPU Timer 0 free-running (10 ns/tick) for burst timing + timestamps
 *  4. Analogue front-end powered via Analog_EN (GPIO17), AFE gain set HIGH
 *  5. MAX14808 initialised in octal three-level mode (MODE0=1, MODE1=0)
 *  6. M24M01E EEPROM on I2CA probed (WC = GPIO33 driven LOW)
 *  7. I2CB slave receives scan command 0x01 from master MCU
 *  8. Supply rails checked (ADCA SOC4–8) BEFORE firing; scan blocked if bad
 *  9. 150 kHz bipolar tone-burst (g_burstCycles = 5..10 cycles) fired on all
 *     8 channels simultaneously; T/R switches close (~12 µs dead time)
 * 10. ePWM1 paces ADCA+ADCC SOC0–3 every SAMPLE_PERIOD_EPWM_TICKS; a tight
 *     polling loop stores WAVE_SAMPLES continuous samples per channel
 * 11. A-scan = the full 8×WAVE_SAMPLES record; time axis is implicit:
 *         t(i) = start_delay_ns + i * sample_period_ns     (t=0 = burst start)
 * 12. Master may save the record to EEPROM under a 16-bit ID (cmd 0x06)
 *     or recall a stored record by ID into the RAM buffer (cmd 0x07)
 *
 * ── I2C protocol (I2CB slave, address 0x21) ───────────────────────────────
 *   WRITE [0x01]              trigger A-scan (rails checked first)
 *   WRITE [0x02][n]           set burst cycles, clamped to 5..10
 *   WRITE [0x03]              select STATUS stream    → READ 2 bytes
 *   WRITE [0x04][ch]          select WAVEFORM stream, channel ch = 0..7
 *                                                     → READ 1024 bytes
 *                               (WAVE_SAMPLES × uint16 LE of that channel)
 *   WRITE [0x05]              select TAPS stream      → READ 10 bytes
 *   WRITE [0x06][idL][idH]    save current A-scan to EEPROM under ID
 *   WRITE [0x07][idL][idH]    load A-scan by ID from EEPROM into RAM buffer
 *   WRITE [0x08]              select HEADER stream    → READ 16 bytes:
 *                               id(2) seq(4) burst(2) samples(2)
 *                               period_ns(2) start_delay_ns(4)   all LE
 *   READ                      bytes of the currently selected stream
 *
 * ── Timing constants (SYSCLK = 100 MHz; EPWMCLK = SYSCLK/2 fixed) ────────
 *   Burst      : 150 kHz → half-periods 333/334 SYSCLK (3.33/3.34 µs)
 *   Sampling   : ePWM1 SOCA every 80 EPWMCLK ticks (20 ns each) = 1.6 µs
 *                → 625 kSps/channel
 *                (4-SOC round robin per ADC ≈ 1.45 µs < 1.6 µs — fits)
 *   Record     : 512 samples × 1.6 µs = 819.2 µs listen window
 *
 * ── RAM budget (F28002x has 24 KB SRAM total) ─────────────────────────────
 *   g_waveBuf  : 8 × 512 uint16 = 4096 words (8 KB) — the dominant consumer.
 *   EEPROM save/load path peaks ≈ 0x250 words of stack (two 256-entry
 *   staging buffers nested) — set stack size ≥ 0x400 in the linker .cmd.
 *   NDT_captureRecord() is placed in .TI.ramfunc: make sure the linker .cmd
 *   maps that section to RAM (LOAD = FLASH, RUN = RAMLS, table copy at boot).
 *
 * ── ADC channel map (g_waveBuf row index) ────────────────────────────────
 *   [0] ADCA CH6   tlv1_out1      [4] ADCC CH6   tlv1_out2
 *   [1] ADCA CH3   tlv1_out3      [5] ADCC CH14  tlv2_out2
 *   [2] ADCA CH2   tlv1_out4      [6] ADCC CH11  tlv2_out3
 *   [3] ADCA CH9   tlv2_out1      [7] ADCC CH10  tlv2_out4
 *
 * ── Voltage-tap map (g_voltBuf index, ADCA SOC4–8) ────────────────────────
 *   [0] vpp_tap      ADCIN12  target 2.912 V   [3] 5v_pulse_tap ADCIN11 1.250 V
 *   [1] 12V_tap      ADCIN5   target 1.714 V   [4] 5v_vfd_tap   ADCIN0  1.250 V
 *   [2] 7v_vfd_tap   ADCIN1   target 1.750 V
 *
 * ── Pin assignments (verified: schematic rev-A + F280025 80QFP pinout) ───
 *   Analog_EN  = GPIO17 (pin 40)   MODE0 = GPIO39 (pin 56)
 *   Pulser_EN  = GPIO25 (pin 42)   MODE1 = GPIO42 (pin 57)
 *   THP        = GPIO13 (pin 35)   EEPROM_WC = GPIO33 (LOW = writes enabled)
 *   EEPROM I2C : I2CA — SDA GPIO26 (pin 43), SCL GPIO27 (pin 44)
 */

#include "driverlib.h"
#include "device.h"
#include "board.h"       /* SysConfig generated — Board_init() lives here   */
#include "max14808.h"
#include "m24m01e.h"
#include "i2ca_eeprom.h"

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
 * (unchanged from previous revision — see that header for derivation)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define GPIOA_A_MASK    0x000084E2UL
#define GPIOA_B_MASK    0x00C04201UL
#define GPIOB_A_MASK    0x00000300UL
#define GPIOB_B_MASK    0x00003004UL
#define GPIOA_ALL_MASK  (GPIOA_A_MASK | GPIOA_B_MASK)
#define GPIOB_ALL_MASK  (GPIOB_A_MASK | GPIOB_B_MASK)

/* ═══════════════════════════════════════════════════════════════════════════
 * Excitation / acquisition timing
 * ═══════════════════════════════════════════════════════════════════════════ */
#define PZT_FREQ_HZ           150000UL

/* 150 kHz period = 666.67 SYSCLK. Alternating 333/334-tick half-periods give
 * a 667-tick cycle → 149.925 kHz (-0.05 % — negligible against PZT Q).      */
#define BURST_HALF_TICKS_A    333U
#define BURST_HALF_TICKS_B    334U

#define BURST_CYCLES_MIN      5U
#define BURST_CYCLES_MAX      10U
#define BURST_CYCLES_DEFAULT  5U

/* ePWM1 SOCA period. On the F28002x, EPWMCLK is FIXED at SYSCLK/2 = 50 MHz
 * (no PERCLKDIVSEL.EPWMCLKDIV divider exists on this device, unlike
 * F2837x/F28004x) → one time-base tick = 20 ns.
 * 80 ticks → 1.6 µs → 625 kSps per channel.
 * Lower bound: the 4-SOC round robin per ADC needs ≈1.45 µs (15-SYSCLK
 * windows + ~210 ns conversions at ADCCLK = 50 MHz).                        */
#define EPWMCLK_TICK_NS            20U    /* EPWMCLK = SYSCLK/2, fixed       */
#define SAMPLE_PERIOD_EPWM_TICKS   80U
#define SAMPLE_PERIOD_NS           (SAMPLE_PERIOD_EPWM_TICKS * EPWMCLK_TICK_NS)

/* Samples per channel — MUST equal M24M01E_ASCAN_SAMPLES (EEPROM geometry). */
#define WAVE_SAMPLES          M24M01E_ASCAN_SAMPLES         /* 512           */
#define WAVE_CHANNELS         M24M01E_ASCAN_CHANNELS        /* 8             */

#define HV_SETTLE_US          500U
#define CPUTIMER_NS_PER_TICK  10U      /* CPU Timer 0, prescaler 0, 100 MHz  */

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
 * ═══════════════════════════════════════════════════════════════════════════ */

/* A-scan waveform buffer, channel-major: g_waveBuf[ch][sample].
 * 4096 words = 8 KB — one third of the F28002x's 24 KB SRAM.               */
volatile uint16_t g_waveBuf[WAVE_CHANNELS][WAVE_SAMPLES];

volatile uint16_t g_voltBuf[NUM_TAPS] = {0};
volatile uint16_t g_statusFlags       = 0U;

/* Header of the record currently in g_waveBuf (captured or loaded). */
static m24m01e_ascan_hdr_t g_lastHdr;

/* Pre-serialised 16-byte header stream for I2CB cmd 0x08 (byte values). */
volatile uint16_t g_hdrStream[16] = {0};

/* Configurable excitation burst length (I2CB cmd 0x02). */
volatile uint16_t g_burstCycles = BURST_CYCLES_DEFAULT;

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

typedef enum {
    NDT_IDLE,           /* waiting for a command via I2CB                    */
    NDT_FIRE,           /* 0x01 received; check rails, burst, capture        */
    NDT_DATA_READY,     /* record complete; housekeeping then back to IDLE   */
    NDT_EEPROM_OP,      /* save/load requested; executed in main loop        */
    NDT_THERMAL_FAULT,  /* MAX14808 over-temperature — HV locked until reset */
    NDT_VOLTAGE_FAULT   /* supply rail(s) out of range — scan blocked        */
} NDT_State;

volatile NDT_State g_ndtState = NDT_IDLE;

/* EEPROM operation request (set by I2CB ISR, executed in main loop). */
typedef enum { EE_NONE = 0, EE_SAVE, EE_LOAD } EE_Op;
volatile EE_Op    g_eeOp = EE_NONE;
volatile uint16_t g_eeId = 0U;

/* Fire moment → first-sample delay of the current record, in ns. */
static volatile uint32_t g_startDelayNs = 0U;

/* ── I2CB slave stream state ─────────────────────────────────────────────── */
volatile uint16_t g_i2cTxPtr = 0U;   /* byte index into the TX stream        */

typedef enum {
    I2C_READ_STATUS = 0,  /*  2 bytes: status-flag word                      */
    I2C_READ_TAPS,        /* 10 bytes: raw voltage-tap counts                */
    I2C_READ_WAVE,        /* WAVE_SAMPLES×2 bytes: one channel's waveform    */
    I2C_READ_HDR          /* 16 bytes: current record header summary         */
} I2C_ReadSel;
volatile I2C_ReadSel g_i2cReadSel = I2C_READ_STATUS;
volatile uint16_t    g_waveCh     = 0U;   /* channel selected by cmd 0x04    */

/* Multi-byte command parser (RX side). */
static volatile uint16_t s_rxCmd    = 0U;
static volatile uint16_t s_rxNeed   = 0U;
static volatile uint16_t s_rxGot    = 0U;
static volatile uint16_t s_rxPar[2] = {0U, 0U};

/* ═══════════════════════════════════════════════════════════════════════════
 * MAX14808 device handle and platform callbacks
 * ═══════════════════════════════════════════════════════════════════════════ */
static max14808_dev_t g_pulser;
static m24m01e_t      g_eeprom;

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
 * NDT_initTimers — CPU Timer 0 free-running (timestamps, burst pacing)
 *                  and ePWM1 as the ADC sample-rate pacer (frozen at boot).
 *
 * ePWM1 is configured entirely here (no SysConfig instance, no pins):
 * Device_init()'s Device_enableAllPeripherals() has already clocked it.
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

    /* NOTE: on the F28002x, EPWMCLK is hard-fixed at SYSCLK/2 = 50 MHz.
     * There is NO SysCtl_setEPWMClockDivider() / PERCLKDIVSEL divider on
     * this device (that API exists only on F2837x/F2838x/F28004x-class
     * parts). All ePWM tick maths below is therefore in 20 ns units.       */
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
 * NDT_initEeprom — WC pin low, transport bind, probe.
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

    GPIO_writePin(PIN_EEPROM_WC, 0U);   /* writes enabled                    */

    if (m24m01e_init(&g_eeprom, &io, 0U) != M24M01E_OK ||
        m24m01e_probe(&g_eeprom)         != M24M01E_OK)
    {
        g_statusFlags |= STATUS_EEPROM_FAIL;
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 * tap_in_range / NDT_checkSupplies — unchanged: forces SOC4–8, polls the
 * ADCA INT2 flag (enabled in SysConfig but never PIE-registered).
 * ───────────────────────────────────────────────────────────────────────── */
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
 * ePWM1 SOCA triggers ADCA SOC0–3 and ADCC SOC0–3 every 1.6 µs (SysConfig:
 * socXTrigger = EPWM1_SOCA). Each sweep completes in ≈1.45 µs; this loop
 * polls both INT1 flags (SOC3 EOC), stores the 8 results, and repeats for
 * WAVE_SAMPLES sweeps. Per-sweep CPU budget is 160 cycles — the store body
 * fits with margin at -O2, and the function runs from RAM (.TI.ramfunc) so
 * flash wait-states can't push it over.
 *
 * The ADC scan ISRs of the previous revision are gone: SysConfig no longer
 * PIE-registers ADC INT1, so the flags set in hardware without vectoring,
 * exactly like the INT2 tap poll.
 * ───────────────────────────────────────────────────────────────────────── */
#pragma CODE_SECTION(NDT_captureRecord, ".TI.ramfunc")
static void NDT_captureRecord(void)
{
    uint16_t n;

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

    /* Arm the pacer: counter to zero, then run. The first SOCA fires on the
     * first zero event, i.e. effectively immediately.                       */
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

/* ─────────────────────────────────────────────────────────────────────────
 * NDT_packHdrStream — serialise g_lastHdr into the 16-byte I2CB stream
 * for command 0x08 (all fields little-endian).
 * ───────────────────────────────────────────────────────────────────────── */
static void NDT_packHdrStream(void)
{
    g_hdrStream[0]  = (uint16_t)( g_lastHdr.id                & 0xFFU);
    g_hdrStream[1]  = (uint16_t)((g_lastHdr.id        >> 8U)  & 0xFFU);
    g_hdrStream[2]  = (uint16_t)( g_lastHdr.seq               & 0xFFU);
    g_hdrStream[3]  = (uint16_t)((g_lastHdr.seq       >> 8U)  & 0xFFU);
    g_hdrStream[4]  = (uint16_t)((g_lastHdr.seq       >> 16U) & 0xFFU);
    g_hdrStream[5]  = (uint16_t)((g_lastHdr.seq       >> 24U) & 0xFFU);
    g_hdrStream[6]  = (uint16_t)( g_lastHdr.burst_cycles      & 0xFFU);
    g_hdrStream[7]  = (uint16_t)((g_lastHdr.burst_cycles >> 8U) & 0xFFU);
    g_hdrStream[8]  = (uint16_t)( g_lastHdr.samples_per_ch    & 0xFFU);
    g_hdrStream[9]  = (uint16_t)((g_lastHdr.samples_per_ch >> 8U) & 0xFFU);
    g_hdrStream[10] = (uint16_t)( g_lastHdr.sample_period_ns  & 0xFFU);
    g_hdrStream[11] = (uint16_t)((g_lastHdr.sample_period_ns >> 8U) & 0xFFU);
    g_hdrStream[12] = (uint16_t)( g_lastHdr.start_delay_ns          & 0xFFU);
    g_hdrStream[13] = (uint16_t)((g_lastHdr.start_delay_ns >> 8U)   & 0xFFU);
    g_hdrStream[14] = (uint16_t)((g_lastHdr.start_delay_ns >> 16U)  & 0xFFU);
    g_hdrStream[15] = (uint16_t)((g_lastHdr.start_delay_ns >> 24U)  & 0xFFU);
}

/* ─────────────────────────────────────────────────────────────────────────
 * NDT_fireAndCapture — one complete A-scan:
 *   1. HV on, settle.
 *   2. t0 := now; N-cycle 150 kHz tone-burst.
 *   3. T/R switches to receive (~12 µs dead time in the driver).
 *   4. Measure actual t0 → capture-start delay (stored in the header).
 *   5. Continuous ePWM-paced capture of WAVE_SAMPLES per channel.
 * ───────────────────────────────────────────────────────────────────────── */
static void NDT_fireAndCapture(void)
{
    uint32_t t_fire, t_cap;

    /* ── 1. Enable HV supply and wait for VP/VN to stabilise ─────────────── */
    GPIO_writePin(PIN_PULSER_EN, 1U);
    DEVICE_DELAY_US(HV_SETTLE_US);

    /* ── 2. Tone-burst (t = 0 for the record's time axis) ────────────────── */
    t_fire = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    NDT_burst(g_burstCycles);

    /* ── 3. Receive mode via driver (~12 µs dead time) ───────────────────── */
    max14808_enter_receive_mode(&g_pulser, true);

    /* ── 4. Fire-to-first-sample delay (down-counter: elapsed = old - new) ─ */
    t_cap = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    g_startDelayNs = (uint32_t)(t_fire - t_cap) * CPUTIMER_NS_PER_TICK;

    /* ── 5. Continuous capture ───────────────────────────────────────────── */
    NDT_captureRecord();

    /* ── Record header for this capture ──────────────────────────────────── */
    g_lastHdr.id               = 0U;              /* assigned on save (0x06) */
    g_lastHdr.seq              = 0U;
    g_lastHdr.burst_cycles     = g_burstCycles;
    g_lastHdr.freq_hz          = PZT_FREQ_HZ;
    g_lastHdr.sample_period_ns = SAMPLE_PERIOD_NS;
    g_lastHdr.start_delay_ns   = g_startDelayNs;
    g_lastHdr.samples_per_ch   = WAVE_SAMPLES;
    g_lastHdr.channels         = WAVE_CHANNELS;
    g_lastHdr.status_flags     = g_statusFlags;
    NDT_packHdrStream();
}

/* ─────────────────────────────────────────────────────────────────────────
 * NDT_runEepromOp — executes the save/load requested over I2CB.
 * Runs in main-loop context; blocking (~0.3 s for a full save). The I2CB
 * slave stays serviceable throughout (its ISR keeps running), and the
 * master can watch STATUS_EEPROM_BUSY via cmd 0x03.
 * ───────────────────────────────────────────────────────────────────────── */
static void NDT_runEepromOp(void)
{
    m24m01e_status_t st;

    g_statusFlags |=  STATUS_EEPROM_BUSY;
    g_statusFlags &= ~STATUS_EEPROM_FAIL;

    if (g_eeOp == EE_SAVE)
    {
        if ((g_statusFlags & STATUS_DATA_VALID) != 0U)
        {
            g_lastHdr.id = g_eeId;
            st = m24m01e_ascan_save(&g_eeprom, &g_lastHdr,
                                    (const uint16_t *)&g_waveBuf[0][0]);
        }
        else
        {
            st = M24M01E_ERR_PARAM;   /* nothing valid to save */
        }
    }
    else /* EE_LOAD */
    {
        st = m24m01e_ascan_load(&g_eeprom, g_eeId, &g_lastHdr,
                                (uint16_t *)&g_waveBuf[0][0]);
        if (st == M24M01E_OK)
        {
            g_statusFlags |= STATUS_DATA_VALID;   /* buffer now holds record */
        }
    }

    if (st != M24M01E_OK)
    {
        g_statusFlags |= STATUS_EEPROM_FAIL;
    }
    NDT_packHdrStream();              /* seq/id updated by save; hdr by load */

    g_statusFlags &= ~STATUS_EEPROM_BUSY;
    g_eeOp = EE_NONE;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * I2CB ISR — slave interface to master MCU
 *   SDA = GPIO2 (chip pin 61),  SCL = GPIO3 (chip pin 60)
 * See the protocol table in the file header. Multi-byte commands are parsed
 * across consecutive RX_DATA_RDY events within one write transaction; a new
 * address match resets the parser.
 * ═══════════════════════════════════════════════════════════════════════════ */
__interrupt void INT_myI2CB_ISR(void)
{
    uint32_t src = I2C_getInterruptSource(I2CB_BASE);

    switch (src)
    {
        /* ── Address match: master opened a new transaction ─────────────── */
        case I2C_INTSRC_ADDR_SLAVE:
            g_i2cTxPtr = 0U;
            s_rxNeed   = 0U;
            s_rxGot    = 0U;
            break;

        /* ── Receive data ready: command byte or parameter byte ─────────── */
        case I2C_INTSRC_RX_DATA_RDY:
        {
            uint16_t d = (uint16_t)(I2C_getData(I2CB_BASE) & 0x00FFU);

            if (s_rxNeed == 0U)
            {
                /* First byte of a transaction: the command. */
                switch (d)
                {
                    case 0x01U:                 /* trigger A-scan            */
                        if (g_ndtState == NDT_IDLE)
                        {
                            g_ndtState = NDT_FIRE;
                            GPIO_togglePin(PIN_MCU_LED);
                        }
                        break;

                    case 0x02U:                 /* set burst cycles [n]      */
                    case 0x04U:                 /* select wave channel [ch]  */
                        s_rxCmd  = d;
                        s_rxNeed = 1U;
                        s_rxGot  = 0U;
                        break;

                    case 0x06U:                 /* save A-scan [idL][idH]    */
                    case 0x07U:                 /* load A-scan [idL][idH]    */
                        s_rxCmd  = d;
                        s_rxNeed = 2U;
                        s_rxGot  = 0U;
                        break;

                    case 0x03U:
                        g_i2cReadSel = I2C_READ_STATUS;
                        break;

                    case 0x05U:
                        g_i2cReadSel = I2C_READ_TAPS;
                        break;

                    case 0x08U:
                        g_i2cReadSel = I2C_READ_HDR;
                        break;

                    default:
                        /* Unknown command — ignored. */
                        break;
                }
            }
            else
            {
                /* Parameter byte of a multi-byte command. */
                s_rxPar[s_rxGot++] = d;
                if (s_rxGot >= s_rxNeed)
                {
                    s_rxNeed = 0U;
                    switch (s_rxCmd)
                    {
                        case 0x02U:
                        {
                            uint16_t n = s_rxPar[0];
                            if (n < BURST_CYCLES_MIN) n = BURST_CYCLES_MIN;
                            if (n > BURST_CYCLES_MAX) n = BURST_CYCLES_MAX;
                            g_burstCycles = n;
                            break;
                        }

                        case 0x04U:
                            g_waveCh     = (uint16_t)(s_rxPar[0] & 0x07U);
                            g_i2cReadSel = I2C_READ_WAVE;
                            break;

                        case 0x06U:
                        case 0x07U:
                            if (g_ndtState == NDT_IDLE)
                            {
                                g_eeId  = (uint16_t)(s_rxPar[0] |
                                          ((uint16_t)s_rxPar[1] << 8U));
                                g_eeOp  = (s_rxCmd == 0x06U) ? EE_SAVE
                                                             : EE_LOAD;
                                g_ndtState = NDT_EEPROM_OP;
                            }
                            break;

                        default:
                            break;
                    }
                }
            }
            break;
        }

        /* ── Transmit data ready: master is reading the selected stream ──── */
        case I2C_INTSRC_TX_DATA_RDY:
        {
            uint16_t txByte;
            uint16_t len;

            switch (g_i2cReadSel)
            {
                case I2C_READ_TAPS:
                {
                    uint16_t w = g_voltBuf[g_i2cTxPtr >> 1U];
                    len    = (uint16_t)(NUM_TAPS * 2U);
                    txByte = (g_i2cTxPtr & 1U)
                             ? (uint16_t)(w >> 8U)
                             : (uint16_t)(w & 0x00FFU);
                    break;
                }

                case I2C_READ_WAVE:
                {
                    uint16_t w = g_waveBuf[g_waveCh][g_i2cTxPtr >> 1U];
                    len    = (uint16_t)(WAVE_SAMPLES * 2U);
                    txByte = (g_i2cTxPtr & 1U)
                             ? (uint16_t)(w >> 8U)
                             : (uint16_t)(w & 0x00FFU);
                    break;
                }

                case I2C_READ_HDR:
                    len    = 16U;
                    txByte = g_hdrStream[g_i2cTxPtr];
                    break;

                case I2C_READ_STATUS:
                default:
                    len    = 2U;
                    txByte = (g_i2cTxPtr & 1U)
                             ? (uint16_t)(g_statusFlags >> 8U)
                             : (uint16_t)(g_statusFlags & 0x00FFU);
                    break;
            }

            I2C_putData(I2CB_BASE, txByte);

            if (++g_i2cTxPtr >= len) { g_i2cTxPtr = 0U; }
            break;
        }

        /* ── Stop condition: transaction complete ────────────────────────── */
        case I2C_INTSRC_STOP_CONDITION:
            g_i2cTxPtr = 0U;
            s_rxNeed   = 0U;
            break;

        default:
            /* TODO (future stage): NACK / arbitration-lost handling.        */
            break;
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════════ */
void main(void)
{
    /* ── 1. Core system init ──────────────────────────────────────────────── */
    /*
     * Device_init() sets SYSCLK = 100 MHz and enables peripheral clocks.
     * When the project defines the predefined symbol _FLASH (Project
     * Properties → C2000 Compiler → Predefined Symbols, add "_FLASH"),
     * Device_init() ALSO copies .TI.ramfunc from flash to RAM and calls
     * Flash_initModule() to program wait-states — so no manual copy is
     * needed here. Build without _FLASH for a RAM target.
     */
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    /* ── 2. SysConfig-generated peripheral init ───────────────────────────── */
    /*
     * Board_init() configures:
     *   ADC  — ADCA: SOC0–3 scan (trigger = EPWM1_SOCA, INT1 flag polled),
     *                SOC4–8 voltage taps (software-forced, INT2 flag polled)
     *          ADCC: SOC0–3 scan (trigger = EPWM1_SOCA, INT1 flag polled)
     *          ADC INT1 is NOT PIE-registered any more — no scan ISRs.
     *   GPIO — all pulser DINP/DINN, control, status pins, EEPROM_WC
     *   I2CB — slave (TARGET) mode, interrupt registered
     *   I2CA — master @400 kHz, polled — M24M01E EEPROM transport
     */
    Board_init();

    /* ── 3. Timers: CPU Timer 0 free-running + ePWM1 sample pacer ─────────── */
    NDT_initTimers();

    /* ── 4. Enable analogue front end + gain ──────────────────────────────── */
    GPIO_writePin(PIN_ANALOG_EN, 1U);
    DEVICE_DELAY_US(500U);
    GPIO_writePin(PIN_AFE_GAIN_A1, 1U);
    GPIO_writePin(PIN_AFE_GAIN_A2, 1U);
    GPIO_writePin(PIN_AFE_GAIN_B1, 1U);
    GPIO_writePin(PIN_AFE_GAIN_B2, 1U);

    /* ── 5. Initialise MAX14808 pulser ────────────────────────────────────── */
    NDT_initPulser();

    /* ── 6. Initialise M24M01E EEPROM (non-fatal on failure) ──────────────── */
    NDT_initEeprom();

    /* ── 7. Enable CPU interrupts ─────────────────────────────────────────── */
    Interrupt_enableGlobal();

    /* ── 8. Status LED on — initialisation complete ───────────────────────── */
    GPIO_writePin(PIN_MCU_LED, 1U);

    /* ══════════════════════════════════════════════════════════════════════
     * Main loop — NDT state machine
     *   NDT_IDLE          → poll THP; I2CB ISR advances state
     *   NDT_FIRE          → check rails; burst + capture; → DATA_READY
     *   NDT_DATA_READY    → restore pulser, HV off, DATA_VALID, → IDLE
     *   NDT_EEPROM_OP     → run save/load; → IDLE
     *   NDT_THERMAL_FAULT → HV off, outputs off, blink LED, latched
     *   NDT_VOLTAGE_FAULT → HV off, → IDLE (master re-issues 0x01)
     * ══════════════════════════════════════════════════════════════════════ */
    for (;;)
    {
        switch (g_ndtState)
        {
            case NDT_IDLE:
                /* MAX14808 THP: open-drain, active-low over-temp flag. */
                if (GPIO_readPin(PIN_THP) == 0U)
                {
                    g_ndtState = NDT_THERMAL_FAULT;
                }
                break;

            case NDT_FIRE:
            {
                uint16_t rails;

                g_statusFlags &= ~(STATUS_DATA_VALID | STATUS_VOLTAGE_FAULT |
                                   STATUS_ALL_RAILS_OK);

                rails = NDT_checkSupplies();
                g_statusFlags |= rails;

                if ((rails & STATUS_ALL_RAILS_OK) == STATUS_ALL_RAILS_OK)
                {
                    /* Blocking: burst + full continuous capture (~0.9 ms). */
                    NDT_fireAndCapture();
                    g_ndtState = NDT_DATA_READY;
                }
                else
                {
                    g_statusFlags |= STATUS_VOLTAGE_FAULT;
                    g_ndtState     = NDT_VOLTAGE_FAULT;
                }
                break;
            }

            case NDT_DATA_READY:
                /* Restore MAX14808 octal-3L ready state (resets DINP/DINN). */
                max14808_set_mode(&g_pulser, MAX14808_MODE_OCTAL_3LEVEL);

                /* HV off until the next scan command. */
                GPIO_writePin(PIN_PULSER_EN, 0U);

                g_statusFlags |= STATUS_DATA_VALID;
                g_ndtState = NDT_IDLE;
                break;

            case NDT_EEPROM_OP:
                NDT_runEepromOp();
                g_ndtState = NDT_IDLE;
                break;

            case NDT_THERMAL_FAULT:
                g_statusFlags |= STATUS_THERMAL_FAULT;
                GPIO_writePin(PIN_PULSER_EN, 0U);
                max14808_set_mode(&g_pulser, MAX14808_MODE_TX_DISABLE);

                GPIO_togglePin(PIN_MCU_LED);
                DEVICE_DELAY_US(100000U);
                break;

            case NDT_VOLTAGE_FAULT:
                GPIO_writePin(PIN_PULSER_EN, 0U);
                g_ndtState = NDT_IDLE;
                break;

            default:
                g_ndtState = NDT_IDLE;
                break;
        }
    }
}