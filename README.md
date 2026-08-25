# F280025 NDT Auxiliary Board Firmware

Firmware for a 150 kHz guided-wave ultrasonic collar (NDT auxiliary board) built around a TI **TMS320F280025** (C2000, 100 MHz). The board fires a bipolar tone-burst on 8 PZT channels via a MAX14808 octal pulser, captures a synchronous 8-channel A-scan at 625 kSps/channel, and serves the record to a master MCU over I2C. Records can be archived to and recalled from an on-board M24M01E EEPROM.

The firmware is split into a policy layer (state machine, configuration), a
board layer (pins, initialisation, acquisition), and three driver stacks
(EEPROM, pulser, I2C slave). See **[Layers.md](Layers.md)** for the layer
hierarchy and the dependency map before making changes.

---

## System overview

```
 Master MCU ──I2CB (slave 0x21)──► F280025 ──I2CA (master)──► M24M01E EEPROM
                                      │
                    ┌─────────────────┼──────────────────────────┐
                    ▼                                            ▲
              MAX14808 pulser                               ADCA / ADCC
              (octal 3-level,                              (SOC0–3 scan,
               8× DINP/DINN)                               ePWM1-paced)
                    │                                            ▲
                    ▼                                            │
               8× PZT elements ──► AD8334 AFE + TLV9354 buffers ─┘
                                        
```

### Scan sequence

1. A trigger arrives — master writes `0x01` over I2CB, or the periodic timer
   elapses, depending on `NDT_TRIGGER_SOURCE` in `ndt_config.h`. Both produce
   the same internal event.
2. Supply rails are measured (ADCA SOC4–8) scan is **blocked** if any rail is out of tolerance.
3. HV supply enabled (`Pulser_EN`), 500 µs settle.
4. 5–10 cycle 150 kHz bipolar tone-burst fired on all 8 channels simultaneously (direct GPIO register writes, drift-free CPU-Timer-0 pacing).
5. ePWM1 paces ADCA+ADCC SOC0–3 every 1.6 µs; a tight RAM-resident polling loop stores 512 samples × 8 channels.
6. Record marked valid; HV disabled; board returns to idle.

While the record buffer is being written (capture, or an EEPROM load) the slave
read side is frozen and returns `0xFF`, so a master read can never observe a
half-overwritten A-scan.

The A-scan time axis is implicit: `t(i) = start_delay_ns + i × sample_period_ns`, with `t = 0` at burst start. `start_delay_ns` is measured per capture and reported in the record header.

---

## I2C command protocol (I2CB slave, address 0x21)

| Write bytes            | Function                                             |
|------------------------|------------------------------------------------------|
| `[0x01]`               | Trigger A-scan (rails checked first)                 |
| `[0x02][n]`            | Set burst cycles, clamped to 5..10                   |
| `[0x03]`               | Select **STATUS** stream → read 2 bytes              |
| `[0x04][ch]`           | Select **WAVEFORM** stream, ch = 0..7 → read 1024 B  |
| `[0x05]`               | Select **TAPS** stream → read 10 bytes               |
| `[0x06][idL][idH]`     | Save current A-scan to EEPROM under 16-bit ID        |
| `[0x07][idL][idH]`     | Load A-scan by ID from EEPROM into RAM buffer        |
| `[0x08]`               | Select **HEADER** stream → read 16 bytes             |

A plain **read** returns bytes from the currently selected stream. All multi-byte values are little-endian.

**Header stream (16 bytes):** `id(2) seq(4) burst_cycles(2) samples_per_ch(2) sample_period_ns(2) start_delay_ns(4)`.

**Status word bits:**

| Bit    | Flag                | Meaning                                  |
|--------|---------------------|------------------------------------------|
| 0x0001 | `VPP_OK`            | HV rail in tolerance                     |
| 0x0002 | `12V_OK`            | 12 V rail in tolerance                   |
| 0x0004 | `7V_VFD_OK`         | 7 V VFD rail in tolerance                |
| 0x0008 | `5V_PULSE_OK`       | 5 V pulser rail in tolerance             |
| 0x0010 | `5V_VFD_OK`         | 5 V VFD rail in tolerance                |
| 0x0020 | `DATA_VALID`        | `g_waveBuf` holds a valid A-scan         |
| 0x0040 | `THERMAL_FAULT`     | MAX14808 over-temperature (latched)      |
| 0x0080 | `VOLTAGE_FAULT`     | Last scan blocked by bad rail(s)         |
| 0x0100 | `EEPROM_BUSY`       | Save/load in progress (~0.3 s for save)  |
| 0x0200 | `EEPROM_FAIL`       | Last EEPROM operation failed / no EEPROM |

**Master usage notes**

- Commands `0x01`, `0x06`, `0x07` are **queued** (8 deep) and executed as soon as the board returns to idle; they are dropped only on queue overflow. *This changed with the state-machine refactor — the previous firmware ignored them outright unless the board was idle.* Poll `DATA_VALID` / `EEPROM_BUSY` to sequence operations.
- A read issued while the record buffer is being rewritten returns `0xFF` bytes rather than partial data. Check `EEPROM_BUSY` / `DATA_VALID` first.
- Unknown command bytes are counted, not acted on.
- A scan (burst + capture) is blocking on the board side and completes in ~0.9 ms; the I2CB slave remains serviceable throughout.
- Rail tolerance is ±5 % of the tap targets listed below.

---

## Configuration (`ndt_config.h`)

Every policy decision lives in one header; physical facts (pin numbers, SYSCLK
tick counts, ADC mapping) stay in `main.c`.

| Symbol | Default | Effect |
|--------|---------|--------|
| `NDT_ASCAN_SAMPLES` / `_CHANNELS` | 512 / 8 | Record geometry. Product × 2 must be a multiple of the 256-byte EEPROM page; a `#error` enforces it. |
| `NDT_BURST_CYCLES_MIN/MAX/DEFAULT` | 5 / 10 / 5 | Clamp applied to command `0x02`. |
| `NDT_TRIGGER_SOURCE` | `NDT_TRIG_I2C` | `NDT_TRIG_I2C`, `NDT_TRIG_PERIODIC`, or `NDT_TRIG_BOTH`. |
| `NDT_TRIG_PERIOD_MS` | 1000 | Period for the periodic source. |
| `NDT_TRIG_SKIP_MISSED` | 1 | Periodic only: skip ticks missed during a long scan, or catch up. |
| `NDT_THERMAL_LATCHED` | 1 | 0 lets the board recover once `THP` releases. |
| `NDT_THERMAL_BLINK_MS` | 100 | Fault LED blink half-period. |
| `NDT_I2CB_ADDRESS` | 0x21 | Slave address (must match SysConfig). |
| `NDT_EVENT_QUEUE_DEPTH` | 8 | ISR → main-loop event ring; power of two. |

Switching to periodic firing requires no code change: an I2CB command and a
timer tick both post `NDT_EV_TRIGGER`, and the state machine cannot tell them
apart.

---

## Timing

SYSCLK = 100 MHz. On the F28002x, **EPWMCLK is hard-fixed at SYSCLK/2 = 50 MHz** there is no `PERCLKDIVSEL` EPWM clock divider on this device family (unlike F2837x/F28004x), so all ePWM tick maths is in 20 ns units.

| Item              | Value                                                      |
|-------------------|------------------------------------------------------------|
| Burst frequency   | 150 kHz nominal (333/334-tick alternating half-periods → 149.925 kHz, −0.05 %) |
| Burst length      | 5–10 cycles (default 5), configurable via cmd `0x02`       |
| Sample pacer      | ePWM1 SOCA every 80 EPWMCLK ticks = **1.6 µs**             |
| Sample rate       | **625 kSps per channel** (8 channels simultaneous)         |
| ADC sweep budget  | 4-SOC round robin per ADC ≈ 1.45 µs < 1.6 µs ✓             |
| Record length     | 512 samples × 1.6 µs = **819.2 µs** listen window          |
| Timestamp base    | CPU Timer 0, free-running, 10 ns/tick                      |
| Millisecond tick  | CPU Timer 1, 1 kHz interrupt (periodic trigger, fault blink) |

---

## Memory

The F28002x has **24 KB SRAM total**  budget carefully:

- `g_waveBuf` (8 × 512 × uint16) = 4096 words (**8 KB**) the dominant consumer.
- The EEPROM save/load path peaks at ≈ 0x250 words of stack (two nested 256-entry staging buffers). **Set stack size ≥ 0x400** in the linker `.cmd`.
- `NDT_captureRecord()` is placed in `.TI.ramfunc`. The linker `.cmd` must map this section **LOAD = FLASH, RUN = RAMLS** with a copy table; `Device_init()` performs the copy at boot when `_FLASH` is defined.

---

## Hardware reference

### ADC channel map (`g_waveBuf` row index)

| Row | ADC / channel | Signal      | Row | ADC / channel | Signal      |
|-----|---------------|-------------|-----|---------------|-------------|
| 0   | ADCA CH6      | tlv1_out1   | 4   | ADCC CH6      | tlv1_out2   |
| 1   | ADCA CH3      | tlv1_out3   | 5   | ADCC CH14     | tlv2_out2   |
| 2   | ADCA CH2      | tlv1_out4   | 6   | ADCC CH11     | tlv2_out3   |
| 3   | ADCA CH9      | tlv2_out1   | 7   | ADCC CH10     | tlv2_out4   |

### Voltage taps (`g_voltBuf`, ADCA SOC4–8)

| Index | Tap           | ADC input | Target   |
|-------|---------------|-----------|----------|
| 0     | vpp_tap       | ADCIN12   | 2.912 V  |
| 1     | 12V_tap       | ADCIN5    | 1.714 V  |
| 2     | 7v_vfd_tap    | ADCIN1    | 1.750 V  |
| 3     | 5v_pulse_tap  | ADCIN11   | 1.250 V  |
| 4     | 5v_vfd_tap    | ADCIN0    | 1.250 V  |

### Key control pins (schematic rev-A, 80QFP)

| Function      | GPIO   | Pin | Function      | GPIO   | Pin |
|---------------|--------|-----|---------------|--------|-----|
| Analog_EN     | GPIO17 | 40  | MODE0         | GPIO39 | 56  |
| Pulser_EN     | GPIO25 | 42  | MODE1         | GPIO42 | 57  |
| THP (fault)   | GPIO13 | 35  | EEPROM_WC     | GPIO33 | —   |
| MCU LED       | GPIO16 | 39  | I2CB SDA/SCL  | GPIO2/3 | 61/60 |
| EEPROM I2CA   | SDA GPIO26 (43), SCL GPIO27 (44) |||||

Pulser DINP/DINN channel pins and the derived GPIO write masks are documented at the top of `main.c`. The MAX14808 CC pins are hard-wired for 2 A on the PCB; MODE0=1 / MODE1=0 selects octal three-level mode.

---

## Fault handling

- **Thermal fault** (`THP` low, active-low open-drain): HV disabled, pulser outputs disabled, LED blinks at `NDT_THERMAL_BLINK_MS` intervals. Latched by default (`NDT_THERMAL_LATCHED`), requiring a reset. The blink is timer-driven, so the main loop keeps draining commands while the fault is active.
- **Voltage fault**: scan is blocked, `VOLTAGE_FAULT` set, board returns to idle. The master may retry `0x01` once rails recover.
- **EEPROM missing/failed**: non-fatal. `EEPROM_FAIL` is set at boot and scans continue to work; only storage is unavailable.

---

## Source files

| File | Layer | Contents |
|------|-------|----------|
| `ndt_config.h` | policy | Every tunable: geometry, trigger source, fault behaviour, queue depth |
| `ndt_sm.c/h` | policy | State machine, event queue, trigger rules. No driverlib — builds on a host compiler |
| `main.c` | board | Pin map, peripheral init, tone-burst, capture loop, shared buffers, hook table, `main()` |
| `ndt_i2cb.c/h` | interface | I2CB slave protocol and ISR; posts events, serves streams |
| `ndt_store.c/h` | record | A-scan slot format: magic, sequence, eviction, checksum, header serialisation |
| `m24m01e.c/h` | chip | M24M01E EEPROM driver — addressing, paging, ACK polling, feature registers |
| `i2ca_eeprom.c/h` | transport | `m24m01e_io_t` callbacks bound to F280025 I2CA, polled, timeout-bounded |
| `max14808.c/h` | driver | MAX14808 octal pulser: mode, current, T/R switching |
| `board.c/h` | platform | SysConfig-generated peripheral init — do not hand-edit |

