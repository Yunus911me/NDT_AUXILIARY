# F280025 NDT Auxiliary Board Firmware

Firmware for a 150 kHz guided-wave ultrasonic collar (NDT auxiliary board) built around a TI **TMS320F280025** (C2000, 100 MHz). The board fires a bipolar tone-burst on 8 PZT channels via a MAX14808 octal pulser, captures a synchronous 8-channel A-scan at 625 kSps/channel, and serves the record to a master MCU over I2C. Records can be archived to and recalled from an on-board M24M01E EEPROM.

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

1. Master writes `0x01` over I2CB.
2. Supply rails are measured (ADCA SOC4–8) scan is **blocked** if any rail is out of tolerance.
3. HV supply enabled (`Pulser_EN`), 500 µs settle.
4. 5–10 cycle 150 kHz bipolar tone-burst fired on all 8 channels simultaneously (direct GPIO register writes, drift-free CPU-Timer-0 pacing).
5. ePWM1 paces ADCA+ADCC SOC0–3 every 1.6 µs; a tight RAM-resident polling loop stores 512 samples × 8 channels.
6. Record marked valid; HV disabled; board returns to idle.

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

- Commands `0x01`, `0x06`, `0x07` are **silently ignored** unless the board is idle. Poll `DATA_VALID` / `EEPROM_BUSY` to sequence operations.
- A scan (burst + capture) is blocking on the board side and completes in ~0.9 ms; the I2CB slave remains serviceable throughout.
- Rail tolerance is ±5 % of the tap targets listed below.

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

---

## Memory

The F28002x has **24 KB SRAM total**  budget carefully:

- `g_waveBuf` (8 × 512 × uint16) = 4096 words (**8 KB**) the dominant consumer.
- The EEPROM save/load path peaks at ≈ 0x250 words of stack (two nested 256-entry staging buffers). **Set stack size ≥ 0x400** in the linker `.cmd`.
- `NDT_captureRecord()` is placed in `.TI.ramfunc`. The linker `.cmd` must map this section **LOAD = FLASH, RUN = RAMLS** with a copy table; `Device_init()` performs the copy at boot when `_FLASH` is defined.

---

## Building

Toolchain: **Code Composer Studio** with the C2000 compiler, **C2000Ware** driverlib for F28002x, and **SysConfig** (generates `board.c/h`).

1. Import the CCS project; confirm the device is F280025.
2. For a flash target, add the predefined symbol `_FLASH` (Project Properties → C2000 Compiler → Predefined Symbols). This makes `Device_init()` copy `.TI.ramfunc` to RAM and program flash wait-states. Omit `_FLASH` for a RAM-only debug target.
3. Build with **`-O2`** the capture loop's 160-cycle-per-sweep budget assumes optimized code.
4. Verify the linker `.cmd` provides the `.TI.ramfunc` LOAD/RUN mapping and ≥ 0x400-word stack.

### SysConfig responsibilities (`Board_init()`)

- **ADCA**: SOC0–3 scan (trigger = `EPWM1_SOCA`, INT1 flag **polled**, not PIE-registered); SOC4–8 voltage taps (software-forced, INT2 flag polled).
- **ADCC**: SOC0–3 scan (trigger = `EPWM1_SOCA`, INT1 flag polled).
- **GPIO**: all pulser DINP/DINN, control, status pins, `EEPROM_WC`.
- **I2CB**: slave (target) mode @ address 0x21, interrupt registered.
- **I2CA**: master @ 400 kHz, polled EEPROM transport.

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

- **Thermal fault** (`THP` low, active-low open-drain): HV disabled, pulser outputs disabled, LED blinks at ~5 Hz. **Latched** requires a reset.
- **Voltage fault**: scan is blocked, `VOLTAGE_FAULT` set, board returns to idle. The master may retry `0x01` once rails recover.
- **EEPROM missing/failed**: non-fatal. `EEPROM_FAIL` is set at boot and scans continue to work; only storage is unavailable.

---

## Source files

| File            | Contents                                                   |
|-----------------|------------------------------------------------------------|
| `main.c`        | State machine, burst generation, capture, I2CB slave ISR   |
| `board.c/h`     | SysConfig-generated peripheral init                        |
| `max14808.c/h`  | MAX14808 octal pulser driver (mode, current, T/R switching)|
| `m24m01e.c/h`   | M24M01E EEPROM driver + A-scan record save/load layout     |
| `i2ca_eeprom.c/h` | I2CA polled-master transport callbacks for the EEPROM    |
