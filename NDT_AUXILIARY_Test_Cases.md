# NDT Auxiliary Board — Firmware Test Case Specification

**Project:** `NDT_AUXILIARY` — F280025 NDT Auxiliary Board Firmware
**DUT:** TI TMS320F280025 (C2000, SYSCLK 100 MHz, EPWMCLK 50 MHz fixed), MAX14808 octal pulser, AD8334 + TLV9354 AFE, M24M01E 1 Mbit EEPROM
**Document version:** 1.0
**Status:** Draft — for review

---

## 1. Scope

This document defines the verification test cases for the NDT auxiliary board firmware: build integrity, boot/initialisation, supply-rail supervision, tone-burst excitation, ADC capture, the I2CB slave command protocol, EEPROM record storage, fault handling, and the top-level state machine.

**In scope:** firmware behaviour observable at the I2C interface, at board GPIO/analog pins, and through the CCS debugger.
**Out of scope:** PCB-level analog performance qualification (AFE noise floor, PZT matching), EMC, thermal qualification of the MAX14808, and master-MCU application software.

### 1.1 Reference documents

| Ref | Item |
|-----|------|
| R1 | `README.md` — protocol, timing, pin map, fault behaviour |
| R2 | `main.c` — state machine, burst, capture, I2CB ISR |
| R3 | `m24m01e.h` / `m24m01e.c` — EEPROM driver + A-scan record layer |
| R4 | `max14808.h` / `max14808.c` — pulser driver |
| R5 | `i2ca_eeprom.h` / `i2ca_eeprom.c` — I2CA polled transport |
| R6 | `ndt_board.syscfg` — SysConfig peripheral configuration |
| R7 | `28002x_generic_ram_lnk.cmd` — linker command file |
| R8 | ST DS13858 Rev 4 — M24M01E datasheet |

---

## 2. Test environment

### 2.1 Equipment

| Item | Requirement |
|------|-------------|
| Debug probe | XDS110 |
| I2C master | Test Kit , clock stretching tolerant, and single-transaction reads of ≥ 1024 bytes |
| Oscilloscope | ≥ 200 MHz, 4 channels, differential probe for HV pulser outputs |
| Logic analyser | ≥ 8 channels, ≥ 50 MSa/s, I2C decode |
| Bench PSU (Keithley 2601B) | Adjustable rails to inject out-of-tolerance supply conditions |
| Signal source | Function generator to inject a known echo into the AFE |
| Thermal tooling | jumper to pull `THP` low |

### 2.2 Firmware configurations under test

| Config | Description |
|--------|-------------|
| **CFG-RAM** | RAM-only debug build; `_FLASH` **not** defined; `28002x_generic_ram_lnk.cmd` |
| **CFG-FLASH** | Production build; `_FLASH` defined; `.TI.ramfunc` LOAD = FLASH / RUN = RAMLS with copy table; stack ≥ 0x400 |
| **CFG-O0** | CFG-FLASH built at `-O0` — used only for negative timing test **PERF-05** |

Unless a case states otherwise, tests run on **CFG-FLASH**.

### 2.3 Instrumented probe points

| Signal | GPIO | Chip pin |
|--------|------|----------|
| `Analog_EN` | GPIO17 | 40 |
| `Pulser_EN` | GPIO25 | 42 |
| `THP` | GPIO13 | 35 |
| `MCU_LED` | GPIO16 | 39 |
| `MODE0` / `MODE1` | GPIO39 / GPIO42 | 56 / 57 |
| `EEPROM_WC` | GPIO33 | — |
| I2CB SDA / SCL | GPIO2 / GPIO3 | 61 / 60 |
| I2CA SDA / SCL | GPIO26 / GPIO27 | 43 / 44 |
| `DINP_CH1` / `DINN_CH1` | GPIO6 / GPIO14 | — |

### 2.4 Conventions

- **Test IDs:** `AREA-nn` where area is `BLD`, `BOOT`, `PWR`, `I2C`, `BST`, `CAP`, `HDR`, `EEP`, `FLT`, `STA`, `RBT`, `PERF`.
- **Priority:** P1 = safety/blocking, P2 = core function, P3 = robustness/nice-to-have.
- **Type:** ST = static/build, UT = unit (host or debugger), HIL = hardware-in-the-loop, INT = integration.
- All I2C multi-byte values are **little-endian**. Slave address **0x21** (7-bit).
- "Idle" means `g_ndtState == NDT_IDLE` (verify by debugger, or infer from `DATA_VALID` / `EEPROM_BUSY` per R1).

### 2.5 Helper procedures

| Name | Definition |
|------|------------|
| **P-STATUS** | Write `[0x03]`, then read 2 bytes in one transaction → 16-bit status word |
| **P-TAPS** | Write `[0x05]`, then read 10 bytes → five raw 12-bit tap counts |
| **P-HDR** | Write `[0x08]`, then read 16 bytes → record header |
| **P-WAVE(ch)** | Write `[0x04][ch]`, then read **1024 bytes in one continuous read** |
| **P-SCAN** | Write `[0x01]`, wait ≥ 5 ms, poll P-STATUS until `DATA_VALID` set |
| **P-RESET** | Assert `XRSn` / power cycle; wait for `MCU_LED` steady high |

### 2.6 Expected tap thresholds (±5 % of target, 3.3 V / 4095 counts)

| Idx | Tap | Target (mV) | Expected counts | Accept range (counts) |
|-----|-----|-------------|-----------------|-----------------------|
| 0 | `vpp_tap` | 2912 | 3613 | 3433 – 3793 |
| 1 | `12V_tap` | 1714 | 2126 | 2020 – 2232 |
| 2 | `7v_vfd_tap` | 1750 | 2171 | 2063 – 2279 |
| 3 | `5v_pulse_tap` | 1250 | 1551 | 1474 – 1628 |
| 4 | `5v_vfd_tap` | 1250 | 1551 | 1474 – 1628 |

---

## 3. Test cases

### 3.1 Build and static verification (BLD)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| BLD-01 | Clean build, no warnings | Fresh clone, CCS + C2000Ware for F28002x, SysConfig installed | Import project, confirm device = F280025, rebuild all | Build succeeds; zero errors; warnings reviewed and either zero or explicitly waived | P1 | ST |
| BLD-02 | `_FLASH` symbol present in flash target | CFG-FLASH | Inspect Project Properties → Predefined Symbols | `_FLASH` defined; `Device_init()` performs `.TI.ramfunc` copy and sets flash wait-states | P1 | ST |
| BLD-03 | `.TI.ramfunc` LOAD/RUN mapping | CFG-FLASH built | Inspect `.map` for `NDT_captureRecord` | Symbol load address in FLASH, run address in RAMLS; copy table emitted | P1 | ST |
| BLD-04 | Stack size ≥ 0x400 words | CFG-FLASH built | Inspect linker `--stack_size` and `.map` | Stack ≥ 0x400 words (EEPROM path peaks ≈ 0x250) | P1 | ST |
| BLD-05 | RAM budget fits 24 KB | CFG-FLASH built | Sum `.bss`/`.data`/`.stack`/`.TI.ramfunc` run sizes from `.map` | Total ≤ 24 KB with ≥ 10 % headroom; `g_waveBuf` = 4096 words (8 KB) | P1 | ST |
| BLD-06 | Optimisation level is `-O2` | CFG-FLASH | Inspect compiler settings | `-O2` set (capture loop assumes ≤ 160 cycles/sweep) | P1 | ST |
| BLD-07 | EEPROM geometry constants consistent | Source available | Verify `WAVE_SAMPLES == M24M01E_ASCAN_SAMPLES` and `WAVE_CHANNELS == M24M01E_ASCAN_CHANNELS`; verify `SAMPLES × CHANNELS × 2` is a multiple of 256 | 512 / 8 / 8192 B = 32 pages exactly; compile-time assert (or manual check) passes | P1 | ST |
| BLD-08 | Slot arithmetic | Source available | Compute `M24M01E_ASCAN_SLOT_BYTES` and `_SLOT_COUNT` | Slot = 33 pages = 8448 B; slot count = 15 | P2 | ST |
| BLD-09 | GPIO burst masks match pin defines | Source available | Derive `GPIOA_A_MASK`, `GPIOA_B_MASK`, `GPIOB_A_MASK`, `GPIOB_B_MASK` from the eight `PIN_DINP_*` / `PIN_DINN_*` defines by hand or script; compare with literals | Derived masks equal the coded literals exactly; no bit belongs to both A and B masks | P1 | ST |
| BLD-10 | Header stream is exactly 16 bytes | Source available | Sum field widths in `NDT_packHdrStream()` | `id(2)+seq(4)+burst(2)+samples(2)+period(2)+start_delay(4) = 16`; `g_hdrStream[16]` fully written, no gaps | P2 | ST |
| BLD-11 | Sample-period constants | Source available | Check `SAMPLE_PERIOD_EPWM_TICKS = 80`, `EPWMCLK_TICK_NS = 20`, ePWM period register = ticks − 1 | `SAMPLE_PERIOD_NS = 1600`; `TBPRD = 79` | P2 | ST |
| BLD-12 | Static analysis clean | Toolchain with MISRA/cppcheck or clang-tidy via `.clangd` | Run analyser on `main.c`, `m24m01e.c`, `max14808.c`, `i2ca_eeprom.c` | No new high-severity findings; all `volatile`/ISR-shared variables correctly qualified | P3 | ST |

### 3.2 Boot and initialisation (BOOT)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| BOOT-01 | Cold boot to idle | Board powered, EEPROM fitted | P-RESET; observe `MCU_LED` and read P-STATUS | `MCU_LED` goes steady HIGH after init; status word = 0x0000 (no faults, `DATA_VALID` clear, `EEPROM_FAIL` clear) | P1 | HIL |
| BOOT-02 | Analog front end enabled before pulser init | Board powered | Scope `Analog_EN` and `MODE0`/`MODE1` from reset | `Analog_EN` rises, ≥ 500 µs delay, then AFE gain pins high, then MODE pins settle to MODE0=1 / MODE1=0 | P2 | HIL |
| BOOT-03 | Pulser in octal three-level mode at boot | Board powered | Read `MODE0`, `MODE1` after boot | MODE0 = 1, MODE1 = 0 | P1 | HIL |
| BOOT-04 | HV disabled at boot | Board powered | Scope `Pulser_EN` from reset through 1 s idle | `Pulser_EN` remains LOW; HV rail not enabled | P1 | HIL |
| BOOT-05 | All DINP/DINN low at boot | Board powered | Probe all 16 drive pins after init | All LOW (clamped state); no pulser output activity | P1 | HIL |
| BOOT-06 | EEPROM write-control asserted | Board powered | Probe `EEPROM_WC` | Driven LOW (writes enabled) before first EEPROM access | P2 | HIL |
| BOOT-07 | EEPROM probe success | EEPROM fitted and healthy | P-RESET, read P-STATUS | `EEPROM_FAIL` (0x0200) **clear** | P2 | HIL |
| BOOT-08 | Missing EEPROM is non-fatal | EEPROM removed / I2CA SDA held high | P-RESET, read P-STATUS, then run P-SCAN | `EEPROM_FAIL` set at boot; board still reaches idle; scan completes and `DATA_VALID` sets | P1 | HIL |
| BOOT-09 | Pulser init failure halts safely | Force `max14808_init()` to return non-OK (debugger patch or disconnected mode pins if detectable) | P-RESET | `ESTOP0` reached; no HV enable, no pulsing. Firmware must not proceed to a state where HV can be asserted | P1 | UT |
| BOOT-10 | ePWM1 frozen at boot | Board powered | Halt after `NDT_initTimers()`; inspect `EPWM1.TBCTL` counter mode and ADC result registers | Counter mode = STOP_FREEZE; no SOCA pulses; no spurious conversions | P2 | UT |
| BOOT-11 | CPU Timer 0 free-running | Board powered | Read `CPUTIMER0` count twice, 1 ms apart | Count decrements by ≈ 100 000 ticks (10 ns/tick), free-run enabled | P2 | UT |
| BOOT-12 | Default burst cycle count | Board powered | P-RESET, then P-SCAN, then P-HDR | `burst_cycles` field = 5 | P2 | HIL |
| BOOT-13 | I2CB slave address | Board powered | Scan the I2C bus from the master | Board ACKs address 0x21 only | P1 | HIL |
| BOOT-14 | I2CA master clock rate | Board powered | Logic-analyse I2CA during EEPROM probe | SCL ≈ 400 kHz, valid timing per M24M01E | P2 | HIL |
| BOOT-15 | Warm reset from fault | Board latched in thermal fault (see FLT-01) | P-RESET | Fault flags cleared; board returns to normal idle | P1 | HIL |

### 3.3 Supply-rail supervision (PWR)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| PWR-01 | All rails nominal | All supplies in tolerance | P-TAPS, then P-STATUS | All five tap counts inside the §2.6 accept ranges; status bits 0x001F all set; `VOLTAGE_FAULT` clear | P1 | HIL |
| PWR-02 | Tap readings map to correct SOCs | Rails nominal | Perturb one rail at a time within tolerance; read P-TAPS after each | Only the corresponding tap index changes; ordering is `vpp, 12V, 7V_VFD, 5V_PULSE, 5V_VFD` (SOC4–8) | P1 | HIL |
| PWR-03 | Low-rail rejection, each rail | Rails nominal | For each rail *i*: drop it to ~85 % of target, write `[0x01]`, then P-STATUS | Corresponding `*_OK` bit clear; `VOLTAGE_FAULT` (0x0080) set; `DATA_VALID` clear; **no pulsing observed on DIN pins**; `Pulser_EN` stays LOW | P1 | HIL |
| PWR-04 | High-rail rejection, each rail | Rails nominal | For each rail *i*: raise it to ~115 % of target, write `[0x01]`, P-STATUS | Same as PWR-03 | P1 | HIL |
| PWR-05 | Tolerance boundary — just inside | Adjustable rail | Set `vpp_tap` to give 3440 counts (inside 3433–3793); write `[0x01]` | `VPP_OK` set; scan proceeds | P2 | HIL |
| PWR-06 | Tolerance boundary — just outside | Adjustable rail | Set `vpp_tap` to give 3420 counts; write `[0x01]` | `VPP_OK` clear; scan blocked; `VOLTAGE_FAULT` set | P2 | HIL |
| PWR-07 | Recovery after voltage fault | Board in `VOLTAGE_FAULT` from PWR-03 | Restore rail, write `[0x01]` again, poll P-STATUS | Scan proceeds; `VOLTAGE_FAULT` cleared at the start of the new FIRE; `DATA_VALID` set on completion | P1 | HIL |
| PWR-08 | Multiple simultaneous rail failures | Rails nominal | Drop two rails together, write `[0x01]`, P-STATUS | Both `*_OK` bits clear; single `VOLTAGE_FAULT`; scan blocked | P2 | HIL |
| PWR-09 | Rails measured **before** HV enable | Rails nominal | Instrument: write `[0x01]`, capture `Pulser_EN` and I2CA/ADC activity on scope with a debugger breakpoint at `NDT_fireAndCapture` entry | Tap conversion completes before `Pulser_EN` rises; a blocked scan never raises `Pulser_EN` | P1 | HIL |
| PWR-10 | Tap ADC poll cannot hang the board | Debugger | Force `ADCA INT2` never to assert (mask SOC4–8 trigger in a patched build) | **Known risk:** `NDT_checkSupplies()` busy-waits with no timeout → firmware hangs. Test documents actual behaviour; expected fix is a bounded wait with `VOLTAGE_FAULT` on timeout | P2 | UT |
| PWR-11 | Tap values persist for later read | Scan just blocked by a bad rail | P-TAPS immediately after | Returns the counts measured during the blocked attempt (diagnostic value preserved) | P3 | HIL |

### 3.4 I2CB slave protocol (I2C)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| I2C-01 | Status stream length and content | Idle, rails nominal | P-STATUS | Exactly 2 bytes, little-endian; matches `g_statusFlags` read via debugger | P1 | HIL |
| I2C-02 | Taps stream length | Idle | P-TAPS | Exactly 10 bytes = 5 × uint16 LE; matches `g_voltBuf` | P1 | HIL |
| I2C-03 | Header stream length | After a scan | P-HDR | Exactly 16 bytes in the documented field order | P1 | HIL |
| I2C-04 | Waveform stream length | `DATA_VALID` set | P-WAVE(0) | Exactly 1024 bytes = 512 × uint16 LE; matches `g_waveBuf[0][]` via debugger | P1 | HIL |
| I2C-05 | Waveform channel select, all 8 | `DATA_VALID` set, distinct data per channel (inject different signals or patch buffer) | P-WAVE(0..7) in turn | Each read returns that channel's data; row order matches the ADC channel map in R1 | P1 | HIL |
| I2C-06 | Channel index masking | `DATA_VALID` set | Write `[0x04][0x08]`, read 1024 B; repeat with `[0x04][0xFF]` | Index masked with 0x07 → 0x08 behaves as ch 0, 0xFF as ch 7. No out-of-bounds access | P2 | HIL |
| I2C-07 | Stream pointer resets on new transaction | `DATA_VALID` set | Write `[0x04][0]`; read 100 bytes (STOP); read another 100 bytes in a new transaction | **Documented limitation:** the second read restarts at byte 0, because both `ADDR_SLAVE` and `STOP` reset `g_i2cTxPtr`. Chunked reads are not supported — record as an expected behaviour and a master-side constraint, or raise as a defect if paged reads are required | P1 | HIL |
| I2C-08 | Over-read wraps, does not overrun | `DATA_VALID` set | Write `[0x03]`, read 6 bytes in one transaction | Bytes repeat the 2-byte status word (pointer wraps at `len`); no invalid memory read, no lockup | P2 | HIL |
| I2C-09 | Read without prior select | Fresh reset | Read 2 bytes with no preceding write | Default stream = STATUS; returns the status word | P2 | HIL |
| I2C-10 | Set burst cycles — valid | Idle | Write `[0x02][7]`, run P-SCAN, P-HDR | `burst_cycles` = 7; scope shows 7 burst cycles | P1 | HIL |
| I2C-11 | Burst clamp low | Idle | Write `[0x02][0]`, `[0x02][1]`, `[0x02][4]`; after each, P-SCAN + P-HDR | `burst_cycles` clamped to 5 in every case | P1 | HIL |
| I2C-12 | Burst clamp high | Idle | Write `[0x02][11]`, `[0x02][255]`; after each, P-SCAN + P-HDR | `burst_cycles` clamped to 10 | P1 | HIL |
| I2C-13 | Burst setting persists across scans | Idle | Write `[0x02][8]`; run three scans; P-HDR after each | `burst_cycles` = 8 for all three | P2 | HIL |
| I2C-14 | Unknown command ignored | Idle | Write `[0x00]`, `[0x09]`, `[0x7F]`, `[0xFF]` | No state change; board stays idle; subsequent valid commands still work | P2 | HIL |
| I2C-15 | Truncated multi-byte command | Idle | Write `[0x02]` then STOP (no parameter); then write `[0x03]` and read 2 bytes | Parser resets on STOP; `g_burstCycles` unchanged; status read works normally | P1 | HIL |
| I2C-16 | Truncated 3-byte command | Idle | Write `[0x06][0x34]` then STOP; P-STATUS | No EEPROM operation triggered; `EEPROM_BUSY` never sets; parser reset | P1 | HIL |
| I2C-17 | Repeated-start read after write | Idle | Write `[0x03]` + repeated START + read 2 bytes (no intervening STOP) | Status word returned correctly; `ADDR_SLAVE` reset of the TX pointer does not corrupt the response | P1 | HIL |
| I2C-18 | Back-to-back commands | Idle | Issue `[0x03]`+read, `[0x05]`+read, `[0x08]`+read, `[0x04][3]`+read 1024, in rapid succession | Every response has the correct length and content; no cross-talk between streams | P2 | HIL |
| I2C-19 | Command during busy state ignored | EEPROM save in progress (`EEPROM_BUSY` set) | Write `[0x01]` | Command silently ignored (state ≠ IDLE); no scan starts; save completes normally | P1 | HIL |
| I2C-20 | Slave serviceable during a scan | Idle | Start a scan, then immediately P-STATUS repeatedly during the ~0.9 ms scan | Status reads are answered throughout; no NACK, no bus hang | P1 | HIL |
| I2C-21 | Bus speed range | Idle | Repeat I2C-01 and I2C-04 at 100 kHz and 400 kHz | Both pass at both speeds | P2 | HIL |
| I2C-22 | Clock stretching tolerated | Idle | Master with stretch detection; run P-WAVE(0) while a scan is running | Any stretching is bounded and the transfer completes; no timeout at the master | P2 | HIL |
| I2C-23 | Bus recovery after abort | Idle | Abort a read mid-transaction (STOP early); issue a fresh P-STATUS | Board recovers; correct 2-byte status returned | P1 | HIL |
| I2C-24 | Wrong slave address ignored | Idle | Address 0x20 and 0x22 | No ACK; no state change | P2 | HIL |
| I2C-25 | LED toggles on scan trigger | Idle | Write `[0x01]` while scoping `MCU_LED` | LED pin toggles once per accepted `0x01`; does **not** toggle when the command is ignored (non-idle) | P3 | HIL |

### 3.5 Burst generation (BST)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| BST-01 | Burst frequency | Rails nominal | Scope `DINP_CH1`; run P-SCAN; measure period | 149.9 kHz ± 0.2 % (333/334-tick alternating half-periods) | P1 | HIL |
| BST-02 | Half-period alternation | Rails nominal | Measure consecutive half-periods on `DINP_CH1` | Alternating 3.33 µs / 3.34 µs (±50 ns); full cycle 6.67 µs | P2 | HIL |
| BST-03 | Cycle count = 5 (default) | Fresh reset | P-SCAN; count cycles on scope | Exactly 5 complete bipolar cycles | P1 | HIL |
| BST-04 | Cycle count = 10 (max) | Write `[0x02][10]` | P-SCAN; count cycles | Exactly 10 cycles; burst length ≈ 66.7 µs | P1 | HIL |
| BST-05 | Bipolar drive, no shoot-through | Rails nominal | Scope `DINP_CH1` and `DINN_CH1` simultaneously at 1 ns/div resolution across a transition | Clear-then-set order gives a brief both-LOW dead gap; **DINP and DINN are never both HIGH** on any transition | P1 | HIL |
| BST-06 | All 8 channels fire simultaneously | Rails nominal | Scope 4 channels at a time (two runs) covering all 8 DINP pins | Rising edges aligned within one SYSCLK cycle (10 ns); same for the GPIOB-resident channels | P1 | HIL |
| BST-07 | Channels return to clamped state | Rails nominal | Scope all DINP/DINN after burst end | All pins LOW within one half-period of the final cycle; stay LOW through the capture window | P1 | HIL |
| BST-08 | No drift across a 10-cycle burst | Write `[0x02][10]` | Measure time from first rising edge to last falling edge | Total = 10 × 6.67 µs ± 100 ns (drift-free `burst_wait` reference advance) | P2 | HIL |
| BST-09 | Timer wrap during burst | Debugger | Preset CPU Timer 0 close to 0 so it wraps mid-burst; run a scan | Unsigned tick arithmetic handles the wrap; burst timing unchanged; no stall | P2 | UT |
| BST-10 | HV settle before burst | Rails nominal | Scope `Pulser_EN` and `DINP_CH1` | ≥ 500 µs between `Pulser_EN` rising and the first burst edge | P1 | HIL |
| BST-11 | T/R switch dead time | Rails nominal | Scope burst end vs. first ADC activity / `start_delay_ns` from P-HDR | ~12 µs receive-mode dead time observed; matches header delay accounting | P2 | HIL |
| BST-12 | HV off after scan | Rails nominal | Scope `Pulser_EN` across a full scan | Falls LOW once capture completes and the state machine reaches `DATA_READY`; stays LOW in idle | P1 | HIL |
| BST-13 | Pulser mode restored after scan | Rails nominal | Read MODE0/MODE1 after a scan completes | Restored to octal three-level (MODE0=1, MODE1=0) | P2 | HIL |
| BST-14 | No pulsing when scan blocked | One rail out of tolerance | Write `[0x01]`; scope all DIN pins for 10 ms | No edges on any drive pin; `Pulser_EN` never rises | P1 | HIL |

### 3.6 Acquisition and capture (CAP)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| CAP-01 | Sample period | Rails nominal | Scope an ADC-synchronous test point or use ePWM1 SOCA on a spare pin; run P-SCAN | SOCA every 1.600 µs ± 20 ns | P1 | HIL |
| CAP-02 | Record length | Rails nominal | Count SOCA pulses per scan | Exactly 512 per channel; capture window 819.2 µs ± 1 µs | P1 | HIL |
| CAP-03 | No dropped or duplicated samples | Inject a known ramp/sine into one AFE input | P-SCAN, then P-WAVE for that channel; reconstruct | Reconstructed waveform is monotonic in phase with no repeated or missing sample; frequency matches the injected signal | P1 | HIL |
| CAP-04 | Channel-to-row mapping | Inject a distinct amplitude on each of the 8 AFE inputs | P-SCAN, then P-WAVE(0..7) | Row *n* carries the signal from the ADC input listed in R1's channel map; no channel swap | P1 | HIL |
| CAP-05 | Simultaneity of ADCA/ADCC pairs | Inject the same signal on one ADCA and one ADCC input via a splitter | P-SCAN; cross-correlate the two rows | Sample-time skew < 200 ns; no systematic one-sample offset | P2 | HIL |
| CAP-06 | ADC sweep fits the pacer period | Rails nominal | Run 100 consecutive scans; after each, check ADC overflow/`OVF` status bits (debugger) | No SOC overflow flags set; sweep ≈ 1.45 µs < 1.6 µs | P1 | UT |
| CAP-07 | ADC full-scale and zero | Drive an AFE input to 0 V and to 3.3 V (within AFE limits) | P-SCAN; inspect that row | Counts near 0 and near 4095; only bits 11:0 significant | P2 | HIL |
| CAP-08 | Pacer frozen outside capture | Idle after a scan | Halt via debugger; check ePWM1 counter mode and re-read ADC results after 10 ms | Counter mode = STOP_FREEZE; ADC result registers unchanged (no free-running conversions) | P1 | UT |
| CAP-09 | Repeatability | Static fixture (fixed target, no motion) | Run 20 scans; compare row 0 across runs | Sample-by-sample deviation within ADC noise budget; start-of-record alignment stable within one sample | P2 | HIL |
| CAP-10 | Capture runs from RAM | CFG-FLASH | Breakpoint inside `NDT_captureRecord`; read PC | PC lies in the RAMLS run address range, not FLASH | P1 | UT |
| CAP-11 | Buffer fully written | Debugger | Fill `g_waveBuf` with 0xA5A5 pattern; run one scan; inspect the whole buffer | No cell retains the sentinel — all 8 × 512 entries overwritten | P1 | UT |
| CAP-12 | Capture with 10-cycle burst | `[0x02][10]` | P-SCAN; P-HDR; P-WAVE(0) | Record still 512 samples; `start_delay_ns` grows by ≈ 33 µs relative to the 5-cycle case | P2 | HIL |
| CAP-13 | Scan duration | Rails nominal | Time from `[0x01]` STOP to `DATA_VALID` set | ≈ 1.4 ms total (500 µs HV settle + burst + ~12 µs + 819.2 µs); capture portion ≈ 0.9 ms. Record the measured value against the R1 figure | P2 | HIL |

### 3.7 Record header and time axis (HDR)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| HDR-01 | Field layout and endianness | After a scan | P-HDR; decode | `id` @0–1, `seq` @2–5, `burst_cycles` @6–7, `samples_per_ch` @8–9, `sample_period_ns` @10–11, `start_delay_ns` @12–15; all little-endian | P1 | HIL |
| HDR-02 | Fixed geometry fields | After a scan | P-HDR | `samples_per_ch` = 512; `sample_period_ns` = 1600 | P1 | HIL |
| HDR-03 | `id` and `seq` after a fresh capture | Fresh scan, not yet saved | P-HDR | `id` = 0, `seq` = 0 (both assigned only on save) | P2 | HIL |
| HDR-04 | `burst_cycles` reflects the setting | `[0x02][9]` then scan | P-HDR | Field = 9 | P2 | HIL |
| HDR-05 | `start_delay_ns` plausibility, 5 cycles | Default burst | P-SCAN, P-HDR | ≈ 45 000–50 000 ns (33.4 µs burst + ~12 µs dead time); consistent run to run within ±1 µs | P1 | HIL |
| HDR-06 | `start_delay_ns` scales with burst length | — | Scan at 5 then at 10 cycles; compare | Difference ≈ 33.4 µs (5 extra cycles) | P2 | HIL |
| HDR-07 | `start_delay_ns` matches scope | Scope on burst start and first SOCA | Measure fire-to-first-sample interval; compare to header | Agreement within ±0.5 µs | P1 | HIL |
| HDR-08 | Time-axis reconstruction | Known echo at a known distance/time | Compute `t(i) = start_delay_ns + i × 1600` for the echo peak index | Reconstructed arrival time matches the scope-measured echo within ±2 µs | P1 | HIL |
| HDR-09 | Header updated after EEPROM load | Record saved with id 0x1234 at some earlier `seq` | Reset, load id 0x1234 (`[0x07]`), P-HDR | Header reflects the **stored** record (its id, seq, burst_cycles, start_delay), not the last live capture | P1 | HIL |
| HDR-10 | Header updated after save | Fresh scan | Save with id 0x00AA, then P-HDR | `id` = 0x00AA; `seq` = driver-assigned value (max existing + 1) | P2 | HIL |

### 3.8 EEPROM storage (EEP)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| EEP-01 | Save/load round trip | Fresh scan, `DATA_VALID` set, EEPROM healthy | Write `[0x06][0x01][0x00]`; poll until `EEPROM_BUSY` clears; reset the board; write `[0x07][0x01][0x00]`; poll; P-WAVE(0..7) | All 8 × 512 samples identical to the pre-save capture; `DATA_VALID` set after load; `EEPROM_FAIL` clear | P1 | HIL |
| EEP-02 | `EEPROM_BUSY` asserted during save | Fresh scan | Issue `[0x06][id]`; poll P-STATUS every 10 ms | `EEPROM_BUSY` set within one poll and clears after ≈ 0.3 s; timing recorded | P1 | HIL |
| EEP-03 | Save duration | Fresh scan | Time from command STOP to `EEPROM_BUSY` clear | ≈ 0.3 s at 400 kHz (33 pages); ≤ 0.5 s upper bound | P2 | HIL |
| EEP-04 | Save with no valid data | Fresh reset, no scan run (`DATA_VALID` clear) | Write `[0x06][0x05][0x00]`; poll P-STATUS | Operation rejected; `EEPROM_FAIL` set; `EEPROM_BUSY` clears; board returns to idle; EEPROM contents unchanged | P1 | HIL |
| EEP-05 | Load non-existent ID | EEPROM with known contents | Write `[0x07][0xEE][0xEE]` for an unused ID; poll | `EEPROM_FAIL` set; `DATA_VALID` not set by the load; `g_waveBuf` not silently populated with garbage | P1 | HIL |
| EEP-06 | Overwrite existing ID | Record id 0x0002 already stored | Capture new data; save with id 0x0002; load id 0x0002 | Same slot reused; loaded data matches the **new** capture; slot count unchanged | P2 | HIL |
| EEP-07 | Fill all 15 slots | Erased/blank EEPROM | Save 15 records with ids 1..15, capturing between each | All 15 saves succeed; each id loads back correctly | P2 | HIL |
| EEP-08 | Oldest-record eviction | 15 slots full (EEP-07) | Save a 16th record with id 16; then attempt to load id 1 | Save succeeds; id 1 (lowest `seq`) is evicted → load of id 1 returns not-found with `EEPROM_FAIL`; ids 2..16 still load | P1 | HIL |
| EEP-09 | Checksum detects corruption | Record id 0x0003 stored | Externally corrupt a data byte in that slot (bench I2C master or debugger-driven write); load id 0x0003 | `EEPROM_FAIL` set (corrupt status); `DATA_VALID` **not** set — the master is not told corrupt data is valid | P1 | HIL |
| EEP-10 | Header-only integrity | Records stored | Reset; load a record; P-HDR | Header fields survive the round trip exactly (id, seq, burst_cycles, sample_period_ns, start_delay_ns, samples_per_ch) | P1 | HIL |
| EEP-11 | Page-boundary handling | — | Save a record whose data contains a distinctive per-page marker (patch `g_waveBuf` via debugger); read back the raw EEPROM with a bench master | Data is page-aligned, 32 data pages after the header page, no page-wrap corruption at any 256-byte boundary | P1 | HIL |
| EEP-12 | Write-cycle ACK polling | — | Logic-analyse I2CA during a save | ACK polling between pages; no fixed over-long delays; each page commits within ~5 ms | P2 | HIL |
| EEP-13 | EEPROM removed mid-operation | Save in progress | Physically interrupt SDA (or hold it high) during a save | Operation aborts within the poll-try budget; `EEPROM_FAIL` set; `EEPROM_BUSY` clears; board returns to idle and remains scan-capable | P1 | HIL |
| EEP-14 | Power loss during save | Save in progress | Cut power mid-save; restore | Board boots normally; the partially written slot either loads correctly or is reported not-found/corrupt — never returned as a valid record | P1 | HIL |
| EEP-15 | Scans still work with no EEPROM | EEPROM removed | Run 10 scans; P-WAVE after each | All scans complete; `DATA_VALID` sets; only `[0x06]`/`[0x07]` fail with `EEPROM_FAIL` | P1 | HIL |
| EEP-16 | ID 0x0000 and 0xFFFF | EEPROM healthy | Save and load with id 0x0000; repeat with id 0xFFFF | Both round-trip correctly, or the firmware documents them as reserved and rejects them consistently | P3 | HIL |
| EEP-17 | Write protection respected | — | Set SWP write protection on the part; attempt a save | Save fails with `EEPROM_FAIL`; no silent success reported to the master | P3 | HIL |
| EEP-18 | Stack headroom on the EEPROM path | CFG-FLASH, stack painted with a known pattern | Run a full save then a full load; inspect the stack watermark | Peak usage ≤ 0x250 words; ≥ 0x100 words of unused stack remain | P1 | UT |

### 3.9 Fault handling (FLT)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| FLT-01 | Thermal fault detection | Board idle | Pull `THP` LOW (jumper or forced over-temperature) | `THERMAL_FAULT` (0x0040) set; `Pulser_EN` driven LOW; pulser set to TX-disable; `MCU_LED` blinks ~5 Hz | P1 | HIL |
| FLT-02 | Thermal fault is latched | FLT-01 active | Release `THP` HIGH; wait 2 s; read P-STATUS; write `[0x01]` | Fault stays latched; `THERMAL_FAULT` remains set; scan command ignored; LED keeps blinking | P1 | HIL |
| FLT-03 | Thermal fault cleared only by reset | FLT-02 | P-RESET with `THP` HIGH | Flags cleared; LED steady; scans work again | P1 | HIL |
| FLT-04 | No HV in thermal fault | FLT-01 active | Scope `Pulser_EN` and DIN pins for 10 s | `Pulser_EN` stays LOW; no drive activity at any time | P1 | HIL |
| FLT-05 | Thermal fault raised during a scan | Board idle | Assert `THP` LOW ~200 µs after issuing `[0x01]` (trigger a FET from the burst edge) | **Known gap:** `THP` is polled only in `NDT_IDLE`, so the in-flight scan completes before the fault is entered. Verify the fault *is* entered on the following idle pass and record the worst-case detection latency (≈ one scan, ~1.4 ms) as an accepted risk or a defect | P1 | HIL |
| FLT-06 | Voltage fault does not latch | Rails nominal | Drop a rail, `[0x01]`, restore rail, `[0x01]` | First blocked with `VOLTAGE_FAULT`; second succeeds; `VOLTAGE_FAULT` cleared at the start of the retry | P1 | HIL |
| FLT-07 | Fault priority | `THP` LOW and a rail out of tolerance simultaneously | Write `[0x01]`, read P-STATUS | Thermal fault takes precedence (HV locked); no pulsing under either condition | P1 | HIL |
| FLT-08 | EEPROM fault does not block scanning | `EEPROM_FAIL` set | Run 5 scans | All succeed; `EEPROM_FAIL` remains set but is independent of `DATA_VALID` | P2 | HIL |
| FLT-09 | `EEPROM_FAIL` cleared at the start of each operation | `EEPROM_FAIL` set from a previous failure | Fit a healthy EEPROM (no reset possible for probe, so use a subsequent valid save) | Flag cleared when a new operation starts and only re-set if that operation fails | P2 | HIL |
| FLT-10 | Status bits are mutually consistent | Various | Sample the status word in each fault state | No impossible combinations (e.g. `DATA_VALID` together with `VOLTAGE_FAULT` from the same attempt); `EEPROM_BUSY` never stuck set | P2 | HIL |

### 3.10 State machine and concurrency (STA)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| STA-01 | Normal state sequence | Idle | Breakpoint/trace `g_ndtState` through one scan | IDLE → FIRE → DATA_READY → IDLE | P1 | UT |
| STA-02 | Blocked-scan sequence | Rail out of tolerance | Trace `g_ndtState` | IDLE → FIRE → VOLTAGE_FAULT → IDLE | P1 | UT |
| STA-03 | EEPROM sequence | `DATA_VALID` set | Trace during a save | IDLE → EEPROM_OP → IDLE | P1 | UT |
| STA-04 | `0x01` ignored while not idle | Scan in progress | Send a second `[0x01]` mid-scan | Second command ignored; no re-entry; first scan completes cleanly | P1 | HIL |
| STA-05 | `0x06`/`0x07` ignored while not idle | Scan in progress | Send `[0x06][id]` mid-scan | Ignored; no EEPROM operation queued; `g_eeOp` stays `EE_NONE` | P1 | HIL |
| STA-06 | Rapid trigger burst | Idle | Send `[0x01]` 50 times as fast as the bus allows | Each accepted trigger completes fully before the next is accepted; no lost or partial records; no state corruption | P1 | HIL |
| STA-07 | Read during capture (tearing) | Idle | Start P-WAVE(0) (long 1024-byte read) and issue `[0x01]` so a capture overwrites `g_waveBuf` mid-read | **Known risk:** no double-buffering — the master may receive a torn record. Verify the actual behaviour and confirm the documented master protocol (poll `DATA_VALID`, do not overlap) prevents it; raise as a defect if a guard is required | P1 | HIL |
| STA-08 | Command parser reset on address match | Idle | Send `[0x06][0x01]` (incomplete) then, without STOP, a repeated START and a read | Parser state resets at address match; no stale parameter is later applied as a save | P1 | HIL |
| STA-09 | `DATA_VALID` lifecycle | Fresh reset | Read status after: reset, after `[0x01]` accepted, mid-scan, after completion, after a blocked scan | Clear at reset; cleared at FIRE entry; set only in DATA_READY; cleared and left clear on a blocked scan | P1 | HIL |
| STA-10 | Soak / endurance | Healthy board | 10 000 scans over ≥ 4 h, with a save every 100th scan and a status read every scan | No hangs, no state-machine lockups, no monotonic drift in `start_delay_ns`, no unexpected fault flags, EEPROM eviction behaves as specified | P1 | HIL |
| STA-11 | ISR execution time | — | Instrument a spare GPIO set/clear around `INT_myI2CB_ISR` | Worst-case ISR duration well inside one I2C byte period at 400 kHz (< 20 µs), including the waveform TX path | P2 | HIL |
| STA-12 | Shared-variable atomicity | Source + debugger | Review 16-bit accesses to `g_statusFlags`, `g_ndtState`, `g_eeOp`, `g_burstCycles` between ISR and main loop | All shared objects are `volatile`; each access is a single 16-bit C28x operation (atomic); no read-modify-write race on `g_statusFlags` between the ISR and the main loop | P1 | ST |

### 3.11 Robustness and negative testing (RBT)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| RBT-01 | Random command fuzzing | Idle | Send 10 000 random 1–4 byte writes (including all opcodes and lengths), interleaved with reads of random lengths 1–2048 | No hang, no ESTOP, no HV assertion without a valid `0x01`, no memory access outside the declared buffers; board still passes BOOT-01 afterwards | P1 | HIL |
| RBT-02 | Read length beyond stream | Idle | Request 4096 bytes after `[0x04][0]` | Data wraps at 1024; no fault; transfer completes | P2 | HIL |
| RBT-03 | Simultaneous I2CA and I2CB activity | Save in progress | Hammer I2CB with status reads during an EEPROM save | Both buses operate correctly; save completes; status reads accurate; no priority inversion or ISR starvation | P1 | HIL |
| RBT-04 | Brown-out during a scan | Scan running | Dip the 3.3 V supply below the BOR threshold mid-capture | Clean reset; board reboots to idle; no HV left enabled and no drive pin left HIGH through the reset | P1 | HIL |
| RBT-05 | Repeated reset stress | — | 500 power-on resets | Boots to idle every time; `EEPROM_FAIL` reflects the true EEPROM presence in every boot | P2 | HIL |
| RBT-06 | I2C bus stuck low | — | Hold I2CB SDA low for 100 ms, then release | Board recovers and responds to the next transaction; watchdog or recovery path documented if not | P2 | HIL |
| RBT-07 | Master ignores `EEPROM_BUSY` | Save in progress | Issue `[0x01]`, `[0x02][7]`, `[0x04][2]`, `[0x03]` during a save | Scan/save/load commands ignored; stream-select commands are accepted (they only touch read state); no corruption of the in-flight save | P2 | HIL |
| RBT-08 | Uninitialised read after reset | Fresh reset, no scan | P-WAVE(0) and P-HDR | Returns zeroed/undefined-but-safe data with `DATA_VALID` clear; no crash. Master is expected to check `DATA_VALID` first | P2 | HIL |

### 3.12 Timing and performance (PERF)

| ID | Title | Precondition | Steps | Expected result | Pri | Type |
|----|-------|--------------|-------|-----------------|-----|------|
| PERF-01 | Per-sweep CPU budget | CFG-FLASH, `-O2` | Instrument the capture loop (GPIO toggle per iteration, or cycle-count in the debugger) | Loop body ≤ 160 SYSCLK cycles per sweep with measurable margin | P1 | UT |
| PERF-02 | Capture jitter | — | Measure SOCA-to-SOCA interval across a full record | Peak-to-peak jitter ≤ 40 ns (hardware-paced, not software-paced) | P1 | HIL |
| PERF-03 | Command latency | Idle | Measure `[0x01]` STOP → `Pulser_EN` rising | ≤ 100 µs (one main-loop pass) | P2 | HIL |
| PERF-04 | Waveform read throughput | `DATA_VALID` set | Time P-WAVE(0) at 400 kHz | ≈ 26 ms for 1024 bytes; no clock stretching beyond the ISR service time | P2 | HIL |
| PERF-05 | Negative: `-O0` build | CFG-O0 | Run CAP-01/CAP-06 | Expected to **fail** the sweep budget (ADC overflow or missed sweeps), confirming the `-O2` requirement is real and not merely advisory | P2 | HIL |
| PERF-06 | Negative: capture running from flash | CFG-FLASH with `.TI.ramfunc` mapping removed | Run CAP-06 | Expected to **fail** or show reduced margin, confirming the RAM-resident requirement | P2 | HIL |

---

## 4. Traceability matrix

| Requirement (source) | Covering test cases |
|----------------------|---------------------|
| 150 kHz bipolar tone-burst on 8 channels (R1 §Scan sequence) | BST-01, BST-02, BST-05, BST-06 |
| Burst length configurable 5–10 cycles (R1 cmd `0x02`) | I2C-10, I2C-11, I2C-12, BST-03, BST-04, HDR-04 |
| 625 kSps/channel, 512 samples, 8 channels | CAP-01, CAP-02, CAP-11, BLD-11 |
| Rails checked before firing; scan blocked if bad | PWR-03, PWR-04, PWR-09, BST-14, STA-02 |
| Status word bit definitions | I2C-01, PWR-01, FLT-01, FLT-06, FLT-10, EEP-02 |
| I2C command set 0x01–0x08 | I2C-01 … I2C-25 |
| Commands ignored unless idle | I2C-19, STA-04, STA-05, RBT-07 |
| Slave serviceable during a scan | I2C-20, I2C-22, STA-11 |
| Implicit time axis via header | HDR-01, HDR-05, HDR-07, HDR-08 |
| EEPROM save/load by 16-bit ID | EEP-01, EEP-06, EEP-10, HDR-09, HDR-10 |
| Slot store: 15 slots, oldest-evicted | BLD-08, EEP-07, EEP-08 |
| Checksum on load | EEP-09 |
| EEPROM failure non-fatal | BOOT-08, EEP-15, FLT-08 |
| Thermal fault latched, HV locked | FLT-01, FLT-02, FLT-03, FLT-04 |
| Voltage fault non-latched | PWR-07, FLT-06 |
| RAM budget / stack ≥ 0x400 | BLD-04, BLD-05, EEP-18 |
| `.TI.ramfunc` placement | BLD-03, CAP-10, PERF-06 |
| `-O2` required | BLD-06, PERF-01, PERF-05 |
| EPWMCLK fixed at SYSCLK/2 | BLD-11, CAP-01 |

---

## 5. Test execution record template

| Field | Value |
|-------|-------|
| Test case ID | |
| Firmware commit SHA | |
| Build config | CFG-RAM / CFG-FLASH / CFG-O0 |
| Board serial | |
| Date / tester | |
| Equipment IDs & cal dates | |
| Result | Pass / Fail / Blocked / N-A |
| Measured values | |
| Evidence (scope capture, log, `.map` excerpt) | |
| Defect reference | |
| Notes | |

**Exit criteria:** all P1 cases pass; P2 failures triaged with an agreed disposition; every item in §5 closed as either an accepted, documented limitation or a tracked defect.
