# NDT Auxiliary Board — Unit Test Specification

**Project:** `NDT_AUXILIARY` — F280025 NDT Auxiliary Board Firmware
**Document version:** 2.0
**Status:** Draft — for review
**Supersedes:** v1.0 (mixed unit / HIL / integration specification)

---

## 1. Scope

This document specifies **unit tests only**. Every case here runs on a host compiler
against a module compiled in isolation, with its dependencies replaced by fakes. No
board, no debugger, no instruments, no I2C bus.

### 1.1 In scope

| Module | Unit-testable because |
|---|---|
| `ndt_sm.c` | Reaches the board only through `ndt_sm_hooks_t` function pointers. Explicitly designed to build without driverlib. |
| `ndt_store.c` | Reaches the EEPROM only through `m24m01e_*` calls, which are themselves callback-driven. |
| `m24m01e.c` | Reaches the bus only through `m24m01e_io_t` callbacks. |
| `max14808.c` | Reaches the pins only through `gpio_write` / `gpio_read` / `delay_us` callbacks. |
| `ndt_i2cb.c` | Needs a small `driverlib.h` / `device.h` shim (§3.5). Worth the shim — the RX parser and TX descriptor are where protocol bugs live. |
| `ndt_config.h`, `ndt_store.h` | Compile-time geometry assertions. |

### 1.2 Out of scope

Deliberately excluded, and **not** simply "not yet written":

- `main.c` register paths — `NDT_burst()`, `NDT_captureRecord()`, `NDT_checkSupplies()`,
  `NDT_initTimers()`. These are ADC/ePWM/GPIO register sequences; a fake proves only
  that the fake was called.
- `i2ca_eeprom.c` — direct I2CA register access, no injection seam.
- Everything timing-, analog- or bus-observable: burst frequency, sample period, ADC
  mapping, rail tolerances, I2C electrical behaviour, EEPROM write-cycle duration,
  soak and endurance.
- Build/link properties (`.TI.ramfunc` placement, stack size, `-O2`, `.map` budget).

Those belong in a separate HIL/integration specification. Where a unit test can only
*partially* cover a requirement, §7 says so rather than implying full coverage.

> **Note on v1.0.** The previous document was written against the pre-refactor
> firmware and references symbols that no longer exist (`g_ndtState`, `g_eeOp`,
> `EE_NONE`, `NDT_packHdrStream()`, `M24M01E_ASCAN_SAMPLES`), and asserts behaviour the
> refactor changed — chiefly that `0x01`/`0x06`/`0x07` are *ignored* when the board is
> busy. They are now **queued**. Cases I2C-19, STA-04, STA-05 and RBT-07 of v1.0 would
> fail against current firmware for that reason, and are replaced here by
> UT-SM-10…UT-SM-13.

---

## 2. Test environment

### 2.1 Toolchain

| Item | Requirement |
|---|---|
| Host compiler | GCC ≥ 9 or Clang ≥ 12, C99, `-Wall -Wextra -Werror` |
| Test framework | Unity, or any xUnit-style C framework. Cases are framework-neutral. |
| Coverage | `gcov` / `llvm-cov`, reported per module |
| Sanitizers | `-fsanitize=address,undefined` on every run |
| Static analysis | `cppcheck` and/or `clang-tidy`, using the existing `.clangd` |

Suggested layout — no source file moves:

```
test/
  fakes/        fake_hooks.c/h  fake_eeprom.c/h  fake_gpio.c/h  driverlib_stub.h
  test_ndt_sm.c  test_ndt_store.c  test_m24m01e.c  test_max14808.c  test_ndt_i2cb.c
  Makefile
```

### 2.2 The C28x / host type hazard

On the C2000 target `uint8_t` is a **16-bit** container and `CHAR_BIT == 16`; on the
host it is 8 bits. Both `ndt_store.c` and `m24m01e.c` mask every byte with `0xFF` on
the way in and out specifically to be correct under both.

Two consequences the suite must respect:

1. Tests must never assert on `sizeof()`, on struct padding, or on raw byte counts
   derived from `sizeof`. Assert on **values** and on **field positions** instead.
2. A host pass does **not** prove the C28x build. UT-CFG-05 pins the masking
   behaviour, and every fake stores bytes in a `uint16_t` array so that an unmasked
   value ≥ 0x100 is visible as a failure rather than being silently truncated by the
   host's 8-bit `uint8_t`.

### 2.3 Fakes

| Fake | Replaces | Capability |
|---|---|---|
| `fake_hooks` | `ndt_sm_hooks_t` | Records an ordered call log; per-hook programmable return values; scriptable `millis()` (test advances it explicitly — no wall clock). |
| `fake_eeprom` | `m24m01e_io_t` | 128 KB RAM array with real 256-byte page-wrap semantics; injectable NACK / bus-error / busy-then-ready on the *n*-th call; full transaction log. |
| `fake_store_dev` | `m24m01e_*` | Thin layer over `fake_eeprom`, so `ndt_store.c` is tested through the real chip driver (integration-flavoured but still hermetic) **and**, in a second configuration, against a link-time-substituted `m24m01e` mock for pure isolation. |
| `fake_gpio` | MAX14808 pin callbacks | 64-pin shadow array, write-order log, programmable read values, `delay_us` accumulator. |
| `driverlib_stub` | `I2C_*`, `Interrupt_*` | Records `I2C_putData()` bytes, feeds a scripted interrupt-source sequence to the ISR. |

### 2.4 Conventions

- **IDs:** `UT-<MOD>-nn`; MOD ∈ `SM` (state machine), `ST` (record store), `EE` (EEPROM
  driver), `PU` (pulser), `IB` (I2CB protocol), `CFG` (compile-time).
- **Priority:** P1 = safety or core correctness; P2 = important behaviour; P3 = hardening.
- Every case is **hermetic**: no shared state between cases, no ordering dependency.
  Each begins with a fresh `*_init()` and a cleared fake.
- Builds under test: modules are compiled with the default `ndt_config.h` unless the
  case names a variant (`CFG-PERIODIC`, `CFG-BOTH`, `CFG-CATCHUP`, `CFG-UNLATCHED`),
  which is produced with a `-D` override.

---

## 3. Test cases

### 3.1 State machine — `ndt_sm.c` (UT-SM)

Fixture: `ndt_sm_init(&fake_hooks)`, `millis` starts at a caller-chosen value.

#### Event queue

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-SM-01 | Init empties the ring | Post 3 events, then call `ndt_sm_init()` again, then `ndt_sm_step()` | No hook fires; state is `NDT_ST_IDLE`; `ndt_sm_dropped()` == 0 | P1 |
| UT-SM-02 | Post/pop round trip | `ndt_sm_post(NDT_EV_SET_BURST, 7)`; one step | `set_burst_cycles` called exactly once with 7 | P1 |
| UT-SM-03 | FIFO ordering | Post SET_BURST(6), SET_BURST(9), SET_BURST(7); three steps | `set_burst_cycles` called with 6, then 9, then 7 — in that order | P1 |
| UT-SM-04 | One event per step | Post 3 events; call step once | Exactly one hook call; two events remain queued | P1 |
| UT-SM-05 | Usable capacity is DEPTH−1 | With `NDT_EVENT_QUEUE_DEPTH` == 8, post 8 events without stepping | First **7** posts return `true`; the 8th returns `false`. Documents the reserved slot: capacity is 7, not 8 | P1 |
| UT-SM-06 | Overflow drops, never blocks | Post 12 events without stepping | Call returns (no spin/hang); `ndt_sm_dropped()` == 5; the first 7 are still delivered intact on subsequent steps | P1 |
| UT-SM-07 | Drop counter is cumulative and survives drain | Overflow by 3, drain fully, overflow by 2 | `ndt_sm_dropped()` == 5 | P3 |
| UT-SM-08 | Index wrap | Post+step 1 event 40 times (5× the ring) | All 40 delivered, in order, no drops — head/tail masking wraps correctly | P1 |
| UT-SM-09 | Unknown event id is inert | `ndt_sm_post((ndt_event_id_t)0x5A, 0)`; step | No hook fires; state stays `NDT_ST_IDLE`; event is consumed (not left blocking the ring) | P2 |

#### Queued-command semantics (the v1.0 behaviour change)

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-SM-10 | Trigger arriving mid-scan is not lost | Post TRIGGER; step into `NDT_ST_FIRE`; while non-idle post a second TRIGGER; run steps to completion | Second trigger is retained and fires a second scan after the first returns to idle. **Contrast with v1.0 STA-04**, which expected it to be discarded | P1 |
| UT-SM-11 | Save arriving mid-scan is not lost | As above with `NDT_EV_STORE_SAVE(0x1234)` | `store_save(0x1234)` runs exactly once, after `after_capture()` | P1 |
| UT-SM-12 | Events are consumed only in IDLE | Post TRIGGER; step; assert state == FIRE; post SET_BURST; step (FIRE arm) | `set_burst_cycles` is **not** called during the FIRE pass — only when idle is reached | P1 |
| UT-SM-13 | Burst of commands drains in order | Post TRIGGER, SET_BURST(8), STORE_SAVE(1) with no interleaved steps; run steps to quiescence | Scan completes, then burst set to 8, then save — the queue order, not a priority order | P2 |

#### Transitions

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-SM-14 | Nominal scan sequence | `supplies_ok` → true; post TRIGGER; step until idle | States observed: IDLE → FIRE → DATA_READY → IDLE. Hook order: `supplies_ok`, `fire_and_capture`, `after_capture` | P1 |
| UT-SM-15 | Blocked scan sequence | `supplies_ok` → false; post TRIGGER; step until idle | IDLE → FIRE → VOLTAGE_FAULT → IDLE; `on_voltage_fault` called once; **`fire_and_capture` never called** | P1 |
| UT-SM-16 | Voltage fault does not latch | Continue UT-SM-15 with `supplies_ok` → true; post TRIGGER | Second attempt reaches DATA_READY normally | P1 |
| UT-SM-17 | Store save transition | Post STORE_SAVE(0xABCD); one step | `store_save(0xABCD)` called once; state is `NDT_ST_IDLE` on return (STORE_OP is entered and left inside the same step) | P1 |
| UT-SM-18 | Store load transition | Post STORE_LOAD(0x0001); one step | `store_load(0x0001)` called once; back to IDLE | P1 |
| UT-SM-19 | 16-bit id passes through unmodified | STORE_SAVE with args 0x0000, 0x00FF, 0xFF00, 0xFFFF | Hook receives each value exactly; no masking or sign extension | P2 |
| UT-SM-20 | `ndt_sm_is_idle()` agrees with `ndt_sm_state()` | Sample both at every step of UT-SM-14 | `is_idle()` true iff state == `NDT_ST_IDLE`, at every point | P2 |
| UT-SM-21 | Burst hook is called raw | Post SET_BURST(0), (4), (11), (255) | Hook receives 0, 4, 11, 255 unmodified — the state machine does **not** clamp; clamping is the hook's job (see UT-SM-22) | P1 |
| UT-SM-22 | Clamp lives in the board hook | Review `hook_set_burst_cycles()` in `main.c`; exercise an extracted copy with 0, 4, 5, 10, 11, 255 | Results 5, 5, 5, 10, 10, 10. If `main.c` cannot be linked hermetically, this is a static-review item and must be recorded as such | P1 |

#### Thermal fault

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-SM-23 | Thermal entry from idle | `thermal_fault` → true; step | `on_thermal_enter` called once; state `NDT_ST_THERMAL_FAULT` | P1 |
| UT-SM-24 | Thermal outranks queued work | Post TRIGGER **and** STORE_SAVE; set `thermal_fault` → true; step | Thermal entry happens; neither `fire_and_capture` nor `store_save` is called; events remain queued (not consumed) | P1 |
| UT-SM-25 | Latched by default | After UT-SM-23, `thermal_fault` → false; advance millis 5 s; many steps | State never leaves `NDT_ST_THERMAL_FAULT`; `on_thermal_enter` not called again | P1 |
| UT-SM-26 | No scan while latched | While latched, post TRIGGER; many steps | `fire_and_capture` never called; queue is not drained | P1 |
| UT-SM-27 | Blink cadence | In thermal state, advance millis in 10 ms increments over 1 s, stepping each time | `thermal_blink` called at exactly `NDT_THERMAL_BLINK_MS` (100 ms) intervals — 10 calls, not one per step | P1 |
| UT-SM-28 | Blink does not block | Same as UT-SM-27 | Each step returns immediately; no hook other than `millis` / `thermal_fault` / `thermal_blink` is called. Guards the regression that `DEVICE_DELAY_US(100000)` caused | P1 |
| UT-SM-29 | Unlatched build recovers | Build `CFG-UNLATCHED` (`NDT_THERMAL_LATCHED=0`); enter thermal; `thermal_fault` → false; step | State returns to `NDT_ST_IDLE`; queued events then drain normally | P2 |
| UT-SM-30 | Unlatched build stays while asserted | `CFG-UNLATCHED`, `thermal_fault` stays true; 100 steps | Remains in thermal state; blinking continues | P2 |

#### Trigger policy

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-SM-31 | Default build ignores the clock | Default `NDT_TRIG_I2C`; advance millis by 10 s; step repeatedly | No scan starts without a posted TRIGGER | P1 |
| UT-SM-32 | Periodic build fires on schedule | `CFG-PERIODIC`; advance millis past `NDT_TRIG_PERIOD_MS`; step | State → FIRE without any posted event | P1 |
| UT-SM-33 | Periodic build ignores command 0x01 | `CFG-PERIODIC`; post TRIGGER before the period elapses; step | Event is consumed but no scan starts — commanded firing is disabled in this build | P1 |
| UT-SM-34 | BOTH build accepts either source | `CFG-BOTH`; (a) post TRIGGER early; (b) let the period elapse | Both produce a FIRE transition; the machine cannot distinguish them | P1 |
| UT-SM-35 | Skip-missed policy | Default `NDT_TRIG_SKIP_MISSED=1`, `CFG-PERIODIC`; make `fire_and_capture` advance millis by 2.5 × period | Exactly one scan follows; the schedule restarts from *now* — missed ticks are dropped | P2 |
| UT-SM-36 | Catch-up policy | `CFG-CATCHUP` (`NDT_TRIG_SKIP_MISSED=0`), same overrun | Successive steps fire back-to-back until the schedule is met, then resume the normal cadence | P2 |
| UT-SM-37 | Periodic branch pre-empts the queue | `CFG-BOTH`; post SET_BURST; advance millis past the period; single step | The periodic branch returns early, so `set_burst_cycles` is deferred by one pass. Documents an accepted design consequence — flag as a defect if command latency ever matters | P2 |
| UT-SM-38 | Millis wrap safety | Set millis to `0xFFFFFF00`; `CFG-PERIODIC`; advance past wrap to `0x00000200` | Trigger still fires at the right elapsed interval; unsigned subtraction in `elapsed()` handles the wrap; no missed or duplicated scan | P1 |
| UT-SM-39 | Blink wrap safety | As UT-SM-38, in the thermal state | Blink cadence is unbroken across the wrap | P2 |

#### Robustness

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-SM-40 | `ndt_sm_init(NULL)` | Call with NULL after a valid init | Previous hook table is retained; state and queue reset; no crash | P2 |
| UT-SM-41 | All-NULL hook table | Init with a zeroed `ndt_sm_hooks_t`; post one of each event; step through every state | No null dereference; the machine still traverses IDLE → FIRE → DATA_READY → IDLE; `millis` absent reads as 0 | P1 |
| UT-SM-42 | Individually NULL hooks | For each of the 11 hooks in turn, NULL only that one and exercise the path that uses it | No crash in any of the 11 runs; other hooks still called | P2 |
| UT-SM-43 | Step from every state is total | Force each `ndt_state_t` value, plus an out-of-range value, then step | Every case terminates and lands in a defined state; the `default` arm returns to IDLE | P2 |
| UT-SM-44 | No hidden re-entrancy | From inside `fire_and_capture`, call `ndt_sm_post()` | Event is queued and delivered later; no recursion into `ndt_sm_step()`. (Also records that the *real* single-producer rule is ISR-only — see UT-SM-45) | P2 |
| UT-SM-45 | Single-producer rule is documented, not enforced | Static review of every `ndt_sm_post()` call site in the tree | Only `ndt_i2cb.c` (ISR context) posts. Any main-loop producer added later breaks the lock-free ring — record as a review gate, not a runtime assertion | P1 |

### 3.2 Record store — `ndt_store.c` (UT-ST)

Fixture: `fake_eeprom` (128 KB, blank = 0xFF), real `m24m01e.c`, `ndt_store_init()`.
A "valid header" means one whose `samples_per_ch` == 512 and `channels` == 8.

#### Geometry and packing

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-ST-01 | Derived geometry | Evaluate the macros | `DATA_BYTES` 8192; `DATA_PAGES` 32; `SLOT_BYTES` 8448; `SLOT_COUNT` 15 | P1 |
| UT-ST-02 | Slots do not overlap or overrun | Compute `slot_addr(k)` for k = 0…14 | Strictly increasing by 8448; `slot_addr(14) + 8448` ≤ 131072 | P1 |
| UT-ST-03 | Header stream field order | Populate every header field with a distinct value; `ndt_store_hdr_to_stream()` | Bytes 0–1 `id`, 2–5 `seq`, 6–7 `burst_cycles`, 8–9 `samples_per_ch`, 10–11 `sample_period_ns`, 12–15 `start_delay_ns`, all little-endian | P1 |
| UT-ST-04 | Stream writes exactly 16 elements | Pre-fill a 20-element array with a sentinel; call | Elements 0–15 written; 16–19 untouched; every written element ≤ 0xFF | P1 |
| UT-ST-05 | Stream omits three header fields | Inspect the output for `freq_hz`, `channels`, `status_flags` | Absent by design — the 16-byte stream is a subset. Confirms `README.md` and the master-side decoder agree | P2 |
| UT-ST-06 | Stream NULL guards | `hdr` NULL; `out` NULL; both NULL | Returns without writing or crashing | P2 |
| UT-ST-07 | Endianness of the widest field | `start_delay_ns` = 0x12345678 | Bytes 12–15 read 0x78, 0x56, 0x34, 0x12 | P1 |

#### Save

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-ST-08 | Save into a blank store | Blank EEPROM; save id 0x0001 with a known ramp | Returns OK; slot 0 written; header magic `'N','D'`, version 1; `hdr->seq` == 1 on return | P1 |
| UT-ST-09 | Data page tiling | Inspect the fake's transaction log for UT-ST-08 | One 30-byte header write at slot base, then exactly 32 writes of 256 bytes at base+256, +512, … +8192. No write crosses a page boundary | P1 |
| UT-ST-10 | Sample byte order in the array | Sample word `w` = 0xBEEF at index 0 | EEPROM bytes at base+256 and +257 are 0xEF then 0xBE | P1 |
| UT-ST-11 | Sequence assignment | Save three records with ids 1, 2, 3 | `seq` values 1, 2, 3; the store assigns them and writes them back into the caller's header | P1 |
| UT-ST-12 | Overwrite same id reuses the slot | Save id 5 (slot 0); save id 5 again with different data | Same slot rewritten; no second slot consumed; loaded data is the **new** payload; `seq` incremented | P1 |
| UT-ST-13 | First-free selection | Save ids 1, 2, 3; erase slot 1 in the fake; save id 9 | Id 9 lands in slot 1 — the first blank slot, not slot 3 | P2 |
| UT-ST-14 | Fill all 15 slots | Save ids 1…15 | All succeed; all 15 slots hold valid magic; `seq` 1…15 | P1 |
| UT-ST-15 | Evict lowest sequence | From UT-ST-14, save id 16 | Succeeds; the slot that held `seq` 1 (id 1) is reused; ids 2…16 all still resolve | P1 |
| UT-ST-16 | Eviction picks by seq, not by slot index | Fill 15 slots, then overwrite id 7 (raising its seq), then save a new id | The evicted record is the one with the lowest seq — not slot 0 by default | P1 |
| UT-ST-17 | Geometry mismatch rejected | Header with `samples_per_ch` = 256, then one with `channels` = 4 | `NDT_STORE_ERR_PARAM` both times; **no EEPROM writes issued** | P1 |
| UT-ST-18 | NULL guards | `st` NULL; `st->dev` NULL; `hdr` NULL; `data` NULL | `NDT_STORE_ERR_PARAM` in all four; no bus traffic | P1 |
| UT-ST-19 | Bus error during the directory scan | Fake returns a bus error on the 4th `read` | Save aborts with `ERR_IO`; no data pages written; nothing partially committed beyond what the fake recorded | P1 |
| UT-ST-20 | Bus error mid-data-write | Fake fails on data page 17 | Returns `ERR_IO`; the caller sees failure; the slot is left detectably inconsistent — confirm a subsequent load reports `ERR_CORRUPT`, not silent success (see UT-ST-29) | P1 |
| UT-ST-21 | Write-protect propagates | Fake NACKs a page write | Chip driver maps NACK → `ERR_WRITE_PROTECT`; the store collapses it to `ERR_IO`. Documents the loss of granularity — the master cannot distinguish protection from a bus fault | P2 |

#### Load and find

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-ST-22 | Round trip | Save a pseudo-random 4096-word payload; load it back | `NDT_STORE_OK`; all 4096 words bit-identical; every header field matches | P1 |
| UT-ST-23 | Header round trip | As above | `id`, `seq`, `burst_cycles`, `freq_hz`, `sample_period_ns`, `start_delay_ns`, `samples_per_ch`, `channels`, `status_flags` all survive exactly | P1 |
| UT-ST-24 | Header-only query | `ndt_store_load(st, id, &hdr, NULL)` | Returns OK; header populated; **no data-page reads** issued (check the fake's log) | P2 |
| UT-ST-25 | Missing id | Load an id never saved | `NDT_STORE_ERR_NOT_FOUND`; caller's data buffer untouched | P1 |
| UT-ST-26 | Blank store | Load from an all-0xFF EEPROM | `ERR_NOT_FOUND` — bad magic is treated as blank, not as an error | P1 |
| UT-ST-27 | `ndt_store_find` hit and miss | Save ids 3 and 8; find 3, 8, 4 | Slots returned for 3 and 8; `ERR_NOT_FOUND` for 4 | P2 |
| UT-ST-28 | Find stops on a real bus error | Fake errors on slot 2's header read | Returns `ERR_IO` immediately — does not silently continue and report not-found | P1 |
| UT-ST-29 | Checksum detects corruption | Save a record, flip one bit in one data byte in the fake, load | `NDT_STORE_ERR_CORRUPT`; the data **is** still returned so the caller may inspect it | P1 |
| UT-ST-30 | Corrupt header magic | Corrupt byte 0 of a slot header | That slot reads as blank (`ERR_NOT_FOUND`); the record is not returned as valid | P1 |
| UT-ST-31 | Version mismatch rejected | Set version byte to 2 | `ERR_NOT_FOUND` — a future format is not misread as the current one | P2 |
| UT-ST-32 | Checksum blind spot | Swap two sample words within a record; load | Returns **OK** — an additive sum cannot detect transposition. Records a known limitation of the format; raise as a defect only if ordering corruption is considered plausible | P3 |
| UT-ST-33 | Checksum overflow wraps cleanly | Payload of all 0xFFFF (sum ≫ 16 bits) | Save and load both succeed; the truncation to 16 bits is consistent between write and verify | P2 |
| UT-ST-34 | Extreme ids | Save and load ids 0x0000 and 0xFFFF | Both round-trip. If either is intended to be reserved, the firmware must reject it consistently — assert whichever behaviour is chosen and document it | P3 |
| UT-ST-35 | Slot index bounds | `ndt_store_slot_header()` with slot 14, 15, 0xFFFF | OK for 14; `ERR_RANGE` for 15 and 0xFFFF | P1 |
| UT-ST-36 | `strerror` totality | Every `ndt_store_status_t` value plus an out-of-range int | Non-NULL, distinct strings; out-of-range yields "unknown" | P3 |

### 3.3 EEPROM chip driver — `m24m01e.c` (UT-EE)

Fixture: `fake_eeprom` with a full transaction log.

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-EE-01 | Init validation | `dev` NULL; `io` NULL; `io.write` NULL; `io.write_read` NULL; `ce` = 4 | `M24M01E_ERR_PARAM` in all five | P1 |
| UT-EE-02 | Init copies the io table | Init, then mutate the caller's `m24m01e_io_t` | Device keeps its own copy; behaviour unchanged | P2 |
| UT-EE-03 | Default poll tries | Init | `poll_tries` set to the driver default, non-zero | P2 |
| UT-EE-04 | Device-select byte, low half | Read at address 0x00000 with `ce` = 0 | A16 bit clear; `ce` bits placed at positions 2:1 | P1 |
| UT-EE-05 | Device-select byte, high half | Read at address 0x10000 | A16 bit set — the 17th address bit rides in the device-select byte | P1 |
| UT-EE-06 | Chip-enable encoding | `ce` = 0, 1, 2, 3 | Each appears at the correct bit positions in the address byte | P2 |
| UT-EE-07 | Read splits at the 64 KB edge | Read 512 bytes starting at 0x0FF00 | Exactly two `write_read` calls: 256 bytes ending at 0x0FFFF, then 256 from 0x10000 with A16 set | P1 |
| UT-EE-08 | Read address bytes | Read at 0x1234 | `wdata` is 0x12 then 0x34 — MSB first, 2 bytes | P1 |
| UT-EE-09 | Read bounds | `addr` = 0x20000; `addr` = 0x1FFFF with `len` = 2 | `ERR_RANGE` both | P1 |
| UT-EE-10 | Zero-length read is a no-op success | `len` = 0 | Returns OK; no bus traffic | P2 |
| UT-EE-11 | Read NULL guards | `dev` NULL; `buf` NULL | `ERR_PARAM`; no bus traffic | P1 |
| UT-EE-12 | Write splits on page boundary | Write 300 bytes at 0x0080 | Two writes: 128 bytes to finish the page, then 172. Neither crosses a 256-byte boundary | P1 |
| UT-EE-13 | Aligned full-page write | Write 256 bytes at 0x0100 | One write of 258 bus bytes (2 address + 256 data) | P1 |
| UT-EE-14 | Multi-page write | Write 1024 bytes at 0x0000 | Exactly four page writes, each followed by an ACK-poll sequence | P1 |
| UT-EE-15 | Write bounds and NULL | `addr` past the array; `len` overrunning the end; NULL `buf` | `ERR_RANGE` / `ERR_RANGE` / `ERR_PARAM`; no partial writes | P1 |
| UT-EE-16 | ACK polling ends on ready | Fake NACKs the address-only probe 3 times, then ACKs | `write_page_chunk` succeeds; exactly 4 probe calls logged | P1 |
| UT-EE-17 | ACK polling times out | Fake NACKs the probe forever | `M24M01E_ERR_TIMEOUT` after `poll_tries` attempts — bounded, no infinite loop | P1 |
| UT-EE-18 | `poll_tries` = 0 falls back to default | Force `dev->poll_tries` = 0; force a timeout | Uses the built-in default count, not zero attempts | P2 |
| UT-EE-19 | `delay_ms` optional | Run UT-EE-16 with `io.delay_ms` NULL | Polling still works; no crash | P1 |
| UT-EE-20 | NACK maps to write-protect | Fake returns `M24M01E_ERR_NACK` on a page write | Returns `M24M01E_ERR_WRITE_PROTECT`, distinct from `ERR_IO` | P1 |
| UT-EE-21 | Generic bus error maps to IO | Fake returns −99 | Returns `M24M01E_ERR_IO` | P2 |
| UT-EE-22 | Probe is address-only | `m24m01e_probe()` | One `write` call with `data` NULL and `len` 0 | P1 |
| UT-EE-23 | Probe failure propagates | Fake NACKs the probe | Non-OK status returned; caller (`NDT_initEeprom`) can set `EEPROM_FAIL` | P1 |
| UT-EE-24 | Feature-register address differs | `m24m01e_read_dti()` etc. | Uses the feature device-select base, not the memory base | P2 |
| UT-EE-25 | ID-page bounds | `offset` + `len` > 256 | `ERR_RANGE` | P2 |
| UT-EE-26 | Lock query without `probe_no_stop` | `io.probe_no_stop` NULL; `m24m01e_id_page_is_locked()` | `M24M01E_ERR_UNSUPPORTED`, not a crash or a false answer | P2 |
| UT-EE-27 | Byte helpers delegate correctly | `read_byte` / `write_byte` | Produce the same bus traffic as the 1-byte block calls | P3 |
| UT-EE-28 | `strerror` totality | Every status value | Non-NULL, distinct strings | P3 |

### 3.4 Pulser driver — `max14808.c` (UT-PU)

Fixture: `fake_gpio` with a pin shadow and an ordered write log.

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-PU-01 | Init argument validation | NULL `dev`, `pins`, `gpio_write`, `gpio_read`, `delay_us` in turn | `MAX14808_ERR_NULL` each time | P1 |
| UT-PU-02 | Variant validation | Variant = 99 | `MAX14808_ERR_PARAM` | P2 |
| UT-PU-03 | Init leaves all drive pins low | Valid init | All 8 DINP and all 8 DINN pins written 0; **no pin left high** | P1 |
| UT-PU-04 | Init lands in shutdown | Valid init | `current_mode` == `MAX14808_MODE_SHUTDOWN`; mode pins reflect it | P1 |
| UT-PU-05 | Init defaults | Valid init | Current = 2 A; SYNC low (transparent); LDO_EN low when the pin is mapped | P2 |
| UT-PU-06 | `MAX14808_PIN_NC` pins are skipped | Init with `cc0`, `cc1`, `sync`, `ldo_en` = `PIN_NC` (the board's actual configuration) | No `gpio_write` to any NC pin; no out-of-range pin id ever passed | P1 |
| UT-PU-07 | Octal three-level mode encoding | `max14808_set_mode(OCTAL_3LEVEL)` | MODE0 = 1, MODE1 = 0 — matches the board's hard requirement | P1 |
| UT-PU-08 | TX-disable mode encoding | `set_mode(TX_DISABLE)` | Mode pins match the datasheet encoding; state recorded in the handle | P1 |
| UT-PU-09 | Receive mode, T/R on | `enter_receive_mode(dev, true)` on a 14808 | Mode set to TX_DISABLE **first**, then DINP = DINN = 1 on all 8 channels, then `delay_us` called once | P1 |
| UT-PU-10 | Receive-mode settle time | Sum the `delay_us` accumulator for UT-PU-09 | **2 µs** — matches `tONTRSW` max 1.2 µs plus guard. **`README.md` and the pre-2.0 flowchart both claim ~12 µs**; one of the two is wrong. Assert the code's value and raise a documentation defect | P1 |
| UT-PU-11 | Ordering safety in receive mode | Inspect the write log for UT-PU-09 | The mode change precedes any DINP/DINN assertion — no window in which both are driven while TX is still enabled | P1 |
| UT-PU-12 | Receive mode, T/R off | `enter_receive_mode(dev, false)` | All DINP and DINN driven 0; no delay required | P2 |
| UT-PU-13 | 14809 variant rejects T/R | `enter_receive_mode(dev, true)` on a 14809 | `MAX14808_ERR_VARIANT`; **channel pins are not touched** — verify the log is empty after the mode change | P2 |
| UT-PU-14 | Channel index bounds | `set_channel_3level()` with ch = 0 and ch = 9 | `MAX14808_ERR_PARAM`; no pin written (channels are 1-based, `MAX14808_CH_MIN` = 1) | P1 |
| UT-PU-15 | Channel index off-by-one | `set_channel_3level(ch = 1)` and `(ch = 8)` | Drive the pins mapped to `dinp[0]`/`dinn[0]` and `dinp[7]`/`dinn[7]` respectively | P1 |
| UT-PU-16 | Three-level output states | Each `max14808_3level_out_t` value | Correct DINP/DINN pair for each; **no state drives both high** unless that state is defined to (T/R damp) | P1 |
| UT-PU-17 | All-channels helper | `set_all_channels_3level()` | All 8 channels written; each exactly once | P2 |
| UT-PU-18 | Thermal read polarity | `gpio_read(thp)` returns 0, then 1 | `overtemp` true for 0 (active-low), false for 1 | P1 |
| UT-PU-19 | Thermal read NULL guard | `overtemp` NULL | `MAX14808_ERR_NULL`; no read issued | P2 |
| UT-PU-20 | Current-drive encoding | Each `max14808_current_t` | CC pin pattern per datasheet — skipped entirely when CC pins are NC | P2 |
| UT-PU-21 | Handle state tracks the pins | After a sequence of mode changes | `dev->current_mode` always matches the last successful `set_mode`; a rejected call leaves it unchanged | P2 |

### 3.5 I2CB protocol — `ndt_i2cb.c` (UT-IB)

Requires `test/fakes/driverlib_stub.h`, which supplies `I2C_getInterruptSource()`,
`I2C_getData()`, `I2C_putData()`, `I2C_clearStatus()`, `Interrupt_clearACKGroup()`,
`I2CB_BASE` and the `I2C_INTSRC_*` constants. The stub feeds the ISR a scripted
sequence of interrupt sources and captures every transmitted byte. `ndt_sm_post()` is
replaced by a spy that records `(id, arg)` pairs.

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-IB-01 | Default stream after init | `ndt_i2cb_init()`, then two TX events with no preceding write | STATUS word served, little-endian — the initial descriptor is valid, not NULL | P1 |
| UT-IB-02 | Status stream is 2 bytes and wraps | Select 0x03; six TX events | Bytes repeat the 2-byte word three times; index wraps at the stream length; no read past the source | P1 |
| UT-IB-03 | Taps stream length | Select 0x05; 12 TX events | 10 distinct bytes = 5 × uint16 LE, then wrap | P1 |
| UT-IB-04 | Header stream is narrow | Select 0x08; 16 TX events | 16 bytes emitted, one per source element, **not** two bytes per element — the `wide` flag must be false here | P1 |
| UT-IB-05 | Waveform channel offset | Select 0x04 with ch = 3; TX events | Source pointer is `wave + 3 × 512`; 1024 bytes served | P1 |
| UT-IB-06 | Channel masking | `[0x04][0x08]` and `[0x04][0xFF]` | 0x08 → channel 0; 0xFF → channel 7. No out-of-bounds pointer is ever formed | P1 |
| UT-IB-07 | Channel guard against a short binding | Bind `wave_channels` = 4; select ch = 7 | `ch` falls back to 0 rather than indexing past the buffer | P2 |
| UT-IB-08 | Commands post, do not act | Send 0x01, 0x02+param, 0x06+2 params, 0x07+2 params | Spy sees `NDT_EV_TRIGGER`, `NDT_EV_SET_BURST`, `NDT_EV_STORE_SAVE`, `NDT_EV_STORE_LOAD` — and the TX descriptor is unchanged by any of them | P1 |
| UT-IB-09 | 16-bit id assembly | `[0x06][0x34][0x12]` | Posted arg is 0x1234 — `p0` low byte, `p1` high byte | P1 |
| UT-IB-10 | Id byte-order regression guard | `[0x07][0xFF][0x00]` and `[0x07][0x00][0xFF]` | 0x00FF and 0xFF00 respectively — never swapped | P1 |
| UT-IB-11 | Parameter accumulation | `[0x02]` then `[0x07]` as separate RX events | Event posted only after the second byte; arg == 7 | P1 |
| UT-IB-12 | Zero-parameter commands act immediately | `[0x03]` | Descriptor retargeted on the same RX event — a master may issue the read with no main-loop turn in between | P1 |
| UT-IB-13 | Unknown command counted | `[0x00]`, `[0x09]`, `[0x7F]`, `[0xFF]` | `ndt_i2cb_bad_commands()` == 4; no event posted; descriptor unchanged | P1 |
| UT-IB-14 | Command byte 0x00 is rejected | `[0x00]` | Counted as bad — guards the lookup table's unused slot 0 | P2 |
| UT-IB-15 | Parser reset on STOP | `[0x02]` then STOP, then `[0x03]` + TX | The orphaned 0x02 never applies; status is served correctly | P1 |
| UT-IB-16 | Parser reset on address match | `[0x06][0x34]` then ADDR_TARGET, then a read | No stale save is posted; the partial parameter is discarded | P1 |
| UT-IB-17 | TX index reset on address match | Serve 3 bytes, then ADDR_TARGET, then serve | Second read restarts at byte 0. **Documents the chunked-read limitation** (v1.0 I2C-07): the master must read a stream in one transaction | P1 |
| UT-IB-18 | NACK / arbitration-lost cleanup | Mid-parameter, inject NO_ACK, then ARB_LOST | `s_need`, `s_got`, `s_txIdx` all reset; `I2C_clearStatus()` called with both flags; the next transaction parses cleanly | P1 |
| UT-IB-19 | TX lock serves 0xFF | `ndt_i2cb_lock_tx(true)`; select any stream; 20 TX events | Every byte is 0xFF; the source buffer is never dereferenced | P1 |
| UT-IB-20 | Unlock restores real data | Lock, serve, unlock, serve | Real bytes resume; the index is not left in a corrupt position | P1 |
| UT-IB-21 | NULL source is safe | Bind `hdr_stream` = NULL; select 0x08; TX event | 0xFF served; no dereference | P1 |
| UT-IB-22 | Zero-length stream is safe | Bind `tap_count` = 0; select 0x05; TX event | 0xFF served; no division or modulo by zero; index does not run away | P2 |
| UT-IB-23 | Source drain loop terminates | Script 5 pending sources, then NONE | ISR services all 5 in one entry, then returns; `Interrupt_clearACKGroup()` called exactly once | P1 |
| UT-IB-24 | Unrecognised interrupt source | Script an undefined source value | Ignored; the loop still terminates | P2 |
| UT-IB-25 | RX data is masked to 8 bits | Stub returns 0x01 with high bits set (e.g. 0xFF01) | Treated as command 0x01 — masking with 0x00FF is applied. C28x-specific hazard | P1 |
| UT-IB-26 | TX data is masked to 8 bits | Source word 0xBEEF, wide stream | Emitted bytes are exactly 0xEF and 0xBE; no value > 0xFF ever reaches `I2C_putData()` | P1 |
| UT-IB-27 | `ndt_i2cb_init(NULL)` | Call with NULL | Retains the previous binding; resets parser and index; no crash | P2 |
| UT-IB-28 | Bad-command counter saturates gracefully | 70000 bad commands | Counter wraps as a `uint16_t` without UB; no other state affected | P3 |
| UT-IB-29 | Full-queue posts are tolerated | Make the `ndt_sm_post` spy return false | ISR ignores the return value and continues; no retry loop, no stall. Confirms a full queue degrades to dropped commands rather than a hung ISR | P1 |

### 3.6 Compile-time and configuration (UT-CFG)

These "run" at build time; a failure is a build failure. Each is exercised by a
deliberately-broken configuration in a negative-compile target.

| ID | Title | Setup / stimulus | Expected result | Pri |
|---|---|---|---|---|
| UT-CFG-01 | Queue depth must be a power of two | Build with `NDT_EVENT_QUEUE_DEPTH` = 6 | `#error` in `ndt_sm.c` fires; build fails | P1 |
| UT-CFG-02 | Record must tile the page grid | Build with `NDT_ASCAN_SAMPLES` = 500 | `#error` in `ndt_store.h` fires (8000 B is not a multiple of 256) | P1 |
| UT-CFG-03 | Record must fit a slot | Build with a geometry exceeding 128 KB | `#error` "not one slot fits" fires | P1 |
| UT-CFG-04 | Default geometry builds clean | Default `ndt_config.h` | No diagnostics; derived constants match UT-ST-01 | P1 |
| UT-CFG-05 | Byte masking is host/target neutral | Build the store and chip driver with `uint8_t` forced to a 16-bit container | Round-trip tests UT-ST-22 and UT-EE-12 still pass — proves the `& 0xFF` discipline, which a plain host build cannot exercise | P1 |
| UT-CFG-06 | Every trigger variant compiles | Build `ndt_sm.c` for `NDT_TRIG_I2C`, `_PERIODIC`, `_BOTH` × `SKIP_MISSED` 0/1 × `THERMAL_LATCHED` 0/1 | All 12 combinations compile warning-free; the corresponding UT-SM cases pass in each | P1 |
| UT-CFG-07 | `ndt_sm.c` is driverlib-free | Compile `ndt_sm.c` with no TI headers on the include path | Compiles — the layering claim in `ndt_sm.h` is enforced by the build, not just by convention | P1 |
| UT-CFG-08 | Header stream length agrees with the packer | `NDT_STORE_HDR_STREAM_LEN` vs the fields written | Static assertion (add one if absent) that the constant is 16 and the packer writes 16 | P2 |

---

## 4. Execution and coverage

### 4.1 Gates

| Gate | Requirement |
|---|---|
| Result | 100 % of P1 cases pass; P2 failures triaged with an agreed disposition |
| Line coverage | ≥ 95 % on `ndt_sm.c` and `ndt_store.c`; ≥ 90 % on `m24m01e.c`, `ndt_i2cb.c`, `max14808.c` |
| Branch coverage | ≥ 90 % on `ndt_sm.c` — the `#if` variants are the point of the module |
| Sanitizers | Zero ASan / UBSan findings |
| Static analysis | No new high-severity `cppcheck` or `clang-tidy` findings |
| Runtime | Full suite under 10 s, so it can run on every commit |

Uncovered lines are expected only in the unreachable `case NDT_ST_STORE_OP` arm of
`ndt_sm_step()` (§6, finding F-4) and in `strerror` default arms. Any other gap needs
a case or a written justification.

### 4.2 Suggested CI order

1. `UT-CFG-*` negative-compile targets (fastest failure).
2. Per-module suites, in dependency order: `EE` → `ST` → `PU` → `SM` → `IB`.
3. Coverage report and gate check.

---

## 5. Traceability

Coverage marked **partial** means the unit test constrains the behaviour but cannot
confirm it on hardware; a HIL case is still required.

| Requirement (source) | Unit tests | Coverage |
|---|---|---|
| Commands queued, executed at idle, dropped only on overflow (`README.md`, `ndt_config.h`) | UT-SM-05, 06, 10, 11, 12, 13; UT-IB-29 | Full |
| Burst length clamped 5–10 (cmd 0x02) | UT-SM-21, 22; UT-IB-11 | Full (clamp logic); burst *waveform* is HIL |
| Rails checked before firing; scan blocked if bad | UT-SM-15, 16 | Partial — sequencing only; thresholds and ADC are HIL |
| Thermal fault latched, HV locked, non-blocking blink | UT-SM-23…30 | Partial — policy only; pin behaviour is HIL |
| Voltage fault does not latch | UT-SM-16 | Full |
| Trigger source is a config-only change | UT-SM-31…37; UT-CFG-06 | Full |
| Millisecond wrap handled | UT-SM-38, 39 | Full |
| I2CB command set 0x01–0x08 | UT-IB-01…14 | Full (parsing); bus behaviour is HIL |
| Stream lengths: status 2 B, taps 10 B, wave 1024 B, header 16 B | UT-IB-02…05; UT-ST-03, 04 | Full |
| Read during a buffer rewrite returns 0xFF | UT-IB-19, 20 | Full |
| Chunked reads unsupported (pointer resets) | UT-IB-17 | Full — a *documented limitation*, verified as such |
| Unknown commands counted, not acted on | UT-IB-13, 14 | Full |
| Header stream field order and endianness | UT-ST-03, 07 | Full |
| Save/load by 16-bit id | UT-ST-08, 22, 23; UT-SM-19; UT-IB-09, 10 | Full |
| 15 slots, overwrite → free → evict-oldest | UT-ST-01, 12, 13, 14, 15, 16 | Full |
| Checksum on load | UT-ST-29, 32, 33 | Full, incl. the known blind spot |
| EEPROM failure is non-fatal | UT-EE-23; UT-ST-19, 20 | Partial — boot path is HIL |
| Page-aware writes, 64 KB read split, bounded ACK polling | UT-EE-07, 12, 13, 14, 16, 17 | Full |
| Pulser starts and ends in a safe state | UT-PU-03, 04, 06, 11 | Partial — pin levels are HIL |
| T/R settle time | UT-PU-10 | Full (code); the *correct* value is a doc question (F-1) |
| `ndt_sm.c` builds without driverlib | UT-CFG-07 | Full |
| Geometry constraints enforced at compile time | UT-CFG-01, 02, 03 | Full |
| 150 kHz burst, 625 kSps, 512 samples, timing, RAM/flash placement | — | **None — HIL only** |

---

## 6. Findings raised during this review

Recorded here because each one is either a documentation defect or a design point a
unit test can only *document* rather than fix.

| # | Finding | Evidence | Suggested action |
|---|---|---|---|
| F-1 | `README.md` and `flowchart.md` (pre-2.0) state a **~12 µs** T/R dead time; `max14808_enter_receive_mode()` waits **2 µs** | `max14808.c` | Decide which is correct. If 2 µs stands, `README.md` and the expected `start_delay_ns` (~45 µs → ~35 µs) both need correcting. UT-PU-10 pins the code's value |
| F-2 | Event-queue usable capacity is **7**, not the 8 implied by `NDT_EVENT_QUEUE_DEPTH` and by "8 deep" in `README.md` | `ndt_sm.c` ring reserves one slot | Reword `README.md` as "7 in flight", or raise the depth to 16 |
| F-3 | `ndt_sm_dropped()` is a diagnostic with no route to the master — a dropped command is silent | `ndt_sm.c`, `ndt_i2cb.c` | Consider a status bit or a byte in the STATUS stream |
| F-4 | `case NDT_ST_STORE_OP` in `ndt_sm_step()` is unreachable: the state is entered and left inside `step_idle()` | `ndt_sm.c` | Harmless, but expect it as an uncovered branch; add a comment or remove |
| F-5 | The store collapses `M24M01E_ERR_WRITE_PROTECT` into `NDT_STORE_ERR_IO`, so the master cannot distinguish protection from a bus fault | `map_dev()` in `ndt_store.c` | Accept and document, or add a status bit |
| F-6 | The additive checksum cannot detect word transposition | `ascan_checksum()` | Accept (UT-ST-32 documents it) or move to CRC-16 |
| F-7 | In `PERIODIC`/`BOTH` builds, a due periodic tick returns before draining the queue, deferring commands by one pass | `step_idle()` | Accept; UT-SM-37 documents it |
| F-8 | v1.0 of this document tests removed symbols and pre-refactor behaviour | §1.2 note | Superseded by this document |

---

## 7. Execution record template

| Field | Value |
|---|---|
| Test case ID | |
| Firmware commit SHA | |
| Test build (compiler, flags, config variant) | |
| Date / engineer | |
| Result | Pass / Fail / Blocked / N-A |
| Coverage delta | |
| Defect reference | |
| Notes | |
