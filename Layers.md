# Architecture — layers and file relationships

This document explains how the firmware is split, why each boundary exists, and
what depends on what. It is the map to read before changing anything; `README.md`
describes what the board *does*, this describes how the code is *arranged*.

The guiding rule: **each file should have exactly one reason to change.**
Re-targeting the MCU, swapping the memory chip, altering the record format, and
changing when a scan fires are four independent events, so they are four
different files.

---

## 1. The stack at a glance

```
                        ┌──────────────────────────────────────────┐
   policy / "when"      │  ndt_sm.c/h        state machine         │
                        │  ndt_config.h      all tunable policy    │
                        └───────────────┬──────────────────────────┘
                                        │ ndt_sm_hooks_t (function pointers)
                                        │ ndt_sm_post()  (events, from ISRs)
                        ┌───────────────┴──────────────────────────┐
   board / "how"        │  main.c            pins, init, capture,  │
                        │                    buffers, hook impls   │
                        └───┬───────────────┬──────────────────┬───┘
                            │               │                  │
        ┌───────────────────┘               │                  └────────────┐
        │                                   │                               │
┌───────▼─────────┐          ┌──────────────▼──────────┐        ┌───────────▼────────┐
│ ndt_i2cb.c/h    │          │ ndt_store.c/h           │        │ max14808.c/h       │
│ I2CB slave      │          │ A-scan record format    │        │ octal pulser       │
│ protocol + ISR  │          └──────────────┬──────────┘        └───────────┬────────┘
└───────┬─────────┘                         │                               │
        │                          ┌────────▼────────┐                      │
        │                          │ m24m01e.c/h     │                      │
        │                          │ EEPROM chip     │                      │
        │                          └────────┬────────┘                      │
        │                                   │ m24m01e_io_t                  │
        │                          ┌────────▼────────┐                      │
        │                          │ i2ca_eeprom.c/h │                      │
        │                          │ I2CA transport  │                      │
        │                          └────────┬────────┘                      │
        │                                   │                               │
┌───────▼───────────────────────────────────▼───────────────────────────────▼────────┐
│  board.c/h  (SysConfig)      device.c/h + driverlib      F280025 silicon           │
└────────────────────────────────────────────────────────────────────────────────────┘
```

Arrows point **downward = "calls into"**. The only upward flow is
`ndt_i2cb.c → ndt_sm_post()`, which is a queue push, not a call into board code.

---

## 2. Layer by layer

### 2.1 Policy layer — `ndt_sm.c/h`, `ndt_config.h`

Answers *when* things happen. Contains no register access and does not include
`driverlib.h`; it compiles on a host compiler, which is how the trigger and
fault configurations are regression-checked.

`ndt_sm.c` owns:

- the state variable (`static`, never exported by address),
- the event queue,
- the trigger policy (commanded / periodic / both),
- fault entry and exit rules.

It reaches hardware only through `ndt_sm_hooks_t`, a table of function pointers
supplied by `main.c` at startup. Consequence: the state machine cannot tell an
I2CB command from a timer tick, because both arrive as `NDT_EV_TRIGGER`.
Switching between commanded and periodic firing is therefore a one-line change
in `ndt_config.h`, with no edit to the machine.

`ndt_config.h` holds every decision that is not a physical fact: record
geometry, burst limits, trigger source and period, fault latching, blink rate,
slave address, queue depth. Physical facts (SYSCLK-derived tick counts, pin
numbers, ADC channel mapping) stay in `main.c`, because they are not choices.

### 2.2 Board layer — `main.c`

Answers *how*, on this PCB. Everything that touches a register in the
acquisition path lives here: pin map, `NDT_initPulser/Timers/Eeprom/Slave`,
supply-tap checking, the tone-burst, the RAM-resident capture loop, the shared
buffers, and the millisecond tick ISR.

The bottom third of the file is `hook_*` functions and the `g_hooks` table.
These are the adaptation between "what the state machine wants" and "what this
board can do" — for example `hook_supplies_ok()` samples the taps, updates the
status word, and returns a plain `bool`, so `ndt_sm.c` never learns the bit
layout of `g_statusFlags`.

`main()` itself is a composition root: bring up hardware, wire the modules,
enable interrupts, then loop on `ndt_sm_step()` forever.

### 2.3 Interface layer — `ndt_i2cb.c/h`

Owns the I2CB slave protocol and its ISR. It knows the command byte values and
the stream layout; it does not know what a command *means*.

Its dependency on the rest of the firmware is deliberately thin:

- **Downward**: `driverlib` I2C calls only.
- **Upward**: `ndt_sm_post()`.
- **Sideways**: a `ndt_i2cb_streams_t` struct of read-only buffer pointers,
  handed to it once by `main.c`. It never dereferences a global by name.

Adding a command means adding a byte to `k_paramLen[]`, a case in
`apply_command()`, and (if it does something) an event in `ndt_sm.h`. Nothing
in `main.c` changes unless the command needs new hardware behaviour.

### 2.4 Record layer — `ndt_store.c/h`

Owns the on-EEPROM record format: magic bytes, slot geometry, sequence numbers,
oldest-record eviction, the additive checksum, and the 16-byte header
serialisation served by command `0x08`.

This is application knowledge, which is why it is not in the chip driver. It
talks to the EEPROM only through the `m24m01e_*` API and has no idea I2C exists.
Changing the record format — more channels, a CRC instead of a sum, a directory
page — touches this file and nothing below it.

### 2.5 Chip layer — `m24m01e.c/h`

Owns M24M01E facts from the datasheet: the 17-bit address whose top bit rides
in the device-select byte, the 256-byte page boundary that a write must not
cross, ACK polling across the internal write cycle, and the DTI/CDA/SWP feature
registers. Portable C, no MCU dependency.

Replacing the memory part with a different EEPROM means writing a sibling to
this file and re-pointing `ndt_store.c`. Nothing else moves.

### 2.6 Transport layer — `i2ca_eeprom.c/h`

The bottom of the EEPROM stack, and the only part of it that is F280025
specific. It implements the three `m24m01e_io_t` callbacks against I2CA in
polled master mode, with iteration-budget timeouts so a stuck bus can never
hang the firmware, and with the C28x byte masking that a 16-bit `uint8_t`
container requires.

Note the direction: `i2ca_eeprom.c` is *below* `m24m01e.c`, not a wrapper around
it. The chip driver calls the transport, never the reverse.

### 2.7 Peripheral driver layer — `max14808.c/h`

MAX14808 octal pulser: mode selection, current setting, T/R switching. Like the
EEPROM chip driver it is MCU-agnostic — `main.c` supplies GPIO read/write and
delay callbacks at init.

### 2.8 Platform layer — `board.c/h`, `device.c/h`, `driverlib`

SysConfig-generated and TI-supplied. `Board_init()` configures ADCA/ADCC SOCs,
GPIO directions and mux, I2CA as master, and I2CB as target at 0x21 with its
interrupt registered. **Do not hand-edit** — regeneration overwrites it.

---

## 3. Dependency table

| File | Includes / uses | Used by | Reason it would change |
|------|-----------------|---------|------------------------|
| `ndt_config.h` | — | everything | A policy decision changes |
| `ndt_sm.c/h` | `ndt_config.h` | `main.c`, `ndt_i2cb.c` | States, events, trigger rules |
| `main.c` | all of the below | — (entry point) | Pins, timing, acquisition, wiring |
| `ndt_i2cb.c/h` | `driverlib`, `ndt_sm.h`, `ndt_config.h` | `main.c` | I2C protocol changes |
| `ndt_store.c/h` | `m24m01e.h`, `ndt_config.h` | `main.c` | Record format changes |
| `m24m01e.c/h` | `stdint`, `inc/hw_types.h` | `ndt_store.c`, `main.c` | Different memory part |
| `i2ca_eeprom.c/h` | `driverlib`, `m24m01e.h` | `main.c` (binding only) | Different MCU or I2C port |
| `max14808.c/h` | `stdint` | `main.c` | Different pulser |
| `board.c/h` | `driverlib` | `main.c` | Regenerated from SysConfig |

Two entries deserve attention:

- `i2ca_eeprom.c` includes `m24m01e.h` only for the `m24m01e_io_t` return-code
  convention. It is a downward include of a type contract, not a dependency on
  the driver's behaviour.
- `main.c` includes `m24m01e.h` only to declare the device handle and bind the
  transport. All actual EEPROM traffic goes through `ndt_store.c`.

---

## 4. Concurrency map

Three execution contexts exist. Every shared object below has exactly one
writer, which is what removes the need for critical sections.

| Object | Written by | Read by |
|--------|-----------|---------|
| `g_statusFlags` | main loop (hooks) | main loop, I2CB ISR |
| `g_waveBuf` | main loop (capture, EEPROM load) | main loop, I2CB ISR |
| `g_hdrStream` | main loop | I2CB ISR |
| `g_voltBuf` | main loop | I2CB ISR |
| `g_msTicks` | Timer 1 ISR | main loop |
| state machine state | main loop | main loop |
| event queue `head` | I2CB ISR | main loop |
| event queue `tail` | main loop | I2CB ISR |
| TX descriptor | I2CB ISR | I2CB ISR |

Rules that keep this true — break them and the guarantees go with them:

1. **Only interrupt context calls `ndt_sm_post()`.** The queue is lock-free
   because it has one producer and one consumer, each owning one index. The
   periodic trigger and the thermal check are handled inline inside
   `ndt_sm_step()` for exactly this reason, rather than being posted.
2. **The ISR never writes application state.** It parses bytes, retargets its
   own TX descriptor, and pushes events. Clamping, ID assembly, state
   transitions and EEPROM traffic all happen in main-loop context.
3. **`g_statusFlags |= x` has one writer.** It is a read-modify-write with no
   guard. If a future ISR ever needs to set a bit, it must go through an event
   instead, or the flag word needs an atomic set/clear pair.
4. **32-bit values shared with an ISR are read with a retry loop.**
   `ndt_millis()` reads `g_msTicks` twice and compares, because a 32-bit load
   on a 16-bit machine is not atomic.
5. **The read side is frozen while a buffer is rewritten.**
   `ndt_i2cb_lock_tx(true)` makes the slave answer `0xFF` during capture and
   during an EEPROM load, so a master read cannot observe a torn record. This
   costs nothing, where ping-pong buffers would cost another 8 KB of 24 KB.

---

## 5. Control flow of a scan

```
  master writes 0x01
        │
        ▼
  INT_myI2CB_ISR            ndt_i2cb.c   — decode byte, no logic
        │  ndt_sm_post(NDT_EV_TRIGGER)
        ▼
  event queue               ndt_sm.c     — lock-free ring
        │
        ▼  (next main-loop turn, machine idle)
  ndt_sm_step → step_idle → state = NDT_ST_FIRE
        │
        ▼  (following turn)
  hooks.supplies_ok()       main.c       — ADCA SOC4-8, tolerance check
        │  true
        ▼
  hooks.fire_and_capture()  main.c       — lock TX, HV on, burst, capture,
        │                                   build header, unlock TX
        ▼
  state = NDT_ST_DATA_READY
        │
        ▼
  hooks.after_capture()     main.c       — pulser mode restored, HV off,
        │                                   DATA_VALID set
        ▼
  state = NDT_ST_IDLE
```

A save (`0x06`) follows the same path as far as the queue, then runs
`hooks.store_save()` → `ndt_store_save()` → `m24m01e_write()` →
`i2ca_ep_write()`, blocking for roughly 0.3 s with `EEPROM_BUSY` set throughout.

---

## 6. Where to make a given change

| You want to… | Edit |
|--------------|------|
| Fire periodically instead of on command | `ndt_config.h` — `NDT_TRIGGER_SOURCE` |
| Change burst limits or the record size | `ndt_config.h` |
| Stop latching the thermal fault | `ndt_config.h` — `NDT_THERMAL_LATCHED` |
| Add a state or change a transition | `ndt_sm.c/h` |
| Add an I2C command | `ndt_i2cb.c` (+ an event in `ndt_sm.h`) |
| Change the stored record format | `ndt_store.c/h` |
| Use a different EEPROM part | new sibling of `m24m01e.c`, re-point `ndt_store.c` |
| Move the EEPROM to I2CB, or change MCU | `i2ca_eeprom.c` |
| Re-map a pin, retime the burst, alter capture | `main.c` |
| Change peripheral configuration | `ndt_board.syscfg`, then regenerate |

---

## 7. Design notes and known trade-offs

**Why `ndt_store.c` is separate from `m24m01e.c`.** Before the refactor the
chip driver contained `M24M01E_ASCAN_MAGIC0 = 'N'`. A generic ST memory driver
knowing the name of this project is the clearest possible signal of a layering
violation. Splitting it made `m24m01e.c` reusable (549 → 291 lines) and made
the record format editable without touching datasheet logic.

**Why the wrapper was kept rather than flattened.** `i2ca_eeprom.c` sits below
the chip driver, so deleting `m24m01e.c` would not remove code — it would move
paging, addressing and ACK polling into a file that is already MCU-specific,
making it both MCU- and chip-specific. Unused driver entry points cost nothing
in flash provided `--gen_func_subsections` is enabled; check the `.map` before
concluding otherwise.

**Why hooks instead of direct calls.** A struct of function pointers costs one
indirection per call, all of them outside the timing-critical path (the burst
and capture loops are entirely inside one hook). In exchange, `ndt_sm.c` builds
and is testable without hardware.

**Events consumed only in the idle state.** This is what makes deferral free:
a command that arrives during a scan simply stays in the ring. It also means a
command is lost only if more than `NDT_EVENT_QUEUE_DEPTH` arrive during one
busy period; `ndt_sm_dropped()` counts those.

**Blocking hooks.** `fire_and_capture` (~0.9 ms) and `store_save` (~0.3 s) run
to completion inside one `ndt_sm_step()`. The I2CB slave stays serviceable
because it is interrupt-driven, but the state machine is unresponsive for the
duration. Any future work that must run *during* a save needs the save broken
into a page-at-a-time sub-state — the structure allows it, the current code
does not do it.
