# F280025 NDT Auxiliary Board — Firmware Flowcharts

150 kHz guided-wave collar: tone-burst A-scan, 8-channel capture, EEPROM record store.

These diagrams describe the firmware **after the state-machine refactor** — the one
where policy moved out of `main.c` into `ndt_sm.c`, and the I2CB ISR stopped writing
the state variable. If a diagram and the source disagree, the source wins; please fix
the diagram.

**Document version:** 2.0
**Covers:** `main.c`, `ndt_sm.c`, `ndt_i2cb.c`, `ndt_store.c`, `ndt_config.h`

---

## 0. Module map and control flow

Where each diagram below lives in the layer stack, and which direction calls travel.

```mermaid
flowchart TD
    subgraph POLICY["policy — no driverlib, host-compilable"]
        CFG["ndt_config.h<br/>geometry, trigger source,<br/>fault policy, queue depth"]
        SM["ndt_sm.c<br/>states, event ring,<br/>trigger rules"]
    end

    subgraph BOARD["board — everything that touches a register"]
        MAIN["main.c<br/>pin map, init, burst,<br/>capture, hook table"]
    end

    subgraph IFACE["interface"]
        I2CB["ndt_i2cb.c<br/>slave protocol + ISR"]
    end

    subgraph RECORD["record / chip / transport"]
        STORE["ndt_store.c<br/>slot format, checksum"]
        CHIP["m24m01e.c<br/>paging, ACK polling"]
        XPORT["i2ca_eeprom.c<br/>F280025 I2CA"]
    end

    MAIN -->|"ndt_sm_step()"| SM
    SM -->|"ndt_sm_hooks_t<br/>(function pointers)"| MAIN
    I2CB -.->|"ndt_sm_post()<br/>queue push, not a call down"| SM
    MAIN --> I2CB
    MAIN --> STORE
    STORE --> CHIP
    CHIP --> XPORT
    CFG --- SM
    CFG --- MAIN

    style POLICY fill:#eef7ee
    style BOARD fill:#eef2fa
    style IFACE fill:#fdf4e8
    style RECORD fill:#f7eef7
```

The only upward edge is `ndt_sm_post()`, and it is a ring-buffer push rather than a
call into board code — which is what keeps `ndt_sm.c` free of driverlib.

---

## 1. Boot — `main()` composition root

```mermaid
flowchart TD
    A["Device_init()<br/>SYSCLK = 100 MHz, flash wait-states,<br/>.TI.ramfunc copy when _FLASH"] --> B["Interrupt_initModule()<br/>Interrupt_initVectorTable()"]
    B --> C["Board_init()  (SysConfig)<br/>ADCA SOC0-3 + SOC4-8, ADCC SOC0-3,<br/>GPIO, I2CB target @0x21, I2CA master"]
    C --> D["NDT_initTimers()"]

    D --> D1["CPU Timer 0: free-running down-counter<br/>10 ns/tick — burst pacing + timestamps"]
    D1 --> D2["CPU Timer 1: 1 kHz IRQ on INT13<br/>ndt_timer1ISR increments g_msTicks"]
    D2 --> D3["ePWM1: TBPRD = 79 (1.6 us),<br/>SOCA on CTR=0, left in STOP_FREEZE"]

    D3 --> E["Analog_EN = 1, wait 500 us<br/>AFE_GAIN A1/A2/B1/B2 = HIGH"]
    E --> F["NDT_initPulser()<br/>MAX14808: 2 A, octal three-level<br/>(MODE0=1, MODE1=0)"]
    F --> F1{"max14808_init /<br/>set_mode OK?"}
    F1 -->|No| F2(["ESTOP0 — halt.<br/>HV is never enabled"])
    F1 -->|Yes| G["NDT_initEeprom()<br/>EEPROM_WC = 0 (writes enabled)"]

    G --> G1{"m24m01e_init + probe +<br/>ndt_store_init all OK?"}
    G1 -->|No| G2["g_statusFlags |= STATUS_EEPROM_FAIL<br/>non-fatal: scans still work,<br/>only storage is unavailable"]
    G1 -->|Yes| G3["store bound to EEPROM"]

    G2 --> H["ndt_sm_init(&g_hooks)<br/>queue emptied, state = NDT_ST_IDLE"]
    G3 --> H
    H --> I["NDT_initSlave()<br/>ndt_i2cb_init(): bind status / taps /<br/>wave / header streams"]
    I --> J["Interrupt_enableGlobal()"]
    J --> K["MCU_LED = 1 — init complete"]
    K --> L(["for(;;) ndt_sm_step()"])
```

**Ordering constraint:** `ndt_sm_init()` and `ndt_i2cb_init()` must both run *before*
`Interrupt_enableGlobal()`. The I2CB ISR is a producer on the state machine's event
ring, so the ring has to be empty and the hook table bound before the first interrupt
can fire.

---

## 2. Main loop — the state machine

States are `ndt_state_t` in `ndt_sm.h`. The state variable lives inside `ndt_sm.c`
and is written by exactly one context: whoever calls `ndt_sm_step()`. No ISR touches it.

```mermaid
stateDiagram-v2
    [*] --> NDT_ST_IDLE

    NDT_ST_IDLE --> NDT_ST_THERMAL_FAULT: thermal_fault() true (checked first, every pass)
    NDT_ST_IDLE --> NDT_ST_FIRE: periodic tick elapsed (PERIODIC / BOTH builds)
    NDT_ST_IDLE --> NDT_ST_FIRE: NDT_EV_TRIGGER popped from queue
    NDT_ST_IDLE --> NDT_ST_IDLE: NDT_EV_SET_BURST popped — hook clamps, no transition

    NDT_ST_IDLE --> NDT_ST_STORE_OP: NDT_EV_STORE_SAVE / NDT_EV_STORE_LOAD popped
    NDT_ST_STORE_OP --> NDT_ST_IDLE: blocking save/load returns (same step_idle call)

    NDT_ST_FIRE --> NDT_ST_DATA_READY: supplies_ok() true, fire_and_capture() done
    NDT_ST_FIRE --> NDT_ST_VOLTAGE_FAULT: supplies_ok() false

    NDT_ST_DATA_READY --> NDT_ST_IDLE: after_capture() — mode restored, HV off, DATA_VALID set
    NDT_ST_VOLTAGE_FAULT --> NDT_ST_IDLE: not latched, master re-issues 0x01

    NDT_ST_THERMAL_FAULT --> NDT_ST_THERMAL_FAULT: latched build — blink every NDT_THERMAL_BLINK_MS
    NDT_ST_THERMAL_FAULT --> NDT_ST_IDLE: NDT_THERMAL_LATCHED == 0 and THP released
```

Two things worth reading carefully:

- `NDT_ST_STORE_OP` is entered and left **inside a single `step_idle()` call**, around
  the blocking `store_save()` / `store_load()` hook. It is not a state the loop parks
  in, so the `case NDT_ST_STORE_OP` arm in `ndt_sm_step()` is unreachable in the
  current design — it exists as a safety default.
- `NDT_ST_VOLTAGE_FAULT` costs one extra `ndt_sm_step()` pass before returning to
  idle. That pass is where a master polling `STATUS` can observe the state.

---

## 3. `step_idle()` — priority order in the idle state

This is the part the old diagram got wrong: commands are **queued**, not applied by
the ISR, and they are consumed only here.

```mermaid
flowchart TD
    S(["step_idle()"]) --> T1{"hooks.thermal_fault()<br/>THP reads LOW?"}
    T1 -->|Yes| T2["on_thermal_enter()<br/>HV off, TX_DISABLE, latch flag"]
    T2 --> T3["s_lastBlinkMs = now<br/>state = NDT_ST_THERMAL_FAULT"] --> T4([return])

    T1 -->|No| P1{"build has PERIODIC<br/>or BOTH trigger?"}
    P1 -->|Yes| P2{"elapsed(s_lastTrigMs,<br/>NDT_TRIG_PERIOD_MS)?"}
    P2 -->|Yes| P3["accept_trigger(periodic = true)"] --> P4([return — queue not drained this pass])
    P2 -->|No| Q1
    P1 -->|No| Q1{"q_pop(&ev)<br/>event available?"}

    Q1 -->|No| Q2([return — nothing to do])
    Q1 -->|Yes| Q3{"ev.id"}

    Q3 -->|NDT_EV_TRIGGER| R1{"build == NDT_TRIG_PERIODIC?"}
    R1 -->|Yes| R2["ignored — commanded firing<br/>disabled in this build"]
    R1 -->|No| R3["accept_trigger(false)<br/>state = NDT_ST_FIRE"]

    Q3 -->|NDT_EV_STORE_SAVE| R4["state = NDT_ST_STORE_OP<br/>hooks.store_save(ev.arg)  [blocking ~0.3 s]<br/>state = NDT_ST_IDLE"]
    Q3 -->|NDT_EV_STORE_LOAD| R5["state = NDT_ST_STORE_OP<br/>hooks.store_load(ev.arg)  [blocking]<br/>state = NDT_ST_IDLE"]
    Q3 -->|NDT_EV_SET_BURST| R6["hooks.set_burst_cycles(ev.arg)<br/>hook clamps to 5..10 — state unchanged"]
    Q3 -->|"NDT_EV_CLEAR_FAULT<br/>/ default"| R7["no-op"]
```

**One event per pass.** `step_idle()` pops at most one event, so a burst of eight
commands takes eight loop iterations to drain — negligible, since a pass with nothing
to do is a handful of instructions.

**Periodic pre-emption.** In a `PERIODIC` or `BOTH` build the periodic branch returns
early, so a tick that is due delays the queue drain by one pass. Only pathological if
`NDT_TRIG_PERIOD_MS` approaches the loop period.

---

## 4. Event queue — single-producer / single-consumer ring

```mermaid
flowchart LR
    subgraph ISRCTX["interrupt context — the ONLY producer"]
        A1["INT_myI2CB_ISR<br/>apply_command()"] --> A2["ndt_sm_post(id, arg)"]
    end

    subgraph RING["s_queue[NDT_EVENT_QUEUE_DEPTH]"]
        B1["s_head — written by producer only"]
        B2["s_tail — written by consumer only"]
    end

    subgraph MAINCTX["main-loop context — the ONLY consumer"]
        C1["ndt_sm_step() → step_idle()"] --> C2["q_pop(&ev)"]
    end

    A2 --> B1
    B2 --> C2

    A2 -.->|"next == s_tail → full"| D1["s_dropped++<br/>return false — never blocks"]
```

Each index is written by exactly one context and only read by the other, so neither
side needs a critical section. That invariant is why the **thermal check and the
periodic tick are handled inline in `step_idle()` rather than posted as events** — a
second producer would break the lock-free guarantee.

```mermaid
flowchart TD
    N1["Capacity note: the ring reserves one slot to<br/>distinguish full from empty."] --> N2["NDT_EVENT_QUEUE_DEPTH = 8<br/>→ 7 events in flight before drops begin"]
    N2 --> N3["ndt_sm_dropped() is the diagnostic counter;<br/>it is not exposed over I2CB today"]
```

---

## 5. `NDT_ST_FIRE` → capture path

```mermaid
flowchart TD
    F0(["ndt_sm_step(), state == NDT_ST_FIRE"]) --> F1["hooks.supplies_ok()<br/>= hook_supplies_ok()"]
    F1 --> F2["clear DATA_VALID, VOLTAGE_FAULT,<br/>ALL_RAILS_OK from g_statusFlags"]
    F2 --> F3["NDT_checkSupplies()<br/>ADC_forceMultipleSOC(SOC4..SOC8)<br/>busy-wait on ADCA INT2 flag"]
    F3 --> F4["for each of 5 taps:<br/>g_voltBuf[i] = result<br/>tap_in_range(meas, expected) ±5%"]
    F4 --> F5{"all 5 rail bits set?"}

    F5 -->|No| V1["hook_on_voltage_fault()<br/>STATUS_VOLTAGE_FAULT, Pulser_EN = 0"]
    V1 --> V2(["state = NDT_ST_VOLTAGE_FAULT"])

    F5 -->|Yes| G1["hook_fire_and_capture()"]
    G1 --> G2["ndt_i2cb_lock_tx(true)<br/>slave reads now answer 0xFF"]
    G2 --> G3["Pulser_EN = 1<br/>DEVICE_DELAY_US(500) — HV settle"]
    G3 --> G4["t_fire = CPUTimer0 count<br/>(t = 0 of the record)"]
    G4 --> G5["NDT_burst(g_burstCycles)"]
    G5 --> G6["max14808_enter_receive_mode(dev, true)<br/>TX_DISABLE, then DINP=DINN=1 on all 8 ch,<br/>delay_us(2) for tONTRSW — see note below"]
    G6 --> G7["t_cap = CPUTimer0 count<br/>g_startDelayNs = (t_fire - t_cap) × 10 ns"]
    G7 --> G8["NDT_captureRecord()  [.TI.ramfunc]"]
    G8 --> G9["populate g_lastHdr<br/>(id = 0, seq = 0 until saved)"]
    G9 --> G10["ndt_store_hdr_to_stream(&g_lastHdr, g_hdrStream)"]
    G10 --> G11["ndt_i2cb_lock_tx(false)"]
    G11 --> G12(["state = NDT_ST_DATA_READY"])

    G12 --> H1["next pass: hook_after_capture()<br/>restore octal-3L, Pulser_EN = 0,<br/>set STATUS_DATA_VALID"]
    H1 --> H2(["state = NDT_ST_IDLE"])
```

> **Discrepancy flagged during review.** `README.md` and the previous version of this
> file both describe a *"~12 µs"* T/R dead time. `max14808_enter_receive_mode()` in
> `max14808.c` actually waits **2 µs** (`tONTRSW` max 1.2 µs plus guard). The eight
> `_write_channel()` calls add only tens of cycles. Either the code or the README is
> wrong; the diagram above follows the code. This also moves the expected
> `start_delay_ns` for a 5-cycle burst from ≈45 µs to ≈35 µs — worth confirming on the
> bench before the number is used as an acceptance limit.

### 5a. `NDT_burst()` — drift-free bipolar excitation

```mermaid
flowchart TD
    B0(["NDT_burst(cycles)"]) --> B1["t_ref = CPUTimer0 count"]
    B1 --> B2{"i < cycles?"}
    B2 -->|No| B7["GPACLEAR/GPBCLEAR = ALL_MASK<br/>all drive pins LOW (clamped)"] --> B8([return])
    B2 -->|Yes| B3["positive half:<br/>CLEAR B-mask, then SET A-mask<br/>(clear-then-set → no shoot-through)"]
    B3 --> B4["burst_wait(&t_ref, 333)"]
    B4 --> B5["negative half:<br/>CLEAR A-mask, then SET B-mask"]
    B5 --> B6["burst_wait(&t_ref, 334)"]
    B6 --> B2
```

`burst_wait()` advances `*t_ref` by exactly the requested tick count rather than
re-reading the timer, so loop-overhead jitter cannot accumulate across the burst.
Alternating 333/334 SYSCLK half-periods give a 667-tick cycle → 149.925 kHz (−0.05 %).

### 5b. `NDT_captureRecord()` — RAM-resident polling loop

```mermaid
flowchart TD
    C0(["NDT_captureRecord()"]) --> C1["clear ADCA INT1, ADCC INT1"]
    C1 --> C2["EPWM1 counter = 0<br/>mode = COUNTER_MODE_UP — pacer armed"]
    C2 --> C3{"n < 512?"}
    C3 -->|Yes| C4["poll ADCA INT1 (SOC3 EOC)<br/>poll ADCC INT1"]
    C4 --> C5["clear both INT1 flags"]
    C5 --> C6["8 × ADC_readResult →<br/>g_waveBuf[0..3][n] from ADCA<br/>g_waveBuf[4..7][n] from ADCC"]
    C6 --> C3
    C3 -->|No| C7["EPWM1 mode = STOP_FREEZE"] --> C8([return])
```

Budget: SOCA every 1.6 µs = 160 SYSCLK cycles per sweep; the 4-SOC round robin takes
≈1.45 µs, leaving the store body the remainder. Requires `-O2` **and** the RAM
placement — flash wait-states alone would push it over.

---

## 6. I2CB slave ISR — command and stream dispatch

The ISR does three things: accumulate parameter bytes, retarget the TX descriptor, or
push one event. It never changes state, never clamps, never touches the EEPROM.

```mermaid
flowchart TD
    S(["INT_myI2CB_ISR"]) --> L{"src = I2C_getInterruptSource()<br/>!= NONE?"}
    L -->|No| ZZ["Interrupt_clearACKGroup(GROUP8)"] --> ZE([return])

    L -->|Yes| T{"interrupt source"}

    T -->|ADDR_TARGET| U["s_txIdx = 0<br/>s_need = 0, s_got = 0<br/>(new transaction — drop partial cmd)"] --> L

    T -->|RX_DATA_RDY| V{"s_need == 0?"}
    V -->|"Yes — this is a command byte"| W{"d == 0 or d > CMD_MAX?"}
    W -->|Yes| W0["s_badCmd++"] --> L
    W -->|No| W1{"k_paramLen[d] == 0?"}
    W1 -->|Yes| W2["apply_command(d, 0, 0)"] --> L
    W1 -->|No| W3["s_cmd = d<br/>s_need = k_paramLen[d]<br/>s_got = 0"] --> L

    V -->|"No — parameter byte"| X["s_par[s_got++] = d"]
    X --> X1{"s_got >= s_need?"}
    X1 -->|No| L
    X1 -->|Yes| X2["s_need = 0<br/>apply_command(s_cmd, s_par[0], s_par[1])"] --> L

    T -->|TX_DATA_RDY| Y{"s_txLock<br/>or src == NULL<br/>or bytes == 0?"}
    Y -->|Yes| Y1["b = 0xFF — buffer busy"]
    Y -->|"No, wide (uint16 LE)"| Y2["w = src[idx >> 1]<br/>b = low or high byte of w"]
    Y -->|"No, narrow (byte values)"| Y3["b = src[idx] & 0xFF"]
    Y1 --> Y4
    Y2 --> Y4
    Y3 --> Y4["I2C_putData(b)<br/>s_txIdx = (++idx >= bytes) ? 0 : idx"] --> L

    T -->|STOP_CONDITION| Z["s_txIdx = 0, s_need = 0"] --> L
    T -->|"NO_ACK / ARB_LOST"| Z2["s_need = 0, s_got = 0, s_txIdx = 0<br/>I2C_clearStatus(NO_ACK | ARB_LOST)"] --> L
```

### 6a. `apply_command()` — two kinds of command

```mermaid
flowchart TD
    A(["apply_command(cmd, p0, p1)"]) --> B{"cmd"}

    B -->|"0x03 SEL_STATUS"| C1["src = &g_statusFlags<br/>bytes = 2, wide = true, idx = 0"]
    B -->|"0x05 SEL_TAPS"| C2["src = g_voltBuf<br/>bytes = 10, wide = true"]
    B -->|"0x04 SEL_WAVE"| C3["ch = p0 & 0x07<br/>src = &wave[ch × 512]<br/>bytes = 1024, wide = true"]
    B -->|"0x08 SEL_HDR"| C4["src = g_hdrStream<br/>bytes = 16, wide = false"]

    C1 --> D1["stream selection is done HERE,<br/>in the ISR: a master may issue the<br/>read with no main-loop turn in between.<br/>Cost is three stores."]
    C2 --> D1
    C3 --> D1
    C4 --> D1

    B -->|"0x01 TRIGGER"| E1["ndt_sm_post(NDT_EV_TRIGGER, 0)"]
    B -->|"0x02 SET_BURST"| E2["ndt_sm_post(NDT_EV_SET_BURST, p0)"]
    B -->|"0x06 STORE_SAVE"| E3["ndt_sm_post(NDT_EV_STORE_SAVE,<br/>p0 | (p1 << 8))"]
    B -->|"0x07 STORE_LOAD"| E4["ndt_sm_post(NDT_EV_STORE_LOAD,<br/>p0 | (p1 << 8))"]
    B -->|default| E5["s_badCmd++"]

    E1 --> F1["anything with a consequence becomes<br/>an event. Clamping, state changes and<br/>EEPROM work happen in main-loop context.<br/>Return value ignored — a full queue drops."]
    E2 --> F1
    E3 --> F1
    E4 --> F1
```

**Behaviour change from the pre-refactor firmware:** `0x01`, `0x06` and `0x07` used to
be ignored outright unless the board was idle. They are now queued (7 deep in flight)
and executed when the board next reaches idle. They are lost only on queue overflow.

---

## 7. EEPROM record store

### 7a. Save — `hook_store_save()` → `ndt_store_save()`

```mermaid
flowchart TD
    A(["hook_store_save(id)"]) --> B["g_statusFlags |= EEPROM_BUSY<br/>g_statusFlags &= ~EEPROM_FAIL"]
    B --> C{"STATUS_DATA_VALID set?"}
    C -->|No| C1["rc = NDT_STORE_ERR_PARAM<br/>nothing valid to save"] --> Z
    C -->|Yes| D["g_lastHdr.id = id<br/>ndt_store_save(&g_store, &g_lastHdr, g_waveBuf)"]

    D --> E{"st / hdr / data NULL,<br/>or geometry != 512×8?"}
    E -->|Yes| E1["ERR_PARAM"] --> Z
    E -->|No| F["Pass 1 — directory scan, all 15 slots<br/>read_hdr() each: magic 'N','D',ver"]

    F --> F1["track: match (same id),<br/>freeslot (first blank),<br/>max_seq, min_seq → oldest"]
    F1 --> F2{"real bus error<br/>during scan?"}
    F2 -->|Yes| F3["return that status"] --> Z
    F2 -->|No| G["target = match ?: freeslot ?: oldest<br/>hdr->seq = max_seq + 1"]

    G --> H["build 30-byte header page:<br/>magic, version, id, seq, burst, freq,<br/>period, start_delay, samples, channels,<br/>status, checksum(sum of all 4096 words)"]
    H --> I["m24m01e_write(slot_addr(target), raw, 30)"]
    I --> J{"OK?"}
    J -->|No| Z
    J -->|Yes| K["32 data pages, 128 words each:<br/>stage[] ← LE bytes of data[w]<br/>m24m01e_write(addr, stage, 256)<br/>addr += 256"]
    K --> K1{"all pages OK?"}
    K1 -->|No| Z
    K1 -->|Yes| L["NDT_STORE_OK"] --> Z

    Z["rc != OK → g_statusFlags |= EEPROM_FAIL<br/>ndt_store_hdr_to_stream() — seq now visible<br/>g_statusFlags &= ~EEPROM_BUSY"] --> ZE([return to step_idle])
```

Slot selection is **overwrite-same-id → first-free → evict-lowest-seq**. Sequence
numbers are assigned by the store, not the caller, and are written back into the
caller's header so `P-HDR` reflects the committed value.

### 7b. Load — `hook_store_load()` → `ndt_store_load()`

```mermaid
flowchart TD
    A(["hook_store_load(id)"]) --> B["g_statusFlags |= EEPROM_BUSY<br/>clear EEPROM_FAIL and DATA_VALID"]
    B --> C["ndt_i2cb_lock_tx(true)<br/>— g_waveBuf is rewritten page by page"]
    C --> D["ndt_store_find(id): linear scan of slots,<br/>read_hdr() until h.id == id"]
    D --> E{"found?"}
    E -->|No| E1["ERR_NOT_FOUND"] --> Y
    E -->|Yes| F["read_hdr(slot, hdr, &stored_ck)"]
    F --> G["32 × m24m01e_read(256 B) →<br/>data[w] reassembled from LE byte pairs"]
    G --> H{"ascan_checksum(data)<br/>== stored_ck?"}
    H -->|Yes| H1["NDT_STORE_OK"] --> Y
    H -->|No| H2["ERR_CORRUPT<br/>(data still returned for inspection)"] --> Y

    Y["ndt_i2cb_lock_tx(false)"] --> Z{"rc == OK?"}
    Z -->|Yes| Z1["g_statusFlags |= DATA_VALID"]
    Z -->|No| Z2["g_statusFlags |= EEPROM_FAIL<br/>DATA_VALID stays clear"]
    Z1 --> ZZ
    Z2 --> ZZ["ndt_store_hdr_to_stream(&g_lastHdr, g_hdrStream)<br/>g_statusFlags &= ~EEPROM_BUSY"] --> ZE([return])
```

A corrupt record is never reported to the master as valid: `DATA_VALID` stays clear
and `EEPROM_FAIL` is set.

---

## 8. TX lock — why a read can never tear

```mermaid
sequenceDiagram
    participant M as Master MCU
    participant ISR as I2CB ISR
    participant ML as Main-loop
    participant Buf as g_waveBuf

    ML->>ISR: ndt_i2cb_lock_tx(true)
    ML->>Buf: NDT_captureRecord() overwrites 8×512
    M->>ISR: read WAVEFORM stream
    ISR-->>M: 0xFF, 0xFF, 0xFF …
    ML->>ISR: ndt_i2cb_lock_tx(false)
    M->>ISR: read STATUS
    ISR-->>M: DATA_VALID set
    M->>ISR: read WAVEFORM stream
    ISR-->>M: real samples
```

Cheaper than double-buffering, which would cost another 8 KB of the 24 KB SRAM. The
lock is set for the whole of `hook_fire_and_capture()` and for the data phase of
`hook_store_load()`.

---

## 9. Reference tables

### 9a. I2CB command set (target address 0x21)

| Write bytes | Effect | Path | Read-back |
|---|---|---|---|
| `0x01` | Trigger A-scan | posts `NDT_EV_TRIGGER` | — |
| `0x02 [n]` | Set burst cycles | posts `NDT_EV_SET_BURST`, hook clamps 5–10 | — |
| `0x03` | Select STATUS | TX descriptor, in ISR | 2 B (LE) |
| `0x04 [ch]` | Select WAVEFORM, ch = `p0 & 7` | TX descriptor, in ISR | 1024 B (512 × u16 LE) |
| `0x05` | Select TAPS | TX descriptor, in ISR | 10 B (5 × u16 LE) |
| `0x06 [idL][idH]` | Save record under ID | posts `NDT_EV_STORE_SAVE` | — |
| `0x07 [idL][idH]` | Load record by ID | posts `NDT_EV_STORE_LOAD` | — |
| `0x08` | Select HEADER | TX descriptor, in ISR | 16 B |

Header stream (16 B, LE): `id(2) seq(4) burst_cycles(2) samples_per_ch(2) sample_period_ns(2) start_delay_ns(4)`.

### 9b. Status word

| Bit | Flag | Set by | Cleared by |
|---|---|---|---|
| 0x0001–0x0010 | rail `*_OK` bits | `hook_supplies_ok()` | start of each `supplies_ok()` |
| 0x0020 | `DATA_VALID` | `hook_after_capture()`, successful load | `supplies_ok()`, start of load |
| 0x0040 | `THERMAL_FAULT` | `hook_on_thermal_enter()` | reset only (latched build) |
| 0x0080 | `VOLTAGE_FAULT` | `hook_on_voltage_fault()` | start of next `supplies_ok()` |
| 0x0100 | `EEPROM_BUSY` | entry to save/load hook | exit of save/load hook |
| 0x0200 | `EEPROM_FAIL` | boot probe failure, failed op | start of each save/load |

### 9c. Where policy lives

| Decision | Symbol (`ndt_config.h`) | Consumed in |
|---|---|---|
| Record geometry | `NDT_ASCAN_SAMPLES`, `_CHANNELS` | `ndt_store.h` (compile-time check), `main.c` |
| Burst clamp | `NDT_BURST_CYCLES_MIN/MAX/DEFAULT` | `hook_set_burst_cycles()` |
| Trigger source | `NDT_TRIGGER_SOURCE` | `step_idle()` (`#if`) |
| Periodic period / catch-up | `NDT_TRIG_PERIOD_MS`, `NDT_TRIG_SKIP_MISSED` | `step_idle()`, `accept_trigger()` |
| Thermal latch / blink | `NDT_THERMAL_LATCHED`, `NDT_THERMAL_BLINK_MS` | `ndt_sm_step()` |
| Queue depth | `NDT_EVENT_QUEUE_DEPTH` | `ndt_sm.c` ring |

Switching from commanded to periodic firing is a `ndt_config.h` edit and a rebuild:
an I2CB command and a timer tick post the same `NDT_EV_TRIGGER`, and the state machine
cannot tell them apart.
