# F280025 NDT Auxiliary Board — Firmware Flowcharts

(150 kHz guided-wave collar: tone-burst A-scan + EEPROM storage).

---

## 1. Boot / Initialization Sequence (`main()`, steps 1–8)

```mermaid
flowchart TD
    A["Device_init()\nSYSCLK = 100 MHz, peripheral clocks enabled"] --> B["Interrupt_initModule()\nInterrupt_initVectorTable()"]
    B --> C["Board_init()  (SysConfig)\nADCA/ADCC SOCs, GPIO, I2CB slave, I2CA master"]
    C --> D["NDT_initTimers()\nCPU Timer0 free-running (10 ns/tick)\nePWM1 configured as ADC pacer, frozen"]
    D --> E["Analog_EN = 1, wait 500 µs\nAFE_GAIN A1/A2/B1/B2 = HIGH"]
    E --> F["NDT_initPulser()\nMAX14808 → octal 3-level mode"]
    F --> G["NDT_initEeprom()\nEEPROM_WC = 0 (writes enabled)\nprobe M24M01E on I2CA"]
    G --> H{"Probe OK?"}
    H -->|No| I["g_statusFlags |= STATUS_EEPROM_FAIL\n(scans still allowed, storage disabled)"]
    H -->|Yes| J["continue"]
    I --> K["Interrupt_enableGlobal()"]
    J --> K
    K --> L["MCU_LED = 1  (init complete)"]
    L --> M(["Enter main loop\nNDT state machine"])
```

---

## 2. Main Loop — NDT State Machine

```mermaid
stateDiagram-v2
    [*] --> NDT_IDLE

    NDT_IDLE --> NDT_THERMAL_FAULT: THP pin reads LOW (MAX14808 over-temp)
    NDT_IDLE --> NDT_FIRE: I2CB ISR sets state on cmd 0x01
    NDT_IDLE --> NDT_EEPROM_OP: I2CB ISR sets state on cmd 0x06 / 0x07

    NDT_FIRE --> NDT_DATA_READY: all 5 rails within ±5% → burst + capture
    NDT_FIRE --> NDT_VOLTAGE_FAULT: any rail out of tolerance

    NDT_DATA_READY --> NDT_IDLE: pulser restored to octal-3L, HV off,\nSTATUS_DATA_VALID set

    NDT_EEPROM_OP --> NDT_IDLE: NDT_runEepromOp() save/load complete

    NDT_VOLTAGE_FAULT --> NDT_IDLE: HV off (master must re-issue 0x01)

    NDT_THERMAL_FAULT --> NDT_THERMAL_FAULT: latched — HV off,\nTX_DISABLE, blink LED every 100 ms
```

---

## 3. `NDT_FIRE` → `NDT_fireAndCapture()` Detail

```mermaid
flowchart TD
    F1["Clear STATUS_DATA_VALID / VOLTAGE_FAULT / ALL_RAILS_OK"] --> F2["NDT_checkSupplies()\nforce ADCA SOC4–8, poll INT2 flag"]
    F2 --> F3{"All 5 taps\nwithin ±5% of target?"}
    F3 -->|No| F4["g_statusFlags |= STATUS_VOLTAGE_FAULT"] --> F5(["→ NDT_VOLTAGE_FAULT"])
    F3 -->|Yes| F6["NDT_fireAndCapture()"]
    F6 --> F7["Pulser_EN = 1, delay 500 µs (HV settle)"]
    F7 --> F8["t_fire = CPUTimer0 snapshot  (t = 0 of record)"]
    F8 --> F9["NDT_burst(g_burstCycles)\n150 kHz bipolar tone-burst,\nall 8 channels via direct GPIO writes"]
    F9 --> F10["max14808_enter_receive_mode()\nT/R switches close, ~12 µs dead time"]
    F10 --> F11["t_cap snapshot → g_startDelayNs\n(fire-to-first-sample delay, ns)"]
    F11 --> F12["NDT_captureRecord()  [runs from RAM]\narm ePWM1 (COUNTER_MODE_UP)\npoll ADCA/ADCC INT1, store 8 ch × 512 samples"]
    F12 --> F13["Freeze ePWM1 (STOP_FREEZE)"]
    F13 --> F14["Populate g_lastHdr\nNDT_packHdrStream()  (16-byte LE header)"]
    F14 --> F15(["→ NDT_DATA_READY"])
```

---

## 4. I2CB Slave ISR — Command / Stream Dispatch

```mermaid
flowchart TD
    S(["I2CB Interrupt"]) --> T{"Interrupt source"}

    T -->|ADDR_SLAVE| U["Reset TX byte pointer\nReset RX command parser"]

    T -->|RX_DATA_RDY| V{"First byte\nof transaction?"}
    V -->|"Yes: command byte"| W{"Command value"}
    W -->|0x01| W1["if state==IDLE:\nstate = NDT_FIRE, toggle LED"]
    W -->|0x02| W2["expect 1 param byte\n(burst cycles)"]
    W -->|0x03| W3["select STATUS stream (2 B)"]
    W -->|0x04| W4["expect 1 param byte\n(wave channel 0–7)"]
    W -->|0x05| W5["select TAPS stream (10 B)"]
    W -->|0x06| W6["expect 2 param bytes\n(save ID, LE)"]
    W -->|0x07| W7["expect 2 param bytes\n(load ID, LE)"]
    W -->|0x08| W8["select HEADER stream (16 B)"]
    W -->|other| W9["ignored"]

    V -->|"No: parameter byte"| X["store in s_rxPar[]"]
    X --> X1{"all params\nreceived?"}
    X1 -->|No| X2["wait for next byte"]
    X1 -->|Yes, cmd 0x02| X3["clamp to 5..10 → g_burstCycles"]
    X1 -->|Yes, cmd 0x04| X4["g_waveCh = param & 0x07\nselect WAVE stream (1024 B)"]
    X1 -->|Yes, cmd 0x06/0x07| X5{"state == IDLE?"}
    X5 -->|Yes| X6["g_eeId = param\ng_eeOp = SAVE/LOAD\nstate = NDT_EEPROM_OP"]
    X5 -->|No| X7["ignored (busy)"]

    T -->|TX_DATA_RDY| Y["Emit next byte of\nselected stream\n(STATUS / TAPS / WAVE / HEADER)\nwrap pointer at stream length"]

    T -->|STOP_CONDITION| Z["Reset TX pointer\nReset RX parser"]
```

---

## Reference — I2CB Command Table

| Write bytes | Effect | Read-back |
|---|---|---|
| `0x01` | Trigger A-scan (rails checked first) | — |
| `0x02 [n]` | Set burst cycles, clamped 5–10 | — |
| `0x03` | Select STATUS stream | 2 bytes |
| `0x04 [ch]` | Select WAVEFORM stream, channel 0–7 | 1024 bytes (512× uint16 LE) |
| `0x05` | Select TAPS stream | 10 bytes |
| `0x06 [idL][idH]` | Save current A-scan to EEPROM under ID | — |
| `0x07 [idL][idH]` | Load A-scan by ID into RAM buffer | — |
| `0x08` | Select HEADER stream | 16 bytes (id, seq, burst, samples, period_ns, start_delay_ns) |
