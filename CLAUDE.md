# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Motor control system based on HC32F460 (Cortex-M4) with RS485/Modbus RTU communication. Supports DC motors (Hall sensor feedback, over-current/voltage detection, rotation angle limiting, fault handling) and BLDC motors (six-step trapezoidal commutation via SDH21263 gate driver, 3-Hall sensor closed-loop control).

All source code lives under `ws_v.2.1/HC32F460_DDL_Rev3.3.0/`.

## Build System

- **Primary IDE:** Keil MDK (uVision 5)
- **Project file:** `ws_v.2.1/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template.uvprojx`
- **MCU:** HC32F460xE (512KB Flash) — linker script is `template/MDK/config/linker/HC32F460xE.sct`
- **Debug probe:** JLink (Cortex-M4)
- **Build output:** `template/MDK/output/debug/` (`.axf`, `.hex`, `.bin`, `.map`)
- Open the `.uvprojx` in Keil, click Build (F7), or use `UV4.exe -r template.uvprojx -o output.txt` from command line
- IAR EWARM and Eclipse/GCC project files also exist under `template/EWARM/` and `template/GCC/` but are not actively maintained

## Source Tree

```
projects/ev_hc32f460_lqfp100_v2/
├── Adp/          # Hardware adapter layer (rs485, PWM, ADC, DMA, GPIO, timers, flash)
├── App/          # Application logic (motor control, comm, fault, realtime)
├── Dev/          # Device drivers (motor, ADC, hall, voltage, sensor, EventBus, DeviceManager)
├── Utils/        # Utilities (ring_buf, msg_queue, lock, param_manager, TickTimer)
├── RTT/          # SEGGER RTT debug output
├── ws/           # BLDC 6-step commutation (dev_commutation, hall_sensor_3ch)
└── template/
    ├── source/   # main.c, main.h, hc32f4xx_conf.h
    └── MDK/      # Keil project, startup, linker scripts, JLink config
```

## 4-Layer Communication Stack (strict top-down dependency)

```
App_Comm.c/h          — Register callbacks, motor control commands, Flash persistence
    ↓ calls
Protocol_ModbusRtu.c/h — CRC16, function codes (0x03/0x06/0x10), exception responses
    ↓ calls
Comm_HAL.c/h           — Ring buffers, frame timeout (3.5 char times), TX queue
    ↓ calls
rs485.c/h              — Pure hardware: USART4 + PA03 direction pin + ISRs
```

- **Each layer only calls the layer directly below it** — no cross-layer access
- Protocol layer knows nothing about register meanings; it calls `on_read`/`on_write`/`on_validate` callbacks
- Comm_HAL knows nothing about Modbus; it just assembles byte streams into frames by idle timeout
- All config aggregated into `App_Comm_Config_t` in main.c

## Key Architecture Patterns

- **EventBus** (`Dev/EventBus.h`): Publish/subscribe for inter-module communication. Uses deferred publish — events published before `EventBus_Enable()` are queued as a bitmask and replayed on enable. Max 4 subscribers per topic, priority-ordered (0 = highest).

  | Index | Topic | Purpose |
  |-------|-------|---------|
  | 0 | `TOPIC_POWER` | Power output state changes |
  | 1 | `TOPIC_LIMIT_HARD` | Hardware limit switches |
  | 2 | `TOPIC_LIMIT_SOFT` | Software limits |
  | 3 | `TOPIC_CAN_EVENT` | CAN bus events (reserved) |
  | 4 | `TOPIC_MOTOR_CMD` | Motor control commands |
  | 5 | `TOPIC_MOTOR_SPEED_FEEDBACK` | Speed feedback from Hall sensor |
  | 6 | `TOPIC_MOTOR_DRIVE_EXEC` | Drive execution |
  | 7 | `TOPIC_MANUAL_IO` | Manual IO button presses |
  | 8 | `TOPIC_ALARM` | Generic alarm events |
  | 9 | `TOPIC_VOLTAGE_ALARM` | Voltage fault alarm (over/under) |
  | 10 | `TOPIC_CURRENT_ALARM` | Overcurrent alarm |
  | 11 | `TOPIC_RTURN_LIMIT` | Rotation angle limit reached |
  | 12 | `TOPIC_FAULT_CLEAR` | Fault clear request |
  | 13 | `TOPIC_MANUAL_RS485` | Manual RS485 control |

- **DeviceManager** (`Dev/device_manager.h`): Uniform device registry with time-sliced update scheduling. Each device gets an `update()` callback called at its configured interval (typically 1ms, voltage bus at 10ms). Mutex-protected access. Device IDs are defined in `App/App_Motor_Project.h`:

  | ID | Constant | Device |
  |----|----------|--------|
  | 0 | `ID_MOTOR` | Motor arbitrator |
  | 1 | `ID_PWR_POS` | Positive power output |
  | 2 | `ID_PWR_NEG` | Negative power output |
  | 3 | `ID_PWR_TEST1` | Test power 1 (PB10) |
  | 4 | `ID_PWR_TEST2` | Test power 2 (PA02) |
  | 5 | `ID_HALL_UP` | Upper Hall limit switch |
  | 6 | `ID_HALL_DOWN` | Lower Hall limit switch |
  | 7 | `ID_IO_FWD` | Forward IO button |
  | 8 | `ID_IO_REV` | Reverse IO button |
  | 9 | `ID_PWM_MOTOR` | Motor PWM output |
  | 10 | `ID_MOTOR_HALL` | Hall sensor (speed/dir) |
  | 11 | `ID_ADC_CURRENT` | Current ADC channel |
  | 12 | `ID_ADC_VOLTAGE` | Voltage ADC channel |
  | 13 | `ID_VOLTAGE_BUS` | Voltage bus monitor |
  | 14 | `ID_SENSOR_CURRENT` | Current sensor (threshold) |
  | 15 | `ID_RTURN` | Rotation angle limiter |

- **Param Manager** (`Utils/param_manager.h`): Register-based parameter storage with Flash persistence. Parameters live in `g_AppParam` (type `AppParamRecord_t`). Read/write via `Param_ReadByReg()`/`Param_WriteByReg()`, save via `Param_Save()`. Uses wear-leveled Flash storage across sectors 56-62 with CRC32 validation, sequence numbers, and magic headers.

- **Motor arbitration** (`Dev/dev_motor.c`): Commands go through the motor arbitrator which decides whether to allow based on mode (auto/remote/manual). Uses `block_fwd`/`block_rev` arrays (`MotorCommandList_t`) — multiple devices can independently block a direction (e.g., overcurrent adds `DEV_ID_OVERCUR_FWD`, limit switches add `DEV_ID_RTURN_FWD`). Arbitration re-evaluates on every block/unblock. Priority order: EMERGENCY(0) > LIMIT(2) > MANUAL(3) > CAN(4) > POWER(5).

- **Simulation mode** (`ENABLE_SIMULATION_MODE=1` in `App_Motor_Project.c`): Enabled by default. The `g_sim` struct holds simulated hardware signals (power, Hall limits, IO buttons, ADC values). The main loop detects state changes on `g_sim` and publishes corresponding EventBus events, allowing full motor arbitration testing without physical hardware.

- **SEGGER RTT** (`RTT/`): Debug logging via `MAIN_D()`, `COMM_DBG()`, `HAL_DEBUG()` macros — these output through RTT, not UART.

## Fault System

Fault bits (stored in `g_RealTimeData.fault_status`, readable via Modbus register 0x2740):

| Bit | Macro | Description |
|-----|-------|-------------|
| bit0 | `FAULT_BIT_OVERVOLTAGE` | Overvoltage |
| bit1 | `FAULT_BIT_OVERCURRENT` | Overcurrent |
| bit2 | `FAULT_BIT_OVERTEMP` | Overtemp (reserved, not currently used) |
| bit3 | `FAULT_BIT_RESET` | Reset |
| bit4 | `FAULT_BIT_OVERLOAD` | Overload |
| bit5 | `FAULT_BIT_STALL` | Motor stall |
| bit6 | `FAULT_BIT_UNDERVOLTAGE` | Undervoltage |

**Note:** Fault bit definitions are inconsistent across code comments and documentation. The `实时数据使用说明.md` maps OVERCURRENT to bit1, while `电流控制逻辑说明.md` maps it to bit2. The authoritative source is the actual `#define FAULT_BIT_*` macros in the source — verify those before trusting any documentation.

- `App_FaultHandler` subscribes to `TOPIC_VOLTAGE_ALARM` and `TOPIC_CURRENT_ALARM`, sets/clears fault bits in realtime data
- Overcurrent triggers **dual blocking**: dev_motor blocks forward via `DEV_ID_OVERCUR_FWD`, dev_rturn also blocks via `TOPIC_RTURN_LIMIT` for redundancy
- **Auto-clear mode**: faults clear automatically when the alarm condition resolves
- **Manual-clear mode**: faults persist until cleared via Modbus write to `REG_FAULT_STATUS` (0x2740), which calls `FaultHandler_ClearFault()`

## Control Commands (REG_CTRL_CMD = 0x2720)

Bits written via Modbus function 0x06 (single write only):

| Bit | Value | Description |
|-----|-------|-------------|
| bit0 | 0x0001 | START — enable RS485 control |
| bit1 | 0x0002 | STOP — disable RS485 control, stop motor |
| bit2 | 0x0004 | ESTOP — emergency stop (motor stops, control stays enabled) |
| bit3 | 0x0008 | RESET — `__NVIC_SystemReset()` after 200ms delay |
| bit4 | 0x0010 | FWD — forward (requires START first, uses `g_AppParam.target_speed`) |
| bit5 | 0x0020 | REV — reverse (requires START first, uses `g_AppParam.target_speed`) |

Typical sequence: START (0x0001) → FWD (0x0011) → STOP (0x0002)

## Startup Initialization Sequence

The startup flow in `main.c` (current BLDC-focused version):

```
Hardware_Init()             → SysTick, TMR0 timers, AOS trigger chains, GPIO
App_Comm_Init(&comm_cfg)    → 4-layer comm stack (RS485 + Modbus RTU, node_id=1, 9600 baud)
// ESystem_Init()           → COMMENTED OUT — DeviceManager + 16 devices (disabled for BLDC testing)
// FaultHandler_Init()      → COMMENTED OUT — EventBus fault subscriptions (disabled)
TMR4_PWM_Config(&pwm_cfg)   → TMR4 PWM 50kHz, 3 channels (U/V/W) SYNC mode
TMR4_PWM_StartOutput()      → Start PWM output
Timer6_Timebase_Init/Start  → μs free-running timebase
// Init coast: all 3 channels 98% SYNC (upper FETs ON → freewheel)
hall_3ch_create(&hall_cfg)   → 3-Hall sensor on PA8/PA9/PA10, INT008-010
EventBus_Enable()            → Unblock deferred publishes (needed even without ESystem)
// Super loop:
while (1) {
    App_Comm_Poll();         → Modbus frame processing
    // Mode switching (comm_mode 0-4): coast / open-loop FWD/REV / closed-loop CW/CCW
    // Open-loop: timer-driven Commutation_Step() via commu_num
    // Closed-loop: hall_3ch_update() for RPM + stall detection
}
```

**Note:** `ESystem_Init()` and `FaultHandler_Init()` are commented out in the current `main.c` — the BLDC commutation loop replaces the old DeviceManager-based motor control. The old DC motor control path (DeviceManager + dev_motor + rotation angle limiter) is preserved in the codebase but not active.

`ESystem_Init()` is defined in `App/App_Motor_Project.c` and wires up all device instances, EventBus subscriptions, and callbacks.

## Important Constraints

- Flash erase/write cycles are limited (~10K-100K). Each `Param_Save()` triggers a sector erase. Avoid calling it per-register in multi-register writes (0x10) — the batch write path calls `Param_Save()` once for the entire batch
- Interrupt safety: Comm_HAL uses `__disable_irq()`/`__enable_irq()` around ring buffer reads. Keep critical sections short
- `ModbusRTU_SendResponse` writes CRC at `raw[len]` and `raw[len+1]` — if `len >= 255`, this overruns the 256-byte buffer. Current code paths keep `len ≤ 253` (3-byte header + max 250 data bytes), but add a bounds check when modifying protocol logic
- `ModbusRTU_ProcessFrame` expects `len <= 256`. Frame buffer is 256 bytes. Modbus RTU max frame is 256 bytes so this is safe
- RS485 direction pin polarity is configurable via `dir_polarity` (0 = high-TX/low-RX, 1 = low-TX/high-RX)
- Realtime data (`g_RealTimeData`) is RAM-only, **not persisted to Flash** — lost on power cycle
- Multi-register writes (0x10) reject batches that include `REG_CTRL_CMD` or `REG_FAULT_STATUS` — those must use single writes (0x06)

## Motor Control Architecture

### PWM Output

The motor has **two** PWM timer configurations:

**TMR4 PWM (active in current `main.c`):** 50kHz center-aligned PWM on 6 pins via `Adp/tmr4_pwm.c/h` — PB9(UH), PB8(UL), PB7(VH), PB6(VL), PB5(WH), PB4(WL). Three independent half-bridge channels, each runtime-switchable between SYNC (THROUGH) and COMPLEMENTARY (DEAD_TMR) modes for SDH21263 gate driver. Coast mode: all 3 channels at 98% SYNC.

**TMRA_4 PWM (commented out in `main.c`, preserved as reference):** Edge-aligned PWM with 4 channels on PB6-PB9 (CH3/CH4 partially commented out in the stop path):

| Channel | Pin | Active Polarity |
|---------|-----|-----------------|
| CH1 | PB6 | Low-active |
| CH2 | PB7 | Low-active |
| CH3 | PB8 | Low-active (partially disabled) |
| CH4 | PB9 | Low-active (partially disabled) |

- PWM frequency: configured via `PWM_Init()` (typically 20kHz)
- Duty cycle range: 2%–98% (`MOTOR_DUTY_MIN`/`MOTOR_DUTY_MAX`)
- Ramp time: 800ms (`MOTOR_RAMP_TIME_MS`) — uses `PWM_StartRamp_TargetFromStart()` for smooth speed changes
- Stop polarity: CH1/CH3 high-active at 50%, CH2/CH4 low-active at 50% (balanced stop)
- Run polarity: all channels low-active — uses `Motor_SetRunPolarity()` to switch from stop mode

### TMR4 PWM Driver (`Adp/tmr4_pwm.c/h`)

TMR4 unit 3 (CM_TMR4_3), 6 pins on GPIO func2: PB9(UH), PB8(UL), PB7(VH), PB6(VL), PB5(WH), PB4(WL). Three half-bridge channels, each independently configurable at runtime.

```c
typedef enum {
    TMR4_CHANNEL_U = 0,    // PB9/PB8
    TMR4_CHANNEL_V = 1,    // PB7/PB6
    TMR4_CHANNEL_W = 2,    // PB5/PB4
    TMR4_CHANNEL_COUNT = 3,
} tmr4_channel_t;

typedef enum {
    TMR4_MODE_COMPLEMENTARY = 0,  // DEAD_TMR PWM, H=invert(L) — both FETs OFF
    TMR4_MODE_SYNC = 1,           // THROUGH PWM, H=L — FETs follow duty
} tmr4_channel_mode_t;

void TMR4_PWM_Config(const tmr4_pwm_config_t *cfg);       // Init all 3 channels
void TMR4_PWM_SetChannelMode(tmr4_channel_t ch, tmr4_channel_mode_t mode, uint16_t duty);
void TMR4_PWM_SetDuty(tmr4_channel_t ch, uint16_t duty);  // 0-10000 = 0.00%-100.00%
void TMR4_PWM_SetDutyFloat(tmr4_channel_t ch, float pct); // 0.0-100.0
void TMR4_PWM_SetFrequency(uint16_t freq_hz);              // Runtime frequency change
void TMR4_PWM_StartOutput(void);
void TMR4_PWM_StopOutput(void);
void TMR4_PWM_EmergencyStop(void);
```

**Per-channel mode behavior:**
- **SYNC mode:** H=L, both follow duty cycle. With SDH21263: HIGH=upper FET ON, LOW=lower FET ON.
- **COMPLEMENTARY mode:** H=invert(L) with dead-time. H≠L → SDH21263 interlock → both FETs OFF (floating phase).

**Internals:**
- Counter: `TMR4_MD_TRIANGLE` (center-aligned). Period formula: `PCLK1 / (freq_hz × 2)`.
- Dead-time: `ticks = dead_time_ns × PCLK1 / 1e9`. Reads PCLK1 via `CLK_GetBusClockFreq(CLK_BUS_PCLK1)` at Config time.
- Shadow registers: All duty/mode writes go to shadow registers, transferred at counter PEAK for glitch-free 3-channel sync.
- Reference: `F:/HC32F460_folder/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/examples/timer4/timer4_pwm_through/`

### BLDC Six-Step Commutation (`ws/dev_commutation.c/h`)

State table driver for trapezoidal BLDC commutation via SDH21263 pre-driver IC.

```
Step 0 (UH_VL): U=SYNC(duty%),  V=SYNC(2%),   W=COMP(50%)  → U→V current
Step 1 (UH_WL): U=SYNC(duty%),  V=COMP(50%),  W=SYNC(2%)   → U→W current
Step 2 (VH_WL): U=COMP(50%),   V=SYNC(duty%), W=SYNC(2%)   → V→W current
Step 3 (VH_UL): U=SYNC(2%),    V=SYNC(duty%), W=COMP(50%)  → V→U current
Step 4 (WH_UL): U=SYNC(2%),    V=COMP(50%),  W=SYNC(duty%) → W→U current
Step 5 (WH_VL): U=COMP(50%),   V=SYNC(2%),   W=SYNC(duty%) → W→V current
```

- **SYNC(duty%):** active phase — H=L=pulse → upper FET PWM/torque
- **SYNC(2%):** low-side return — nearly always LOW → lower FET conducts
- **COMP(50%):** floating phase — H≠L → interlock → both FETs OFF

Each step calls `TMR4_PWM_SetChannelMode()` for each of the 3 channels. Lazy-update: skips channels whose mode+duty haven't changed since the last step.

```c
void Commutation_Init(void);                                     // All 3 phases to COMP OFF
void Commutation_Step(uint8_t state, uint16_t freq_hz, float duty_pct);  // state 0-5
void Commutation_Stop(void);                                     // All phases to COMP OFF
// Convenience macros:
COMM_STEP_UH_VL(freq, duty)  // Step 0
COMM_STEP_UH_WL(freq, duty)  // Step 1
COMM_STEP_VH_WL(freq, duty)  // Step 2
COMM_STEP_VH_UL(freq, duty)  // Step 3
COMM_STEP_WH_UL(freq, duty)  // Step 4
COMM_STEP_WH_VL(freq, duty)  // Step 5
```

### 3-Channel Hall Sensor (`ws/hall_sensor_3ch.c/h`)

Three Hall sensors on GPIO EXINT (both edges): PA10(Hall U, INT009), PA9(Hall V, INT010), PA8(Hall W, INT008).

**ISR flow:** Read 3 GPIOs → form 3-bit state (0b000-0b111) → debounce (50μs min interval + unchanged-state skip) → lookup `hall_to_step[state]` (0-5 step, 0xFF=fault) → call `on_step(step, dir)` callback which invokes `Commutation_Step`.

**Hall state sequence (CW rotation):** `0x03 → 0x02 → 0x06 → 0x04 → 0x05 → 0x01 → 0x03`

**Corrected Hall-to-step table (table index 12):**
```
hall_to_step[8] = {0xFF, 1, 5, 0, 3, 2, 4, 0xFF}
//                 0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07
```
Invalid states 0x00 and 0x07 → 0xFF (fault callback).

**Alignment startup:** Apply `align_step` for `align_duration_ms` (pulls rotor to known position), then kick one step forward/reverse, then transition to RUNNING (ISR takes over).

**RPM:** `60,000,000 / (avg_interval_us × pole_pairs × 6)` with 6-sample sliding window. RPM measurement has three layers of noise protection:
1. **ISR-level:** Only valid step transitions (diff = ±1 mod 6) record intervals. Invalid jumps (vibration/noise) are discarded before interval recording.
2. **Dedup:** `last_pulse_id` prevents the same ISR interval from being added to the sliding window multiple times in successive `hall_3ch_update()` calls.
3. **Outlier rejection:** Intervals deviating >4x from the current 6-sample moving average are discarded (motor inertia prevents instantaneous 4x speed changes).

**Runtime Hall table calibration (16 tables in `hall_tables[16][8]`):**

`main.c` contains 16 pre-computed Hall-to-step lookup tables. Mode 3 (CW) uses `hall_tables[hall_table_cw]`, mode 4 (CCW) uses `hall_tables[hall_table_ccw]`. Both indices are independently adjustable in Keil Watch — e.g., change `hall_table_cw=14` and `hall_table_ccw=15` to test CW/CCW without recompiling.

| Index | Purpose |
|-------|---------|
| 0-5 | Synchronous alignment offsets 0-5 (magnetic field CW rotation) |
| 6-11 | Reverse-direction tables (magnetic field CCW rotation) |
| 12 | Empirically corrected CW table from open-loop test data |
| 13 | Empirically corrected CCW table (opposite of 12) |
| 14 | **Lead-angle forward** (sector+90° voltage vector) — **default** |
| 15 | **Lead-angle reverse** (sector-90° voltage vector) |

Table 12 was derived by running open-loop CW and recording which Hall state corresponds to which commutation step. Tables 14/15 apply a +90°/-90° phase advance for better torque at speed (field-oriented control approximation).

The `hall_cfg.hall_to_step` initialization table (`{0xFF,1,3,2,5,0,4,0xFF}`) is only used during alignment startup. After alignment, `hall_3ch_set_table()` replaces it with the selected `hall_tables[index]` before transitioning to RUNNING state. The alignment table is a "cogging torque" alignment sequence; the runtime table is the actual commutation mapping.

**Operating modes** (set by `comm_mode` variable in `main.c`):
| Mode | Description |
|------|-------------|
| 0 | Coast — all 3 phases 98% SYNC (all upper FETs → same potential → no current) |
| 1 | Open-loop forward (timer-driven step sequence, constant ~667 RPM) |
| 2 | Open-loop reverse (same, reversed step direction) |
| 3 | **Open-loop ramp → flying start CW** (2s ramp 167→1111 RPM → read Hall → skip alignment → closed-loop) |
| 4 | **Open-loop ramp → flying start CCW** (same, reversed direction) |

Mode 3/4 use a `comm_sub_phase` state machine:
- **Phase 0** (`comm_sub_phase=0`): Open-loop forced commutation with linear ramp over `OL_RAMP_DURATION_MS` (2000ms). Interval ramps from `OL_START_INTERVAL_US` (20000μs, ~167 RPM) to `OL_TARGET_INTERVAL_US` (3000μs, ~1111 RPM). Uses `g_comm_duty_pct`.
- **Phase 1** (`comm_sub_phase=1`): Ramp complete → `hall_3ch_start_flying()` reads current Hall GPIOs, looks up the corresponding commutation step, sets `last_step`, and enters RUNNING directly — **no alignment, no coast**. Mode 3 uses `hall_tables[hall_table_cw]`, mode 4 uses `hall_tables[hall_table_ccw]`. The Hall ISR takes over seamlessly. Stall detection triggers coast (mode 0).

Modes 1/2 are pure open-loop with fixed 5000μs step interval (COMM_PWM_FREQ_HZ=50kHz). The ramp infrastructure (RAMP_START→RAMP_TARGET over RAMP_DURATION) is present but configured with start=target=5000μs (constant speed).

**Runtime Debug Variables** (modifiable in Keil Watch window without recompiling):
| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `g_comm_duty_pct` | `volatile float` | 80.0 | PWM duty cycle for both open-loop and closed-loop commutation |
| `hall_table_cw` | `volatile int` | 14 | Hall table selector for mode 3 (CW closed-loop) |
| `hall_table_ccw` | `volatile int` | 15 | Hall table selector for mode 4 (CCW closed-loop) |
| `comm_mode` | `volatile int` | 0 | Operating mode (0=coast, 1=open-loop FWD, 2=open-loop REV, 3=closed-loop CW, 4=closed-loop CCW) |

The Hall sensor module also exports debug globals: `g_hall_rpm`, `g_hall_state`, `g_hall_dir`, `g_hall_running`, `g_hall_stalled`, `g_hall_last_step`.

### Motor Ramp Control

Motor speed is controlled via `Motor_RampForward()`/`Motor_RampReverse()` in `Dev/dev_motor.c`. These `__weak` callbacks:
1. Call `Motor_LimitDuty()` to clamp to 2%–98%
2. Switch from stop polarity to run polarity via `Motor_SetRunPolarity()`
3. Kick off a PWM ramp via `PWM_StartRamp_TargetFromStart()`
4. On PWM ramp completion callback, set final duty via `Motor_SetRunDutyDirect()`

### Hall Sensor (`Adp/Motor_hall.c/h`)

- Configuration: `motor_hall_config_t` struct with GPIO, interrupt, pole-pairs, and hall-count fields
- Dual Hall sensors on PA9 (Hall A, EXTINT_CH09, INT009_IRQn) and PA10 (Hall B, EXTINT_CH10, INT010_IRQn)
- Hall count: 2 (`DEFAULT_HALL_COUNT`), pole pairs: 3 (`DEFAULT_POLE_PAIRS`)
- RPM calculation: uses both edges per hall pulse via `CALC_PULSES_PER_REV`

### Hall Sensor IRQ Names (HC32F460)

Correct interrupt names (no underscore between INT and number):
- `INT008_IRQn` for PA8 (EXTINT_CH08)
- `INT009_IRQn` for PA9 (EXTINT_CH09)
- `INT010_IRQn` for PA10 (EXTINT_CH10)

### Key Motor Files

| File | Role |
|------|------|
| `Adp/Pwm.c/h` | PWM driver with ramp (TMRA_4 channels) |
| `Adp/Template_Pwm.c/h` | PWM HAL config helpers |
| `Adp/tmr4_pwm.c/h` | TMR4 3-channel PWM driver (per-channel SYNC/COMP mode switching, shadow registers) |
| `Adp/Motor_hall.c/h` | Hall sensor driver: GPIO interrupts, RPM/direction calculation |
| `Adp/Timer0_Unit1.c/h` | Timer0 unit 1 — timebase and timing |
| `Adp/timer6_timebase.c/h` | TMR6 timebase — free-running μs counter for `tickTimer_GetCount()` |
| `ws/dev_commutation.c/h` | BLDC 6-step commutation state machine (via SDH21263) |
| `ws/hall_sensor_3ch.c/h` | 3-Hall sensor: ISR-driven state lookup, debounce, alignment startup, RPM |
| `Dev/dev_motor.c/h` | Motor arbitrator: block/allow lists, direction arbitration, ramp callbacks |
| `Dev/dev_motor_hall.c/h` | Device-layer wrapper for Hall sensor |
| `Dev/dev_rturn.c/h` | Rotation angle limiter: angle integration, calibration, lock/release |
| `App/App_Motor_Project.c/h` | Hardware pin definitions, device registration, simulation data |
| `Utils/Params.h` | Modbus register map definitions (REG_*) + Flash record layout (`AppParamRecord_t`) |
| `Utils/rtt_manager.h` | Per-module debug macro switches (enable via uncommenting `#define` lines) |
| `RTT/rtt_log.h` | Central debug macros: `MAIN_D()`, `COMM_DBG()`, `HAL_DEBUG()` |

### RTT Debug Switch Mechanism (`Utils/rtt_manager.h`)

Each module has a commented-out `#define MODULE_NAME` line. Uncomment to enable debug output via RTT:

```
// #define DEV_MOTOR       → uncomment to enable motor debug
// #define ADP_RS485_DEBUG → uncomment to enable RS485 debug
#define DEV_SENSOR          → currently enabled (high-frequency sensor output)
```

Also provides `INTERVAL_DECLARE()` macro for rate-limited debug printing to avoid flooding the RTT channel at high frequencies.

### Rotation Angle Limiting (`Dev/dev_rturn.c/h`)

Tracks motor rotation angle by integrating Hall sensor RPM over time. Blocks motor direction when limits are reached, publishes `TOPIC_RTURN_LIMIT`.

**Key parameters** (defined in `App_Motor_Project.h`):
- Reduction ratio: 1183:1 (`RTURN_REDUCTION_RATIO`)
- Max angle: 88.0° (`RTURN_MAX_ANGLE`)
- Min angle: -2.0° (`RTURN_MIN_ANGLE`)
- Update interval: 1ms

**Calibration:** Device starts uncalibrated. On first overcurrent event in the reverse direction, angle snaps to `fMinAngle` (-2.0°) and calibration flag is set. Forward overcurrent before calibration only locks direction — no angle adjustment.

**Lock/release:** Overcurrent alarm (`TOPIC_CURRENT_ALARM`) triggers a directional lock. Lock releases when desired motor direction reverses (e.g., locked forward → command reverse = release). After release, if overcurrent persists in the new direction, it re-locks. Also has software position limit: angle ≥ `fMaxAngle` triggers a forward lock.

**Debug globals** (watchable in Keil): `g_fDbgRTurnAngle`, `g_fDbgRTurnSpeed`, `g_u8DbgRTurnDir`, `g_u8DbgRTurnDesiredDir`, `g_u8DbgRTurnLockedDir`, `g_u8DbgRTurnLockActive`, `g_u8DbgRTurnCalibrated`, `g_u8DbgRTurnLimitTrig`

### GBK File Encoding & Safe Edit Workflow (MANDATORY for .c/.h files)

Source files use GBK encoding for Chinese comments. The Read/Edit/Write tools operate in UTF-8 only — **never edit .c/.h files directly** or Chinese comments will be permanently corrupted. Always use this 3-step process:

Every edit to a `.c` or `.h` file MUST follow this 3-step process:

```bash
# Step 1: Convert GBK → UTF-8 before reading/editing
iconv -f GBK -t UTF-8 path/to/file.c > path/to/file_utf8.c

# Step 2: Edit file_utf8.c using Read/Edit tools (normal UTF-8 editing)

# Step 3: Convert UTF-8 → GBK after editing, then remove temp file
iconv -f UTF-8 -t GBK path/to/file_utf8.c > path/to/file.c
rm path/to/file_utf8.c
```

If `iconv` fails with "cannot convert" on Step 1, the file is already corrupted from a previous bad edit. Restore it from git (`git show <commit>:path`) and re-apply changes with the safe workflow.

**For new files**: Write directly in UTF-8, then convert to GBK with `iconv -f UTF-8 -t GBK`.

## Register Map

The authoritative register definitions are in `Utils/Params.h` — both the `REG_*` address macros and the `AppParamRecord_t` struct that gets persisted to Flash. The register layout:

| Range | Type | Read/Write |
|-------|------|------------|
| 0x2710–0x271F | Persisted params (Flash) | R/W via 0x03/0x06/0x10 |
| 0x2720 | Control command | Write-only via 0x06 |
| 0x2730–0x273F | Realtime data (RAM) | Read-only via 0x03 |
| 0x2740 | Fault status | R/W via 0x06 |

See `Utils/Params.h` for the full list of `REG_*` constants and `AppParamRecord_t` field layout.

## Documentation

- `ws_v.2.1/通信栈架构说明.md` — Full 4-layer communication stack explanation (Chinese)
- `ws_v.2.1/电流控制逻辑说明.md` — Over-current detection flow, dual blocking, fault recovery (Chinese)
- `ws_v.2.1/实时数据使用说明.md` — Real-time data register map and usage (Chinese)
- `ws_v.2.1/六步方波.md` — BLDC six-step square wave commutation: SDH21263 pre-driver, TMR4 per-channel PWM, 3-Hall sensor, operating modes (Chinese)
- `ws_v.2.1/modbus_test_cmds.py` — Generates Modbus RTU hex command frames for testing (`python modbus_test_cmds.py` to see examples)
- `安全审查报告.md` — Security audit of the RS485/Modbus comm stack. Key findings:
  - **Critical:** Potential buffer overrun in `ModbusRTU_SendResponse` if `len >= 255` (CRC write at `raw[len]`/`raw[len+1]`). Current code paths keep `len ≤ 253` but lacks defensive bounds check.
  - **High:** `HAL_FrameParser` silently truncates frames > 256 bytes (acceptable for Modbus RTU max frame size).
  - **Medium:** Every register write triggers `Param_Save()` (Flash erase). Multi-register writes (0x10) batch-save correctly, but repeated single writes cause excessive Flash wear.
  - **Low:** `Comm_HAL_Init` returns silently on NULL config — no `m_bInitialized` flag means `Comm_HAL_Poll` operates on uninitialized data structures.
  - No catastrophic bugs found. All findings are defensive-hardening recommendations.
