# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Motor control system based on HC32F460 (Cortex-M4) with RS485/Modbus RTU communication. Controls DC motors with Hall sensor feedback, over-current/voltage detection, rotation angle limiting, and fault handling.

All source code lives under `ws_v.1.1/HC32F460_DDL_Rev3.3.0/`.

## Build System

- **Primary IDE:** Keil MDK (uVision 5)
- **Project file:** `ws_v.1.1/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template.uvprojx`
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

The startup flow in `main.c`:

```
Hardware_Init()           → SysTick, TMR0 timers, AOS trigger chains, GPIO
App_Comm_Init(&comm_cfg)  → 4-layer comm stack (RS485 + Modbus RTU)
ESystem_Init()            → DeviceManager + EventBus + all 16 devices registered
FaultHandler_Init()       → Subscribe to TOPIC_VOLTAGE_ALARM + TOPIC_CURRENT_ALARM
TMR4_PWM_Config(&pwm_cfg) → TMR4 PWM at 50kHz, SYNC output, active-high
EventBus_Enable()         → Unblock deferred publishes, replay queued events
// Super loop:
while (1) {
    ESystem_MainLoop();   // DeviceManager update scheduling
    App_Comm_Poll();      // Modbus frame processing
    TMR4_PWM_SetDuty();   // PWM duty update
}
```

`ESystem_Init()` is defined in `App/App_Motor_Project.c` and wires up all device instances, EventBus subscriptions, and callbacks.

## Important Constraints

- Flash erase/write cycles are limited (~10K-100K). Each `Param_Save()` triggers a sector erase. Avoid calling it per-register in multi-register writes (0x10) — the batch write path calls `Param_Save()` once for the entire batch
- Interrupt safety: Comm_HAL uses `__disable_irq()`/`__enable_irq()` around ring buffer reads. Keep critical sections short
- `ModbusRTU_ProcessFrame` expects `len <= 256`. Frame buffer is 256 bytes. Modbus RTU max frame is 256 bytes so this is safe
- RS485 direction pin polarity is configurable via `dir_polarity` (0 = high-TX/low-RX, 1 = low-TX/high-RX)
- Realtime data (`g_RealTimeData`) is RAM-only, **not persisted to Flash** — lost on power cycle
- Multi-register writes (0x10) reject batches that include `REG_CTRL_CMD` or `REG_FAULT_STATUS` — those must use single writes (0x06)

## Motor Control Architecture

### PWM Output

The motor has **two** PWM timer configurations:

**TMR4 PWM (active in current `main.c`):** 50kHz center-aligned PWM on PB8/PB9 via `Adp/tmr4_pwm.c/h`. Configured with `TMR4_OUTPUT_SYNC` mode for external gate-driver IC with built-in dead-time. Set via `TMR4_PWM_SetDuty(2500)` for 25% duty in the super loop.

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

TMR4 unit 3 (CM_TMR4_3) on PB8/PB9. Two output modes, configurable via a single struct:

```c
typedef struct {
    tmr4_output_type_t output_type;    // TMR4_OUTPUT_COMPLEMENTARY or TMR4_OUTPUT_SYNC
    uint16_t           freq_hz;        // PWM frequency in Hz
    uint16_t           dead_time_ns;   // Dead-time in nanoseconds (complementary mode only)
    bool               active_high;    // true = active high, false = active low
} tmr4_pwm_config_t;

void TMR4_PWM_Config(const tmr4_pwm_config_t *cfg);
void TMR4_PWM_StartOutput(void);
void TMR4_PWM_StopOutput(void);
void TMR4_PWM_EmergencyStop(void);
void TMR4_PWM_SetDuty(uint16_t u16Duty);              // 0-10000 = 0.00%-100.00%
```

**Output types and dead-time behavior:**

| output_type | PWM mode | Dead-time | Use case |
|---|---|---|---|
| `COMPLEMENTARY` | `DEAD_TMR` | `dead_time_ns` → PDAR/PDBR | Direct MOSFET drive, or pre-driver IC without built-in dead-time |
| `SYNC` | `THROUGH` | Ignored (hardware doesn't support) | External gate-driver IC with built-in dead-time, or optocoupler |

**Internals:**
- Counter: `TMR4_MD_TRIANGLE` (center-aligned). Period formula: `PCLK1 / (freq_hz × 2)`.
- Dead-time conversion: `ticks = dead_time_ns × PCLK1 / 1e9`. Reads PCLK1 via `CLK_GetBusClockFreq(CLK_BUS_PCLK1)` at Config time — immune to Sysclk.h macro changes.
- Polarity: `active_high=true` → `OXH_HOLD_OXL_HOLD`, `false` → `OXH_INVT_OXL_INVT`.
- SYNC mode OC configuration follows HC32 official example `timer4_pwm_through` (UH compare mode 0x225F, UL compare mode 0x2250_225F, buffer cond `PEAK`).
- Reference: `F:/HC32F460_folder/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/examples/timer4/timer4_pwm_through/`

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
| `Adp/tmr4_pwm.c/h` | TMR4 complementary PWM (standalone driver) |
| `Adp/Motor_hall.c/h` | Hall sensor driver: GPIO interrupts, RPM/direction calculation |
| `Adp/Timer0_Unit1.c/h` | Timer0 unit 1 — timebase and timing |
| `Adp/timer6_timebase.c/h` | TMR6 timebase — free-running μs counter for `tickTimer_GetCount()` |
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

- `ws_v.1.1/通信栈架构说明.md` — Full 4-layer communication stack explanation (Chinese)
- `ws_v.1.1/电流控制逻辑说明.md` — Over-current detection flow, dual blocking, fault recovery (Chinese)
- `ws_v.1.1/实时数据使用说明.md` — Real-time data register map and usage (Chinese)
- `ws_v.1.1/modbus_test_cmds.py` — Generates Modbus RTU hex command frames for testing (`python modbus_test_cmds.py` to see examples)
