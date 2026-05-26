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

- **EventBus** (`Dev/EventBus.h`): Publish/subscribe for inter-module communication. 14 topics including `TOPIC_MANUAL_IO` (motor commands), `TOPIC_VOLTAGE_ALARM`, `TOPIC_CURRENT_ALARM`, `TOPIC_RTURN_LIMIT`. Uses deferred publish — events published before `EventBus_Enable()` are queued as a bitmask and replayed on enable. Max 4 subscribers per topic, priority-ordered (0 = highest).

- **DeviceManager** (`Dev/device_manager.h`): Uniform device registry with time-sliced update scheduling. Each device gets an `update()` callback called at its configured interval (typically 1ms, voltage bus at 10ms). Mutex-protected access. 16 devices registered (motor arbitrator, power outputs, Hall switches, IO buttons, PWM, ADC channels, sensors).

- **Param Manager** (`Utils/param_manager.h`): Register-based parameter storage with Flash persistence. Parameters live in `g_AppParam` (type `AppParamRecord_t`). Read/write via `Param_ReadByReg()`/`Param_WriteByReg()`, save via `Param_Save()`. Uses wear-leveled Flash storage across sectors 56-62 with CRC32 validation, sequence numbers, and magic headers.

- **Motor arbitration** (`Dev/dev_motor.c`): Commands go through the motor arbitrator which decides whether to allow based on mode (auto/remote/manual). Uses a `block_fwd`/`block_rev` bitmask — multiple devices can independently block a direction (e.g., overcurrent adds `DEV_ID_OVERCUR_FWD`, limit switches add `DEV_ID_RTURN_FWD`). Arbitration re-evaluates on every block/unblock.

- **Simulation mode** (`ENABLE_SIMULATION_MODE=1` in `App_Motor_Project.c`): Enabled by default. The `g_sim` struct holds simulated hardware signals (power, Hall limits, IO buttons, ADC values). The main loop detects state changes on `g_sim` and publishes corresponding EventBus events, allowing full motor arbitration testing without physical hardware.

- **SEGGER RTT** (`RTT/`): Debug logging via `MAIN_D()`, `COMM_DBG()`, `HAL_DEBUG()` macros — these output through RTT, not UART.

## Fault System

Fault bits (stored in `g_RealTimeData.fault_status`, readable via Modbus register 0x2740):

| Bit | Macro | Description |
|-----|-------|-------------|
| bit0 | `FAULT_BIT_OVERVOLTAGE` | Overvoltage |
| bit2 | `FAULT_BIT_OVERCURRENT` | Overcurrent |
| bit4 | `FAULT_BIT_OVERTEMP` | Overtemp |
| bit6 | `FAULT_BIT_UNDERVOLTAGE` | Undervoltage |
| bit5 | `FAULT_BIT_STALL` | Motor stall |

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

## Important Constraints

- Flash erase/write cycles are limited (~10K-100K). Each `Param_Save()` triggers a sector erase. Avoid calling it per-register in multi-register writes (0x10) — the batch write path calls `Param_Save()` once for the entire batch
- Interrupt safety: Comm_HAL uses `__disable_irq()`/`__enable_irq()` around ring buffer reads. Keep critical sections short
- `ModbusRTU_ProcessFrame` expects `len <= 256`. Frame buffer is 256 bytes. Modbus RTU max frame is 256 bytes so this is safe
- RS485 direction pin polarity is configurable via `dir_polarity` (0 = high-TX/low-RX, 1 = low-TX/high-RX)
- Realtime data (`g_RealTimeData`) is RAM-only, **not persisted to Flash** — lost on power cycle
- Multi-register writes (0x10) reject batches that include `REG_CTRL_CMD` or `REG_FAULT_STATUS` — those must use single writes (0x06)

## BLDC Motor Control — TMR4 Six-Step Commutation

The motor control layer uses TMR4 timer (CM_TMR4_1) for six-step block commutation. This replaced the original TMRA_4 edge-aligned PWM. TMR4 provides 6 complementary output channels (UH/UL, VH/VL, WH/WL) on PB4-PB9 with dead-time insertion.

### Mode Selection Macros (`App_Motor_Project.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `MOTOR_HALL_TRIPLE_ENABLE` | `0` (in `motor_hall.h`) | `0` = dual Hall (PA9, PA10), `1` = triple Hall (PA8, PA9, PA10). Gates all Hall C code and commutation path |
| `MOTOR_COMMUTATION_SENSORLESS` | `0` | `0` = Hall-dependent commutation, `1` = open-loop timed commutation. Only active when `MOTOR_HALL_TRIPLE_ENABLE=1` |
| `ENABLE_SIMULATION_MODE` | `1` | Simulation mode (no physical hardware needed) |

**Three compilation paths** controlled by these macros:

```
MOTOR_HALL_TRIPLE_ENABLE=0:  Original TMRA_4 PWM (PWM_Update + Motor_RampForward/Reverse)
MOTOR_HALL_TRIPLE_ENABLE=1 + MOTOR_COMMUTATION_SENSORLESS=0:  TMR4 + Hall GPIO read (PA8/9/10)
MOTOR_HALL_TRIPLE_ENABLE=1 + MOTOR_COMMUTATION_SENSORLESS=1:  TMR4 + timed step advance (open-loop)
```

### Open-Loop Timing Config

When `MOTOR_COMMUTATION_SENSORLESS=1`, commutation step interval scales linearly with duty cycle:
- `COMM_STEP_INTERVAL_MIN_US` (800us) → used at 100% duty (fastest)
- `COMM_STEP_INTERVAL_MAX_US` (5000us) → used at 0% duty (slowest)
- Interval = MAX_US/1000 - ((MAX_US - MIN_US)/1000) * duty / 100 (in ms ticks)

### Key Files

| File | Role |
|------|------|
| `Adp/Template_tmr4_pwm.c/h` | TMR4 PWM config + six-step commutation tables + open-loop step advance |
| `Adp/Motor_hall.c/h` | Hall sensor driver: GPIO interrupts on PA8/9/10, RPM/direction calculation |
| `Dev/dev_motor_hall.c/h` | Device-layer wrapper for Hall sensor, `MotorHall_Device_t` struct |
| `Dev/dev_motor.c` | Motor arbitrator: `Motor_Update()` dispatches to TMR4 or TMRA based on macros |
| `App/App_Motor_Project.c/h` | Hardware pin definitions, device registration, simulation data |

### Commutation Tables (`Template_tmr4_pwm.c`)

Forward sequence (hall_state order): **5→4→6→2→3→1**
```
Hall 101(5): U→W    Hall 100(4): U→V    Hall 110(6): W→V
Hall 010(2): W→U    Hall 011(3): V→U    Hall 001(1): V→W
```

Reverse sequence (hall_state order): **1→3→2→6→4→5** (reversed pairs)

Open-loop `TMR4_PWM_CommutationNextStep(direction)` advances through a step index (0-5) and maps to the corresponding hall_state to reuse the same `TMR4_PWM_CommutationStep()` function.

### TMR4 API Summary

```c
void TMR4_PWM_Config(void);                          // Init TMR4, GPIOs, dead-time
void TMR4_PWM_StartOutput(void);                     // Enable all 6 channels
void TMR4_PWM_StopOutput(void);                      // Disable all outputs
void TMR4_PWM_EmergencyStop(void);                   // Immediate all-off
void TMR4_PWM_CommutationStep(hall_state, dir);      // Set one commutation sector
void TMR4_PWM_SetCommutationDuty(uint16_t duty);      // duty 0-10000 (0.00%-100.00%)
void TMR4_PWM_CommutationStop(void);                  // Clear all, reset state
void TMR4_PWM_CommutationNextStep(uint8_t dir);       // Open-loop: advance to next step
void TMR4_PWM_CommutationResetSequence(void);         // Open-loop: reset step index
```

### Hall Sensor IRQ Names (HC32F460)

Correct interrupt names (no underscore between INT and number):
- `INT008_IRQn` for PA8 (EXTINT_CH08)
- `INT009_IRQn` for PA9 (EXTINT_CH09)
- `INT010_IRQn` for PA10 (EXTINT_CH10)

### Known Limitations

- **Dual Hall fallback is broken**: When `MOTOR_HALL_TRIPLE_ENABLE=0`, `Motor_OnArbitrationFwd/Rev` still call `Motor_RampForward/Reverse`, but the TMRA path in `Motor_Update` remains functional via `#else` blocks. The callbacks are wrapped in `#if MOTOR_HALL_TRIPLE_ENABLE` / `#else` preserving both paths.
- **Hall state 000/111**: When Hall sensors fail (all high or all low), commutation is skipped (guarded by `hall_state >= 1 && hall_state <= 6`).
- **File encoding**: Source files use GBK encoding for Chinese comments. The Read/Edit/Write tools operate in UTF-8 only — **never edit .c/.h files directly** or Chinese comments will be permanently corrupted. Always use the GBK-safe edit workflow below.

### GBK-Safe Edit Workflow (MANDATORY for .c/.h files)

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

## Documentation

- `ws_v.1.1/通信栈架构说明.md` — Full 4-layer communication stack explanation (Chinese)
- `ws_v.1.1/电流控制逻辑说明.md` — Over-current detection flow, dual blocking, fault recovery (Chinese)
- `ws_v.1.1/实时数据使用说明.md` — Real-time data register map and usage (Chinese)
- `ws_v.1.1/modbus_test_cmds.py` — Generates Modbus RTU hex command frames
