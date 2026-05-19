# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Motor control system based on HC32F460 (Cortex-M4) with RS485/Modbus RTU communication. Controls DC motors with Hall sensor feedback, over-current/voltage detection, rotation angle limiting, and fault handling.

All source code lives under `T_v.4.36.15_20260428/HC32F460_DDL_Rev3.3.0/`.

## Build System

- **IDE:** Keil MDK (uVision 5)
- **Project file:** `T_v.4.36.15_20260428/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template.uvprojx`
- **MCU:** HC32F460xE (512KB Flash) — linker script is `template/MDK/config/linker/HC32F460xE.sct`
- **Debug probe:** JLink (Cortex-M4)
- **Build output:** `template/MDK/output/debug/` (`.axf`, `.hex`, `.bin`, `.map`)
- Open the `.uvprojx` in Keil, click Build (F7), or use `UV4.exe -r template.uvprojx -o output.txt` from command line

No IAR or GCC toolchain is configured for this project.

## Source Tree

```
projects/ev_hc32f460_lqfp100_v2/
├── Adp/          # Hardware adapter layer (rs485, PWM, ADC, DMA, GPIO, timers, flash)
├── App/          # Application logic (motor control, comm, fault, realtime)
├── Dev/          # Device drivers (motor, ADC, hall, voltage, sensor, EventBus)
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

- **EventBus** (`Dev/EventBus.h`): Publish/subscribe for inter-module communication. Topics include `TOPIC_MANUAL_IO` (motor commands), fault events, voltage/current events
- **Param Manager** (`Utils/param_manager.h`): Register-based parameter storage with Flash persistence. Parameters live in `g_AppParam` (type `AppParamRecord_t`). Read/write via `Param_ReadByReg()`/`Param_WriteByReg()`, save via `Param_Save()`
- **Motor arbitration** (`App_Motor_Project`): Commands go through `Motor_OnArbitrationFwd/Rev/Stop()` which decides whether to allow based on mode (auto/remote/manual)
- **SEGGER RTT** (`RTT/`): Debug logging via `MAIN_D()`, `COMM_DBG()`, `HAL_DEBUG()` macros — these output through RTT, not UART

## Important Constraints

- Flash erase/write cycles are limited (~10K-100K). Each `Param_Save()` triggers a sector erase. Avoid calling it per-register in multi-register writes (0x10)
- Interrupt safety: Comm_HAL uses `__disable_irq()`/`__enable_irq()` around ring buffer reads. Keep critical sections short
- `ModbusRTU_ProcessFrame` expects `len <= 256`. Frame buffer is 256 bytes. Modbus RTU max frame is 256 bytes so this is safe
- RS485 direction pin polarity is configurable via `dir_polarity` (0 = high-TX/low-RX, 1 = low-TX/high-RX)

## Documentation

- `T_v.4.36.15_20260428/通信栈架构说明.md` — Full 4-layer architecture explanation (Chinese)
- `T_v.4.36.15_20260428/电流控制逻辑说明.md` — Over-current detection flow (Chinese)
- `T_v.4.36.15_20260428/实时数据使用说明.md` — Real-time data register map (Chinese)
- `T_v.4.36.15_20260428/modbus_test_cmds.py` — Modbus test command generator
