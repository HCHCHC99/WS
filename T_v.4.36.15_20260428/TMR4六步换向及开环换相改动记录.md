# TMR4 Six-Step Commutation & Open-Loop Changes

> Date: 2026-05-26
> Based on commit: `55bd302` (add six-step commutation + triple Hall)

## 1. Files Changed

| File | Description |
|------|-------------|
| `Adp/Template_tmr4_pwm.c` | Added open-loop commutation step logic |
| `Adp/Template_tmr4_pwm.h` | Added two function prototypes |
| `App/App_Motor_Project.h` | Added open-loop control macros and timing config macros |
| `Dev/dev_motor.c` | Completed TMR4 changes + three-way commutation branch |

---

## 2. Template_tmr4_pwm.c Changes

### 2.1 New variable (line ~378)

```c
static uint8_t  s_u8OpenLoopStepIndex;  // current step index 0..5 (open-loop mode)
```

### 2.2 New step-to-Hall mapping tables

```c
// Forward: step index -> hall_state
static const uint8_t s_au8StepToHallFwd[6] = { 5, 4, 6, 2, 3, 1 };

// Reverse: step index -> hall_state
static const uint8_t s_au8StepToHallRev[6] = { 1, 3, 2, 6, 4, 5 };
```

### 2.3 New functions

```c
// Reset open-loop sequence (called on direction change or stop)
void TMR4_PWM_CommutationResetSequence(void)
{
    s_u8OpenLoopStepIndex = 0;
    s_u8LastHallState = 0;
    s_u8LastDirection = 0;
}

// Advance to the next commutation step
void TMR4_PWM_CommutationNextStep(uint8_t direction)
{
    // direction: 1=forward, 2=reverse
    // Step index auto-increments (wraps 0..5), maps to hall_state via table,
    // calls TMR4_PWM_CommutationStep()
}
```

### 2.4 TMR4_PWM_CommutationStop change

Now also resets `s_u8OpenLoopStepIndex = 0` on stop.

---

## 3. Template_tmr4_pwm.h Changes

New prototypes:

```c
void TMR4_PWM_CommutationNextStep(uint8_t direction);
void TMR4_PWM_CommutationResetSequence(void);
```

---

## 4. App_Motor_Project.h Changes

### 4.1 New macros

```c
// Commutation mode: 0=Hall-dependent, 1=open-loop sensorless
#ifndef MOTOR_COMMUTATION_SENSORLESS
#define MOTOR_COMMUTATION_SENSORLESS  0
#endif

// Open-loop step interval (only effective when MOTOR_COMMUTATION_SENSORLESS=1)
#define COMM_STEP_INTERVAL_MIN_US   800   // min step interval (us), for max speed
#define COMM_STEP_INTERVAL_MAX_US   5000  // max step interval (us), for min speed
```

### 4.2 Interval calculation

Interval scales linearly with duty cycle — higher duty = shorter interval:

```
interval_ms = MAX_US/1000 - ((MAX_US - MIN_US)/1000) * duty / 100
```

- duty=100% → 0.8ms (fastest commutation)
- duty=0% → 5ms (slowest commutation)

---

## 5. dev_motor.c Changes

### 5.1 Completed TMR4 changes (deferred from previous session)

| Area | Change |
|------|--------|
| Includes | `#if MOTOR_HALL_TRIPLE_ENABLE` pulls in `Template_tmr4_pwm.h`/`Motor_hall.h`, `#else` keeps original TMRA headers |
| extern pwm_t | Wrapped in `#if !MOTOR_HALL_TRIPLE_ENABLE` |
| Motor_OnArbitrationStop | TMR4 version calls `TMR4_PWM_CommutationStop()`, original in `#else` |
| Motor_OnArbitrationFwd | TMR4 version calls `TMR4_PWM_SetCommutationDuty(duty * 100U)`, original in `#else` |
| Motor_OnArbitrationRev | Same as above |
| Motor_Init | TMRA register sync wrapped in `#if !MOTOR_HALL_TRIPLE_ENABLE` |

### 5.2 New static variables

```c
static uint32_t s_u32LastCommStepTime = 0;  // timestamp of last commutation step (ms)
static uint8_t  s_u8CommDirCache = 0;        // cached commutation direction
```

### 5.3 Motor_Update three-way branch

```c
#if MOTOR_HALL_TRIPLE_ENABLE
  #if MOTOR_COMMUTATION_SENSORLESS
      // Open-loop: compute interval from duty, call TMR4_PWM_CommutationNextStep() on timer
      // On stop: call TMR4_PWM_CommutationResetSequence()
  #else
      // Hall-dependent: read PA8/PA9/PA10 GPIOs, call TMR4_PWM_CommutationStep(hall_state, dir)
  #endif
#else
  // Original TMRA: PWM_Update + stop polarity check
#endif
```

---

## 6. Three Compilation Paths

| MOTOR_HALL_TRIPLE_ENABLE | MOTOR_COMMUTATION_SENSORLESS | Commutation method |
|--------------------------|------------------------------|--------------------|
| 0 | (ignored) | Original TMRA_4 PWM |
| 1 | 0 | TMR4 + Hall sensor GPIO read |
| 1 | 1 | TMR4 + open-loop timed advance |

---

## 7. Known Limitations

1. **Dual-Hall fallback uses separate path**: `MOTOR_HALL_TRIPLE_ENABLE=0` uses the original TMRA PWM path (in `#else` branch), which works independently
2. **Hall state 000/111**: Commutation is skipped when all sensors read the same (guarded by `hall_state >= 1 && hall_state <= 6`)
3. **Source file encoding**: Files use GBK encoding. When matching with regex, prefer ASCII-only patterns
4. **HC32F460 IRQ naming**: Correct form is `INT008_IRQn` (no underscore between INT and 008)
