#ifndef __DEV_COMMUTATION_H__
#define __DEV_COMMUTATION_H__

#include "tmr4_pwm.h"

/*=============================================================================
 * SDH21263 pre-driver truth table (per phase):
 *   HIN=H, LIN=H  -> high-side ON
 *   HIN=L, LIN=L  -> low-side ON
 *   HIN!=LIN      -> interlock (both OFF, built-in dead-time)
 *
 * Six-step commutation states:
 *   State   High-side(PWM)  Low-side(ON)   Current
 *   -----   --------------  ------------   -------
 *   UH_VL   U (HIN=PWM)     V (LIN=L)      U -> V
 *   UH_WL   U (HIN=PWM)     W (LIN=L)      U -> W
 *   VH_WL   V (HIN=PWM)     W (LIN=L)      V -> W
 *   VH_UL   V (HIN=PWM)     U (LIN=L)      V -> U
 *   WH_UL   W (HIN=PWM)     U (LIN=L)      W -> U
 *   WH_VL   W (HIN=PWM)     V (LIN=L)      W -> V
 *=============================================================================*/

/* Pre-driver duty limits */
#define COMM_DUTY_MIN  200U   /* 2% */
#define COMM_DUTY_MAX  9800U  /* 98% */

typedef enum {
    COMM_STATE_UH_VL = 0,
    COMM_STATE_UH_WL = 1,
    COMM_STATE_VH_WL = 2,
    COMM_STATE_VH_UL = 3,
    COMM_STATE_WH_UL = 4,
    COMM_STATE_WH_VL = 5,
} comm_state_t;

/* Enable all 6 output pins, set to safe OFF state (interlock on all phases) */
void Commutation_Init(void);

/* Switch to the specified commutation state with PWM duty (clamped to 2%-98%) */
void Commutation_Step(comm_state_t state, uint16_t duty);

/* Return all phases to interlock OFF state */
void Commutation_Stop(void);

#endif /* __DEV_COMMUTATION_H__ */
