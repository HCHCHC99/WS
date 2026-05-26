#ifndef __TMR4_PWM_H__
#define __TMR4_PWM_H__

#include "main.h"

/*=============================================================================
 * TMR4 unit 3 complementary PWM on PB9 (TIM4_3_OUH) and PB8 (TIM4_3_OUL)
 * GPIO_FUNC_2 for both pins
 *=============================================================================*/

/* Duty cycle range: 0-10000 (representing 0.00% - 100.00%) */
#define TMR4_PWM_DUTY_MAX  10000U

/* Initialize TMR4, GPIO, counter, OC channels, PWM dead-timer mode */
void TMR4_PWM_Config(void);

/* Enable PWM outputs (start counter + enable OC output) */
void TMR4_PWM_StartOutput(void);

/* Disable PWM outputs (stop counter + disable OC output) */
void TMR4_PWM_StopOutput(void);

/* Immediate all-off (emergency stop) */
void TMR4_PWM_EmergencyStop(void);

/* Set duty cycle: 0 = 0.00%, 10000 = 100.00% */
void TMR4_PWM_SetDuty(uint16_t u16Duty);

#endif /* __TMR4_PWM_H__ */
