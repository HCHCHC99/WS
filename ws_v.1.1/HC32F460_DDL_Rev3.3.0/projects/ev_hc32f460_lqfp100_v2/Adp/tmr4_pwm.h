#ifndef __TMR4_PWM_H__
#define __TMR4_PWM_H__

#include "main.h"

/*=============================================================================
 * TMR4 unit 3 complementary PWM on PB9 (TIM4_3_OUH) and PB8 (TIM4_3_OUL)
 * GPIO_FUNC_2 for both pins
 *=============================================================================*/

/* Duty cycle range: 0-10000 (representing 0.00% - 100.00%) */
#define TMR4_PWM_DUTY_MAX  10000U

/* Output mode */
typedef enum {
    TMR4_OUTPUT_MODE_COMPLEMENTARY = 0,  /* Complementary + dead-time (direct MOSFET drive) */
    TMR4_OUTPUT_MODE_SYNC          = 1,  /* Same signal on both outputs (external gate-driver IC) */
} tmr4_output_mode_t;

/* Configuration structure for one TMR4 PWM channel pair */
typedef struct {
    tmr4_output_mode_t mode;              /* Complementary vs sync */
    uint16_t           polarity;          /* Output polarity, use TMR4_PWM_OXH_*_OXL_* macros */
    uint16_t           freq_hz;           /* PWM frequency in Hz (period auto-calculated from PCLK1) */
    uint16_t           dead_time_rising;  /* PDAR dead-time, unit: PCLK1 ticks */
    uint16_t           dead_time_falling; /* PDBR dead-time, unit: PCLK1 ticks */
} tmr4_pwm_config_t;

/* Initialize TMR4, GPIO, counter, OC channels, PWM mode */
void TMR4_PWM_Config(const tmr4_pwm_config_t *pConfig);

/* Enable PWM outputs (start counter) */
void TMR4_PWM_StartOutput(void);

/* Disable PWM outputs (stop counter + disable OC output) */
void TMR4_PWM_StopOutput(void);

/* Immediate all-off (emergency stop) */
void TMR4_PWM_EmergencyStop(void);

/* Set duty cycle: 0 = 0.00%, 10000 = 100.00% */
void TMR4_PWM_SetDuty(uint16_t u16Duty);

#endif /* __TMR4_PWM_H__ */
