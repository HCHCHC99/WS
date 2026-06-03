#ifndef __TMR4_PWM_H__
#define __TMR4_PWM_H__

#include "main.h"

/*=============================================================================
 * TMR4 unit 3 complementary PWM on PB9 (TIM4_3_OUH) and PB8 (TIM4_3_OUL)
 * GPIO_FUNC_2 for both pins
 *=============================================================================*/

/* Duty cycle range: 0-10000 (representing 0.00% - 100.00%) */
#define TMR4_PWM_DUTY_MAX  10000U

/* Output type */
typedef enum {
    TMR4_OUTPUT_COMPLEMENTARY = 0,  /* Complementary PWM + hardware dead-time */
    TMR4_OUTPUT_SYNC          = 1,  /* Same signal on both outputs (external driver handles dead-time) */
} tmr4_output_type_t;

/* Configuration structure for one TMR4 PWM channel pair */
typedef struct {
    tmr4_output_type_t output_type;    /* Complementary or sync */
    uint16_t           freq_hz;        /* PWM frequency in Hz (period auto-calculated from PCLK1) */
    uint16_t           dead_time_ns;   /* Dead-time in nanoseconds (only effective in COMPLEMENTARY mode) */
    bool               active_high;    /* true = active high, false = active low */
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
