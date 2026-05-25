#ifndef __TEMPLATE_TMR4_PWM_H__
#define __TEMPLATE_TMR4_PWM_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
#include "Hardware.h"
#include "hc32_ll.h"

/*******************************************************************************
 * PWM configuration macros
 ******************************************************************************/
#define TMR4_UNIT                       (CM_TMR4_1)
#define TMR4_PERIPH_CLK                 (FCG2_PERIPH_TMR4_1)

/* PWM frequency (Hz) - carrier frequency for motor control */
#define TMR4_PWM_FREQ                   (20000UL)

/* Dead time (in timer clock ticks) - adjust based on MOSFET/IGBT specs */
#define TMR4_DEAD_TIME_NS               (1000U)     /* 1us dead time */

/* 6 PWM output pins: PB4~PB9, GPIO_FUNC_4 for TMR4_1 */
/* Phase U: UH=PB4, UL=PB5 */
#define TMR4_UH_PORT                    (GPIO_PORT_B)
#define TMR4_UH_PIN                     (GPIO_PIN_04)
#define TMR4_UH_PIN_FUNC                (GPIO_FUNC_2)
#define TMR4_UL_PORT                    (GPIO_PORT_B)
#define TMR4_UL_PIN                     (GPIO_PIN_05)
#define TMR4_UL_PIN_FUNC                (GPIO_FUNC_2)

/* Phase V: VH=PB6, VL=PB7 */
#define TMR4_VH_PORT                    (GPIO_PORT_B)
#define TMR4_VH_PIN                     (GPIO_PIN_06)
#define TMR4_VH_PIN_FUNC                (GPIO_FUNC_2)
#define TMR4_VL_PORT                    (GPIO_PORT_B)
#define TMR4_VL_PIN                     (GPIO_PIN_07)
#define TMR4_VL_PIN_FUNC                (GPIO_FUNC_2)

/* Phase W: WH=PB8, WL=PB9 */
#define TMR4_WH_PORT                    (GPIO_PORT_B)
#define TMR4_WH_PIN                     (GPIO_PIN_08)
#define TMR4_WH_PIN_FUNC                (GPIO_FUNC_2)
#define TMR4_WL_PORT                    (GPIO_PORT_B)
#define TMR4_WL_PIN                     (GPIO_PIN_09)
#define TMR4_WL_PIN_FUNC                (GPIO_FUNC_2)

/* Duty cycle range: 0-10000 = 0.00%-100.00% */
#define TMR4_DUTY_MAX                   (10000U)

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/
void TMR4_PWM_Config(void);
void TMR4_PWM_SetDutyU(uint16_t u16Duty);
void TMR4_PWM_SetDutyV(uint16_t u16Duty);
void TMR4_PWM_SetDutyW(uint16_t u16Duty);
void TMR4_PWM_StartOutput(void);
void TMR4_PWM_StopOutput(void);
void TMR4_PWM_EmergencyStop(void);

/* Six-step block commutation */
void TMR4_PWM_CommutationStep(uint8_t hall_state, uint8_t direction);
void TMR4_PWM_SetCommutationDuty(uint16_t u16Duty);
uint16_t TMR4_PWM_GetCommutationDuty(void);
void TMR4_PWM_CommutationStop(void);

#endif /* __TEMPLATE_TMR4_PWM_H__ */
