#include "tmr4_pwm.h"
#include "hc32_ll_tmr4.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_utility.h"
#include "rtt_log.h"
#include <stdbool.h>

/*=============================================================================
 * Timing parameters
 * PCLK2 = 100MHz, clock div = 1
 * PWM frequency: 20kHz -> period = 100MHz / 20000 = 5000
 * Dead time: ~1us (100 * 10ns at 100MHz PCLK)
 *=============================================================================*/
#define TMR4_PWM_PERIOD      5000U
#define TMR4_PWM_DEAD_TIME   100U

static bool s_bConfigured = false;

/*=============================================================================
 * Public API — follows official timer4_pwm_dead_timer example structure
 *
 * Key rules learned from the official example:
 *   1. Only configure the LOW (UL) OC channel — UH is driven by PWM block
 *   2. Enable OC (TMR4_OC_Cmd) BEFORE initializing PWM (TMR4_PWM_Init)
 *   3. Do NOT call StartReloadTimer or ExtendControlCmd
 *   4. Use default POCR polarity (HOLD_HOLD) — PWM block handles complement
 *=============================================================================*/

void TMR4_PWM_Config(void)
{
    stc_tmr4_init_t stcTmr4Init;
    stc_tmr4_oc_init_t stcTmr4OcInit;
    stc_tmr4_pwm_init_t stcTmr4PwmInit;
    un_tmr4_oc_ocmrl_t unOcmrl;

    /* Enable TMR4_3 peripheral clock */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_3, ENABLE);

    /* GPIO: PB9=TIM4_3_OUH(func3), PB8=TIM4_3_OUL(func3) */
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_3);
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_3);
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /************************* Counter *************************/
    TMR4_StructInit(&stcTmr4Init);
    stcTmr4Init.u16ClockDiv    = TMR4_CLK_DIV1;
    stcTmr4Init.u16PeriodValue = TMR4_PWM_PERIOD - 1U;
    TMR4_Init(CM_TMR4_3, &stcTmr4Init);

    /************************* OC low channel (UL) *************************/
    TMR4_OC_StructInit(&stcTmr4OcInit);
    stcTmr4OcInit.u16CompareValue = 0U;
    TMR4_OC_Init(CM_TMR4_3, TMR4_OC_CH_UL, &stcTmr4OcInit);

    unOcmrl.OCMRx = 0U;
    unOcmrl.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET;
    unOcmrl.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET;
    unOcmrl.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET;
    unOcmrl.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET;
    unOcmrl.OCMRx_f.OPDCL   = TMR4_OC_INVT;
    unOcmrl.OCMRx_f.OPPKL   = TMR4_OC_INVT;
    unOcmrl.OCMRx_f.OPUCL   = TMR4_OC_INVT;
    unOcmrl.OCMRx_f.OPZRL   = TMR4_OC_INVT;
    unOcmrl.OCMRx_f.OPNPKL  = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.OPNZRL  = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPDCL  = TMR4_OC_INVT;
    unOcmrl.OCMRx_f.EOPPKL  = TMR4_OC_INVT;
    unOcmrl.OCMRx_f.EOPUCL  = TMR4_OC_INVT;
    unOcmrl.OCMRx_f.EOPZRL  = TMR4_OC_INVT;
    unOcmrl.OCMRx_f.EOPNPKL = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPNZRL = TMR4_OC_HOLD;
    TMR4_OC_SetLowChCompareMode(CM_TMR4_3, TMR4_OC_CH_UL, unOcmrl);

    /* Enable OC BEFORE PWM init (matches official example order) */
    TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UL, ENABLE);

    /************************* PWM dead-timer mode *************************/
    TMR4_PWM_StructInit(&stcTmr4PwmInit);
    stcTmr4PwmInit.u16Mode     = TMR4_PWM_MD_DEAD_TMR;
    stcTmr4PwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
    TMR4_PWM_Init(CM_TMR4_3, TMR4_PWM_CH_U, &stcTmr4PwmInit);

    TMR4_PWM_SetDeadTimeValue(CM_TMR4_3, TMR4_PWM_CH_U,
                              TMR4_PWM_PDAR_IDX, TMR4_PWM_DEAD_TIME);
    TMR4_PWM_SetDeadTimeValue(CM_TMR4_3, TMR4_PWM_CH_U,
                              TMR4_PWM_PDBR_IDX, TMR4_PWM_DEAD_TIME);

    s_bConfigured = true;
    MAIN_D("[TMR4_PWM] PB9=OUH, PB8=OUL, 20kHz, dead-time=%d ticks\r\n",
           TMR4_PWM_DEAD_TIME);
}

void TMR4_PWM_StartOutput(void)
{
    if (!s_bConfigured) {
        return;
    }
    TMR4_Start(CM_TMR4_3);
}

void TMR4_PWM_StopOutput(void)
{
    if (!s_bConfigured) {
        return;
    }
    TMR4_Stop(CM_TMR4_3);
    TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UL, DISABLE);
}

void TMR4_PWM_EmergencyStop(void)
{
    if (!s_bConfigured) {
        return;
    }
    TMR4_Stop(CM_TMR4_3);
    TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UL, DISABLE);
    TMR4_ClearCountValue(CM_TMR4_3);
    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 0U);
}

void TMR4_PWM_SetDuty(uint16_t u16Duty)
{
    uint16_t u16Compare;

    if (!s_bConfigured) {
        return;
    }

    if (u16Duty > TMR4_PWM_DUTY_MAX) {
        u16Duty = TMR4_PWM_DUTY_MAX;
    }

    u16Compare = (uint16_t)(((uint32_t)TMR4_PWM_PERIOD * u16Duty) /
                            TMR4_PWM_DUTY_MAX);

    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, u16Compare);
}
