#include "tmr4_pwm.h"
#include "hc32_ll_tmr4.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_clk.h"
#include "hc32_ll_utility.h"
#include "rtt_log.h"
#include <stdbool.h>

static bool               s_bConfigured = false;
static uint16_t           s_u16Period   = 0U;
static tmr4_output_type_t s_output_type = TMR4_OUTPUT_COMPLEMENTARY;

/* Nanoseconds to TMR4 dead-time ticks (PDAR/PDBR unit = timer clock cycle) */
static uint16_t DeadTimeNsToTicks(uint16_t dead_time_ns, uint32_t timer_hz)
{
    return (uint16_t)((uint64_t)dead_time_ns * timer_hz / 1000000000ULL);
}

void TMR4_PWM_Config(const tmr4_pwm_config_t *pConfig)
{
    stc_tmr4_init_t stcTmr4Init;
    stc_tmr4_oc_init_t stcTmr4OcInit;
    stc_tmr4_pwm_init_t stcTmr4PwmInit;
    un_tmr4_oc_ocmrl_t unOcmrl;
    un_tmr4_oc_ocmrh_t unOcmrh;
    uint32_t u32TimerClock;
    uint16_t u16Polarity;

    if (pConfig == NULL) {
        return;
    }

    s_output_type = pConfig->output_type;

    u32TimerClock = CLK_GetBusClockFreq(CLK_BUS_PCLK1);
    s_u16Period  = (uint16_t)(u32TimerClock / pConfig->freq_hz);

    MAIN_D("[TMR4] mode=%s clk=%lu Hz period=%u ticks",
           (pConfig->output_type == TMR4_OUTPUT_COMPLEMENTARY) ? "COMP" : "SYNC",
           u32TimerClock, s_u16Period);

    /* Map active_high to TMR4 polarity macros */
    if (pConfig->active_high) {
        u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
    } else {
        u16Polarity = TMR4_PWM_OXH_INVT_OXL_INVT;
    }

    /* Enable TMR4_3 peripheral clock */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_3, ENABLE);

    /* GPIO: PB9=TIM4_3_OUH(func2), PB8=TIM4_3_OUL(func2) */
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_2);
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_2);
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /************************* Counter *************************/
    TMR4_StructInit(&stcTmr4Init);
    stcTmr4Init.u16ClockDiv = TMR4_CLK_DIV1;

    /* Triangle wave: one PWM cycle = 2 * period ticks */
    stcTmr4Init.u16CountMode = TMR4_MD_TRIANGLE;
    s_u16Period = (uint16_t)(u32TimerClock / (pConfig->freq_hz * 2U));
    stcTmr4Init.u16PeriodValue = s_u16Period - 1U;
    TMR4_Init(CM_TMR4_3, &stcTmr4Init);

    /************************* OC channels *************************/
    TMR4_OC_StructInit(&stcTmr4OcInit);
    stcTmr4OcInit.u16CompareValue = 0U;

    if (pConfig->output_type == TMR4_OUTPUT_COMPLEMENTARY) {
        /* Dead-timer mode: UL only, PWM block auto-generates UH */
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

        TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UL, ENABLE);

    } else {
        /* Through mode: per HC32 official example timer4_pwm_through */
        stcTmr4OcInit.u16CompareValue = stcTmr4Init.u16PeriodValue / 2U;
        stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;

        TMR4_OC_Init(CM_TMR4_3, TMR4_OC_CH_UH, &stcTmr4OcInit);
        TMR4_OC_Init(CM_TMR4_3, TMR4_OC_CH_UL, &stcTmr4OcInit);

        /* UH compare mode: 0x225F - center-aligned PWM */
        unOcmrh.OCMRx = 0U;
        unOcmrh.OCMRx_f.OCFDCH = TMR4_OC_OCF_SET;
        unOcmrh.OCMRx_f.OCFPKH = TMR4_OC_OCF_SET;
        unOcmrh.OCMRx_f.OCFUCH = TMR4_OC_OCF_SET;
        unOcmrh.OCMRx_f.OCFZRH = TMR4_OC_OCF_SET;
        unOcmrh.OCMRx_f.OPDCH  = TMR4_OC_HIGH;
        unOcmrh.OCMRx_f.OPPKH  = TMR4_OC_HIGH;
        unOcmrh.OCMRx_f.OPUCH  = TMR4_OC_LOW;
        unOcmrh.OCMRx_f.OPZRH  = TMR4_OC_HOLD;
        unOcmrh.OCMRx_f.OPNPKH = TMR4_OC_LOW;
        unOcmrh.OCMRx_f.OPNZRH = TMR4_OC_HOLD;
        TMR4_OC_SetHighChCompareMode(CM_TMR4_3, TMR4_OC_CH_UH, unOcmrh);

        /* UL compare mode: 0x2250_225F - same as UH via EOP fields */
        unOcmrl.OCMRx = 0U;
        unOcmrl.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET;
        unOcmrl.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET;
        unOcmrl.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET;
        unOcmrl.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET;
        unOcmrl.OCMRx_f.OPDCL   = TMR4_OC_HIGH;
        unOcmrl.OCMRx_f.OPPKL   = TMR4_OC_HIGH;
        unOcmrl.OCMRx_f.OPUCL   = TMR4_OC_LOW;
        unOcmrl.OCMRx_f.OPZRL   = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.OPNPKL  = TMR4_OC_LOW;
        unOcmrl.OCMRx_f.OPNZRL  = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.EOPDCL  = TMR4_OC_HIGH;
        unOcmrl.OCMRx_f.EOPPKL  = TMR4_OC_HIGH;
        unOcmrl.OCMRx_f.EOPUCL  = TMR4_OC_LOW;
        unOcmrl.OCMRx_f.EOPZRL  = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.EOPNPKL = TMR4_OC_LOW;
        unOcmrl.OCMRx_f.EOPNZRL = TMR4_OC_HOLD;
        TMR4_OC_SetLowChCompareMode(CM_TMR4_3, TMR4_OC_CH_UL, unOcmrl);

        TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UH, ENABLE);
        TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UL, ENABLE);
    }

    /************************* PWM mode *************************/
    TMR4_PWM_StructInit(&stcTmr4PwmInit);
    stcTmr4PwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;

    if (pConfig->output_type == TMR4_OUTPUT_COMPLEMENTARY) {
        stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_DEAD_TMR;
    } else {
        stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_THROUGH;
    }
    TMR4_PWM_Init(CM_TMR4_3, TMR4_PWM_CH_U, &stcTmr4PwmInit);

    /* Set output polarity */
    TMR4_PWM_SetPolarity(CM_TMR4_3, TMR4_PWM_CH_U, u16Polarity);

    /* Dead-time only applies in complementary mode */
    if (pConfig->output_type == TMR4_OUTPUT_COMPLEMENTARY) {
        uint16_t u16DeadTicks = DeadTimeNsToTicks(pConfig->dead_time_ns, u32TimerClock);
        TMR4_PWM_SetDeadTimeValue(CM_TMR4_3, TMR4_PWM_CH_U,
                                  TMR4_PWM_PDAR_IDX, u16DeadTicks);
        TMR4_PWM_SetDeadTimeValue(CM_TMR4_3, TMR4_PWM_CH_U,
                                  TMR4_PWM_PDBR_IDX, u16DeadTicks);
    }

    s_bConfigured = true;
    MAIN_D("[TMR4] Config done: mode=%s count=%s period=%u polar=%s",
           (pConfig->output_type == TMR4_OUTPUT_COMPLEMENTARY) ? "COMP" : "SYNC",
           (stcTmr4Init.u16CountMode == TMR4_MD_TRIANGLE) ? "TRI" : "SAW",
           s_u16Period,
           pConfig->active_high ? "HIGH" : "LOW");
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
    if (s_output_type == TMR4_OUTPUT_SYNC) {
        TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UH, DISABLE);
    }
    TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UL, DISABLE);
}

void TMR4_PWM_EmergencyStop(void)
{
    if (!s_bConfigured) {
        return;
    }
    TMR4_Stop(CM_TMR4_3);
    if (s_output_type == TMR4_OUTPUT_SYNC) {
        TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UH, DISABLE);
    }
    TMR4_OC_Cmd(CM_TMR4_3, TMR4_OC_CH_UL, DISABLE);
    TMR4_ClearCountValue(CM_TMR4_3);
    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 0U);
    if (s_output_type == TMR4_OUTPUT_SYNC) {
        TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 0U);
    }
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

    u16Compare = (uint16_t)(((uint32_t)s_u16Period * u16Duty) /
                            TMR4_PWM_DUTY_MAX);

    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, u16Compare);
    if (s_output_type == TMR4_OUTPUT_SYNC) {
        TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, u16Compare);
    }
}
