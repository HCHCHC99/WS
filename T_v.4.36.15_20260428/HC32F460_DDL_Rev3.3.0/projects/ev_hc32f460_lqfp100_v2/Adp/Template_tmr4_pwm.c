#include "Template_tmr4_pwm.h"

/*******************************************************************************
 * Static variables
 ******************************************************************************/
static uint32_t s_u32PeriodValue;   /* PWM period value (timer counts) */
static uint32_t s_u32TimerClk;      /* Timer clock frequency (Hz) */
static uint16_t s_u16DeadTimeTicks; /* Dead time in timer clock ticks */
static bool    s_bInitialized;      /* Init flag */

/*******************************************************************************
 * Helper: calculate dead time register value from nanoseconds
 ******************************************************************************/
static uint16_t TMR4_CalcDeadTime(uint32_t u32DeadTimeNs)
{
    /* Dead time = (PDAR_value) / TimerClock */
    /* PDAR_value = DeadTime_ns * TimerClock_Hz / 1e9 */
    uint32_t u32Ticks = (u32DeadTimeNs * s_u32TimerClk) / 1000000000UL;
    if (u32Ticks > 0xFFFFU) {
        u32Ticks = 0xFFFFU;
    }
    if (u32Ticks < 1U) {
        u32Ticks = 1U;
    }
    return (uint16_t)u32Ticks;
}

/*******************************************************************************
 * Helper: configure OC high channel compare mode (center-aligned, active-high)
 ******************************************************************************/
static void TMR4_ConfigOCHigh(CM_TMR4_TypeDef *TMR4x, uint32_t u32Ch)
{
    un_tmr4_oc_ocmrh_t unOcmrh;

    /* Read current high channel mode register */
    unOcmrh.OCMRx = TMR4_OC_GetHighChCompareMode(TMR4x, u32Ch);

    /* Center-aligned, active-high PWM on high side:
     * - Counting up match:   set OP high
     * - Counting down match: set OP low
     * - At peak (no match):  hold
     * - At zero (no match):  hold
     * - When match doesn't occur at peak: hold
     * - When match doesn't occur at zero: hold
     */
    unOcmrh.OCMRx_f.OPUCH  = TMR4_OC_HIGH;   /* up match -> high */
    unOcmrh.OCMRx_f.OPDCH  = TMR4_OC_LOW;    /* down match -> low */
    unOcmrh.OCMRx_f.OPPKH  = TMR4_OC_HOLD;   /* at peak -> hold */
    unOcmrh.OCMRx_f.OPZRH  = TMR4_OC_HOLD;   /* at zero -> hold */
    unOcmrh.OCMRx_f.OPNPKH = TMR4_OC_HOLD;   /* no match at peak -> hold */
    unOcmrh.OCMRx_f.OPNZRH = TMR4_OC_HOLD;   /* no match at zero -> hold */

    /* Disable OCF flags for clean operation */
    unOcmrh.OCMRx_f.OCFDCH = TMR4_OC_OCF_HOLD;
    unOcmrh.OCMRx_f.OCFPKH = TMR4_OC_OCF_HOLD;
    unOcmrh.OCMRx_f.OCFUCH = TMR4_OC_OCF_HOLD;
    unOcmrh.OCMRx_f.OCFZRH = TMR4_OC_OCF_HOLD;

    TMR4_OC_SetHighChCompareMode(TMR4x, u32Ch, unOcmrh);
}

/*******************************************************************************
 * Helper: configure OC low channel compare mode (active-low, complementary)
 ******************************************************************************/
static void TMR4_ConfigOCLow(CM_TMR4_TypeDef *TMR4x, uint32_t u32Ch)
{
    un_tmr4_oc_ocmrl_t unOcmrl;

    /* Read current low channel mode register */
    unOcmrl.OCMRx = TMR4_OC_GetLowChCompareMode(TMR4x, u32Ch);

    /* Low side: complementary to high side
     * - Counting down (high OFF, low ON):  set OP high
     * - Counting up (high ON, low OFF): set OP low
     * Active-low operation for low-side MOSFET
     */
    unOcmrl.OCMRx_f.OPDCL   = TMR4_OC_HIGH;   /* down match -> low-side ON */
    unOcmrl.OCMRx_f.OPUCL   = TMR4_OC_LOW;    /* up match -> low-side OFF */
    unOcmrl.OCMRx_f.OPPKL   = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.OPZRL   = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.OPNPKL  = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.OPNZRL  = TMR4_OC_HOLD;

    /* High matches & low doesn't match: hold (dead timer manages this) */
    unOcmrl.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;

    /* Both high and low match: hold */
    unOcmrl.OCMRx_f.EOPDCL  = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPPKL  = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPUCL  = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPZRL  = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPNPKL = TMR4_OC_HOLD;
    unOcmrl.OCMRx_f.EOPNZRL = TMR4_OC_HOLD;

    /* Disable OCF flags for clean operation */
    unOcmrl.OCMRx_f.OCFDCL  = TMR4_OC_OCF_HOLD;
    unOcmrl.OCMRx_f.OCFPKL  = TMR4_OC_OCF_HOLD;
    unOcmrl.OCMRx_f.OCFUCL  = TMR4_OC_OCF_HOLD;
    unOcmrl.OCMRx_f.OCFZRL  = TMR4_OC_OCF_HOLD;

    TMR4_OC_SetLowChCompareMode(TMR4x, u32Ch, unOcmrl);
}

/*******************************************************************************
 * Helper: init one OC channel (high or low side)
 ******************************************************************************/
static void TMR4_InitOCChannel(CM_TMR4_TypeDef *TMR4x, uint32_t u32Ch)
{
    stc_tmr4_oc_init_t stcOcInit;

    (void)TMR4_OC_StructInit(&stcOcInit);
    stcOcInit.u16CompareValue        = 0U;
    stcOcInit.u16OcInvalidPolarity   = TMR4_OC_INVD_LOW;
    stcOcInit.u16CompareModeBufCond  = TMR4_OC_BUF_COND_PEAK_VALLEY;
    stcOcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK_VALLEY;
    stcOcInit.u16BufLinkTransObject  = (TMR4_OC_BUF_CMP_VALUE | TMR4_OC_BUF_CMP_MD);
    (void)TMR4_OC_Init(TMR4x, u32Ch, &stcOcInit);
}

/*******************************************************************************
 * Helper: init one PWM couple channel (U/V/W)
 ******************************************************************************/
static void TMR4_InitPWMCouple(CM_TMR4_TypeDef *TMR4x, uint32_t u32Ch)
{
    stc_tmr4_pwm_init_t stcPwmInit;

    (void)TMR4_PWM_StructInit(&stcPwmInit);
    stcPwmInit.u16Mode     = TMR4_PWM_MD_DEAD_TMR;
    stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
    stcPwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_INVT;
    (void)TMR4_PWM_Init(TMR4x, u32Ch, &stcPwmInit);

    /* Set dead time for both PDAR and PDBR */
    TMR4_PWM_SetDeadTimeValue(TMR4x, u32Ch, TMR4_PWM_PDAR_IDX, s_u16DeadTimeTicks);
    TMR4_PWM_SetDeadTimeValue(TMR4x, u32Ch, TMR4_PWM_PDBR_IDX, s_u16DeadTimeTicks);
}

/*******************************************************************************
 * Helper: configure GPIO pin for TMR4 output
 ******************************************************************************/
static void TMR4_ConfigPin(uint8_t u8Port, uint16_t u16Pin, uint16_t u16Func)
{
    GPIO_SetFunc(u8Port, u16Pin, u16Func);
}

/*******************************************************************************
 * TMR4_PWM_Config - Main initialization for 3-phase center-aligned PWM
 *
 * Configures TMR4_1 for 6-channel brushless motor PWM output on PB4~PB9:
 *   PB4=UH, PB5=UL, PB6=VH, PB7=VL, PB8=WH, PB9=WL
 *
 * Center-aligned (triangle wave) with dead timer insertion.
 * All outputs start disabled (0% duty).
 ******************************************************************************/
void TMR4_PWM_Config(void)
{
    stc_tmr4_init_t     stcTmr4Init;
    stc_clock_freq_t    stcClkFreq;

    /* Get PCLK1 frequency (TMR4 uses PCLK1) */
    (void)CLK_GetClockFreq(&stcClkFreq);
    s_u32TimerClk = stcClkFreq.u32Pclk1Freq;

    /* Calculate period for triangle mode: period = PCLK / (2 * PWM_FREQ) - 1 */
    s_u32PeriodValue = s_u32TimerClk / (TMR4_PWM_FREQ * 2UL) - 1UL;

    /* Calculate dead time ticks */
    s_u16DeadTimeTicks = TMR4_CalcDeadTime(TMR4_DEAD_TIME_NS);

    /* ---- Enable TMR4 peripheral clock ---- */
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg2PeriphClockCmd(TMR4_PERIPH_CLK, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    /* ---- Configure TMR4 counter ---- */
    (void)TMR4_StructInit(&stcTmr4Init);
    stcTmr4Init.u16ClockSrc    = TMR4_CLK_SRC_INTERNCLK;
    stcTmr4Init.u16ClockDiv    = TMR4_CLK_DIV1;
    stcTmr4Init.u16CountMode   = TMR4_MD_TRIANGLE;    /* Center-aligned */
    stcTmr4Init.u16PeriodValue = (uint16_t)s_u32PeriodValue;
    (void)TMR4_Init(TMR4_UNIT, &stcTmr4Init);

    /* ---- Configure GPIO pins for 6 PWM outputs ---- */
    LL_PERIPH_WE(LL_PERIPH_GPIO);

    TMR4_ConfigPin(TMR4_UH_PORT, TMR4_UH_PIN, TMR4_UH_PIN_FUNC);  /* PB4: UH */
    TMR4_ConfigPin(TMR4_UL_PORT, TMR4_UL_PIN, TMR4_UL_PIN_FUNC);  /* PB5: UL */
    TMR4_ConfigPin(TMR4_VH_PORT, TMR4_VH_PIN, TMR4_VH_PIN_FUNC);  /* PB6: VH */
    TMR4_ConfigPin(TMR4_VL_PORT, TMR4_VL_PIN, TMR4_VL_PIN_FUNC);  /* PB7: VL */
    TMR4_ConfigPin(TMR4_WH_PORT, TMR4_WH_PIN, TMR4_WH_PIN_FUNC);  /* PB8: WH */
    TMR4_ConfigPin(TMR4_WL_PORT, TMR4_WL_PIN, TMR4_WL_PIN_FUNC);  /* PB9: WL */

    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /* ---- Init OC channels (6 channels: UH/UL/VH/VL/WH/WL) ---- */
    TMR4_InitOCChannel(TMR4_UNIT, TMR4_OC_CH_UH);
    TMR4_InitOCChannel(TMR4_UNIT, TMR4_OC_CH_UL);
    TMR4_InitOCChannel(TMR4_UNIT, TMR4_OC_CH_VH);
    TMR4_InitOCChannel(TMR4_UNIT, TMR4_OC_CH_VL);
    TMR4_InitOCChannel(TMR4_UNIT, TMR4_OC_CH_WH);
    TMR4_InitOCChannel(TMR4_UNIT, TMR4_OC_CH_WL);

    /* ---- Configure OC compare modes for center-aligned complementary PWM ---- */
    /* High-side: active-high, center-aligned */
    TMR4_ConfigOCHigh(TMR4_UNIT, TMR4_OC_CH_UH);
    TMR4_ConfigOCHigh(TMR4_UNIT, TMR4_OC_CH_VH);
    TMR4_ConfigOCHigh(TMR4_UNIT, TMR4_OC_CH_WH);

    /* Low-side: active-low, complementary */
    TMR4_ConfigOCLow(TMR4_UNIT, TMR4_OC_CH_UL);
    TMR4_ConfigOCLow(TMR4_UNIT, TMR4_OC_CH_VL);
    TMR4_ConfigOCLow(TMR4_UNIT, TMR4_OC_CH_WL);

    /* ---- Init PWM couple channels (U/V/W) with dead timer ---- */
    TMR4_InitPWMCouple(TMR4_UNIT, TMR4_PWM_CH_U);
    TMR4_InitPWMCouple(TMR4_UNIT, TMR4_PWM_CH_V);
    TMR4_InitPWMCouple(TMR4_UNIT, TMR4_PWM_CH_W);

    /* ---- Set all compare values to 0 (motor stopped) ---- */
    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_UH, 0U);
    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_UL, 0U);
    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_VH, 0U);
    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_VL, 0U);
    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_WH, 0U);
    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_WL, 0U);

    /* ---- Set comparator interval (summit/valley) for triangle mode ---- */
    TMR4_SetCountValue(TMR4_UNIT, 1UL);

    /* ---- Start PWM reload timers ---- */
    TMR4_PWM_StartReloadTimer(TMR4_UNIT, TMR4_PWM_CH_U);
    TMR4_PWM_StartReloadTimer(TMR4_UNIT, TMR4_PWM_CH_V);
    TMR4_PWM_StartReloadTimer(TMR4_UNIT, TMR4_PWM_CH_W);

    /* ---- Enable all OC outputs ---- */
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_UH, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_UL, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_VH, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_VL, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_WH, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_WL, ENABLE);

    /* ---- Start counter ---- */
    TMR4_Start(TMR4_UNIT);

    s_bInitialized = true;

    MAIN_D("TMR4 PWM initialized: 3-phase, %luHz center-aligned, dead time=%luns\r\n",
           TMR4_PWM_FREQ, TMR4_DEAD_TIME_NS);
}

/*******************************************************************************
 * TMR4_PWM_SetDutyU - Set U-phase duty cycle
 *   u16Duty: 0~10000 = 0.00%~100.00%
 ******************************************************************************/
void TMR4_PWM_SetDutyU(uint16_t u16Duty)
{
    uint16_t u16Compare;

    if (u16Duty > TMR4_DUTY_MAX) {
        u16Duty = TMR4_DUTY_MAX;
    }
    u16Compare = (uint16_t)(((uint32_t)s_u32PeriodValue * u16Duty) / TMR4_DUTY_MAX);

    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_UH, u16Compare);
    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_UL, u16Compare);
}

/*******************************************************************************
 * TMR4_PWM_SetDutyV - Set V-phase duty cycle
 ******************************************************************************/
void TMR4_PWM_SetDutyV(uint16_t u16Duty)
{
    uint16_t u16Compare;

    if (u16Duty > TMR4_DUTY_MAX) {
        u16Duty = TMR4_DUTY_MAX;
    }
    u16Compare = (uint16_t)(((uint32_t)s_u32PeriodValue * u16Duty) / TMR4_DUTY_MAX);

    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_VH, u16Compare);
    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_VL, u16Compare);
}

/*******************************************************************************
 * TMR4_PWM_SetDutyW - Set W-phase duty cycle
 ******************************************************************************/
void TMR4_PWM_SetDutyW(uint16_t u16Duty)
{
    uint16_t u16Compare;

    if (u16Duty > TMR4_DUTY_MAX) {
        u16Duty = TMR4_DUTY_MAX;
    }
    u16Compare = (uint16_t)(((uint32_t)s_u32PeriodValue * u16Duty) / TMR4_DUTY_MAX);

    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_WH, u16Compare);
    TMR4_OC_SetCompareValue(TMR4_UNIT, TMR4_OC_CH_WL, u16Compare);
}

/*******************************************************************************
 * TMR4_PWM_StartOutput - Enable PWM outputs (after init they're already on)
 *   Call after TMR4_PWM_Config() if outputs were stopped via StopOutput()
 ******************************************************************************/
void TMR4_PWM_StartOutput(void)
{
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_UH, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_UL, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_VH, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_VL, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_WH, ENABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_WL, ENABLE);
}

/*******************************************************************************
 * TMR4_PWM_StopOutput - Disable all PWM outputs (idle state)
 ******************************************************************************/
void TMR4_PWM_StopOutput(void)
{
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_UH, DISABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_UL, DISABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_VH, DISABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_VL, DISABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_WH, DISABLE);
    TMR4_OC_Cmd(TMR4_UNIT, TMR4_OC_CH_WL, DISABLE);
}

/*******************************************************************************
 * TMR4_PWM_EmergencyStop - Immediately shut down all PWM outputs and stop counter
 ******************************************************************************/
void TMR4_PWM_EmergencyStop(void)
{
    TMR4_Stop(TMR4_UNIT);
    TMR4_PWM_StopOutput();
    MAIN_D("TMR4 PWM emergency stop\r\n");
}

/*******************************************************************************
 * Six-step block commutation
 *
 * Hall state (3-bit: C,B,A) -> commutation sector 1..6
 * Forward: 101(5)->100(4)->110(6)->010(2)->011(3)->001(1)
 * For each sector, one high-side and one low-side OC channel are PWM-driven,
 * all others are off (compare=0). Dead timer handles complementary switching.
 ******************************************************************************/

typedef struct {
    uint32_t high_ch;
    uint32_t low_ch;
} tmr4_comm_step_t;

/* Forward commutation table indexed by hall_state [1..6] */
static const tmr4_comm_step_t s_comm_fwd[8] = {
    [0] = { 0, 0 },
    [1] = { TMR4_OC_CH_VH, TMR4_OC_CH_WL },  /* 001: V -> W */
    [2] = { TMR4_OC_CH_WH, TMR4_OC_CH_UL },  /* 010: W -> U */
    [3] = { TMR4_OC_CH_VH, TMR4_OC_CH_UL },  /* 011: V -> U */
    [4] = { TMR4_OC_CH_UH, TMR4_OC_CH_VL },  /* 100: U -> V */
    [5] = { TMR4_OC_CH_UH, TMR4_OC_CH_WL },  /* 101: U -> W */
    [6] = { TMR4_OC_CH_WH, TMR4_OC_CH_VL },  /* 110: W -> V */
};

/* Reverse commutation table indexed by hall_state [1..6] */
static const tmr4_comm_step_t s_comm_rev[8] = {
    [0] = { 0, 0 },
    [1] = { TMR4_OC_CH_UH, TMR4_OC_CH_WL },  /* 001 -> fwd[5]: U -> W */
    [2] = { TMR4_OC_CH_VH, TMR4_OC_CH_UL },  /* 010 -> fwd[3]: V -> U */
    [3] = { TMR4_OC_CH_VH, TMR4_OC_CH_WL },  /* 011 -> fwd[1]: V -> W */
    [4] = { TMR4_OC_CH_WH, TMR4_OC_CH_VL },  /* 100 -> fwd[6]: W -> V */
    [5] = { TMR4_OC_CH_UH, TMR4_OC_CH_VL },  /* 101 -> fwd[4]: U -> V */
    [6] = { TMR4_OC_CH_WH, TMR4_OC_CH_UL },  /* 110 -> fwd[2]: W -> U */
};

static uint16_t s_u16CommutationDuty;   /* current commutation duty 0..10000 */
static uint8_t  s_u8LastHallState;      /* last hall state for change detection */
static uint8_t  s_u8LastDirection;      /* last direction */
static uint8_t  s_u8OpenLoopStepIndex;  /* current step index 0..5 for open-loop */

/* All OC channels in order for easy iteration */
static const uint32_t s_au32AllOcChannels[6] = {
    TMR4_OC_CH_UH, TMR4_OC_CH_UL,
    TMR4_OC_CH_VH, TMR4_OC_CH_VL,
    TMR4_OC_CH_WH, TMR4_OC_CH_WL,
};

static void TMR4_PWM_ClearAllCompareValues(void)
{
    uint8_t i;
    for (i = 0; i < 6; i++) {
        TMR4_OC_SetCompareValue(TMR4_UNIT, s_au32AllOcChannels[i], 0U);
    }
}

void TMR4_PWM_SetCommutationDuty(uint16_t u16Duty)
{
    if (u16Duty > TMR4_DUTY_MAX) {
        u16Duty = TMR4_DUTY_MAX;
    }
    s_u16CommutationDuty = u16Duty;
}

uint16_t TMR4_PWM_GetCommutationDuty(void)
{
    return s_u16CommutationDuty;
}

void TMR4_PWM_CommutationStep(uint8_t hall_state, uint8_t direction)
{
    const tmr4_comm_step_t *step = NULL;
    uint16_t u16Compare;

    /* Clamp hall_state to 1..6 */
    if (hall_state < 1 || hall_state > 6) {
        return;
    }

    /* No change? Skip. */
    if (hall_state == s_u8LastHallState && direction == s_u8LastDirection) {
        return;
    }
    s_u8LastHallState = hall_state;
    s_u8LastDirection = direction;

    if (direction == 0) {
        /* Stop: all off */
        TMR4_PWM_ClearAllCompareValues();
        return;
    }

    if (direction == 1) {
        step = &s_comm_fwd[hall_state];
    } else {
        step = &s_comm_rev[hall_state];
    }

    if (step->high_ch == 0 && step->low_ch == 0) {
        return;
    }

    u16Compare = (uint16_t)(((uint32_t)s_u32PeriodValue * s_u16CommutationDuty) / TMR4_DUTY_MAX);

    /* Clear all first, then set only the active pair */
    TMR4_PWM_ClearAllCompareValues();
    TMR4_OC_SetCompareValue(TMR4_UNIT, step->high_ch, u16Compare);
    TMR4_OC_SetCompareValue(TMR4_UNIT, step->low_ch,  u16Compare);
}

void TMR4_PWM_CommutationStop(void)
{
    TMR4_PWM_ClearAllCompareValues();
    s_u8LastHallState = 0;
    s_u8LastDirection = 0;
    s_u8OpenLoopStepIndex = 0;
}

/*******************************************************************************
 * Open-loop (sensorless) commutation: advance to the next step in sequence
 *
 * Forward sequence: hall_states 5,4,6,2,3,1
 * Reverse sequence: hall_states 1,3,2,6,4,5
 ******************************************************************************/

/* Step index -> hall_state mapping for forward direction */
static const uint8_t s_au8StepToHallFwd[6] = { 5, 4, 6, 2, 3, 1 };

/* Step index -> hall_state mapping for reverse direction */
static const uint8_t s_au8StepToHallRev[6] = { 1, 3, 2, 6, 4, 5 };

void TMR4_PWM_CommutationResetSequence(void)
{
    s_u8OpenLoopStepIndex = 0;
    s_u8LastHallState = 0;
    s_u8LastDirection = 0;
}

void TMR4_PWM_CommutationNextStep(uint8_t direction)
{
    uint8_t hall_state;
    const uint8_t *seq;

    if (direction == 1) {
        seq = s_au8StepToHallFwd;
    } else if (direction == 2) {
        seq = s_au8StepToHallRev;
    } else {
        TMR4_PWM_CommutationStop();
        return;
    }

    /* Advance step index (wrap 0..5) */
    s_u8OpenLoopStepIndex++;
    if (s_u8OpenLoopStepIndex >= 6) {
        s_u8OpenLoopStepIndex = 0;
    }

    hall_state = seq[s_u8OpenLoopStepIndex];
    TMR4_PWM_CommutationStep(hall_state, direction);
}
