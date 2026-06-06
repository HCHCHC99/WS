#include "dev_commutation.h"
#include "tmr4_pwm.h"

/*=============================================================================
 * Per-pin duty table for each commutation state
 *
 * Pin order: {UH, UL, VH, VL, WH, WL}
 * Value: 0 = LOW, 1 = HIGH, 2 = PWM
 *
 * SDH21263 truth table: H,H→HS ON | L,L→LS ON | H≠L→interlock OFF
 *
 * PWM phase:  HIN & LIN same duty → toggles between H,H(drive) and L,L(sync-rect).
 * ON phase:   both at COMM_DUTY_MIN → mostly L,L → low-side ON.
 * OFF phase:  H pin disabled with INVD_HIGH, L pin disabled with INVD_LOW
 *             → static H,L → guaranteed interlock.
 *=============================================================================*/
static const uint8_t s_pin_map[6][6] = {
    /*         UH  UL  VH  VL  WH  WL */
    [COMM_STATE_UH_VL] = { 2,  2,  0,  0,  2,  2 },  /* TEST: W same as U */
    [COMM_STATE_UH_WL] = { 2,  2,  0,  1,  0,  0 },  /* OFF: V */
    [COMM_STATE_VH_WL] = { 0,  1,  2,  2,  0,  0 },  /* OFF: U */
    [COMM_STATE_VH_UL] = { 0,  0,  2,  2,  0,  1 },  /* OFF: W */
    [COMM_STATE_WH_UL] = { 0,  0,  0,  1,  2,  2 },  /* OFF: V */
    [COMM_STATE_WH_VL] = { 0,  1,  0,  0,  2,  2 },  /* OFF: U */
};

#define PIN_IDX(ch, hs)  (((int)(ch) << 1) | ((hs) ? 0 : 1))
/*  UH=0, UL=1,  VH=2, VL=3,  WH=4, WL=5 */

/*=============================================================================
 * Commutation_Init - Set all phases to interlock OFF via static levels
 *=============================================================================*/
void Commutation_Init(void)
{
    int ch;

    /* All phases: H=HIGH(disabled), L=LOW(disabled) → H,L → interlock */
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_PinSetInvalidLevel((tmr4_pwm_channel_t)ch, true,  true);
        TMR4_PWM_PinSetInvalidLevel((tmr4_pwm_channel_t)ch, false, false);
        TMR4_PWM_PinCmd((tmr4_pwm_channel_t)ch, true,  false);
        TMR4_PWM_PinCmd((tmr4_pwm_channel_t)ch, false, false);
    }
}

/*=============================================================================
 * Commutation_Step - Switch to target commutation state
 *
 * PWM (both=2): restore INVD_LOW, enable both, set same duty.
 * ON  (both=0): restore INVD_LOW, enable both, set COMM_DUTY_MIN.
 * OFF (H=0,L=1): H→INVD_HIGH+disable, L→INVD_LOW+disable → static H,L.
 *=============================================================================*/
void Commutation_Step(comm_state_t state, uint16_t duty)
{
    const uint8_t *pin;
    int ch;

    if (state > COMM_STATE_WH_VL) {
        return;
    }

    if (duty < COMM_DUTY_MIN) {
        duty = COMM_DUTY_MIN;
    }
    if (duty > COMM_DUTY_MAX) {
        duty = COMM_DUTY_MAX;
    }

    pin = s_pin_map[state];

    for (ch = 0; ch < 3; ch++) {
        uint8_t h_val = pin[PIN_IDX(ch, true)];
        uint8_t l_val = pin[PIN_IDX(ch, false)];

        if (h_val == 0U && l_val == 1U) {
            /* --- OFF: static levels via disabled OC --- */
            TMR4_PWM_PinSetInvalidLevel((tmr4_pwm_channel_t)ch, true,  true);
            TMR4_PWM_PinSetInvalidLevel((tmr4_pwm_channel_t)ch, false, false);
            TMR4_PWM_PinCmd((tmr4_pwm_channel_t)ch, true,  false);
            TMR4_PWM_PinCmd((tmr4_pwm_channel_t)ch, false, false);
        } else {
            /* --- PWM or ON: restore normal invalid level, enable, set duty --- */
            TMR4_PWM_PinSetInvalidLevel((tmr4_pwm_channel_t)ch, true,  false);
            TMR4_PWM_PinSetInvalidLevel((tmr4_pwm_channel_t)ch, false, false);

            uint16_t h_duty = (h_val == 2U) ? duty :
                              (h_val == 1U) ? COMM_DUTY_MAX : COMM_DUTY_MIN;
            uint16_t l_duty = (l_val == 2U) ? duty :
                              (l_val == 1U) ? COMM_DUTY_MAX : COMM_DUTY_MIN;

            TMR4_PWM_PinSetDuty((tmr4_pwm_channel_t)ch, true,  h_duty);
            TMR4_PWM_PinSetDuty((tmr4_pwm_channel_t)ch, false, l_duty);

            TMR4_PWM_PinCmd((tmr4_pwm_channel_t)ch, true,  true);
            TMR4_PWM_PinCmd((tmr4_pwm_channel_t)ch, false, true);
        }
    }
}

/*=============================================================================
 * Commutation_Stop - All phases to static interlock OFF
 *=============================================================================*/
void Commutation_Stop(void)
{
    int ch;

    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_PinSetInvalidLevel((tmr4_pwm_channel_t)ch, true,  true);
        TMR4_PWM_PinSetInvalidLevel((tmr4_pwm_channel_t)ch, false, false);
        TMR4_PWM_PinCmd((tmr4_pwm_channel_t)ch, true,  false);
        TMR4_PWM_PinCmd((tmr4_pwm_channel_t)ch, false, false);
    }
}
