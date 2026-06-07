#include "dev_commutation.h"
#include "tmr4_pwm.h"

/*=============================================================================
 * Commutation state table
 *
 * Each row = { U_mode, U_duty_flag,  V_mode, V_duty_flag,  W_mode, W_duty_flag }
 *
 * mode:  0 = SYNC (PWM/ON),  1 = COMPLEMENTARY (OFF)
 * duty:  D = PWM duty (from parameter),
 *        L = COMM_DUTY_MIN (2%),
 *        O = COMM_DUTY_OFF (50% complementary)
 *=============================================================================*/
enum { M_SYNC = 0, M_COMP = 1 };
enum { D_PWM = 0, D_MIN = 1, D_OFF = 2 };

static const uint8_t s_states[6][6] = {
    /*         U_mode   U_duty  V_mode  V_duty  W_mode  W_duty */
    /* UH_VL */ { M_SYNC, D_PWM, M_SYNC, D_MIN, M_COMP, D_OFF },
    /* UH_WL */ { M_SYNC, D_PWM, M_COMP, D_OFF, M_SYNC, D_MIN },
    /* VH_WL */ { M_COMP, D_OFF, M_SYNC, D_PWM, M_SYNC, D_MIN },
    /* VH_UL */ { M_SYNC, D_MIN, M_SYNC, D_PWM, M_COMP, D_OFF },
    /* WH_UL */ { M_SYNC, D_MIN, M_COMP, D_OFF, M_SYNC, D_PWM },
    /* WH_VL */ { M_COMP, D_OFF, M_SYNC, D_MIN, M_SYNC, D_PWM },
};

/* Last applied state: 0xFF = never applied, forces first call to configure */
static uint8_t  s_last_state = 0xFFU;
static uint16_t s_last_freq  = 0U;
static float    s_last_duty  = 0.0f;

/*=============================================================================
 * Commutation_Init - All 3 phases to complementary OFF
 *=============================================================================*/
void Commutation_Init(void)
{
    int ch;
    s_last_state = 0xFFU;  /* force next Commutation_Step to apply */
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch,
                                TMR4_MODE_COMPLEMENTARY, COMM_DUTY_OFF_F);
    }
}

/*=============================================================================
 * Commutation_Step
 *=============================================================================*/
void Commutation_Step(uint8_t state, uint16_t freq_hz, float duty_pct)
{
    int ch;

    if (state > 5U) {
        return;
    }

    /* Clamp duty to 2%~98% */
    if (duty_pct < COMM_DUTY_MIN_F) {
        duty_pct = COMM_DUTY_MIN_F;
    }
    if (duty_pct > COMM_DUTY_MAX_F) {
        duty_pct = COMM_DUTY_MAX_F;
    }

    /* Skip if nothing changed */
    if (state == s_last_state && freq_hz == s_last_freq && duty_pct == s_last_duty) {
        return;
    }
    s_last_state = state;
    s_last_freq  = freq_hz;
    s_last_duty  = duty_pct;

    /* Update frequency first (affects all channels) */
    TMR4_PWM_SetFrequency(freq_hz);

    /* Apply per-channel mode + duty */
    for (ch = 0; ch < 3; ch++) {
        uint8_t mode  = s_states[state][ch * 2U];
        uint8_t dflag = s_states[state][ch * 2U + 1U];
        float ch_duty;

        if (dflag == D_PWM) {
            ch_duty = duty_pct;
        } else if (dflag == D_MIN) {
            ch_duty = COMM_DUTY_MIN_F;
        } else {
            ch_duty = COMM_DUTY_OFF_F;
        }

        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch,
            (mode == M_COMP) ? TMR4_MODE_COMPLEMENTARY : TMR4_MODE_SYNC,
            ch_duty);
    }
}

/*=============================================================================
 * Commutation_Stop - All phases to complementary OFF
 *=============================================================================*/
void Commutation_Stop(void)
{
    int ch;
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch,
                                TMR4_MODE_COMPLEMENTARY, COMM_DUTY_OFF_F);
    }
}
