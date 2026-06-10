#include "dev_comm_runner.h"
#include "dev_commutation.h"
#include "tmr4_pwm.h"
#include "timer6_timebase.h"
#include "rtt_log.h"
#include <string.h>

/*=============================================================================
 * Hall-to-step lookup tables.
 *
 * Physical Hall sequence (measured on this motor):
 *   CW:  0x02 -> 0x06 -> 0x04 -> 0x05 -> 0x01 -> 0x03  (decreasing sector#)
 *   CCW: 0x01 -> 0x05 -> 0x04 -> 0x06 -> 0x02 -> 0x03  (increasing sector#)
 *
 * For this motor, CW = decreasing angle = needs sector-90deg (reverse_map).
 * CCW = increasing angle = needs sector+90deg (forward_map).
 * So CW mode uses reverse_map, CCW mode uses forward_map �� they are SWAPPED
 * relative to their names because of the motor's Hall mounting orientation.
 *=============================================================================*/
static const uint8_t s_hall2step_cw[8]  = {0xFF, 5, 3, 4, 1, 0, 2, 0xFF};  /* reverse_map: sector -90deg */
static const uint8_t s_hall2step_ccw[8] = {0xFF, 2, 0, 1, 4, 3, 5, 0xFF};  /* forward_map: sector +90deg */

/*=============================================================================
 * State
 *=============================================================================*/
static comm_runner_config_t s_cfg;
static hall_3ch_handle_t    s_hall     = NULL;
static comm_runner_mode_t   s_mode     = COMM_RUNNER_STOP;
static float                s_duty     = 80.0f;
static int                  s_comm_step = 0;
static int                  s_sub_phase = 0;
static uint8_t              s_initialized = 0;
static volatile uint8_t     s_fault_pending = 0;

/* ��定时变量 (mode 1/2 恒� & mode 3/4 飞启共用��) */
static uint64_t s_ol_ramp_start_us  = 0;
static uint64_t s_ol_last_step_us   = 0;
static uint32_t s_ol_interval_us    = 0;
static uint32_t s_ol_start_interval = 0;
static uint32_t s_ol_target_interval = 0;
static uint32_t s_ol_ramp_duration_ms = 0;

/*=============================================================================
 * Hall 回调 (ISR 上下�)
 *=============================================================================*/
static void runner_on_hall_step(uint8_t step, hall3_direction_t dir)
{
    (void)dir;
    Commutation_Step(step, s_cfg.pwm_freq_hz, s_duty);
}

static void runner_on_hall_fault(uint8_t hall_state)
{
    (void)hall_state;
    /* ISR 上下�: ��标志, 不执行停� (避免 ISR 内大量寄存器操作 + 重入风险) */
    s_fault_pending = 1;
}

/*=============================================================================
 * 内部: �动开�强拖 (mode 1/2 � mode 3/4 飞启阶�共�)
 *=============================================================================*/
static void start_open_loop(uint32_t start_interval, uint32_t target_interval,
                            uint32_t ramp_ms, int dir_fw)
{
    s_ol_start_interval   = start_interval;
    s_ol_target_interval  = target_interval;
    s_ol_ramp_duration_ms = ramp_ms;

    s_comm_step = 0;
    s_ol_ramp_start_us = Timer6_Timebase_GetTimestamp();
    s_ol_interval_us   = start_interval;
    s_ol_last_step_us  = s_ol_ramp_start_us;

    Commutation_Init();
    /* ���: UH_VL */
    COMM_STEP_UH_VL(s_cfg.pwm_freq_hz, s_duty);

    if (dir_fw) {
        MAIN_D("[CommRunner] Open-loop FW start: %lu->%lu us, ramp=%lu ms",
               start_interval, target_interval, ramp_ms);
    } else {
        MAIN_D("[CommRunner] Open-loop RV start: %lu->%lu us, ramp=%lu ms",
               start_interval, target_interval, ramp_ms);
    }
}

/*=============================================================================
 * 内部: ��斜坡计算 + 定时换相 (� Update 调用)
 *=============================================================================*/
static void open_loop_tick(uint64_t now, int dir_fw)
{
    uint64_t ramp_elapsed = now - s_ol_ramp_start_us;
    uint64_t ramp_total   = (uint64_t)s_ol_ramp_duration_ms * 1000UL;

    /* 线�斜� */
    if (ramp_elapsed < ramp_total) {
        s_ol_interval_us = s_ol_start_interval
            - (uint32_t)((s_ol_start_interval - s_ol_target_interval)
                         * ramp_elapsed / ramp_total);
    } else {
        s_ol_interval_us = s_ol_target_interval;
    }

    /* 到时间就��� */
    if ((now - s_ol_last_step_us) >= s_ol_interval_us) {
        s_ol_last_step_us = now;
        if (dir_fw) {
            s_comm_step = (s_comm_step + 1) % 6;
        } else {
            s_comm_step = (s_comm_step + 5) % 6;
        }
        Commutation_Step((uint8_t)s_comm_step, s_cfg.pwm_freq_hz, s_duty);
    }
}

/*=============================================================================
 * 内部: 停� -> 全至关断
 *=============================================================================*/
static void do_stop(void)
{
    int ch;
    s_sub_phase = 0;
    if (s_hall) {
        hall_3ch_stop(s_hall);
    }
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_SYNC, 98.0f);
    }
    MAIN_D("[CommRunner] STOP");
}

/*=============================================================================
 * CommRunner_Init
 *=============================================================================*/
void CommRunner_Init(const comm_runner_config_t *cfg)
{
    int ch;

    if (!cfg) return;

    memcpy(&s_cfg, cfg, sizeof(comm_runner_config_t));
    s_duty = cfg->default_duty_pct;

    /* ---- PWM ---- */
    tmr4_pwm_config_t pwm_cfg = {
        .output_type_u = TMR4_OUTPUT_SYNC,
        .output_type_v = TMR4_OUTPUT_SYNC,
        .output_type_w = TMR4_OUTPUT_SYNC,
        .freq_hz       = cfg->pwm_freq_hz,
        .dead_time_ns  = 0,
        .active_high   = true,
    };
    TMR4_PWM_Config(&pwm_cfg);
    TMR4_PWM_StartOutput();

    /* ---- Timebase ---- */
    Timer6_Timebase_Init();
    Timer6_Timebase_Start();

    /* ---- 上电默�: 全高� ON (上�全�=刹车, 待机安全) ---- */
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_SYNC, 98.0f);
    }

    /* ---- Hall 传感� ---- */
    /* 覆写回调� Runner 内部函数 */
    s_cfg.hall_cfg.on_step  = runner_on_hall_step;
    s_cfg.hall_cfg.on_fault = runner_on_hall_fault;
    s_hall = hall_3ch_create(&s_cfg.hall_cfg);
    MAIN_D("[CommRunner] Hall sensor created");

    if (cfg->on_init_done) {
        cfg->on_init_done();
    }

    s_mode        = COMM_RUNNER_STOP;
    s_initialized = 1;

    MAIN_D("[CommRunner] Init done, freq=%u Hz", cfg->pwm_freq_hz);
}

/*=============================================================================
 * CommRunner_SetMode
 *=============================================================================*/
void CommRunner_SetMode(comm_runner_mode_t mode)
{
    if (!s_initialized) return;
    if (mode == s_mode) return;

    s_mode      = mode;

    switch (mode) {

    case COMM_RUNNER_STOP:
        do_stop();
        break;

    case COMM_RUNNER_OPEN_FW:
        if (s_hall) hall_3ch_stop(s_hall);
        start_open_loop(s_cfg.ol_const_start_us, s_cfg.ol_const_target_us,
                        s_cfg.ol_const_ramp_ms, 1);
        s_sub_phase = 0;
        break;

    case COMM_RUNNER_OPEN_RV:
        if (s_hall) hall_3ch_stop(s_hall);
        start_open_loop(s_cfg.ol_const_start_us, s_cfg.ol_const_target_us,
                        s_cfg.ol_const_ramp_ms, 0);
        s_sub_phase = 0;
        break;

    case COMM_RUNNER_CLOSED_FW:
        if (s_hall) hall_3ch_stop(s_hall);
        start_open_loop(s_cfg.ol_fly_start_us, s_cfg.ol_fly_target_us,
                        s_cfg.ol_fly_ramp_ms, 1);
        s_sub_phase = 0;
        MAIN_D("[CommRunner] Mode=CLOSED_FW: Open-loop ramp -> flying start");
        break;

    case COMM_RUNNER_CLOSED_RV:
        if (s_hall) hall_3ch_stop(s_hall);
        start_open_loop(s_cfg.ol_fly_start_us, s_cfg.ol_fly_target_us,
                        s_cfg.ol_fly_ramp_ms, 0);
        s_sub_phase = 0;
        MAIN_D("[CommRunner] Mode=CLOSED_RV: Open-loop ramp -> flying start");
        break;
    }
}

/*=============================================================================
 * CommRunner_GetMode
 *=============================================================================*/
comm_runner_mode_t CommRunner_GetMode(void)
{
    return s_mode;
}

/*=============================================================================
 * CommRunner_SetDuty
 *=============================================================================*/
void CommRunner_SetDuty(float duty_pct)
{
    if (duty_pct < 2.0f) duty_pct = 2.0f;
    if (duty_pct > 98.0f) duty_pct = 98.0f;
    s_duty = duty_pct;

    /* ��模式�, 下� Hall ISR 触发 on_step 时自然带新占空比 */
    /* ��模式�, 下�定时换相时带新占空� */
}

/*=============================================================================
 * CommRunner_GetDuty
 *=============================================================================*/
float CommRunner_GetDuty(void)
{
    return s_duty;
}

/*=============================================================================
 * CommRunner_Update - 主循� 1ms 调用
 *=============================================================================*/
void CommRunner_Update(void)
{
    if (!s_initialized) return;

    /* 处理 ISR 报告� Hall 故障 (000/111) */
    if (s_fault_pending) {
        s_fault_pending = 0;
        MAIN_D("[CommRunner] Hall fault, coast");
        CommRunner_SetMode(COMM_RUNNER_STOP);
    }

    Timer6_Timebase_UpdateTimestamp();
    uint64_t now = Timer6_Timebase_GetTimestamp();

    switch (s_mode) {

    /* ---- ��� (mode 1/2) ---- */
    case COMM_RUNNER_OPEN_FW:
        open_loop_tick(now, 1);
        break;

    case COMM_RUNNER_OPEN_RV:
        open_loop_tick(now, 0);
        break;

    /* ---- 飞启->�� (mode 3/4) ---- */
    case COMM_RUNNER_CLOSED_FW:
    case COMM_RUNNER_CLOSED_RV: {
        int is_fw = (s_mode == COMM_RUNNER_CLOSED_FW);

        if (s_sub_phase == 0) {
            /* === 阶�0: ��强拖 + 斜坡 === */
            open_loop_tick(now, is_fw);

            /* 斜坡结束 -> 飞启切入�� */
            uint64_t ramp_elapsed = now - s_ol_ramp_start_us;
            uint64_t ramp_total   = (uint64_t)s_ol_ramp_duration_ms * 1000UL;
            if (ramp_elapsed >= ramp_total) {
                if (is_fw) {
                    hall_3ch_set_table(s_hall, s_hall2step_cw);
                    hall_3ch_start_flying(s_hall, HALL3_DIR_FORWARD);
                } else {
                    hall_3ch_set_table(s_hall, s_hall2step_ccw);
                    hall_3ch_start_flying(s_hall, HALL3_DIR_REVERSE);
                }
                s_sub_phase = 1;
                MAIN_D("[CommRunner] Flying start -> closed-loop (%s)",
                       is_fw ? "CW" : "CCW");
            }
        } else {
            /* === 阶�1: ��运� (Hall ISR 驱动换相) === */
            hall_3ch_update(s_hall);
            if (hall_3ch_is_stalled(s_hall)) {
                MAIN_D("[CommRunner] Closed-loop stall, coast");
                CommRunner_SetMode(COMM_RUNNER_STOP);
            }
        }
        break;
    }

    case COMM_RUNNER_STOP:
    default:
        break;
    }
}

/*=============================================================================
 * CommRunner_GetRPM
 *=============================================================================*/
float CommRunner_GetRPM(void)
{
    if (!s_hall) return 0.0f;
    return hall_3ch_get_rpm(s_hall);
}

/*=============================================================================
 * CommRunner_IsRunning
 *=============================================================================*/
uint8_t CommRunner_IsRunning(void)
{
    if (!s_hall) return 0;
    return hall_3ch_is_running(s_hall);
}

/*=============================================================================
 * CommRunner_IsStalled
 *=============================================================================*/
uint8_t CommRunner_IsStalled(void)
{
    if (!s_hall) return 0;
    return hall_3ch_is_stalled(s_hall);
}
