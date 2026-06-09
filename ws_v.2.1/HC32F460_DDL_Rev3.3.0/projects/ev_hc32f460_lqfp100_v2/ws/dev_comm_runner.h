#ifndef __DEV_COMM_RUNNER_H__
#define __DEV_COMM_RUNNER_H__

#include "hall_sensor_3ch.h"
#include <stdint.h>

/*=============================================================================
 * CommRunner: 六步方波换相控制器
 *
 * 封装换相状态机（停止 / 开环 / 飞启→闭环），替代 main.c 中的分散逻辑。
 * 主循环只需调用 CommRunner_Update，模式切换调用 CommRunner_SetMode。
 *=============================================================================*/

/* 换相模式 (与原有 comm_mode 0~4 兼容) */
typedef enum {
    COMM_RUNNER_STOP       = 0,  /* 惯性滑行 / 停止 */
    COMM_RUNNER_OPEN_FW    = 1,  /* 开环正转 (恒速斜坡) */
    COMM_RUNNER_OPEN_RV    = 2,  /* 开环反转 (恒速斜坡) */
    COMM_RUNNER_CLOSED_FW  = 3,  /* 飞启→闭环正转 */
    COMM_RUNNER_CLOSED_RV  = 4,  /* 飞启→闭环反转 */
} comm_runner_mode_t;

/* 配置结构 */
typedef struct {
    /* PWM 频率 */
    uint16_t pwm_freq_hz;

    /* Hall 传感器配置 (传给 hall_3ch_create) */
    hall_3ch_config_t hall_cfg;

    /* Hall 映射表 (16×8 表指针) 及其 CW/CCW 索引 */
    const uint8_t (*hall_tables)[8];
    uint8_t  hall_table_cw;   /* 闭环正转用的表索引 */
    uint8_t  hall_table_ccw;  /* 闭环反转用的表索引 */

    /* 开环恒速 (mode 1/2): 斜坡参数 */
    uint32_t ol_const_start_us;     /* 起始间隔 (us) */
    uint32_t ol_const_target_us;    /* 目标间隔 (us) */
    uint32_t ol_const_ramp_ms;      /* 斜坡时长 (ms) */

    /* 飞启开环 (mode 3/4): 斜坡参数 */
    uint32_t ol_fly_start_us;       /* 起始间隔 (us, 高扭矩低速) */
    uint32_t ol_fly_target_us;      /* 目标间隔 (us, 高速) */
    uint32_t ol_fly_ramp_ms;        /* 斜坡时长 (ms) */

    /* 占空比默认值 */
    float    default_duty_pct;

    /* PWM 初始化后回调 (可选): 让 main.c 做额外 GPIO 设置 */
    void (*on_init_done)(void);
} comm_runner_config_t;

/*=============================================================================
 * API
 *=============================================================================*/

/* 初始化: 创建 PWM / Hall / 时间基准, 进入 STOP 状态 */
void CommRunner_Init(const comm_runner_config_t *cfg);

/* 切换模式: 内部处理所有过渡 (停止 / 开环起步 / 飞启) */
void CommRunner_SetMode(comm_runner_mode_t mode);

/* 获取当前模式 */
comm_runner_mode_t CommRunner_GetMode(void);

/* 动态设置占空比 (闭环模式下实时生效) */
void CommRunner_SetDuty(float duty_pct);

/* 获取当前占空比 */
float CommRunner_GetDuty(void);

/* 主循环调用: 1ms 周期, 驱动开环换相 + 闭环监控 */
void CommRunner_Update(void);

/* 获取滤波后 RPM (来自 Hall 传感器 M 法) */
float CommRunner_GetRPM(void);

/* 是否正在运行 (闭环模式下 Hall 处于 RUNNING 状态) */
uint8_t CommRunner_IsRunning(void);

/* 是否堵转 */
uint8_t CommRunner_IsStalled(void);

#endif /* __DEV_COMM_RUNNER_H__ */
