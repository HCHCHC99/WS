#ifndef __HALL_SENSOR_3CH_H__
#define __HALL_SENSOR_3CH_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 方向 */
typedef enum {
    HALL3_DIR_NONE = 0,
    HALL3_DIR_FORWARD,
    HALL3_DIR_REVERSE,
} hall3_direction_t;

/* 回调: Hall 触发换相时调用, ISR 内执行 */
typedef void (*hall3_step_callback_t)(uint8_t step, hall3_direction_t dir);
/* 回调: 检测到 000/111 时调用 */
typedef void (*hall3_fault_callback_t)(uint8_t hall_state);

/* 配置 */
typedef struct {
    uint8_t   port[3];           /* 0=U(PA10) 1=V(PA9) 2=W(PA8) */
    uint16_t  pin[3];
    uint32_t  eirq_ch[3];
    IRQn_Type irqn[3];
    uint32_t  irq_src[3];
    uint8_t   irq_priority;

    uint8_t   pole_pairs;
    uint8_t   hall_to_step[8];   /* 3bit 状态码 → 换相步 0~5, 0xFF=故障 */

    hall3_step_callback_t  on_step;    /* ISR 内调用 */
    hall3_fault_callback_t on_fault;   /* ISR 内调用 */

    /* 对齐启动 */
    uint8_t   align_step;         /* 对齐用的换相步 */
    float     align_duty_pct;     /* 对齐占空比 */
    uint16_t  align_duration_ms;  /* 对齐持续时间 */

    uint16_t  stall_timeout_ms;
} hall_3ch_config_t;

/* 不透明句柄 */
typedef struct hall_3ch_instance_t* hall_3ch_handle_t;

/* API */
void              hall_3ch_system_init(void);
hall_3ch_handle_t hall_3ch_create(const hall_3ch_config_t *cfg);
void              hall_3ch_destroy(hall_3ch_handle_t h);

void              hall_3ch_start(hall_3ch_handle_t h, hall3_direction_t dir);
void              hall_3ch_stop(hall_3ch_handle_t h);    /* 停, 回 IDLE */
void              hall_3ch_set_table(hall_3ch_handle_t h, const uint8_t table[8]);

void              hall_3ch_update(hall_3ch_handle_t h);  /* 主循环定时调 */

float             hall_3ch_get_rpm(hall_3ch_handle_t h);
hall3_direction_t hall_3ch_get_direction(hall_3ch_handle_t h);
uint8_t           hall_3ch_is_running(hall_3ch_handle_t h);
uint8_t           hall_3ch_is_stalled(hall_3ch_handle_t h);

#endif
