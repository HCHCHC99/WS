#include "hall_sensor_3ch.h"
#include "timer6_timebase.h"
#include "TickTimer.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_interrupts.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

/* ========== 常量 ========== */
#define MAX_INSTANCES             2
#define MIN_PULSE_INTERVAL_US     50u
#define MAX_PULSE_INTERVAL_US     200000u
#define RPM_WINDOW_SIZE           6
#define DIR_CONFIRM_COUNT         3
#define HALL_STATE_MASK           0x07u

/* ========== 状态机 ========== */
enum {
    STATE_IDLE = 0,
    STATE_ALIGNING,
    STATE_RUNNING,
    STATE_FAULT,
};

/* ========== 实例内部结构 ========== */
typedef struct hall_3ch_instance_t {
    uint8_t id;
    uint8_t valid;
    uint8_t state;

    hall_3ch_config_t config;

    /* 保存 gpio 引用用于 ISR 内读电平 */
    uint8_t  gpio_port[3];
    uint16_t gpio_pin[3];

    /* ISR 写入, update 读取 */
    volatile uint8_t  last_hall_state;
    volatile uint8_t  last_valid_state;
    volatile uint8_t  last_step;
    volatile uint32_t last_pulse_interval_us;
    volatile uint64_t last_pulse_time_us;
    volatile uint32_t pulse_counter;

    /* 方向 */
    volatile hall3_direction_t current_dir;
    volatile uint8_t  dir_confirm_count;
    volatile uint8_t  dir_data_ready;

    /* RPM */
    volatile float    current_rpm;
    volatile float    filtered_rpm;
    float             rpm_window[RPM_WINDOW_SIZE];
    uint8_t           rpm_write_idx;
    uint8_t           rpm_valid_count;

    uint32_t          interval_history[6];
    uint8_t           interval_idx;
    uint8_t           interval_valid_count;

    /* 对齐启动 */
    hall3_direction_t  target_dir;
    uint64_t           align_start_time_us;

    /* 堵转 */
    volatile uint8_t  stalled;

} hall_3ch_instance_t;

/* ========== 全局实例池 ========== */
static uint8_t g_system_initialized = 0;
static hall_3ch_instance_t g_instances[MAX_INSTANCES] = {{{0}}};

/* ISR 快速索引: eirq_ch → 实例指针 */
static hall_3ch_instance_t *g_irq_map[3] = {NULL, NULL, NULL};

/* ========== 内部声明 ========== */
static void hall_common_handler(uint8_t ch);
static void update_direction(hall_3ch_instance_t *inst, uint8_t step);

/* ========== 系统初始化 ========== */
void hall_3ch_system_init(void)
{
    if (g_system_initialized) return;
    Timer6_Timebase_Init();
    Timer6_Timebase_Start();
    memset(g_instances, 0, sizeof(g_instances));
    g_system_initialized = 1;
}

/* ========== 注册单路中断 (EXTINT + GPIO + IRQ, 一步完成) ========== */
static void register_hall_irq(hall_3ch_instance_t *inst, uint8_t ch, func_ptr_t cb)
{
    stc_extint_init_t       stcExti;
    stc_irq_signin_config_t stcIrq;
    stc_gpio_init_t         stcGpio;

    uint32_t  eirq_ch = inst->config.eirq_ch[ch];
    IRQn_Type irqn    = inst->config.irqn[ch];
    uint32_t  irq_src = inst->config.irq_src[ch];
    uint8_t   port    = inst->config.port[ch];
    uint16_t  pin     = inst->config.pin[ch];

    g_irq_map[ch] = inst;

    /* EXTINT: 双边沿, 无滤波 */
    memset(&stcExti, 0, sizeof(stcExti));
    stcExti.u32Edge        = EXTINT_TRIG_BOTH;
    stcExti.u32Filter      = EXTINT_FILTER_OFF;
    stcExti.u32FilterClock = EXTINT_FCLK_DIV1;
    EXTINT_Init(eirq_ch, &stcExti);

    /* GPIO: 数字输入, 上拉 */
    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinDir  = PIN_DIR_IN;
    stcGpio.u16PinAttr = PIN_ATTR_DIGITAL;
    stcGpio.u16PullUp  = PIN_PU_ON;
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_Init(port, pin, &stcGpio);
    GPIO_ExtIntCmd(port, pin, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /* 中断注册 (带回调) */
    memset(&stcIrq, 0, sizeof(stcIrq));
    stcIrq.enIntSrc    = (en_int_src_t)irq_src;
    stcIrq.enIRQn      = irqn;
    stcIrq.pfnCallback = cb;
    INTC_IrqSignIn(&stcIrq);

    EXTINT_ClearExtIntStatus(eirq_ch);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_SetPriority(irqn, inst->config.irq_priority);
    NVIC_EnableIRQ(irqn);
}

/* ========== ISR 入口 ========== */
static void hall_u_isr(void) { hall_common_handler(0); }
static void hall_v_isr(void) { hall_common_handler(1); }
static void hall_w_isr(void) { hall_common_handler(2); }

/* ========== 共用的 Hall 处理 ========== */
static void hall_common_handler(uint8_t ch)
{
    hall_3ch_instance_t *inst = g_irq_map[ch];
    if (!inst || !inst->valid) return;

    /* 清该路中断标志 */
    EXTINT_ClearExtIntStatus(inst->config.eirq_ch[ch]);

    /* 读三路 GPIO 电平, 拼状态码 */
    uint8_t state = 0;
    state |= (GPIO_ReadInputPins(inst->gpio_port[0], inst->gpio_pin[0]) == PIN_SET) ? 0x04u : 0x00u;
    state |= (GPIO_ReadInputPins(inst->gpio_port[1], inst->gpio_pin[1]) == PIN_SET) ? 0x02u : 0x00u;
    state |= (GPIO_ReadInputPins(inst->gpio_port[2], inst->gpio_pin[2]) == PIN_SET) ? 0x01u : 0x00u;

    /* 去重1: 状态没变 → 抖动 */
    if (state == inst->last_hall_state) {
        return;
    }

    /* 去重2: 间隔太短 → 抖动回退 */
    uint32_t interval = Timer6_Timebase_GetDelta();
    uint32_t interval_us = Timer6_Timebase_DeltaToUs(interval);
    if (interval_us < MIN_PULSE_INTERVAL_US) {
        return;
    }
    inst->last_hall_state = state;

    /* 查表 */
    uint8_t step = inst->config.hall_to_step[state];

    /* 故障状态 000/111 */
    if (step == 0xFFu) {
        if (inst->state == STATE_RUNNING) {
            inst->state = STATE_FAULT;
            if (inst->config.on_fault) {
                inst->config.on_fault(state);
            }
        }
        return;
    }

    /* 记录间隔 */
    inst->last_pulse_interval_us = interval_us;
    inst->last_pulse_time_us     = Timer6_Timebase_GetTimestamp();
    inst->pulse_counter++;

    /* 非 RUNNING 状态: 只观测, 不换相 */
    if (inst->state != STATE_RUNNING) {
        MAIN_D("[HALL] ch=%d raw=0x%02X -> step=%d (%s)",
               ch, state, step,
               (inst->state == STATE_ALIGNING) ? "align" : "idle");
        return;
    }

    /* RUNNING 状态: 判向 + 换相 */
    update_direction(inst, step);

    if (inst->config.on_step) {
        MAIN_D("[HALL] ch=%d raw=0x%02X -> step=%d dir=%d",
               ch, state, step, (int)inst->current_dir);
        inst->config.on_step(step, inst->current_dir);
    }
}

/* ========== 方向判定: 基于 step 序列 ========== */
static void update_direction(hall_3ch_instance_t *inst, uint8_t step)
{
    hall3_direction_t tentative;

    int8_t diff = (int8_t)step - (int8_t)inst->last_step;
    if (diff < 0) diff += 6;

    if (diff == 1) {
        tentative = HALL3_DIR_FORWARD;
    } else if (diff == 5) {
        tentative = HALL3_DIR_REVERSE;
    } else {
        /* 乱序跳转, 忽略 */
        inst->last_step = step;
        return;
    }

    inst->last_step = step;

    if (tentative != inst->current_dir) {
        inst->dir_confirm_count++;
        if (inst->dir_confirm_count >= DIR_CONFIRM_COUNT) {
            inst->current_dir = tentative;
            inst->dir_confirm_count = 0;
        }
    } else {
        inst->dir_confirm_count = 0;
    }
}

/* ========== RPM 滤波 ========== */
static void update_rpm_filter(hall_3ch_instance_t *inst, float raw)
{
    inst->rpm_window[inst->rpm_write_idx] = raw;
    inst->rpm_write_idx = (inst->rpm_write_idx + 1) % RPM_WINDOW_SIZE;
    if (inst->rpm_valid_count < RPM_WINDOW_SIZE) {
        inst->rpm_valid_count++;
    }
    float sum = 0.0f;
    for (uint8_t i = 0; i < inst->rpm_valid_count; i++) {
        sum += inst->rpm_window[i];
    }
    inst->filtered_rpm = sum / (float)inst->rpm_valid_count;
}

/* ========== 平均间隔 ========== */
static float average_interval_us(hall_3ch_instance_t *inst)
{
    if (inst->last_pulse_interval_us >= MIN_PULSE_INTERVAL_US
        && inst->last_pulse_interval_us <= MAX_PULSE_INTERVAL_US) {
        inst->interval_history[inst->interval_idx] = inst->last_pulse_interval_us;
        inst->interval_idx = (inst->interval_idx + 1) % 6;
        if (inst->interval_valid_count < 6) {
            inst->interval_valid_count++;
        }
    }
    if (inst->interval_valid_count < 2) return 0.0f;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < inst->interval_valid_count; i++) {
        sum += inst->interval_history[i];
    }
    return (float)sum / (float)inst->interval_valid_count;
}

/* ========== 间隔 → RPM ========== */
static float interval_to_rpm(hall_3ch_instance_t *inst, float interval_us)
{
    if (interval_us < 1.0f) return 0.0f;
    uint16_t pulses_per_rev = inst->config.pole_pairs * 6u;
    float rpm = 60000000.0f / (interval_us * (float)pulses_per_rev);
    if (rpm > 100000.0f) rpm = 100000.0f;
    return rpm;
}

/* ========== 公开接口 ========== */

hall_3ch_handle_t hall_3ch_create(const hall_3ch_config_t *cfg)
{
    if (!g_system_initialized) hall_3ch_system_init();
    if (!cfg) return NULL;

    uint8_t i;
    for (i = 0; i < MAX_INSTANCES; i++) {
        if (!g_instances[i].valid) break;
    }
    if (i >= MAX_INSTANCES) return NULL;

    hall_3ch_instance_t *inst = &g_instances[i];
    memset(inst, 0, sizeof(*inst));

    inst->id    = i;
    inst->valid = 1;
    inst->state = STATE_IDLE;

    /* 拷贝配置 */
    memcpy(&inst->config, cfg, sizeof(hall_3ch_config_t));

    /* 保存 gpio 引用 */
    for (uint8_t ch = 0; ch < 3; ch++) {
        inst->gpio_port[ch] = cfg->port[ch];
        inst->gpio_pin[ch]  = cfg->pin[ch];
    }

    inst->last_hall_state = 0xFFu;
    inst->current_dir     = HALL3_DIR_NONE;

    /* 注册三路中断 */
    register_hall_irq(inst, 0, (func_ptr_t)hall_u_isr);
    register_hall_irq(inst, 1, (func_ptr_t)hall_v_isr);
    register_hall_irq(inst, 2, (func_ptr_t)hall_w_isr);

    MAIN_D("[HALL3] Created instance %d", i);
    return (hall_3ch_handle_t)inst;
}

void hall_3ch_destroy(hall_3ch_handle_t h)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    for (uint8_t ch = 0; ch < 3; ch++) {
        NVIC_DisableIRQ(inst->config.irqn[ch]);
    }
    inst->valid = 0;
}

void hall_3ch_start(hall_3ch_handle_t h, hall3_direction_t dir)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    if (!inst->valid) return;

    inst->target_dir = dir;

    /* 对齐: 给 align_step 通电 */
    inst->state = STATE_ALIGNING;
    inst->align_start_time_us = Timer6_Timebase_GetTimestamp();

    if (inst->config.on_step) {
        inst->config.on_step(inst->config.align_step, dir);
    }

    MAIN_D("[HALL3] Start aligning step=%d dir=%d",
           inst->config.align_step, (int)dir);
}

void hall_3ch_stop(hall_3ch_handle_t h)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    inst->state    = STATE_IDLE;
    inst->stalled  = 0;
    inst->last_hall_state = 0xFFu;
}

void hall_3ch_set_table(hall_3ch_handle_t h, const uint8_t table[8])
{
    if (!h || !table) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    uint8_t i;
    for (i = 0; i < 8; i++) {
        inst->config.hall_to_step[i] = table[i];
    }
}

/* ========== 定时轮询 ========== */
void hall_3ch_update(hall_3ch_handle_t h)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    if (!inst->valid) return;

    Timer6_Timebase_UpdateTimestamp();
    uint64_t now = Timer6_Timebase_GetTimestamp();

    switch (inst->state) {

    case STATE_ALIGNING: {
        uint64_t elapsed = now - inst->align_start_time_us;
        if (elapsed >= (uint64_t)inst->config.align_duration_ms * 1000UL) {
            /* 对齐完成: 踢一脚 */
            inst->state = STATE_RUNNING;
            inst->last_step = inst->config.align_step;

            uint8_t kick_step;
            if (inst->target_dir == HALL3_DIR_FORWARD) {
                kick_step = (inst->config.align_step + 1) % 6;
            } else {
                kick_step = (inst->config.align_step + 5) % 6;
            }

            if (inst->config.on_step) {
                inst->config.on_step(kick_step, inst->target_dir);
            }
            MAIN_D("[HALL3] Align done, kick step=%d", kick_step);
        }
        break;
    }

    case STATE_RUNNING: {
        /* RPM */
        float avg = average_interval_us(inst);
        if (avg > 0.0f) {
            float raw = interval_to_rpm(inst, avg);
            inst->current_rpm = raw;
            update_rpm_filter(inst, raw);
        }

        /* 堵转检测 */
        if (inst->config.stall_timeout_ms > 0) {
            uint64_t since_pulse = now - inst->last_pulse_time_us;
            if (since_pulse > (uint64_t)inst->config.stall_timeout_ms * 1000UL) {
                inst->stalled = 1;
                inst->state   = STATE_IDLE;
                MAIN_D("[HALL3] Stall! timeout=%lu us", (uint32_t)since_pulse);
            }
        }
        break;
    }

    case STATE_FAULT:
    case STATE_IDLE:
    default:
        break;
    }
}

/* ========== 查询接口 ========== */

float hall_3ch_get_rpm(hall_3ch_handle_t h)
{
    if (!h) return 0.0f;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    return inst->filtered_rpm;
}

hall3_direction_t hall_3ch_get_direction(hall_3ch_handle_t h)
{
    if (!h) return HALL3_DIR_NONE;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    return inst->current_dir;
}

uint8_t hall_3ch_is_running(hall_3ch_handle_t h)
{
    if (!h) return 0;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    return (inst->state == STATE_RUNNING) ? 1 : 0;
}

uint8_t hall_3ch_is_stalled(hall_3ch_handle_t h)
{
    if (!h) return 1;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    return inst->stalled;
}
