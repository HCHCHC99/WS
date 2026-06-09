#include "hall_sensor_3ch.h"
#include "timer6_timebase.h"
#include "TickTimer.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_interrupts.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

/* ========== ���� ========== */
#define MAX_INSTANCES             2
#define MIN_PULSE_INTERVAL_US     50u
#define MAX_PULSE_INTERVAL_US     200000u
#define RPM_WINDOW_SIZE           6
#define DIR_CONFIRM_COUNT         3
#define HALL_STATE_MASK           0x07u

/* ========== ״̬�� ========== */
enum {
    STATE_IDLE = 0,
    STATE_ALIGNING,
    STATE_RUNNING,
    STATE_FAULT,
};

/* ========== ʵ���ڲ��ṹ ========== */
typedef struct hall_3ch_instance_t {
    uint8_t id;
    uint8_t valid;
    uint8_t state;

    hall_3ch_config_t config;

    /* ���� gpio �������� ISR �ڶ���ƽ */
    uint8_t  gpio_port[3];
    uint16_t gpio_pin[3];

    /* ISR д��, update ��ȡ */
    volatile uint8_t  last_hall_state;
    volatile uint8_t  last_valid_state;
    volatile uint8_t  last_step;
    volatile uint32_t last_pulse_interval_us;
    volatile uint64_t last_pulse_time_us;
    volatile uint32_t pulse_counter;

    /* ���� */
    volatile hall3_direction_t current_dir;
    volatile uint8_t  dir_confirm_count;
    volatile uint8_t  dir_data_ready;

    /* RPM */
    volatile float    current_rpm;
    volatile float    filtered_rpm;
    float             rpm_window[RPM_WINDOW_SIZE];
    uint8_t           rpm_write_idx;
    uint8_t           rpm_valid_count;

    /* M-method RPM: �̶�ʱ�䴰�������ۼ� + ������ */
    volatile uint32_t last_pulse_count;
    uint32_t          rpm_accum_pulses;
    uint64_t          last_rpm_update_us;

    /* �������� */
    hall3_direction_t  target_dir;
    uint64_t           align_start_time_us;

    /* ��ת */
    volatile uint8_t  stalled;

} hall_3ch_instance_t;

/* ========== ȫ��ʵ���� ========== */
static uint8_t g_system_initialized = 0;
static hall_3ch_instance_t g_instances[MAX_INSTANCES] = {{{0}}};

/* ISR ��������: eirq_ch �� ʵ��ָ�� */
static hall_3ch_instance_t *g_irq_map[3] = {NULL, NULL, NULL};

/* Keil Watch ���Ա��� (�� static, volatile) */
volatile float    g_hall_rpm        = 0.0f;
volatile uint8_t  g_hall_state      = 0;
volatile uint8_t  g_hall_dir        = 0;
volatile uint8_t  g_hall_running    = 0;
volatile uint8_t  g_hall_stalled    = 0;
volatile uint8_t  g_hall_last_step  = 0;

/* ========== �ڲ����� ========== */
static void hall_common_handler(uint8_t ch);

/* ========== ϵͳ��ʼ�� ========== */
void hall_3ch_system_init(void)
{
    if (g_system_initialized) return;
    Timer6_Timebase_Init();
    Timer6_Timebase_Start();
    memset(g_instances, 0, sizeof(g_instances));
    g_system_initialized = 1;
}

/* ========== ע�ᵥ·�ж� (EXTINT + GPIO + IRQ, һ�����) ========== */
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

    /* EXTINT: ˫����, Ӳ���˲� (PCLK1/64��1.56MHz, �˳�<~10��s ë��) */
    memset(&stcExti, 0, sizeof(stcExti));
    stcExti.u32Edge        = EXTINT_TRIG_BOTH;
    stcExti.u32Filter      = EXTINT_FILTER_ON;
    stcExti.u32FilterClock = EXTINT_FCLK_DIV64;
    EXTINT_Init(eirq_ch, &stcExti);

    /* GPIO: ��������, ���� */
    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinDir  = PIN_DIR_IN;
    stcGpio.u16PinAttr = PIN_ATTR_DIGITAL;
    stcGpio.u16PullUp  = PIN_PU_ON;
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_Init(port, pin, &stcGpio);
    GPIO_ExtIntCmd(port, pin, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /* �ж�ע�� (���ص�) */
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

/* ========== ISR ��� ========== */
static void hall_u_isr(void) { hall_common_handler(0); }
static void hall_v_isr(void) { hall_common_handler(1); }
static void hall_w_isr(void) { hall_common_handler(2); }

/* ========== ���õ� Hall ���� ========== */

/* ���ж�· Hall GPIO ƴ�� 3bit ״̬�� */
static uint8_t read_hall_state_raw(const hall_3ch_instance_t *inst)
{
    uint8_t s = 0;
    s |= (GPIO_ReadInputPins(inst->gpio_port[0], inst->gpio_pin[0]) == PIN_SET) ? 0x04u : 0x00u;
    s |= (GPIO_ReadInputPins(inst->gpio_port[1], inst->gpio_pin[1]) == PIN_SET) ? 0x02u : 0x00u;
    s |= (GPIO_ReadInputPins(inst->gpio_port[2], inst->gpio_pin[2]) == PIN_SET) ? 0x01u : 0x00u;
    return s;
}

static void hall_common_handler(uint8_t ch)
{
    hall_3ch_instance_t *inst = g_irq_map[ch];
    if (!inst || !inst->valid) return;

    /* ���·�жϱ�־ */
    EXTINT_ClearExtIntStatus(inst->config.eirq_ch[ch]);

    /*
     * ���� 1: ������ GPIO, �˳� PWM ���ر�Ե��
     *   ���β���һ��˵���Ƕ���ë��, ֱ�Ӷ���.
     *   GetDelta() �ڴ˻�δ����, ����������Ȼָ����һ�κϷ����䡣
     */
    uint8_t state = read_hall_state_raw(inst);
    for (volatile int32_t _d = 0; _d < 100; _d++) { __NOP(); }  /* ~0.5us */
    uint8_t state2 = read_hall_state_raw(inst);
    if (state != state2) {
        return;
    }

    /*
     * ���� 2: ״̬δ�� → ë�� (���ظ�ͬһë��)
     */
    if (state == inst->last_hall_state) {
        return;
    }

    /*
     * ���� 3: ���״̬ 000/111 → ���ϻ�����
     */
    uint8_t step = inst->config.hall_to_step[state];
    if (step == 0xFFu) {
        if (inst->state == STATE_RUNNING) {
            inst->state = STATE_FAULT;
            if (inst->config.on_fault) {
                inst->config.on_fault(state);
            }
        }
        return;
    }

    /*
     * �� RUNNING: ֻ��ʱ���, ������ last_hall_state ��ֹ�ظ�����
     */
    if (inst->state != STATE_RUNNING) {
        inst->last_hall_state = state;
        inst->last_pulse_time_us = Timer6_Timebase_GetTimestamp();
        MAIN_D("[HALL] ch=%d raw=0x%02X -> step=%d (%s)",
               ch, state, step,
               (inst->state == STATE_ALIGNING) ? "align" : "idle");
        return;
    }

    /*
     * ���� 4: �����ж� — ���������� (diff=1) ����� (diff=5) ���ǺϷ�����
     *   �Ƿ���Ծ������, ������ last_step, ������ GetDelta() δ����,
     *   ����������Ȼָ����һ�κϷ����䡣
     */
    hall3_direction_t tentative;
    int8_t diff = (int8_t)step - (int8_t)inst->last_step;
    if (diff < 0) diff += 6;
    if (diff == 1) {
        tentative = HALL3_DIR_FORWARD;
    } else if (diff == 5) {
        tentative = HALL3_DIR_REVERSE;
    } else {
        return;  /* PWM ë��: ���Ϸ���Ծ, ���򶪹� */
    }

    /*
     * ============================================================
     * ������ͨ��ȫ���ṹУ��, �˴��ſ�ʼ��ʱ.
     * GetDelta() ��������һ�κϷ��������ʱ, ��� PWM ë��û�и��Ź�.
     * ============================================================
     */
    uint32_t interval = Timer6_Timebase_GetDelta();
    uint32_t interval_us = Timer6_Timebase_DeltaToUs(interval);
    if (interval_us < MIN_PULSE_INTERVAL_US) {
        return;  /* ���̫�� (Ӧ�������ڱ������Ϸ�ת��) */
    }

    /*
     * �Ϸ�ת��: ���¼�״̬����
     */
    inst->last_hall_state = state;
    inst->last_step        = step;

    /* ��¼���� */
    inst->last_pulse_interval_us = interval_us;
    inst->last_pulse_time_us     = Timer6_Timebase_GetTimestamp();
    inst->pulse_counter++;

    /* ����ȷ�� (���������ͬ��) */
    if (tentative != inst->current_dir) {
        inst->dir_confirm_count++;
        if (inst->dir_confirm_count >= DIR_CONFIRM_COUNT) {
            inst->current_dir = tentative;
            inst->dir_confirm_count = 0;
        }
    } else {
        inst->dir_confirm_count = 0;
    }

    /* ���ಽ�ص� */
    if (inst->config.on_step) {
        MAIN_D("[HALL] ch=%d raw=0x%02X -> step=%d dir=%d",
               ch, state, step, (int)inst->current_dir);
        inst->config.on_step(step, inst->current_dir);
    }
}


/* ========== RPM �˲� ========== */
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

/* ========== M-method RPM ���� (�̶�ʱ�䴰�� + ������) ========== */
#define RPM_UPDATE_MIN_US      20000u   /* ��� 20ms ����һ�� */
#define RPM_UPDATE_MIN_PULSES  6u       /* ��� 6 ������ (1 ��������) */
#define RPM_TIMEOUT_US         500000u  /* 500ms 超时: >0 脉冲强制更新, =0 归零 */

static float calc_rpm_from_pulses(uint32_t pulses, uint64_t dt_us, uint8_t pole_pairs)
{
    if (dt_us == 0 || pole_pairs == 0) return 0.0f;
    float rpm = (float)pulses * 60000000.0f / ((float)dt_us * (float)(pole_pairs * 6u));
    if (rpm > 100000.0f) rpm = 100000.0f;
    return rpm;
}

/* ========== �����ӿ� ========== */

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

    /* �������� */
    memcpy(&inst->config, cfg, sizeof(hall_3ch_config_t));

    /* ���� gpio ���� */
    for (uint8_t ch = 0; ch < 3; ch++) {
        inst->gpio_port[ch] = cfg->port[ch];
        inst->gpio_pin[ch]  = cfg->pin[ch];
    }

    inst->last_hall_state = 0xFFu;
    inst->current_dir     = HALL3_DIR_NONE;

    /* ע����·�ж� */
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

    inst->target_dir  = dir;
    inst->stalled     = 0;

    /* ��ʼ������ʱ�������ֹ stall ����ʱ��������㵼���� */
    inst->last_pulse_time_us = Timer6_Timebase_GetTimestamp();

    /* M-method RPM: ���´��� */
    Timer6_Timebase_UpdateTimestamp();
    inst->last_pulse_count   = inst->pulse_counter;
    inst->rpm_accum_pulses   = 0;
    inst->last_rpm_update_us = Timer6_Timebase_GetTimestamp();

    /* ����: �� align_step ͨ�� */
    inst->state = STATE_ALIGNING;
    inst->align_start_time_us = inst->last_pulse_time_us;

    if (inst->config.on_step) {
        inst->config.on_step(inst->config.align_step, dir);
    }

    MAIN_D("[HALL3] Start aligning step=%d dir=%d",
           inst->config.align_step, (int)dir);
}

void hall_3ch_start_flying(hall_3ch_handle_t h, hall3_direction_t dir)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    if (!inst->valid) return;

    inst->target_dir  = dir;
    inst->stalled     = 0;
    inst->last_pulse_time_us = Timer6_Timebase_GetTimestamp();

    /* M-method RPM: ���´��� */
    inst->last_pulse_count   = inst->pulse_counter;
    inst->rpm_accum_pulses   = 0;
    inst->last_rpm_update_us = inst->last_pulse_time_us;

    /* 读取当前 Hall 位置: 双采样确认 (复用 ISR 的 read_hall_state_raw) */
    uint8_t hall_state = read_hall_state_raw(inst);
    for (volatile int32_t _d = 0; _d < 100; _d++) { __NOP(); }
    uint8_t hall_state2 = read_hall_state_raw(inst);

    if (hall_state != hall_state2) {
        hall_state = 0;  /* 毛刺, fallback 到 000 */
    }

    uint8_t step = inst->config.hall_to_step[hall_state];
    if (step == 0xFFu) {
        step = 0;  /* 000/111 �쳣, fallback �� step 0 */
    }

    inst->state           = STATE_RUNNING;
    inst->last_step       = step;
    inst->last_hall_state = hall_state;

    if (inst->config.on_step) {
        inst->config.on_step(step, dir);
    }

    MAIN_D("[HALL3] Flying start: hall=0x%02X step=%d dir=%d",
           hall_state, step, (int)dir);
}

void hall_3ch_stop(hall_3ch_handle_t h)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    inst->state    = STATE_IDLE;
    inst->stalled  = 0;
    inst->last_hall_state = 0xFFu;

    /* M-method & RPM filter reset */
    inst->rpm_accum_pulses   = 0;
    inst->last_rpm_update_us = 0;
    inst->current_rpm        = 0.0f;
    inst->filtered_rpm       = 0.0f;
    inst->rpm_valid_count    = 0;
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

/* ========== ��ʱ��ѯ ========== */
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
            /* �������: ��һ�� */
            inst->state = STATE_RUNNING;
            inst->last_step = inst->config.align_step;

            /* M-method RPM: �л��� RUNNING ʱ���»�׼ */
            inst->last_pulse_count = inst->pulse_counter;
            inst->rpm_accum_pulses = 0;
            inst->last_rpm_update_us = now;

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
        /*
         * M-method RPM: �̶�ʱ�䴰�������� pulse_counter ����
         *  ��� T-method (ÿ��������), M-method ��Ȼƽ������ʱ�䴰�ڵ�ë��,
         *  ���Ҳ������� pulse ��� timebase ������
         */
        {
            uint32_t delta = inst->pulse_counter - inst->last_pulse_count;
            inst->last_pulse_count = inst->pulse_counter;
            inst->rpm_accum_pulses += delta;

            uint64_t elapsed = now - inst->last_rpm_update_us;
            if ((elapsed >= RPM_UPDATE_MIN_US && inst->rpm_accum_pulses >= RPM_UPDATE_MIN_PULSES) ||
                (elapsed >= RPM_TIMEOUT_US && inst->rpm_accum_pulses > 0)) {
                /* �������� (>=6 ������) ���ʱǿ�Ƹ��� (>0 ������) */
                float raw = calc_rpm_from_pulses(inst->rpm_accum_pulses, elapsed,
                                                 inst->config.pole_pairs);
                inst->current_rpm = raw;
                update_rpm_filter(inst, raw);
                inst->rpm_accum_pulses = 0;
                inst->last_rpm_update_us = now;
            } else if (elapsed > RPM_TIMEOUT_US && inst->rpm_accum_pulses == 0) {
                /* 500ms ������ -> ͣת */
                inst->current_rpm = 0.0f;
                update_rpm_filter(inst, 0.0f);
                inst->rpm_accum_pulses = 0;
                inst->last_rpm_update_us = now;
            }
        }

        /* ��ת��� */
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

    /* ͬ���� Keil Watch ���Ա��� */
    g_hall_rpm       = inst->filtered_rpm;
    g_hall_state     = inst->state;
    g_hall_dir       = (uint8_t)inst->current_dir;
    g_hall_running   = (inst->state == STATE_RUNNING) ? 1 : 0;
    g_hall_stalled   = inst->stalled;
    g_hall_last_step = inst->last_step;
}

/* ========== ��ѯ�ӿ� ========== */

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
