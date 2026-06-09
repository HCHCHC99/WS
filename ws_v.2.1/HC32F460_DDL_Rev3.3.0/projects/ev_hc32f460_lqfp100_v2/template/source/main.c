
  #include "main.h"
  #include "Hardware.h"
  #include "rtt_log.h"
  #include "timer6_timebase.h"
  #include "TickTimer.h"
  #include "App_Motor_Project.h"
  #include "App_Comm.h"
  #include "App_FaultHandler.h"
  #include "rtt_manager.h"
  #include "hc32_ll_utility.h"
  #include "tmr4_pwm.h"
#include "dev_comm_runner.h"

/*=============================================================================
 * Keil Watch 可修改变量 (调试接口)
 *=============================================================================*/
volatile int   comm_mode        = 0;     /* 0=停, 1=开环正转, 2=开环反转, 3=闭环正转, 4=闭环反转 */
volatile float g_comm_duty_pct  = 80.0f; /* 占空比 2%~98% */

/* 旧 PWM 全局变量 (dev_motor 模块引用, 不可删除) */
pwm_t g_motor_pwm_ch1;
pwm_t g_motor_pwm_ch2;
pwm_t g_motor_pwm_ch3;
pwm_t g_motor_pwm_ch4;

/*=============================================================================
 * Hall 映射表 (16张 × 8项)
 *   0~5: 同步 (霍尔+1=磁场CW)
 *   6~11: 偏移 (霍尔+1=磁场CCW)
 *   12: 实测校准 CW
 *   13: CCW (磁场CCW超前)
 *   14: 强拖正转 [sector+90°]
 *   15: 强拖反转 [sector-90°]
 *=============================================================================*/
static const uint8_t hall_tables[16][8] = {
    /* === 同步: 霍尔+1 = 磁场CW === */
    {0xFF, 0, 2, 1, 4, 5, 3, 0xFF},   /*  0: 磁场正转(重合) */
    {0xFF, 1, 3, 2, 5, 0, 4, 0xFF},   /*  1: 磁场超前 1 拍 */
    {0xFF, 2, 4, 3, 0, 1, 5, 0xFF},   /*  2: 磁场超前 2 拍 */
    {0xFF, 3, 5, 4, 1, 2, 0, 0xFF},   /*  3: 磁场超前 3 拍 */
    {0xFF, 4, 0, 5, 2, 3, 1, 0xFF},   /*  4: 磁场超前 4 拍 */
    {0xFF, 5, 1, 0, 3, 4, 2, 0xFF},   /*  5: 磁场超前 5 拍 */
    /* === 偏移: 霍尔+1 = 磁场CCW === */
    {0xFF, 5, 3, 4, 1, 0, 2, 0xFF},   /*  6 */
    {0xFF, 0, 4, 5, 2, 1, 3, 0xFF},   /*  7 */
    {0xFF, 1, 5, 0, 3, 2, 4, 0xFF},   /*  8 */
    {0xFF, 2, 0, 1, 4, 3, 5, 0xFF},   /*  9 */
    {0xFF, 3, 1, 2, 5, 4, 0, 0xFF},   /* 10 */
    {0xFF, 4, 2, 3, 0, 5, 1, 0xFF},   /* 11 */
    /* === 实测校准 === */
    {0xFF, 1, 5, 0, 3, 2, 4, 0xFF},   /* 12: CW */
    {0xFF, 2, 0, 1, 3, 5, 4, 0xFF},   /* 13: CCW */
    /* === 强拖: 超前/滞后 90° === */
    {0xFF, 2, 0, 1, 4, 3, 5, 0xFF},   /* 14: 强拖正转 [sector+90°] */
    {0xFF, 5, 3, 4, 1, 0, 2, 0xFF},   /* 15: 强拖反转 [sector-90°] */
};

int main(void)
{
    /* ---- 硬件初始化 ---- */
    Hardware_Init();

    /* ---- 通信栈 (RS485 + Modbus RTU) ---- */
    static const App_Comm_Config_t comm_cfg = {
        .phy.baudrate     = 9600,
        .phy.dir_polarity = 0,
        .hal.rx_buf_size  = 500,
        .hal.tx_buf_size  = 500,
        .hal.rx_frame_queue_depth = 10,
        .hal.tx_queue_depth       = 10,
        .hal.frame_timeout_ms     = 0,
        .proto.node_id            = 1,
        .proto.enable_write_multi = true,
    };
    App_Comm_Init(&comm_cfg);

    tickTimer_DelayMs(5);

    /* ---- 换相控制器初始化 ---- */
    static const comm_runner_config_t runner_cfg = {
        .pwm_freq_hz       = 50000,
        .hall_tables       = hall_tables,
        .hall_table_cw     = 14,
        .hall_table_ccw    = 15,

        /* Hall 传感器配置: 3路, PA10=U, PA9=V, PA8=W, 3对极 */
        .hall_cfg = {
            .port      = {GPIO_PORT_A, GPIO_PORT_A, GPIO_PORT_A},
            .pin       = {GPIO_PIN_10, GPIO_PIN_09, GPIO_PIN_08},
            .eirq_ch   = {EXTINT_CH10, EXTINT_CH09, EXTINT_CH08},
            .irqn      = {INT010_IRQn, INT009_IRQn, INT008_IRQn},
            .irq_src   = {INT_SRC_PORT_EIRQ10, INT_SRC_PORT_EIRQ9, INT_SRC_PORT_EIRQ8},
            .irq_priority = DDL_IRQ_PRIO_02,
            .pole_pairs   = 3,
            /* 默认磁场对齐表: step0→0x01, 磁场正向 */
            .hall_to_step = {0xFF,1,3,2,5,0,4,0xFF},
            /* on_step/on_fault 由 CommRunner 内部覆写 */
            .on_step      = NULL,
            .on_fault     = NULL,
            .align_step        = 0,
            .align_duty_pct    = 80.0f,
            .align_duration_ms = 500,
            .stall_timeout_ms  = 500,
        },

        /* 开环恒速 (mode 1/2): 667 RPM 定速 3s 斜坡 */
        .ol_const_start_us  = 5000,
        .ol_const_target_us = 5000,
        .ol_const_ramp_ms   = 3000,

        /* 飞启开环 (mode 3/4): 167→1111 RPM, 2s 斜坡 */
        .ol_fly_start_us    = 20000,
        .ol_fly_target_us   = 3000,
        .ol_fly_ramp_ms     = 2000,

        .default_duty_pct   = 80.0f,
        .on_init_done       = NULL,
    };
    CommRunner_Init(&runner_cfg);

    EventBus_Enable();

    /* ---- 主循环 ---- */
    static int   s_prev_mode     = -1;
    static float s_prev_duty     = 80.0f;

    while (1) {
        App_Comm_Poll();

        /* Keil Watch → CommRunner (调试器/Modbus 下发的模式切换) */
        if (comm_mode != s_prev_mode) {
            s_prev_mode = comm_mode;
            CommRunner_SetMode((comm_runner_mode_t)comm_mode);
        }
        if (g_comm_duty_pct != s_prev_duty) {
            s_prev_duty = g_comm_duty_pct;
            CommRunner_SetDuty(g_comm_duty_pct);
        }

        /* 驱动换相状态机 */
        CommRunner_Update();

        /* CommRunner → Keil Watch (堵转等内部触发的 STOP 同步回来) */
        {
            int actual = (int)CommRunner_GetMode();
            if (actual != comm_mode) {
                comm_mode   = actual;
                s_prev_mode = actual;
            }
        }
    }
}
