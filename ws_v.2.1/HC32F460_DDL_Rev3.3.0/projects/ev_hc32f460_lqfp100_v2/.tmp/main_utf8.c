
  #include "main.h"
  #include "Hardware.h"
  #include "rtt_log.h"
  #include "timer6_timebase.h"
  #include "Motor_hall.h"
  #include "TickTimer.h"
  #include "device_manager.h"
  #include "App_Motor_Project.h"
  #include "param_manager.h"
  #include "Gpio_io.h"
  #include "App_Comm.h"
  #include "Params.h"
  #include "App_FaultHandler.h"
  #include "rtt_manager.h"
  #include "Pwm.h"
  #include "hc32_ll_utility.h"
  #include "tmr4_pwm.h"
#include "dev_commutation.h"
#include "hall_sensor_3ch.h"

//   /*=============================================================================
//    * 全锟斤拷PWM实锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟绞癸拷茫锟?????
//    *=============================================================================*/
  pwm_t g_motor_pwm_ch1;  // PB6
  pwm_t g_motor_pwm_ch2;  // PB7
  pwm_t g_motor_pwm_ch3;  // PB8
  pwm_t g_motor_pwm_ch4;  // PB9

//   /*=============================================================================
//    * 锟斤拷锟斤拷锟斤拷锟斤拷
//    *=============================================================================*/
//   static void Motor_Pwm_Init(void);

//   /*=============================================================================
//    * 锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷玫锟絇WM锟斤拷4锟斤拷通锟斤拷锟斤拷全锟斤拷锟斤拷锟斤拷效锟斤拷
//    *=============================================================================*/
//   static void Motor_Pwm_Init(void)
//   {
//       // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷?4锟斤拷通锟斤拷全锟斤拷锟斤拷锟斤拷效锟斤拷锟斤拷锟斤拷转通锟斤拷占锟秸比凤拷锟斤拷实锟街ｏ拷
//       // 频锟绞ｏ拷20kHz锟斤拷锟斤拷始占锟秸比ｏ拷0%

//       // 锟斤拷锟斤拷GPIO锟斤拷锟借（锟斤拷锟斤拷锟睫革拷GPIO锟斤拷锟斤拷锟斤拷锟矫ｏ拷
//       LL_PERIPH_WE(LL_PERIPH_GPIO);

//       // CH1: PB6 - 锟斤拷锟斤拷效
//       g_motor_pwm_ch1 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH1,
//                                   GPIO_PORT_B, GPIO_PIN_06, GPIO_FUNC_4,
//                                   TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
//                                   6000, 0, PWM_ACTIVE_LOW);

//       // CH2: PB7 - 锟斤拷锟斤拷效
//       g_motor_pwm_ch2 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH2,
//                                   GPIO_PORT_B, GPIO_PIN_07, GPIO_FUNC_4,
//                                   TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
//                                   6000, 0, PWM_ACTIVE_LOW);

//       // CH3: PB8 - 锟斤拷锟斤拷效
//       g_motor_pwm_ch3 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH3,
//                                   GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_4,
//                                   TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
//                                   6000, 0, PWM_ACTIVE_LOW);

//       // CH4: PB9 - 锟斤拷锟斤拷效
//       g_motor_pwm_ch4 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH4,
//                                   GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_4,
//                                   TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
//                                   6000, 0, PWM_ACTIVE_LOW);

//       // 锟斤拷锟斤拷GPIO锟斤拷锟借（锟斤拷锟斤拷锟斤拷煤锟斤拷锟斤拷锟斤拷锟?????
//       LL_PERIPH_WP(LL_PERIPH_GPIO);

//       // 锟斤拷锟斤拷FCG锟斤拷锟借（使锟杰讹拷时锟斤拷时锟接ｏ拷
//       LL_PERIPH_WE(LL_PERIPH_FCG);

//       // 锟斤拷锟斤拷锟斤拷锟斤拷PWM锟斤拷时锟斤拷
//       PWM_Start(&g_motor_pwm_ch1);
//       PWM_Start(&g_motor_pwm_ch2);
//       PWM_Start(&g_motor_pwm_ch3);
//       PWM_Start(&g_motor_pwm_ch4);

//       // 使锟斤拷锟斤拷锟?????
//       PWM_OutputCmd(&g_motor_pwm_ch1, PWM_OUTPUT_ENABLE);
//       PWM_OutputCmd(&g_motor_pwm_ch2, PWM_OUTPUT_ENABLE);
//       PWM_OutputCmd(&g_motor_pwm_ch3, PWM_OUTPUT_ENABLE);
//       PWM_OutputCmd(&g_motor_pwm_ch4, PWM_OUTPUT_ENABLE);

//       // 锟斤拷锟斤拷FCG锟斤拷锟斤拷
//       LL_PERIPH_WP(LL_PERIPH_FCG);

//       MAIN_D("Motor PWM initialized: 4 channels, 20kHz, low active\r\n");
//   }

  /*=============================================================================
   * 锟斤拷锟斤拷锟斤拷
   *=============================================================================*/
  volatile int commu_num = 0;
  volatile int comm_mode = 0;   /* 0=停, 1=开环正转, 2=开环反转, 3=闭环正转, 4=闭环反转 */
  static int s_prev_mode = 0;
  static uint64_t s_last_step_time_us = 0;
  static uint64_t s_ramp_start_time_us = 0;
  static uint32_t s_current_interval_us = 0;
  #define COMM_PWM_FREQ_HZ           50000UL
  #define COMM_DUTY_PCT              50.0f
  /* 固定换相间隔, 无斜坡 */
  #define RAMP_START_INTERVAL_US   5000UL   /* 起步 ~667 RPM */
  #define RAMP_TARGET_INTERVAL_US  5000UL   /* 目标 ~667 RPM */
  #define RAMP_DURATION_MS         3000UL

  /* 闭环 Hall 传感器 */
  static hall_3ch_handle_t s_hall_handle = NULL;

  /* 映射表索引: 0~12, Keil Watch 里改, 无需重新编译 */
  volatile int hall_table_index = 12;

  /* 13 种映射: 0~5同向, 6~11反向, 12开环实测校正 */
  static const uint8_t hall_tables[14][8] = {
      /* === 同向: 步进+1 = 磁场CW === */
      {0xFF, 0, 2, 1, 4, 5, 3, 0xFF},   /*  0: 磁场跟转子重合 */
      {0xFF, 1, 3, 2, 5, 0, 4, 0xFF},   /*  1: 磁场领先 1 步 */
      {0xFF, 2, 4, 3, 0, 1, 5, 0xFF},   /*  2: 磁场领先 2 步 */
      {0xFF, 3, 5, 4, 1, 2, 0, 0xFF},   /*  3: 磁场领先 3 步 */
      {0xFF, 4, 0, 5, 2, 3, 1, 0xFF},   /*  4: 磁场领先 4 步 */
      {0xFF, 5, 1, 0, 3, 4, 2, 0xFF},   /*  5: 磁场领先 5 步 */
      /* === 反向: 步进+1 = 磁场CCW (与Hall序列CW相反) === */
      {0xFF, 5, 3, 4, 1, 0, 2, 0xFF},   /*  6: CW领先1 */
      {0xFF, 0, 4, 5, 2, 1, 3, 0xFF},   /*  7: CW领先2 */
      {0xFF, 1, 5, 0, 3, 2, 4, 0xFF},   /*  8: CW领先3 */
      {0xFF, 2, 0, 1, 4, 3, 5, 0xFF},   /*  9: CW领先4 */
      {0xFF, 3, 1, 2, 5, 4, 0, 0xFF},   /* 10: CW领先5 */
      {0xFF, 4, 2, 3, 0, 5, 1, 0xFF},   /* 11: CW领先0(重合) */
      /* === 开环实测校正 === */
      {0xFF, 1, 5, 0, 3, 2, 4, 0xFF},   /* 12: CW→step递减 [0x03→0,0x02→5,0x06→4,0x04→3,0x05→2,0x01→1] */
      {0xFF, 2, 0, 1, 3, 5, 4, 0xFF},   /* 13: CCW,磁场CCW超前 [0x01→2,0x02→0,0x03→1,0x04→3,0x05→5,0x06→4] */
  };

  /* Hall 回调: ISR 内调, 直接换相 */
  static void on_hall_step(uint8_t step, hall3_direction_t dir)
  {
      (void)dir;
      Commutation_Step(step, COMM_PWM_FREQ_HZ, COMM_DUTY_PCT);
  }

  /* Hall 故障回调: 000/111 → 切到滑行 */
  static void on_hall_fault(uint8_t hall_state)
  {
      (void)hall_state;
      comm_mode = 0;
      MAIN_D("[COMM] Hall fault state=0x%02X, coast", hall_state);
  }

  int main(void)
  {
      Hardware_Init();

      /* 通锟斤拷栈锟斤拷始锟斤拷 (RS485 + Modbus RTU) */
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

    //   ESystem_Init();

      /* 锟斤拷始锟斤拷锟斤拷锟较达拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥碉拷压/锟斤拷锟斤拷锟铰硷拷锟斤拷锟斤拷锟铰癸拷锟斤拷锟诫） */
    //   FaultHandler_Init();

      /* 锟斤拷始锟斤拷锟斤拷锟絇WM锟斤拷锟节碉拷锟斤拷璞革拷锟绞硷拷锟斤拷前锟斤拷? */
    //   Motor_Pwm_Init();
      tickTimer_DelayMs(5);
      static const tmr4_pwm_config_t pwm_cfg = {
          .output_type_u = TMR4_OUTPUT_SYNC,
          .output_type_v = TMR4_OUTPUT_SYNC,
          .output_type_w = TMR4_OUTPUT_SYNC,
          .freq_hz       = 50000,
          .dead_time_ns  = 0,
          .active_high   = true,
      };
      TMR4_PWM_Config(&pwm_cfg);

      MAIN_D("[MAIN] TMR4 Config done, starting output");
      TMR4_PWM_StartOutput();

      /* 初始化定时器, 用于换相计时 */
      Timer6_Timebase_Init();
      Timer6_Timebase_Start();

      /* 六步换相: 上电默认滑行态 (三相98% SYNC -> 上管全通 -> 同电位 -> 自由滑行) */
      int ch;
      for (ch = 0; ch < 3; ch++) {
          TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_SYNC, 98.0f);
      }

      /* 初始化 Hall 传感器 (3路闭环) */
      static const hall_3ch_config_t hall_cfg = {
          .port      = {GPIO_PORT_A, GPIO_PORT_A, GPIO_PORT_A},
          .pin       = {GPIO_PIN_10, GPIO_PIN_09, GPIO_PIN_08},  /* U=PA10, V=PA9, W=PA8 */
          .eirq_ch   = {EXTINT_CH10, EXTINT_CH09, EXTINT_CH08},
          .irqn      = {INT010_IRQn, INT009_IRQn, INT008_IRQn},
          .irq_src   = {INT_SRC_PORT_EIRQ10, INT_SRC_PORT_EIRQ9, INT_SRC_PORT_EIRQ8},
          .irq_priority = DDL_IRQ_PRIO_02,
          .pole_pairs   = 3,
          .hall_to_step = {0xFF,1,3,2,5,0,4,0xFF},   /* step0→0x01, 磁场领先: 0x01→1,0x02→3,0x03→2,0x04→5,0x05→0,0x06→4 */
          .on_step      = on_hall_step,
          .on_fault     = on_hall_fault,
          .align_step        = 0,
          .align_duty_pct    = COMM_DUTY_PCT,
          .align_duration_ms = 500,
          .stall_timeout_ms  = 500,
      };
      s_hall_handle = hall_3ch_create(&hall_cfg);
      MAIN_D("[MAIN] Hall sensor created");

      EventBus_Enable();

      while (1) {
          App_Comm_Poll();

          /* 模式切换检测 */
          if (comm_mode != s_prev_mode) {
              s_prev_mode = comm_mode;

              if (comm_mode == 0) {
                  /* 滑行 */
                  int ch;
                  hall_3ch_stop(s_hall_handle);
                  for (ch = 0; ch < 3; ch++) {
                      TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_SYNC, 98.0f);
                  }
                  MAIN_D("[COMM] Mode=0: COAST");
              } else if (comm_mode == 1 || comm_mode == 2) {
                  /* 开环启动 */
                  hall_3ch_stop(s_hall_handle);
                  commu_num = 0;
                  s_ramp_start_time_us = Timer6_Timebase_GetTimestamp();
                  s_current_interval_us = RAMP_START_INTERVAL_US;
                  s_last_step_time_us = s_ramp_start_time_us;
                  Commutation_Init();
                  COMM_STEP_UH_VL(COMM_PWM_FREQ_HZ, COMM_DUTY_PCT);
                  MAIN_D("[COMM] Mode=%d: OPEN-LOOP START", comm_mode);
              } else if (comm_mode == 3) {
                  /* CW: 表12, 踢step+1 */
                  hall_3ch_set_table(s_hall_handle, hall_tables[12]);
                  hall_3ch_start(s_hall_handle, HALL3_DIR_FORWARD);
                  MAIN_D("[COMM] Mode=3: CW (table12)");
              } else if (comm_mode == 4) {
                  /* CCW: 表12(同表), 踢step+5 */
                  hall_3ch_set_table(s_hall_handle, hall_tables[12]);
                  hall_3ch_start(s_hall_handle, HALL3_DIR_REVERSE);
                  MAIN_D("[COMM] Mode=4: CCW (table12, rev kick)");
              }
          }

          /* 开环运行: 定时步进 */
          if (comm_mode == 1 || comm_mode == 2) {
              Timer6_Timebase_UpdateTimestamp();
              uint64_t now = Timer6_Timebase_GetTimestamp();

              uint64_t ramp_elapsed = now - s_ramp_start_time_us;
              uint64_t ramp_total = RAMP_DURATION_MS * 1000UL;
              if (ramp_elapsed < ramp_total) {
                  s_current_interval_us = RAMP_START_INTERVAL_US
                      - (uint32_t)((RAMP_START_INTERVAL_US - RAMP_TARGET_INTERVAL_US)
                                   * ramp_elapsed / ramp_total);
              } else {
                  s_current_interval_us = RAMP_TARGET_INTERVAL_US;
              }

              if ((now - s_last_step_time_us) >= s_current_interval_us) {
                  s_last_step_time_us = now;
                  if (comm_mode == 1) {
                      commu_num = (commu_num + 1) % 6;
                  } else {
                      commu_num = (commu_num + 5) % 6;
                  }
                  Commutation_Step((uint8_t)commu_num, COMM_PWM_FREQ_HZ, COMM_DUTY_PCT);
              }
          }

          /* 闭环运行: Hall ISR 驱动, 只做维护 */
          if (comm_mode == 3 || comm_mode == 4) {
              hall_3ch_update(s_hall_handle);
              if (hall_3ch_is_stalled(s_hall_handle)) {
                  comm_mode = 0;
                  MAIN_D("[COMM] Closed-loop stall, coast");
              }
          }
      }
  }
