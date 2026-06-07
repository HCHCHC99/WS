
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

//   /*=============================================================================
//    * È«ï¿½ï¿½PWMÊµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¹ï¿½Ã£ï¿?????
//    *=============================================================================*/
  pwm_t g_motor_pwm_ch1;  // PB6
  pwm_t g_motor_pwm_ch2;  // PB7
  pwm_t g_motor_pwm_ch3;  // PB8
  pwm_t g_motor_pwm_ch4;  // PB9

//   /*=============================================================================
//    * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//    *=============================================================================*/
//   static void Motor_Pwm_Init(void);

//   /*=============================================================================
//    * ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãµï¿½PWMï¿½ï¿½4ï¿½ï¿½Í¨ï¿½ï¿½ï¿½ï¿½È«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð§ï¿½ï¿½
//    *=============================================================================*/
//   static void Motor_Pwm_Init(void)
//   {
//       // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?4ï¿½ï¿½Í¨ï¿½ï¿½È«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð§ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ªÍ¨ï¿½ï¿½Õ¼ï¿½Õ±È·ï¿½ï¿½ï¿½Êµï¿½Ö£ï¿½
//       // Æµï¿½Ê£ï¿½20kHzï¿½ï¿½ï¿½ï¿½Ê¼Õ¼ï¿½Õ±È£ï¿½0%

//       // ï¿½ï¿½ï¿½ï¿½GPIOï¿½ï¿½ï¿½è£¨ï¿½ï¿½ï¿½ï¿½ï¿½Þ¸ï¿½GPIOï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã£ï¿½
//       LL_PERIPH_WE(LL_PERIPH_GPIO);

//       // CH1: PB6 - ï¿½ï¿½ï¿½ï¿½Ð§
//       g_motor_pwm_ch1 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH1,
//                                   GPIO_PORT_B, GPIO_PIN_06, GPIO_FUNC_4,
//                                   TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
//                                   6000, 0, PWM_ACTIVE_LOW);

//       // CH2: PB7 - ï¿½ï¿½ï¿½ï¿½Ð§
//       g_motor_pwm_ch2 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH2,
//                                   GPIO_PORT_B, GPIO_PIN_07, GPIO_FUNC_4,
//                                   TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
//                                   6000, 0, PWM_ACTIVE_LOW);

//       // CH3: PB8 - ï¿½ï¿½ï¿½ï¿½Ð§
//       g_motor_pwm_ch3 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH3,
//                                   GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_4,
//                                   TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
//                                   6000, 0, PWM_ACTIVE_LOW);

//       // CH4: PB9 - ï¿½ï¿½ï¿½ï¿½Ð§
//       g_motor_pwm_ch4 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH4,
//                                   GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_4,
//                                   TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
//                                   6000, 0, PWM_ACTIVE_LOW);

//       // ï¿½ï¿½ï¿½ï¿½GPIOï¿½ï¿½ï¿½è£¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãºï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?????
//       LL_PERIPH_WP(LL_PERIPH_GPIO);

//       // ï¿½ï¿½ï¿½ï¿½FCGï¿½ï¿½ï¿½è£¨Ê¹ï¿½Ü¶ï¿½Ê±ï¿½ï¿½Ê±ï¿½Ó£ï¿½
//       LL_PERIPH_WE(LL_PERIPH_FCG);

//       // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½PWMï¿½ï¿½Ê±ï¿½ï¿½
//       PWM_Start(&g_motor_pwm_ch1);
//       PWM_Start(&g_motor_pwm_ch2);
//       PWM_Start(&g_motor_pwm_ch3);
//       PWM_Start(&g_motor_pwm_ch4);

//       // Ê¹ï¿½ï¿½ï¿½ï¿½ï¿?????
//       PWM_OutputCmd(&g_motor_pwm_ch1, PWM_OUTPUT_ENABLE);
//       PWM_OutputCmd(&g_motor_pwm_ch2, PWM_OUTPUT_ENABLE);
//       PWM_OutputCmd(&g_motor_pwm_ch3, PWM_OUTPUT_ENABLE);
//       PWM_OutputCmd(&g_motor_pwm_ch4, PWM_OUTPUT_ENABLE);

//       // ï¿½ï¿½ï¿½ï¿½FCGï¿½ï¿½ï¿½ï¿½
//       LL_PERIPH_WP(LL_PERIPH_FCG);

//       MAIN_D("Motor PWM initialized: 4 channels, 20kHz, low active\r\n");
//   }

  /*=============================================================================
   * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
   *=============================================================================*/
  volatile int commu_num = 0;
  volatile int comm_mode = 0;   /* 0=Í£, 1=Õý×ª, 2=·´×ª */
  static int s_prev_mode = 0;
  static uint64_t s_last_step_time_us = 0;
  static uint64_t s_ramp_start_time_us = 0;
  static uint32_t s_current_interval_us = 0;
  #define COMM_PWM_FREQ_HZ           50000UL
  #define COMM_DUTY_PCT              50.0f
  /* Ð±ÆÂ¼ÓËÙ: ´ÓÂýµ½¿ì, 3ÃëÍê³É */
  #define RAMP_START_INTERVAL_US  10000UL   /* Æð²½ ~333 RPM */
  #define RAMP_TARGET_INTERVAL_US  3333UL   /* Ä¿±ê ~1000 RPM */
  #define RAMP_DURATION_MS         3000UL   /* ¼ÓËÙÊ±¼ä 3Ãë */
  int main(void)
  {
      Hardware_Init();

      /* Í¨ï¿½ï¿½Õ»ï¿½ï¿½Ê¼ï¿½ï¿½ (RS485 + Modbus RTU) */
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

      /* ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½Ï´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Äµï¿½Ñ¹/ï¿½ï¿½ï¿½ï¿½ï¿½Â¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â¹ï¿½ï¿½ï¿½ï¿½ë£© */
    //   FaultHandler_Init();

      /* ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½PWMï¿½ï¿½ï¿½Úµï¿½ï¿½ï¿½è±¸ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½Ç°ï¿½ï¿½? */
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

      /* ³õÊ¼»¯¶¨Ê±Æ÷, ÓÃÓÚ»»Ïà¼ÆÊ± */
      Timer6_Timebase_Init();
      Timer6_Timebase_Start();

      /* Áù²½»»Ïà: ÉÏµçÄ¬ÈÏ»¬ÐÐÌ¬ (ÈýÏà98% SYNC -> ÉÏ¹ÜÈ«Í¨ -> Í¬µçÎ» -> ×ÔÓÉ»¬ÐÐ) */
      int ch;
      for (ch = 0; ch < 3; ch++) {
          TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_SYNC, 98.0f);
      }
      s_prev_mode = 0;

      EventBus_Enable();

      while (1) {
          App_Comm_Poll();

          /* Ä£Ê½ÇÐ»»¼ì²â */
          if (comm_mode != s_prev_mode) {
              s_prev_mode = comm_mode;

              if (comm_mode == 0) {
                  /* »¬ÐÐ: ÈýÏàÈ«ÉèÎª98% SYNC -> H=L -> H=HIGH/L=HIGH ÉÏ¹ÜÈ«Í¨ -> ÈýÏàÍ¬µçÎ» -> ×ÔÓÉ»¬ÐÐ */
                  int ch;
                  for (ch = 0; ch < 3; ch++) {
                      TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_SYNC, 98.0f);
                  }
                  MAIN_D("[COMM] Mode=0: COAST");
              } else {
                  /* Æô¶¯: ¸´Î»µ½ step0, ¿ªÊ¼Ð±ÆÂ¼ÓËÙ */
                  commu_num = 0;
                  s_ramp_start_time_us = Timer6_Timebase_GetTimestamp();
                  s_current_interval_us = RAMP_START_INTERVAL_US;
                  s_last_step_time_us = s_ramp_start_time_us;
                  Commutation_Init();
                  COMM_STEP_UH_VL(COMM_PWM_FREQ_HZ, COMM_DUTY_PCT);
                  MAIN_D("[COMM] Mode=%d: START, ramp %lu -> %lu us",
                         comm_mode, (uint32_t)RAMP_START_INTERVAL_US, (uint32_t)RAMP_TARGET_INTERVAL_US);
              }
          }

          /* ÔËÐÐ×´Ì¬: Ð±ÆÂ¼ÓËÙ + ¶¨Ê±²½½ø */
          if (comm_mode != 0) {
              Timer6_Timebase_UpdateTimestamp();
              uint64_t now = Timer6_Timebase_GetTimestamp();

              /* Ð±ÆÂ: ÏßÐÔ¼ÓËÙ, ´Ó RAMP_START µ½ RAMP_TARGET */
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
                      /* Õý×ª: step +1 */
                      commu_num = (commu_num + 1) % 6;
                  } else {
                      /* ·´×ª: step -1 (µÈ¼ÛÓÚ +5 mod 6) */
                      commu_num = (commu_num + 5) % 6;
                  }
                  Commutation_Step((uint8_t)commu_num, COMM_PWM_FREQ_HZ, COMM_DUTY_PCT);
              }
          }
      }
  }
