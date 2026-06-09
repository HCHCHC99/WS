
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
  volatile int comm_mode = 0;   /* 0=Í£, 1=¿ª»·Õý×ª, 2=¿ª»··´×ª, 3=±Õ»·Õý×ª, 4=±Õ»··´×ª */
  static int s_prev_mode = 0;
  static uint64_t s_last_step_time_us = 0;
  static uint64_t s_ramp_start_time_us = 0;
  static uint32_t s_current_interval_us = 0;
  #define COMM_PWM_FREQ_HZ           50000UL
  #define COMM_DUTY_PCT              50.0f
  /* ¹Ì¶¨»»Ïà¼ä¸ô, ÎÞÐ±ÆÂ */
  #define RAMP_START_INTERVAL_US   5000UL   /* Æð²½ ~667 RPM */
  #define RAMP_TARGET_INTERVAL_US  5000UL   /* Ä¿±ê ~667 RPM */
  #define RAMP_DURATION_MS         3000UL

  /* ±Õ»· Hall ´«¸ÐÆ÷ */
  static hall_3ch_handle_t s_hall_handle = NULL;

  /* Ó³Éä±íË÷Òý: 0~15, Keil Watch Àï¸Ä, ÎÞÐèÖØÐÂ±àÒë */
  volatile int hall_table_index = 14;

  /* 16 ÖÖÓ³Éä: 0~5Í¬Ïò, 6~11·´Ïò, 12¿ª»·Êµ²âÐ£Õý, 13 CCW(´Å³¡³¬Ç°), 14 ÍÆµ¼Õý×ª, 15 ÍÆµ¼·´×ª */
  static const uint8_t hall_tables[16][8] = {
      /* === Í¬Ïò: ²½½ø+1 = ´Å³¡CW === */
      {0xFF, 0, 2, 1, 4, 5, 3, 0xFF},   /*  0: ´Å³¡¸ú×ª×ÓÖØºÏ */
      {0xFF, 1, 3, 2, 5, 0, 4, 0xFF},   /*  1: ´Å³¡ÁìÏÈ 1 ²½ */
      {0xFF, 2, 4, 3, 0, 1, 5, 0xFF},   /*  2: ´Å³¡ÁìÏÈ 2 ²½ */
      {0xFF, 3, 5, 4, 1, 2, 0, 0xFF},   /*  3: ´Å³¡ÁìÏÈ 3 ²½ */
      {0xFF, 4, 0, 5, 2, 3, 1, 0xFF},   /*  4: ´Å³¡ÁìÏÈ 4 ²½ */
      {0xFF, 5, 1, 0, 3, 4, 2, 0xFF},   /*  5: ´Å³¡ÁìÏÈ 5 ²½ */
      /* === ·´Ïò: ²½½ø+1 = ´Å³¡CCW (ÓëHallÐòÁÐCWÏà·´) === */
      {0xFF, 5, 3, 4, 1, 0, 2, 0xFF},   /*  6: CWÁìÏÈ1 */
      {0xFF, 0, 4, 5, 2, 1, 3, 0xFF},   /*  7: CWÁìÏÈ2 */
      {0xFF, 1, 5, 0, 3, 2, 4, 0xFF},   /*  8: CWÁìÏÈ3 */
      {0xFF, 2, 0, 1, 4, 3, 5, 0xFF},   /*  9: CWÁìÏÈ4 */
      {0xFF, 3, 1, 2, 5, 4, 0, 0xFF},   /* 10: CWÁìÏÈ5 */
      {0xFF, 4, 2, 3, 0, 5, 1, 0xFF},   /* 11: CWÁìÏÈ0(ÖØºÏ) */
      /* === ¿ª»·Êµ²âÐ£Õý === */
      {0xFF, 1, 5, 0, 3, 2, 4, 0xFF},   /* 12: CW¡ústepµÝ¼õ [0x03¡ú0,0x02¡ú5,0x06¡ú4,0x04¡ú3,0x05¡ú2,0x01¡ú1] */
      {0xFF, 2, 0, 1, 3, 5, 4, 0xFF},   /* 13: CCW,´Å³¡CCW³¬Ç° [0x01¡ú2,0x02¡ú0,0x03¡ú1,0x04¡ú3,0x05¡ú5,0x06¡ú4] */
      /* === ÀíÂÛÍÆµ¼: ÉÈÇø¡À90¡ã ¡ú µçÑ¹Ê¸Á¿ ¡ú step === */
      {0xFF, 2, 0, 1, 4, 3, 5, 0xFF},   /* 14: ÍÆµ¼Õý×ª [sector+90¡ã, hall¡ústep: 0x01¡ú2,0x02¡ú0,0x03¡ú1,0x04¡ú4,0x05¡ú3,0x06¡ú5] */
      {0xFF, 5, 3, 4, 1, 0, 2, 0xFF},   /* 15: ÍÆµ¼·´×ª [sector-90¡ã, hall¡ústep: 0x01¡ú5,0x02¡ú3,0x03¡ú4,0x04¡ú1,0x05¡ú0,0x06¡ú2] */
  };

  /* Hall »Øµ÷: ISR ÄÚµ÷, Ö±½Ó»»Ïà */
  static void on_hall_step(uint8_t step, hall3_direction_t dir)
  {
      (void)dir;
      Commutation_Step(step, COMM_PWM_FREQ_HZ, COMM_DUTY_PCT);
  }

  /* Hall ¹ÊÕÏ»Øµ÷: 000/111 ¡ú ÇÐµ½»¬ÐÐ */
  static void on_hall_fault(uint8_t hall_state)
  {
      (void)hall_state;
      comm_mode = 0;
      MAIN_D("[COMM] Hall fault state=0x%02X, coast", hall_state);
  }

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

      /* ³õÊ¼»¯ Hall ´«¸ÐÆ÷ (3Â·±Õ»·) */
      static const hall_3ch_config_t hall_cfg = {
          .port      = {GPIO_PORT_A, GPIO_PORT_A, GPIO_PORT_A},
          .pin       = {GPIO_PIN_10, GPIO_PIN_09, GPIO_PIN_08},  /* U=PA10, V=PA9, W=PA8 */
          .eirq_ch   = {EXTINT_CH10, EXTINT_CH09, EXTINT_CH08},
          .irqn      = {INT010_IRQn, INT009_IRQn, INT008_IRQn},
          .irq_src   = {INT_SRC_PORT_EIRQ10, INT_SRC_PORT_EIRQ9, INT_SRC_PORT_EIRQ8},
          .irq_priority = DDL_IRQ_PRIO_02,
          .pole_pairs   = 3,
          .hall_to_step = {0xFF,1,3,2,5,0,4,0xFF},   /* step0¡ú0x01, ´Å³¡ÁìÏÈ: 0x01¡ú1,0x02¡ú3,0x03¡ú2,0x04¡ú5,0x05¡ú0,0x06¡ú4 */
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

          /* Ä£Ê½ÇÐ»»¼ì²â */
          if (comm_mode != s_prev_mode) {
              s_prev_mode = comm_mode;

              if (comm_mode == 0) {
                  /* »¬ÐÐ */
                  int ch;
                  hall_3ch_stop(s_hall_handle);
                  for (ch = 0; ch < 3; ch++) {
                      TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_SYNC, 98.0f);
                  }
                  MAIN_D("[COMM] Mode=0: COAST");
              } else if (comm_mode == 1 || comm_mode == 2) {
                  /* ¿ª»·Æô¶¯ */
                  hall_3ch_stop(s_hall_handle);
                  commu_num = 0;
                  s_ramp_start_time_us = Timer6_Timebase_GetTimestamp();
                  s_current_interval_us = RAMP_START_INTERVAL_US;
                  s_last_step_time_us = s_ramp_start_time_us;
                  Commutation_Init();
                  COMM_STEP_UH_VL(COMM_PWM_FREQ_HZ, COMM_DUTY_PCT);
                  MAIN_D("[COMM] Mode=%d: OPEN-LOOP START", comm_mode);
              } else if (comm_mode == 3) {
                  /* CW: ±í14, ÉÈÇø+90¡ãÕý×ª */
                  hall_3ch_set_table(s_hall_handle, hall_tables[14]);
                  hall_3ch_start(s_hall_handle, HALL3_DIR_FORWARD);
                  MAIN_D("[COMM] Mode=3: CW (table14, sector+90)");
              } else if (comm_mode == 4) {
                  /* CCW: ±í15, ÉÈÇø-90¡ã·´×ª */
                  hall_3ch_set_table(s_hall_handle, hall_tables[15]);
                  hall_3ch_start(s_hall_handle, HALL3_DIR_REVERSE);
                  MAIN_D("[COMM] Mode=4: CCW (table15, sector-90)");
              }
          }

          /* ¿ª»·ÔËÐÐ: ¶¨Ê±²½½ø */
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

          /* ±Õ»·ÔËÐÐ: Hall ISR Çý¶¯, Ö»×öÎ¬»¤ */
          if (comm_mode == 3 || comm_mode == 4) {
              hall_3ch_update(s_hall_handle);
              if (hall_3ch_is_stalled(s_hall_handle)) {
                  comm_mode = 0;
                  MAIN_D("[COMM] Closed-loop stall, coast");
              }
          }
      }
  }
