
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

//   /*=============================================================================
//    * 全锟斤拷PWM实锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟绞癸拷茫锟?
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
//       // 锟斤拷锟斤拷锟斤拷锟斤拷锟揭?4锟斤拷通锟斤拷全锟斤拷锟斤拷锟斤拷效锟斤拷锟斤拷锟斤拷转通锟斤拷占锟秸比凤拷锟斤拷实锟街ｏ拷
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

//       // 锟斤拷锟斤拷GPIO锟斤拷锟借（锟斤拷锟斤拷锟斤拷煤锟斤拷锟斤拷锟斤拷锟?
//       LL_PERIPH_WP(LL_PERIPH_GPIO);

//       // 锟斤拷锟斤拷FCG锟斤拷锟借（使锟杰讹拷时锟斤拷时锟接ｏ拷
//       LL_PERIPH_WE(LL_PERIPH_FCG);

//       // 锟斤拷锟斤拷锟斤拷锟斤拷PWM锟斤拷时锟斤拷
//       PWM_Start(&g_motor_pwm_ch1);
//       PWM_Start(&g_motor_pwm_ch2);
//       PWM_Start(&g_motor_pwm_ch3);
//       PWM_Start(&g_motor_pwm_ch4);

//       // 使锟斤拷锟斤拷锟?
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

      ESystem_Init();

      /* 锟斤拷始锟斤拷锟斤拷锟较达拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥碉拷压/锟斤拷锟斤拷锟铰硷拷锟斤拷锟斤拷锟铰癸拷锟斤拷锟诫） */
      FaultHandler_Init();

      /* 锟斤拷始锟斤拷锟斤拷锟絇WM锟斤拷锟节碉拷锟斤拷璞革拷锟绞硷拷锟街
iconv: D:/WS/ws_v.1.1/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/source/main.c:117:73: cannot convert
