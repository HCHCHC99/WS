
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

  /*=============================================================================
   * 全局PWM实例（供电机控制使用）
   *=============================================================================*/
  pwm_t g_motor_pwm_ch1;  // PB6
  pwm_t g_motor_pwm_ch2;  // PB7
  pwm_t g_motor_pwm_ch3;  // PB8
  pwm_t g_motor_pwm_ch4;  // PB9

  /*=============================================================================
   * 函数声明
   *=============================================================================*/
  static void Motor_Pwm_Init(void);

  /*=============================================================================
   * 初始化电机控制用的PWM（4个通道，全部低有效）
   *=============================================================================*/
  static void Motor_Pwm_Init(void)
  {
      // 电机控制需要4个通道全部低有效（正反转通过占空比分配实现）
      // 频率：20kHz，初始占空比：0%

      // 解锁GPIO外设（允许修改GPIO功能配置）
      LL_PERIPH_WE(LL_PERIPH_GPIO);

      // CH1: PB6 - 低有效
      g_motor_pwm_ch1 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH1,
                                  GPIO_PORT_B, GPIO_PIN_06, GPIO_FUNC_4,
                                  TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                  6000, 0, PWM_ACTIVE_LOW);

      // CH2: PB7 - 低有效
      g_motor_pwm_ch2 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH2,
                                  GPIO_PORT_B, GPIO_PIN_07, GPIO_FUNC_4,
                                  TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                  6000, 0, PWM_ACTIVE_LOW);

      // CH3: PB8 - 低有效
      g_motor_pwm_ch3 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH3,
                                  GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_4,
                                  TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                  6000, 0, PWM_ACTIVE_LOW);

      // CH4: PB9 - 低有效
      g_motor_pwm_ch4 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH4,
                                  GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_4,
                                  TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                  6000, 0, PWM_ACTIVE_LOW);

      // 锁定GPIO外设（完成配置后锁定）
      LL_PERIPH_WP(LL_PERIPH_GPIO);

      // 解锁FCG外设（使能定时器时钟）
      LL_PERIPH_WE(LL_PERIPH_FCG);

      // 启动所有PWM定时器
      PWM_Start(&g_motor_pwm_ch1);
      PWM_Start(&g_motor_pwm_ch2);
      PWM_Start(&g_motor_pwm_ch3);
      PWM_Start(&g_motor_pwm_ch4);

      // 使能输出
      PWM_OutputCmd(&g_motor_pwm_ch1, PWM_OUTPUT_ENABLE);
      PWM_OutputCmd(&g_motor_pwm_ch2, PWM_OUTPUT_ENABLE);
      PWM_OutputCmd(&g_motor_pwm_ch3, PWM_OUTPUT_ENABLE);
      PWM_OutputCmd(&g_motor_pwm_ch4, PWM_OUTPUT_ENABLE);

      // 锁定FCG外设
      LL_PERIPH_WP(LL_PERIPH_FCG);

      MAIN_D("Motor PWM initialized: 4 channels, 20kHz, low active\r\n");
  }

  /*=============================================================================
   * 主函数
   *=============================================================================*/
  int main(void)
  {
      Hardware_Init();

      /* 通信栈初始化 (RS485 + Modbus RTU) */
      static const App_Comm_Config_t comm_cfg = {
          .phy.baudrate     = 9600,
          .phy.dir_polarity = 0,
          .hal.rx_buf_size  = 500,
          .hal.tx_buf_size  = 500,
          .hal.rx_frame_queue_depth = 10,
          .hal.tx_queue_depth       = 10,
          .hal.frame_timeout_ms     = 0,
          .proto.node_id            = 1,
          .proto.enable_write_multi = false,
      };
      App_Comm_Init(&comm_cfg);

      ESystem_Init();

      /* 初始化故障处理器（订阅电压/电流事件，更新故障码） */
      FaultHandler_Init();

      /* 初始化电机PWM（在电机设备初始化之前） */
      Motor_Pwm_Init();

      /*=========================================================================
       * 电机控制模式变量（可在Keil Watch窗口中修改）
       * 0: 停止, 1: 正转, 2: 反转
       *=========================================================================*/
      // volatile uint8_t motor_mode = 0;

      // MotorDevice_t* motor = NULL;       // TODO: 获取电机设备指针
      EventBus_Enable();

      while (1)
      {
          ESystem_MainLoop();
          App_Comm_Poll();

          // 更新PWM状态（处理缓启动）
          PWM_Update(&g_motor_pwm_ch1);
          PWM_Update(&g_motor_pwm_ch2);
          PWM_Update(&g_motor_pwm_ch3);
          PWM_Update(&g_motor_pwm_ch4);

          // // 每次循环都根据 motor_mode 调用对应函数
          // if (motor_mode == 0) {
          //     Motor_OnArbitrationStop(motor);
          // } else if (motor_mode == 1) {
          //     Motor_OnArbitrationFwd(motor, 0.0f);
          // } else if (motor_mode == 2) {
          //     Motor_OnArbitrationRev(motor, 0.0f);
          // }
          Param_PrintAllValues();
      }
  }
  