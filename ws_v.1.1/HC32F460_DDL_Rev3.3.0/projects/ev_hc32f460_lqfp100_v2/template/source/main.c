
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
#include "Template_tmr4_pwm.h"

  /*=============================================================================
   * ȫ��PWMʵ�������������ʹ�ã�?
   *=============================================================================*/
  pwm_t g_motor_pwm_ch1;  // PB6
  pwm_t g_motor_pwm_ch2;  // PB7
  pwm_t g_motor_pwm_ch3;  // PB8
  pwm_t g_motor_pwm_ch4;  // PB9

  /*=============================================================================
   * ��������
   *=============================================================================*/
  static void Motor_Pwm_Init(void);

  /*=============================================================================
   * ��ʼ����������õ�PWM��4��ͨ����ȫ������Ч��
   *=============================================================================*/
  static void Motor_Pwm_Init(void)
  {
      // ����������?4��ͨ��ȫ������Ч������תͨ��ռ�ձȷ���ʵ�֣�
      // Ƶ�ʣ�20kHz����ʼռ�ձȣ�0%

      // ����GPIO���裨�����޸�GPIO�������ã�
      LL_PERIPH_WE(LL_PERIPH_GPIO);

      // CH1: PB6 - ����Ч
      g_motor_pwm_ch1 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH1,
                                  GPIO_PORT_B, GPIO_PIN_06, GPIO_FUNC_4,
                                  TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                  6000, 0, PWM_ACTIVE_LOW);

      // CH2: PB7 - ����Ч
      g_motor_pwm_ch2 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH2,
                                  GPIO_PORT_B, GPIO_PIN_07, GPIO_FUNC_4,
                                  TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                  6000, 0, PWM_ACTIVE_LOW);

      // CH3: PB8 - ����Ч
      g_motor_pwm_ch3 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH3,
                                  GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_4,
                                  TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                  6000, 0, PWM_ACTIVE_LOW);

      // CH4: PB9 - ����Ч
      g_motor_pwm_ch4 = PWM_Init(CM_TMRA_4, FCG2_PERIPH_TMRA_4, TMRA_CH4,
                                  GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_4,
                                  TMRA_MD_SAWTOOTH, TMRA_DIR_UP,
                                  6000, 0, PWM_ACTIVE_LOW);

      // ����GPIO���裨������ú�������?
      LL_PERIPH_WP(LL_PERIPH_GPIO);

      // ����FCG���裨ʹ�ܶ�ʱ��ʱ�ӣ�
      LL_PERIPH_WE(LL_PERIPH_FCG);

      // ��������PWM��ʱ��
      PWM_Start(&g_motor_pwm_ch1);
      PWM_Start(&g_motor_pwm_ch2);
      PWM_Start(&g_motor_pwm_ch3);
      PWM_Start(&g_motor_pwm_ch4);

      // ʹ�����?
      PWM_OutputCmd(&g_motor_pwm_ch1, PWM_OUTPUT_ENABLE);
      PWM_OutputCmd(&g_motor_pwm_ch2, PWM_OUTPUT_ENABLE);
      PWM_OutputCmd(&g_motor_pwm_ch3, PWM_OUTPUT_ENABLE);
      PWM_OutputCmd(&g_motor_pwm_ch4, PWM_OUTPUT_ENABLE);

      // ����FCG����
      LL_PERIPH_WP(LL_PERIPH_FCG);

      MAIN_D("Motor PWM initialized: 4 channels, 20kHz, low active\r\n");
  }

  /*=============================================================================
   * ������
   *=============================================================================*/
  int main(void)
  {
      Hardware_Init();

      /* ͨ��ջ��ʼ�� (RS485 + Modbus RTU) */
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

      /* ��ʼ�����ϴ����������ĵ�ѹ/�����¼������¹����룩 */
      FaultHandler_Init();

      /* ��ʼ�� TMR4 6·����PWM (PB4~PB9: UH UL VH VL WH WL) */
      TMR4_PWM_Config();

      /*=========================================================================
       * �������ģʽ����������Keil Watch�������޸ģ�
       * 0: ֹͣ, 1: ��ת, 2: ��ת
       *=========================================================================*/
      // volatile uint8_t motor_mode = 0;

      // MotorDevice_t* motor = NULL;       // TODO: ��ȡ����豸ָ��?
      EventBus_Enable();

      while (1)
      {
          ESystem_MainLoop();
          App_Comm_Poll();

          // ����PWM״̬��������������
        //   PWM_Update(&g_motor_pwm_ch1);
        //   PWM_Update(&g_motor_pwm_ch2);
        //   PWM_Update(&g_motor_pwm_ch3);
        //   PWM_Update(&g_motor_pwm_ch4);

          // // ÿ��ѭ�������� motor_mode ���ö�Ӧ����
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
  