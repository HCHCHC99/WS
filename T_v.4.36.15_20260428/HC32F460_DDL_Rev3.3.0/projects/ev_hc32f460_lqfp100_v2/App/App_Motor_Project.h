#ifndef APP_MOTOR_PROJECT_H_
#define APP_MOTOR_PROJECT_H_

#include "device_manager.h"
#include "EventBus.h"
#include "dev_motor.h"          // ï¿½ï¿½ï¿½ï¿½è±¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù²ï¿½ï¿½ß¼ï¿½ï¿½ï¿?
#include "dev_pwm.h"            // PWMï¿½è±¸
#include "dev_power.h"          // ï¿½ï¿½Ô´ï¿½è±¸
#include "dev_io.h"             // IOï¿½è±¸
#include "dev_hall.h"           // ï¿½ï¿½ï¿½ï¿½ï¿½è±¸

// ï¿½ï¿½Ê±×¢ï¿½Íµï¿½Î´Êµï¿½Öµï¿½ï¿½è±¸
#include "dev_adc.h"            // ADCï¿½è±¸
#include "dev_voltage.h"        // ï¿½ï¿½Ñ¹Ä¸ï¿½ï¿½ï¿½è±¸
#include "dev_sensor.h"         // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è±¸
// #include "dev_actuator.h"       // Ö´ï¿½ï¿½ï¿½ï¿½ï¿½è±¸
// #include "dev_can.h"            // CANï¿½è±¸
#include "dev_motor_hall.h"     // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è±?

// --- Ä£ï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½Æºê£¨1=ï¿½ï¿½ï¿½ï¿½Ä£ï¿½ï¿½, 0=Ê¹ï¿½ï¿½ï¿½ï¿½ÊµÓ²ï¿½ï¿½ï¿½ï¿½---
#ifndef ENABLE_SIMULATION_MODE
#define ENABLE_SIMULATION_MODE  1

// --- »»ÏàÄ£Ê½¿ØÖÆºê£¨0=ÒÀÀµHall´«¸ÐÆ÷, 1=¿ª»·ÎÞ´«¸ÐÆ÷£©---
#ifndef MOTOR_COMMUTATION_SENSORLESS
#define MOTOR_COMMUTATION_SENSORLESS  0
#endif

// --- ¿ª»·»»Ïà¼ä¸ôÅäÖÃ£¨½ö MOTOR_COMMUTATION_SENSORLESS=1 Ê±ÓÐÐ§£©---
// »»Ïà¼ä¸ô = ×îÐ¡¼ä¸ô + (×î´ó¼ä¸ô-×îÐ¡¼ä¸ô) * (1 - duty/100)
// duty=100% Ê±Ê¹ÓÃ×îÐ¡¼ä¸ô£¨×î¿ì£©£¬duty=0% Ê±Ê¹ÓÃ×î´ó¼ä¸ô£¨×îÂý£©
#define COMM_STEP_INTERVAL_MIN_US   800     // ×îÐ¡»»Ïà²½¼ä¸ô(us)£¬¶ÔÓ¦×î¸ß×ªËÙ
#define COMM_STEP_INTERVAL_MAX_US   5000    // ×î´ó»»Ïà²½¼ä¸ô(us)£¬¶ÔÓ¦×îµÍ×ªËÙ

#endif

// ========== ï¿½ï¿½ï¿½×´Ì¬Ã¶ï¿½ï¿? ==========
#define MOTOR_STOPPED    0
#define MOTOR_FORWARD    1
#define MOTOR_REVERSE    2
#define MOTOR_FAULT      3

// ========== ï¿½ï¿½Ô´×´Ì¬Ã¶ï¿½ï¿½ ==========
#define POWER_BOTH_OFF   0
#define POWER_POS_ON     1
#define POWER_NEG_ON     2
#define POWER_BOTH_ON    3

// ========== ï¿½ï¿½ï¿½ï¿½×´Ì¬Ã¶ï¿½ï¿½ ==========
#define HALL_NO_LIMIT    0
#define HALL_UP_LIMIT    1
#define HALL_DOWN_LIMIT  2
#define HALL_BOTH_LIMIT  3

// ========== È«ï¿½ï¿½ï¿½è±¸IDï¿½ï¿½ï¿½ï¿½ ==========
#define ID_PWM_MOTOR        9   // ï¿½ï¿½ï¿½PWMï¿½è±¸
#define ID_PWR_POS          1   // ï¿½ï¿½ï¿½ï¿½Ô´
#define ID_PWR_NEG          2   // ï¿½ï¿½ï¿½ï¿½Ô´
#define ID_PWR_TEST1        3   // ï¿½ï¿½ï¿½Ôµï¿½Ô´1 (PB10)
#define ID_PWR_TEST2        4   // ï¿½ï¿½ï¿½Ôµï¿½Ô´2 (PA02)
#define ID_HALL_UP          5   // ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½ï¿½ï¿½ (ï¿½ï¿½×¢ï¿½ï¿½)
#define ID_HALL_DOWN        6   // ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½ï¿½ï¿½ (ï¿½ï¿½×¢ï¿½ï¿½)
#define ID_IO_FWD           7   // ï¿½ï¿½×ªIOï¿½è±¸
#define ID_IO_REV           8   // ï¿½ï¿½×ªIOï¿½è±¸
#define ID_MOTOR            0   // ï¿½ï¿½ï¿½ï¿½Ù²ï¿½ï¿½è±?
#define ID_MOTOR_HALL       10  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
#define ID_ADC_CURRENT      11  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
#define ID_ADC_VOLTAGE      12  // Ä¸ï¿½ßµï¿½Ñ¹ï¿½ï¿½ï¿½ï¿½
#define ID_VOLTAGE_BUS      13  // ï¿½ï¿½Ñ¹Ä¸ï¿½ï¿½ï¿½è±¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ADC_VOLTAGEï¿½ï¿½ï¿½ã£©
#define ID_SENSOR_CURRENT   14  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è±¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ADC_CURRENTï¿½ï¿½ï¿½ã£©
#define ID_RTURN            15  // Ô²ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è±¸

// ========== Ó²ï¿½ï¿½ï¿½ï¿½ï¿½Åºê¶¨ï¿½ï¿½ (ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½) ==========

// --- ï¿½ï¿½ï¿½ï¿½Ô´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ---
#define PIN_PWR_POS_PORT        GPIO_PORT_C
#define PIN_PWR_POS_PIN         GPIO_PIN_13

// --- ï¿½ï¿½ï¿½ï¿½Ô´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ---
#define PIN_PWR_NEG_PORT        GPIO_PORT_C
#define PIN_PWR_NEG_PIN         GPIO_PIN_14   // ï¿½ï¿½ï¿½è£¬ï¿½ï¿½ï¿½ï¿½ï¿½Êµï¿½ï¿½ï¿½Þ¸ï¿?

// --- ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿? ---
#define PIN_HALL_A_PORT         GPIO_PORT_A
#define PIN_HALL_A_PIN          GPIO_PIN_10
#define PIN_HALL_B_PORT         GPIO_PORT_A
#define PIN_HALL_B_PIN          GPIO_PIN_09
#if MOTOR_HALL_TRIPLE_ENABLE
#define PIN_HALL_C_PORT         GPIO_PORT_A
#define PIN_HALL_C_PIN          GPIO_PIN_08
#endif

// --- ADCï¿½ï¿½ï¿½ï¿½ ---
#define PIN_ADC_CURRENT_PORT    GPIO_PORT_A
#define PIN_ADC_CURRENT_PIN     GPIO_PIN_05
#define PIN_ADC_CURRENT_CH      (5)           // ï¿½ï¿½Ó¦ADCÍ¨ï¿½ï¿½

#define PIN_ADC_VOLTAGE_PORT    GPIO_PORT_A
#define PIN_ADC_VOLTAGE_PIN     GPIO_PIN_06
#define PIN_ADC_VOLTAGE_CH      (6)           // ï¿½ï¿½Ó¦ADCÍ¨ï¿½ï¿½

// --- ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½IOï¿½ï¿½ï¿½ï¿½ ---
#define PIN_IO_FWD_PORT         GPIO_PORT_B   // ï¿½ï¿½ï¿½è£¬ï¿½ï¿½ï¿½ï¿½ï¿½Êµï¿½ï¿½ï¿½Þ¸ï¿?
#define PIN_IO_FWD_PIN          GPIO_PIN_00
#define PIN_IO_REV_PORT         GPIO_PORT_B
#define PIN_IO_REV_PIN          GPIO_PIN_01

// ========== Ó²ï¿½ï¿½ï¿½ï¿½ï¿½Ã²ï¿½ï¿½ï¿½ï¿½ï¿½ ==========

// --- ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿? ---
#define MOTOR_HALL_POLE_PAIRS   (3)
#if MOTOR_HALL_TRIPLE_ENABLE
#define MOTOR_HALL_COUNT        (3)
#else
#define MOTOR_HALL_COUNT        (2)
#endif
#define MOTOR_HALL_UPDATE_MS    (1)

// --- ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð¶ï¿½ï¿½ï¿½ï¿½ï¿? ---
#define HALL_EIRQ_CH_A          EXTINT_CH10
#define HALL_EIRQ_CH_B          EXTINT_CH09
#define HALL_IRQN_A             INT010_IRQn
#define HALL_IRQN_B             INT009_IRQn
#define HALL_IRQ_SRC_A          INT_SRC_PORT_EIRQ10
#define HALL_IRQ_SRC_B          INT_SRC_PORT_EIRQ9
#if MOTOR_HALL_TRIPLE_ENABLE
#define HALL_EIRQ_CH_C          EXTINT_CH08
#define HALL_IRQN_C             INT008_IRQn
#define HALL_IRQ_SRC_C          INT_SRC_PORT_EIRQ8
#endif
#define HALL_IRQ_PRIORITY       (2)

// --- Ô²ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ---
#define RTURN_REDUCTION_RATIO           (1183.0f)
#define RTURN_MAX_ANGLE                 (88.0f)
#define RTURN_MIN_ANGLE                 (-2.0f)
#define RTURN_UPDATE_INTERVAL_MS        (1)
#define RTURN_REVERSE_OUTPUT            (0)

// ========== ï¿½ï¿½Ñ¹ï¿½æ¾¯ï¿½ï¿½Öµï¿½ï¿½ï¿½Ã£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ Params.hï¿½ï¿½ ==========

// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð©ï¿½ê£¨ï¿½ï¿½ï¿½ï¿½Ê±ï¿½Ì¶ï¿½ï¿½ï¿½Ó²ï¿½ï¿½ï¿½ï¿½Ø£ï¿?
#define OVERCURRENT_MODE                    OVERCURRENT_MODE_TIME_WINDOW  // ï¿½Ì¶ï¿½Ê¹ï¿½ï¿½Ê±ï¿½ï¿½Ä£Ê½
#define CURRENT_TRIGGER_WINDOW_SIZE         (0)      // ï¿½ï¿½ï¿½ï¿½Ä£Ê½Ê±ï¿½ï¿½ï¿½ï¿½
#define CURRENT_RELEASE_WINDOW_SIZE         (0)      // ï¿½ï¿½ï¿½ï¿½Ä£Ê½Ê±ï¿½ï¿½ï¿½ï¿½

// ========== Ä£ï¿½ï¿½ï¿½ï¿½ï¿½Ý½á¹¹ï¿½ï¿½ ==========
typedef struct {
    uint8_t sim_pwr_pos;
    uint8_t sim_pwr_neg;
    uint8_t sim_hall_up;
    uint8_t sim_hall_down;
    uint8_t sim_io_fwd;
    uint8_t sim_io_rev;
    float   sim_io_speed;
    uint16_t sim_adc_val;
    int32_t sim_motor_speed;
    uint8_t sim_motor_dir;
} SystemSim_t;

// ========== ×´Ì¬Ö¸Ê¾ï¿½ï¿½ï¿½ï¿½ ==========
typedef struct {
    uint8_t motor_status;
    uint8_t power_status;
    uint8_t hall_status;
    uint8_t io_status;
    float   current_duty;
} SystemStatus_t;

extern SystemSim_t g_sim;
extern SystemStatus_t g_status;

void ESystem_Init(void);
void ESystem_MainLoop(void);

#if ENABLE_SIMULATION_MODE
void Sim_ProcessInput(void);
void Sim_PublishEvents(void);
#endif

#endif /* APP_MOTOR_PROJECT_H_ */
