#ifndef __MOTOR_HALL_H__
#define __MOTOR_HALL_H__

#include "hc32_ll.h"
#include "Adapter.h"

/* ========== ˫����/����������ѡ�� ========== */
#ifndef MOTOR_HALL_TRIPLE_ENABLE
#define MOTOR_HALL_TRIPLE_ENABLE    (1)     /* 0=˫����(A+B), 1=������(A+B+C) */
#endif

/* ========== ������ò�������ת�١�ת����أ� ========== */

/**
 * @brief ��������������
 */
typedef struct {
    /* GPIO���� */
    uint8_t hall_a_port;        /* GPIO_PORT_A �� */
    uint16_t hall_a_pin;        /* GPIO_PIN_xx */
    uint8_t hall_b_port;
    uint16_t hall_b_pin;
    
    /* �ж����� */
    uint32_t eirq_ch_a;         /* EXTINT_CHxx */
    uint32_t eirq_ch_b;
    uint8_t irqn_a;             /* INTxxx_IRQn */
    uint8_t irqn_b;
    uint32_t irq_src_a;         /* INT_PORT_EIRQx */
    uint32_t irq_src_b;
    uint8_t irq_priority;

#if MOTOR_HALL_TRIPLE_ENABLE
    /* ������Hall C��GPIO���ж����ã� */
    uint8_t hall_c_port;        /* GPIO_PORT_x */
    uint16_t hall_c_pin;        /* GPIO_PIN_xx */
    uint32_t eirq_ch_c;         /* EXTINT_CHxx */
    uint8_t irqn_c;             /* INTxxx_IRQn */
    uint32_t irq_src_c;         /* INT_PORT_EIRQx */
#endif
    
    /* ���������ת��ת����أ� */
    uint8_t pole_pairs;
    uint8_t hall_count;
    uint16_t custom_pulses_per_rev;
    
} motor_hall_config_t;


/* ========== Ĭ������ʾ����ԭ�������? - PA9, PA10�� ========== */
#define DEFAULT_HALL_A_PORT      GPIO_PORT_A
#define DEFAULT_HALL_A_PIN       GPIO_PIN_09
#define DEFAULT_HALL_B_PORT      GPIO_PORT_A
#define DEFAULT_HALL_B_PIN       GPIO_PIN_10

#define DEFAULT_HALL_A_EIRQ_CH   EXTINT_CH09
#define DEFAULT_HALL_B_EIRQ_CH   EXTINT_CH10
#define DEFAULT_HALL_A_IRQN      INT009_IRQn
#define DEFAULT_HALL_B_IRQN      INT010_IRQn
#define DEFAULT_HALL_A_IRQ_SRC   INT_PORT_EIRQ9
#define DEFAULT_HALL_B_IRQ_SRC   INT_PORT_EIRQ10

#define DEFAULT_HALL_IRQ_PRIORITY DDL_IRQ_PRIORITY_02

/* Ĭ�ϵ������? */
#define DEFAULT_POLE_PAIRS       (3)     
#define DEFAULT_HALL_COUNT       (2)     

/* �Զ�����ÿת�������������� �� ������ �� 2��˫���أ� */
#define CALC_PULSES_PER_REV(pole_pairs, hall_count) ((pole_pairs) * (hall_count) * 2)

/* ========== ����״̬ö�� ========== */
typedef enum {
    MOTOR_DIRECTION_NONE = 0,
    MOTOR_DIRECTION_FORWARD,
    MOTOR_DIRECTION_REVERSE,
    MOTOR_DIRECTION_STOP,
} motor_direction_t;

/* ========== ��������״̬ö�� ========== */
typedef enum {
    HALL_STATUS_NONE = 0,
    HALL_STATUS_A_ONLY,
    HALL_STATUS_B_ONLY,
    HALL_STATUS_BOTH,
    HALL_STATUS_ERROR
} hall_working_status_t;

/* ========== �����������͸��ָ��? ========== */
typedef struct motor_hall_handle_t* motor_hall_handle_t;

/* ========== ����/���ٽӿ� ========== */

motor_hall_handle_t motor_hall_create(const motor_hall_config_t* config);
void motor_hall_destroy(motor_hall_handle_t handle);

/* ========== ��ʼ��/���½ӿ� ========== */

void motor_hall_system_init(void);
void motor_hall_start(motor_hall_handle_t handle);
void motor_hall_stop(motor_hall_handle_t handle);
void motor_hall_update(motor_hall_handle_t handle);

/* ========== ת����ؽӿ�? ========== */

float motor_hall_get_rpm(motor_hall_handle_t handle);
float motor_hall_get_rpm_raw(motor_hall_handle_t handle);
uint32_t motor_hall_get_pulse_interval_us(motor_hall_handle_t handle);
uint8_t motor_hall_is_running(motor_hall_handle_t handle);
uint8_t motor_hall_is_stalled(motor_hall_handle_t handle);

/* ========== ������ؽӿ�? ========== */

motor_direction_t motor_hall_get_direction(motor_hall_handle_t handle);
uint8_t motor_hall_get_direction_confidence(motor_hall_handle_t handle);
uint8_t motor_hall_is_direction_changed(motor_hall_handle_t handle);

/* ========== ���������ӿ� ========== */

uint32_t motor_hall_get_hall_a_count(motor_hall_handle_t handle);
uint32_t motor_hall_get_hall_b_count(motor_hall_handle_t handle);
uint32_t motor_hall_get_total_pulse_count(motor_hall_handle_t handle);
void motor_hall_reset_counts(motor_hall_handle_t handle);
#if MOTOR_HALL_TRIPLE_ENABLE
uint32_t motor_hall_get_hall_c_count(motor_hall_handle_t handle);
#endif

hall_working_status_t motor_hall_get_status(motor_hall_handle_t handle);
uint8_t motor_hall_get_active_hall_count(motor_hall_handle_t handle);
uint16_t motor_hall_get_pulses_per_rev(motor_hall_handle_t handle);
uint8_t motor_hall_get_pole_pairs(motor_hall_handle_t handle);

#endif /* MOTOR_HALL_H */
