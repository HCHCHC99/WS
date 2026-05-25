#ifndef DEV_MOTOR_HALL_H_
#define DEV_MOTOR_HALL_H_

#include "device_manager.h"
#include "EventBus.h"
#include "Motor_hall.h"
#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

// ========== ���Ժ궨�� ==========
// ������ rtt_manager.h ��ͳһ������DEV_MOTOR_HALL / DEV_MOTOR_HALL_OUTPUT

#ifdef DEV_MOTOR_HALL
    #define MOTOR_HALL_DEBUG(fmt, ...)    MAIN_D("[MOTOR_HALL_DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define MOTOR_HALL_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef DEV_MOTOR_HALL_OUTPUT
    #define MOTOR_HALL_OUT(fmt, ...)      MAIN_D("[MOTOR_HALL_OUT] " fmt, ##__VA_ARGS__)
#else
    #define MOTOR_HALL_OUT(fmt, ...)      ((void)0)
#endif

// ========== ��������豸������ ==========
#define CMD_MOTOR_HALL_GET_RPM           (CMD_BASE_MOTOR_HALL + 0x01)
#define CMD_MOTOR_HALL_GET_DIRECTION     (CMD_BASE_MOTOR_HALL + 0x02)
#define CMD_MOTOR_HALL_GET_PULSE_COUNT   (CMD_BASE_MOTOR_HALL + 0x03)
#define CMD_MOTOR_HALL_GET_HALL_STATUS   (CMD_BASE_MOTOR_HALL + 0x04)
#define CMD_MOTOR_HALL_RESET_COUNTS      (CMD_BASE_MOTOR_HALL + 0x05)
#define CMD_MOTOR_HALL_GET_RUNNING_STATE (CMD_BASE_MOTOR_HALL + 0x06)

// ========== ��������豸���� ==========
// �豸�����е����ã���չ�ֶΣ�
typedef struct {
    uint8_t motor_id;               // ���ID
    uint16_t update_interval_ms;    // ���¼��(ms)
} MotorHall_DeviceConfig_t;

// ========== ��������豸�ṹ�� ==========
typedef struct {
    motor_hall_handle_t handle;         // �ײ�������
    motor_hall_config_t config;         // �ײ�������ã�ֱ��ʹ�� Motor_hall �Ķ��壩
    MotorHall_DeviceConfig_t dev_cfg;   // �豸����������
    uint8_t initialized;                // ��ʼ����־
    
    // ��������
    float cached_rpm;
    float cached_rpm_raw;
    motor_direction_t cached_direction;
    uint8_t direction_confidence;
    uint32_t cached_pulse_interval;
    uint32_t hall_a_count;
    uint32_t hall_b_count;
#if MOTOR_HALL_TRIPLE_ENABLE
    uint32_t hall_c_count;
#endif
    uint32_t total_pulse_count;
    uint8_t is_running;
    uint8_t is_stalled;
    hall_working_status_t hall_status;
    
    // ʱ�����
    uint32_t last_update_time;
    uint8_t direction_changed;
} MotorHall_Device_t;

// ========== ��������ṹ�� ==========
typedef struct {
    float rpm;
} MotorHall_RpmResponse_t;

typedef struct {
    uint8_t direction;
    uint8_t confidence;
} MotorHall_DirectionResponse_t;

typedef struct {
    uint32_t hall_a_count;
    uint32_t hall_b_count;
    uint32_t total_count;
} MotorHall_PulseCountResponse_t;

// ========== ��׼�豸���� ==========
DeviceResult_t MotorHall_Device_Init(void* handle);
DeviceResult_t MotorHall_Device_Deinit(void* handle);
DeviceResult_t MotorHall_Device_Read(void* handle, void* data, uint32_t size);
DeviceResult_t MotorHall_Device_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t MotorHall_Device_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t MotorHall_Device_Update(void* handle);

// ========== ��������ض��ӿ� ==========
float MotorHall_Device_GetRPM(MotorHall_Device_t* dev);
float MotorHall_Device_GetRPMRaw(MotorHall_Device_t* dev);
motor_direction_t MotorHall_Device_GetDirection(MotorHall_Device_t* dev);
uint8_t MotorHall_Device_GetDirectionConfidence(MotorHall_Device_t* dev);
uint8_t MotorHall_Device_IsDirectionChanged(MotorHall_Device_t* dev);
uint32_t MotorHall_Device_GetHallACount(MotorHall_Device_t* dev);
uint32_t MotorHall_Device_GetHallBCount(MotorHall_Device_t* dev);
#if MOTOR_HALL_TRIPLE_ENABLE
uint32_t MotorHall_Device_GetHallCCount(MotorHall_Device_t* dev);
#endif
uint32_t MotorHall_Device_GetTotalPulseCount(MotorHall_Device_t* dev);
void MotorHall_Device_ResetCounts(MotorHall_Device_t* dev);
uint8_t MotorHall_Device_IsRunning(MotorHall_Device_t* dev);
uint8_t MotorHall_Device_IsStalled(MotorHall_Device_t* dev);
hall_working_status_t MotorHall_Device_GetHallStatus(MotorHall_Device_t* dev);
MotorHall_Device_t* MotorHall_Device_Create(const motor_hall_config_t* config, const MotorHall_DeviceConfig_t* dev_cfg);

// ========== EventBus �¼����� ==========
// ���豸���������� TOPIC_MOTOR_SPEED_FEEDBACK �¼�

// ========== ȫ�ֲ��������� ==========
extern const DeviceOps_t g_motor_hall_ops;

#endif /* DEV_MOTOR_HALL_H_ */
