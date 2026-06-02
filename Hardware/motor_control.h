#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 喷淋电机硬件模式选择。
 *
 * BASE_SPRAY_MOTOR_DC:
 *   默认模式，使用 TIM1_CH1(PA8/PWM_A) 和 TIM1_CH4(PA11/PWM_B)
 *   组成直流电机正反转 PWM 控制。
 *
 * BASE_SPRAY_MOTOR_STEPPER:
 *   预留步进电机模式，使用 TIM3_CH2(PA7/STP_MS) 输出 STEP PWM，
 *   TIM3_CH3(PB0/DIR_MS) 表示方向，TIM3_CH4(PB1/ENABLE_MS) 表示使能。
 */
#define BASE_SPRAY_MOTOR_DC       1U
#define BASE_SPRAY_MOTOR_STEPPER  2U

/* 如需切换到步进电机，在工程宏或本文件前定义 BASE_SPRAY_MOTOR_MODE。 */
#ifndef BASE_SPRAY_MOTOR_MODE
#define BASE_SPRAY_MOTOR_MODE     BASE_SPRAY_MOTOR_DC
#endif

/* 喷淋电机方向。直流模式表示 PWM_A/PWM_B 的输出侧，步进模式表示 DIR_MS 电平。 */
typedef enum
{
  MOTOR_DIR_FORWARD = 0,
  MOTOR_DIR_REVERSE = 1
} MotorDirection_t;

/* 初始化喷淋电机相关 PWM 通道，并默认停止输出。 */
void Motor_Init(void);

/* 直接按百分比速度运行：speed=0-100，direction=0 正向，非 0 反向。 */
void Motor_Run(uint8_t speed, uint8_t direction);

/* 立即停止喷淋电机，并清零当前速度/档位状态。 */
void Motor_Stop(void);

/* 按档位运行：0 停止，1/2/3 分别映射为低/中/高 PWM 占空比。 */
void Motor_SetLevel(uint8_t level);

/* 在当前速度不变的情况下切换方向。 */
void Motor_SetDirection(MotorDirection_t direction);

/* 兼容旧接口的预留函数；当前基站第一版未启用自动换向。 */
void Motor_SetAutoReverse(uint8_t enable, uint32_t interval_ms);

/* 以下接口用于状态上报或调试日志。 */
uint8_t Motor_GetLevel(void);
uint8_t Motor_GetDutyPercent(void);
MotorDirection_t Motor_GetDirection(void);
uint8_t Motor_IsRunning(void);
uint8_t Motor_HasFault(void);

/* 当前无电流/堵转检测输入，故障位暂由软件预留。 */
void Motor_ClearFault(void);

/* FreeRTOS 周期任务入口，用于保持/刷新当前 PWM 输出。 */
void Motor_TaskProcess(void);

#ifdef __cplusplus
}
#endif

#endif
