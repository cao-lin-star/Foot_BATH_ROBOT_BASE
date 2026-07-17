#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 喷淋杆电机驱动模式。
 * 当前硬件使用 MS4988 步进电机驱动器，DC 模式仅为兼容早期方案保留。
 */
#define BASE_SPRAY_MOTOR_DC       1U
#define BASE_SPRAY_MOTOR_STEPPER  2U

#ifndef BASE_SPRAY_MOTOR_MODE
#define BASE_SPRAY_MOTOR_MODE     BASE_SPRAY_MOTOR_STEPPER
#endif

/*
 * 电机整步角为 1.8 度，1:1 传动时从 0 度到 90 度需要 50 个整步。
 * 原理图中 MS1/MS2/MS3 均为高电平，MS4988 工作在 1/16 细分模式，
 * 所以正式运行所需脉冲数应为 50 * 16 = 800。
 *
 * MOTOR_FULL_STEPS_0_TO_90 当前为 1U 时属于调试值，只输出 16 个微步；
 * 正式运行时应将其设置为 50U。
 */
#define MOTOR_STEP_FREQUENCY_HZ   1000U
#define MOTOR_FULL_STEPS_0_TO_90  50U
#define MOTOR_MICROSTEP_DIVISOR   2U
#define MOTOR_STEPS_0_TO_90       (MOTOR_FULL_STEPS_0_TO_90 * MOTOR_MICROSTEP_DIVISOR)

/* MS4988 ENABLE 为低电平有效：低电平开启输出，高电平关闭输出。 */
#define MOTOR_ENABLE_ACTIVE_LOW   1U

/*
 * 从 0 度转向 90 度时 DIR_MS 的电平。
 * 现场方向相反时将该宏改为 0U，不需要交换电机相线。
 */
#ifndef MOTOR_DIR_TO_SPRAY_HIGH
#define MOTOR_DIR_TO_SPRAY_HIGH   1U
#endif

/* 电机方向，主要用于兼容旧接口和状态查询。 */
typedef enum
{
  MOTOR_DIR_FORWARD = 0,
  MOTOR_DIR_REVERSE = 1
} MotorDirection_t;

/*
 * 喷淋杆逻辑位置。
 * 当前没有零位开关，位置依赖软件计步；堵转、丢步或断电移动后会失准。
 */
typedef enum
{
  MOTOR_POSITION_DRAIN_0 = 0,       /* 0 度：排水、烘干、待机等非喷淋位置。 */
  MOTOR_POSITION_SPRAY_90 = 1,      /* 90 度：清水喷淋和清洁液喷淋位置。 */
  MOTOR_POSITION_UNKNOWN = 0xFF     /* 运动中途停止，无法确认实际角度。 */
} MotorPosition_t;

/* 初始化电机；上电前必须人工确保喷淋杆处于 0 度。 */
void Motor_Init(void);

/* 异步移动到 0 度或 90 度，STEP 脉冲由 TIM3 产生和计数。 */
void Motor_MoveToPosition(MotorPosition_t position);

/* 立即停止 STEP 并关闭驱动器，运动中停止会丢失当前位置。 */
void Motor_Stop(void);

/* 返回 1 表示正在执行有限步数定位。 */
uint8_t Motor_IsBusy(void);

/* 获取软件记录的喷淋杆位置。 */
MotorPosition_t Motor_GetPosition(void);

/* TIM3 周期中断入口，每产生一个完整 STEP 周期调用一次。 */
void Motor_StepTimerElapsed(void);

/*
 * 以下为旧版兼容或调试接口。
 * 正常喷淋杆定位应优先使用 Motor_MoveToPosition()。
 */
void Motor_Run(uint8_t speed, uint8_t direction);
void Motor_SetLevel(uint8_t level);
void Motor_SetDirection(MotorDirection_t direction);
void Motor_SetAutoReverse(uint8_t enable, uint32_t interval_ms);
uint8_t Motor_GetLevel(void);
uint8_t Motor_GetDutyPercent(void);
MotorDirection_t Motor_GetDirection(void);
uint8_t Motor_IsRunning(void);
uint8_t Motor_HasFault(void);
void Motor_ClearFault(void);
void Motor_TaskProcess(void);

#ifdef __cplusplus
}
#endif

#endif
