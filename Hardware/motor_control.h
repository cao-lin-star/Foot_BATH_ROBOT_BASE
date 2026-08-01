#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 喷淋杆使用 28BYJ-48/ULN2003 八拍半步驱动。
 * 按约 4096 半步/输出轴一圈计算，0 度到 90 度需要 1024 半步。
 * TIM3 基本更新中断周期配置为 3 ms，每次更新推进一个半步。
 */
#define MOTOR_STEPS_0_TO_90       1024U
#define MOTOR_STEP_INTERVAL_MS    3U

/* 电机方向，正向按 1000、1100、0100...相序推进。 */
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

/* 初始化电机；上电前必须人工确保喷淋杆处于 0 度，初始化后四相全低。 */
void Motor_Init(void);

/* 异步移动到 0 度或 90 度，八拍相位由 TIM3 更新中断推进。 */
void Motor_MoveToPosition(MotorPosition_t position);

/* 立即停止 TIM3 并将四相拉低；定位途中停止会丢失当前位置。 */
void Motor_Stop(void);

/* 返回 1 表示正在执行有限半步数定位。 */
uint8_t Motor_IsBusy(void);

/* 获取软件记录的喷淋杆位置。 */
MotorPosition_t Motor_GetPosition(void);

/* TIM3 每 3 ms 的周期中断入口，每次调用推进一个半步。 */
void Motor_StepTimerElapsed(void);

/*
 * 以下为旧版兼容或调试接口。
 * 正常喷淋杆定位应优先使用 Motor_MoveToPosition()。
 * Motor_Run() 的 speed 只作状态记录，连续运行仍按固定 3 ms 半步周期执行。
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
