#include "motor_control.h"
#include "main.h"
#include "tim.h"

/* 剩余 STEP 脉冲数，由 TIM3 中断递减。 */
static volatile uint16_t motor_steps_remaining;
/* 有限步数定位忙标志，1 表示正在向目标位置运动。 */
static volatile uint8_t motor_busy;
/* 以下状态用于兼容旧版档位、速度和状态查询接口。 */
static uint8_t motor_level;
static uint8_t motor_duty;
static uint8_t motor_running;
/* 软件故障标志，当前位置未知时会禁止继续绝对定位。 */
static uint8_t motor_fault;
static MotorDirection_t motor_direction;
/* 软件记录的当前位置及本次运动的目标位置。 */
static MotorPosition_t motor_position;
static MotorPosition_t motor_target_position;

#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_DC)
/* 将百分比占空比换算为定时器 CCR 值，仅用于直流电机兼容模式。 */
static uint32_t Motor_DutyToPulse(TIM_HandleTypeDef *htim, uint8_t duty_percent)
{
  uint32_t period;
  uint32_t pulse;

  if (duty_percent > 100U)
  {
    duty_percent = 100U;
  }

  period = htim->Init.Period;
  pulse = ((period + 1U) * duty_percent) / 100U;
  if (pulse > period)
  {
    pulse = period;
  }
  return pulse;
}
#endif

/* 将旧版低/中/高档位映射为经验占空比。 */
static uint8_t Motor_LevelToDuty(uint8_t level)
{
  switch (level)
  {
    case 1U:
      return 35U;
    case 2U:
      return 60U;
    case 3U:
      return 85U;
    default:
      return 0U;
  }
}

/*
 * 控制 MS4988 的 ENABLE 引脚。
 * 当前硬件为低电平有效，因此 enabled=1 时输出低电平。
 */
static void Motor_SetDriverEnabled(uint8_t enabled)
{
#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_STEPPER)
#if (MOTOR_ENABLE_ACTIVE_LOW != 0U)
  HAL_GPIO_WritePin(ENABLE_MS_GPIO_Port, ENABLE_MS_Pin,
                    (enabled != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(ENABLE_MS_GPIO_Port, ENABLE_MS_Pin,
                    (enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
#else
  (void)enabled;
#endif
}

/* toward_spray=1 表示朝 90 度运动，否则朝 0 度运动。 */
static void Motor_SetStepDirection(uint8_t toward_spray)
{
  uint8_t direction_high;

  direction_high = (toward_spray != 0U) ? MOTOR_DIR_TO_SPRAY_HIGH :
                                          (uint8_t)(MOTOR_DIR_TO_SPRAY_HIGH == 0U);
  HAL_GPIO_WritePin(DIR_MS_GPIO_Port, DIR_MS_Pin,
                    (direction_high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  motor_direction = (toward_spray != 0U) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
}

/*
 * 停止 TIM3 STEP 波形和更新中断。
 * disable_driver=1 时同时关闭驱动器：ENABLE_MS 输出高电平，线圈停止励磁。
 */
static void Motor_StopStepTimer(uint8_t disable_driver)
{
#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_STEPPER)
  HAL_TIM_Base_Stop_IT(&htim3);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  if (disable_driver != 0U)
  {
    Motor_SetDriverEnabled(0U);
  }
#else
  (void)disable_driver;
#endif
}

#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_DC)
/* 将速度和方向应用到早期直流电机的两路 H 桥 PWM。 */
static void Motor_ApplyDc(uint8_t duty_percent, MotorDirection_t direction)
{
  if (direction == MOTOR_DIR_FORWARD)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Motor_DutyToPulse(&htim1, duty_percent));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Motor_DutyToPulse(&htim1, duty_percent));
  }
}
#endif

/* 初始化输出和软件状态，并假定上电时喷淋杆已经人工放在 0 度。 */
void Motor_Init(void)
{
#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_STEPPER)
  Motor_StopStepTimer(1U);
#else
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  Motor_ApplyDc(0U, MOTOR_DIR_FORWARD);
#endif

  motor_steps_remaining = 0U;
  motor_busy = 0U;
  motor_level = 0U;
  motor_duty = 0U;
  motor_running = 0U;
  motor_fault = 0U;
  motor_direction = MOTOR_DIR_FORWARD;
  motor_position = MOTOR_POSITION_DRAIN_0;
  motor_target_position = MOTOR_POSITION_DRAIN_0;
}

/*
 * 异步移动到 0 度或 90 度。
 * 正常端点切换输出 MOTOR_STEPS_0_TO_90 个微步；
 * 运动途中收到反向命令时，按已经走出的步数退回原端点。
 */
void Motor_MoveToPosition(MotorPosition_t position)
{
#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_STEPPER)
  uint8_t toward_spray;

  if ((position != MOTOR_POSITION_DRAIN_0) &&
      (position != MOTOR_POSITION_SPRAY_90))
  {
    return;
  }

  if (motor_busy != 0U)
  {
    uint16_t steps_to_new_target;

    if (position == motor_target_position)
    {
      return;
    }

    /* 已走出的微步数，也就是反向返回原端点所需的微步数。 */
    steps_to_new_target = (uint16_t)(MOTOR_STEPS_0_TO_90 - motor_steps_remaining);
    Motor_StopStepTimer(0U);
    if (steps_to_new_target == 0U)
    {
      Motor_SetDriverEnabled(0U);
      motor_busy = 0U;
      motor_running = 0U;
      motor_level = 0U;
      motor_duty = 0U;
      motor_position = position;
      motor_target_position = position;
      return;
    }

    toward_spray = (position == MOTOR_POSITION_SPRAY_90) ? 1U : 0U;
    Motor_SetStepDirection(toward_spray);
    Motor_SetDriverEnabled(1U);
    motor_target_position = position;
    motor_steps_remaining = steps_to_new_target;
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (htim3.Init.Period + 1U) / 2U);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim3);
    return;
  }

  if ((motor_busy == 0U) && (motor_position == MOTOR_POSITION_UNKNOWN))
  {
    motor_fault = 1U;
    return;
  }

  if ((motor_busy == 0U) && (motor_position == position))
  {
    Motor_SetDriverEnabled(0U);
    return;
  }

  Motor_StopStepTimer(0U);
  toward_spray = (position == MOTOR_POSITION_SPRAY_90) ? 1U : 0U;
  Motor_SetStepDirection(toward_spray);
  Motor_SetDriverEnabled(1U);

  motor_target_position = position;
  motor_steps_remaining = MOTOR_STEPS_0_TO_90;
  motor_busy = 1U;
  motor_running = 1U;
  motor_level = 1U;
  motor_duty = 50U;

  /* STEP 输出固定为 50% 占空比，并从计数器 0 开始。 */
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (htim3.Init.Period + 1U) / 2U);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
#else
  (void)position;
#endif
}

/*
 * 立即停止电机并关闭驱动器。
 * 定位途中停止后无法确认实际角度，因此将位置标记为 UNKNOWN。
 */
void Motor_Stop(void)
{
#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_STEPPER)
  uint8_t was_busy;

  was_busy = motor_busy;
  Motor_StopStepTimer(1U);
  motor_steps_remaining = 0U;
  motor_busy = 0U;
  if (was_busy != 0U)
  {
    motor_position = MOTOR_POSITION_UNKNOWN;
  }
#else
  Motor_ApplyDc(0U, motor_direction);
#endif

  motor_level = 0U;
  motor_duty = 0U;
  motor_running = 0U;
}

/* 查询有限步数定位是否正在执行。 */
uint8_t Motor_IsBusy(void)
{
  return motor_busy;
}

/* 返回软件记录的喷淋杆位置。 */
MotorPosition_t Motor_GetPosition(void)
{
  return motor_position;
}

/*
 * TIM3 更新中断处理。
 * TIM3 每完成一个 PWM 周期进入一次中断，对应一个 STEP 脉冲。
 */
void Motor_StepTimerElapsed(void)
{
#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_STEPPER)
  if ((motor_busy == 0U) || (motor_steps_remaining == 0U))
  {
    return;
  }

  motor_steps_remaining--;
  if (motor_steps_remaining == 0U)
  {
    /* 到达目标后关闭 STEP 和驱动使能，依靠机械结构保持喷淋杆位置。 */
    Motor_StopStepTimer(1U);
    motor_busy = 0U;
    motor_running = 0U;
    motor_level = 0U;
    motor_duty = 0U;
    motor_position = motor_target_position;
  }
#endif
}

/*
 * 旧版连续运行接口。
 * 步进模式下 speed 不改变 STEP 频率，调用后绝对位置会变为未知。
 */
void Motor_Run(uint8_t speed, uint8_t direction)
{
  if (speed > 100U)
  {
    speed = 100U;
  }

  motor_direction = (direction != 0U) ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;
  motor_level = (speed == 0U) ? 0U : 1U;
  motor_duty = speed;
  motor_running = (speed != 0U) ? 1U : 0U;

#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_STEPPER)
  if (speed == 0U)
  {
    Motor_Stop();
    return;
  }

  Motor_StopStepTimer(0U);
  Motor_SetStepDirection((direction == 0U) ? 1U : 0U);
  Motor_SetDriverEnabled(1U);
  motor_position = MOTOR_POSITION_UNKNOWN;
  motor_busy = 0U;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (htim3.Init.Period + 1U) / 2U);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
#else
  Motor_ApplyDc(speed, motor_direction);
#endif
}

/* 旧版档位接口，将 1-3 档转换为占空比后调用 Motor_Run()。 */
void Motor_SetLevel(uint8_t level)
{
  if (level > 3U)
  {
    level = 3U;
  }
  Motor_Run(Motor_LevelToDuty(level), (uint8_t)motor_direction);
  motor_level = level;
}

/* 修改方向；步进模式直接设置 DIR_MS 电平。 */
void Motor_SetDirection(MotorDirection_t direction)
{
  motor_direction = direction;
#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_STEPPER)
  Motor_SetStepDirection((direction == MOTOR_DIR_FORWARD) ? 1U : 0U);
#else
  Motor_ApplyDc(motor_duty, motor_direction);
#endif
}

/* 自动换向当前未启用，保留接口兼容性。 */
void Motor_SetAutoReverse(uint8_t enable, uint32_t interval_ms)
{
  (void)enable;
  (void)interval_ms;
}

/* 获取旧版接口记录的档位。 */
uint8_t Motor_GetLevel(void)
{
  return motor_level;
}

/* 获取旧版接口记录的占空比。 */
uint8_t Motor_GetDutyPercent(void)
{
  return motor_duty;
}

/* 获取当前软件方向。 */
MotorDirection_t Motor_GetDirection(void)
{
  return motor_direction;
}

/* 查询电机是否处于运行状态。 */
uint8_t Motor_IsRunning(void)
{
  return motor_running;
}

/* 查询软件故障标志。 */
uint8_t Motor_HasFault(void)
{
  return motor_fault;
}

/* 清除软件故障标志，但不会重新建立机械零位。 */
void Motor_ClearFault(void)
{
  motor_fault = 0U;
}

/* 周期任务预留入口，当前有限步数计数由 TIM3 中断完成。 */
void Motor_TaskProcess(void)
{
}
