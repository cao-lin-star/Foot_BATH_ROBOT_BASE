#include "motor_control.h"
#include "main.h"
#include "tim.h"

/* 28BYJ-48/ULN2003 八拍半步励磁表，1 表示对应 ULN2003 输入为高电平。 */
static const uint8_t motor_phase_table[8][4] =
{
  {1U, 0U, 0U, 0U},
  {1U, 1U, 0U, 0U},
  {0U, 1U, 0U, 0U},
  {0U, 1U, 1U, 0U},
  {0U, 0U, 1U, 0U},
  {0U, 0U, 1U, 1U},
  {0U, 0U, 0U, 1U},
  {1U, 0U, 0U, 1U}
};

/* 剩余半步数，由 3 ms 的 TIM3 更新中断递减。 */
static volatile uint16_t motor_steps_remaining;
/* 有限步数定位忙标志，1 表示正在向目标位置运动。 */
static volatile uint8_t motor_busy;
/* 以下状态用于兼容旧版档位、速度和状态查询接口。 */
static volatile uint8_t motor_level;
static volatile uint8_t motor_duty;
static volatile uint8_t motor_running;
/* 软件故障标志，当前位置未知时会禁止继续绝对定位。 */
static volatile uint8_t motor_fault;
static volatile MotorDirection_t motor_direction;
/* 软件记录的当前位置及本次运动的目标位置。 */
static volatile MotorPosition_t motor_position;
static volatile MotorPosition_t motor_target_position;
/* 当前八拍相位。输出关闭时仍保留索引，以便再次启动或途中反向。 */
static volatile uint8_t motor_phase_index;

/* 将旧版低/中/高档位映射为兼容的速度百分比记录值。 */
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

/* 输出一个半步相位；ULN2003 输入高电平时对应线圈励磁。 */
static void Motor_WritePhase(uint8_t phase_index)
{
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin,
                    (motor_phase_table[phase_index][0] != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin,
                    (motor_phase_table[phase_index][1] != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin,
                    (motor_phase_table[phase_index][2] != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin,
                    (motor_phase_table[phase_index][3] != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 四相全部关闭，停止时不持续给电机线圈通电。 */
static void Motor_Deenergize(void)
{
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);
}

/* toward_spray=1 表示按正向相序朝 90 度运动，否则按反向相序朝 0 度运动。 */
static void Motor_SetStepDirection(uint8_t toward_spray)
{
  motor_direction = (toward_spray != 0U) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
}

/* 按当前方向推进一个八拍半步并立即更新四路输出。 */
static void Motor_AdvancePhase(void)
{
  if (motor_direction == MOTOR_DIR_FORWARD)
  {
    motor_phase_index = (uint8_t)((motor_phase_index + 1U) & 0x07U);
  }
  else
  {
    motor_phase_index = (motor_phase_index == 0U) ? 7U : (uint8_t)(motor_phase_index - 1U);
  }

  Motor_WritePhase((uint8_t)motor_phase_index);
}

/* 停止 TIM3 基本更新中断；deenergize=1 时同时关闭全部线圈。 */
static void Motor_StopStepTimer(uint8_t deenergize)
{
  HAL_TIM_Base_Stop_IT(&htim3);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);

  if (deenergize != 0U)
  {
    Motor_Deenergize();
  }
}

/* 从完整的 3 ms 周期开始异步推进半步。 */
static void Motor_StartStepTimer(void)
{
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);
}

/* 初始化输出和软件状态，并假定上电时喷淋杆已经人工放在 0 度。 */
void Motor_Init(void)
{
  Motor_StopStepTimer(1U);

  motor_steps_remaining = 0U;
  motor_busy = 0U;
  motor_level = 0U;
  motor_duty = 0U;
  motor_running = 0U;
  motor_fault = 0U;
  motor_direction = MOTOR_DIR_FORWARD;
  motor_position = MOTOR_POSITION_DRAIN_0;
  motor_target_position = MOTOR_POSITION_DRAIN_0;
  motor_phase_index = 0U;
}

/*
 * 异步移动到 0 度或 90 度。
 * 正常端点切换输出 MOTOR_STEPS_0_TO_90 个半步；
 * 运动途中收到反向命令时，按已经走出的半步数退回原端点。
 */
void Motor_MoveToPosition(MotorPosition_t position)
{
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

    /* 先停止更新中断，再取得一致的已走半步数。 */
    Motor_StopStepTimer(0U);
    steps_to_new_target = (uint16_t)(MOTOR_STEPS_0_TO_90 - motor_steps_remaining);
    if (steps_to_new_target == 0U)
    {
      Motor_Deenergize();
      motor_steps_remaining = 0U;
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
    motor_target_position = position;
    motor_steps_remaining = steps_to_new_target;
    Motor_StartStepTimer();
    return;
  }

  if (motor_position == MOTOR_POSITION_UNKNOWN)
  {
    motor_fault = 1U;
    return;
  }

  if (motor_position == position)
  {
    Motor_StopStepTimer(1U);
    return;
  }

  Motor_StopStepTimer(0U);
  toward_spray = (position == MOTOR_POSITION_SPRAY_90) ? 1U : 0U;
  Motor_SetStepDirection(toward_spray);

  motor_target_position = position;
  motor_steps_remaining = MOTOR_STEPS_0_TO_90;
  motor_busy = 1U;
  motor_running = 1U;
  motor_level = 1U;
  motor_duty = 50U;

  /* 先输出当前相位，再由每个 TIM3 更新中断推进一个新相位。 */
  Motor_WritePhase((uint8_t)motor_phase_index);
  Motor_StartStepTimer();
}

/*
 * 立即停止电机并关闭全部线圈。
 * 定位途中停止后无法确认实际角度，因此将位置标记为 UNKNOWN。
 */
void Motor_Stop(void)
{
  uint8_t was_busy;

  was_busy = motor_busy;
  Motor_StopStepTimer(1U);
  motor_steps_remaining = 0U;
  motor_busy = 0U;
  if (was_busy != 0U)
  {
    motor_position = MOTOR_POSITION_UNKNOWN;
  }

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

/* TIM3 每 3 ms 调用一次；有限定位和连续运行都在此非阻塞推进。 */
void Motor_StepTimerElapsed(void)
{
  if (motor_busy != 0U)
  {
    if (motor_steps_remaining == 0U)
    {
      Motor_StopStepTimer(1U);
      motor_busy = 0U;
      motor_running = 0U;
      motor_level = 0U;
      motor_duty = 0U;
      motor_position = motor_target_position;
      return;
    }

    Motor_AdvancePhase();
    motor_steps_remaining--;
    if (motor_steps_remaining == 0U)
    {
      Motor_StopStepTimer(1U);
      motor_busy = 0U;
      motor_running = 0U;
      motor_level = 0U;
      motor_duty = 0U;
      motor_position = motor_target_position;
    }
    return;
  }

  if (motor_running != 0U)
  {
    Motor_AdvancePhase();
    return;
  }

  /* 防止无有效运动请求时因残留更新事件使线圈保持通电。 */
  Motor_StopStepTimer(1U);
}

/*
 * 旧版连续运行接口。
 * speed 仅保留为状态百分比，实际半步周期固定由 TIM3 的 3 ms 周期决定；
 * 调用后绝对位置变为未知，直至重新人工建立零位。
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

  if (speed == 0U)
  {
    Motor_Stop();
    return;
  }

  Motor_StopStepTimer(0U);
  motor_steps_remaining = 0U;
  motor_position = MOTOR_POSITION_UNKNOWN;
  motor_busy = 0U;
  Motor_WritePhase((uint8_t)motor_phase_index);
  Motor_StartStepTimer();
}

/* 旧版档位接口，将 1-3 档转换为兼容的百分比记录值。 */
void Motor_SetLevel(uint8_t level)
{
  if (level > 3U)
  {
    level = 3U;
  }
  Motor_Run(Motor_LevelToDuty(level), (uint8_t)motor_direction);
  motor_level = level;
}

/* 修改方向；连续运行时从下一个 TIM3 更新中断开始按相反相序推进。 */
void Motor_SetDirection(MotorDirection_t direction)
{
  motor_direction = (direction == MOTOR_DIR_REVERSE) ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;
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

/* 获取旧版接口记录的百分比。 */
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

/* 周期任务预留入口，电机相位由 TIM3 中断推进。 */
void Motor_TaskProcess(void)
{
}
