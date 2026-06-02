#include "motor_control.h"
#include "tim.h"

/* 当前喷淋电机的缓存状态，用于状态查询和周期任务刷新输出。 */
static uint8_t motor_level;
static uint8_t motor_duty;
static uint8_t motor_running;
static uint8_t motor_fault;
static MotorDirection_t motor_direction;

/*
 * 将百分比占空比转换为定时器 CCR 值。
 * TIM 的 Period 由 CubeMX 配置决定；这里用 Period+1 计算完整周期，
 * 再限制到 Period，避免 100% 时越界。
 */
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

static uint8_t Motor_LevelToDuty(uint8_t level)
{
  /*
   * 档位到 PWM 的经验值。
   * 低档避免电机低占空比无法启动，因此从 35% 起步。
   */
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

static void Motor_ApplyDc(uint8_t duty_percent, MotorDirection_t direction)
{
  /*
   * 直流电机模式：
   *   PWM_A 有占空比、PWM_B 为 0 -> 正向
   *   PWM_A 为 0、PWM_B 有占空比 -> 反向
   * 两路不同时输出 PWM，避免 H 桥直通风险。
   */
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

static void Motor_Apply(uint8_t duty_percent)
{
#if (BASE_SPRAY_MOTOR_MODE == BASE_SPRAY_MOTOR_STEPPER)
  /*
   * 步进模式预留：
   *   CH2 输出 STEP 脉冲；
   *   CH3 用高/低电平表达方向；
   *   CH4 用高/低电平表达使能。
   * 当前未加入步数闭环，只提供可编译、可点动的输出框架。
   */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Motor_DutyToPulse(&htim3, duty_percent));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
                        (motor_direction == MOTOR_DIR_FORWARD) ? htim3.Init.Period : 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (duty_percent != 0U) ? htim3.Init.Period : 0U);
#else
  Motor_ApplyDc(duty_percent, motor_direction);
#endif
}

void Motor_Init(void)
{
  /*
   * 两种电机硬件通道都启动 PWM：
   *   默认直流模式实际只使用 TIM1；
   *   步进模式切换宏后使用 TIM3。
   * 提前启动不会产生有效输出，因为后面会统一写 0。
   */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  motor_level = 0U;
  motor_duty = 0U;
  motor_running = 0U;
  motor_fault = 0U;
  motor_direction = MOTOR_DIR_FORWARD;
  Motor_Apply(0U);
}

void Motor_Run(uint8_t speed, uint8_t direction)
{
  /* speed 作为直接百分比控制，主要给喷淋动作或调试点动使用。 */
  motor_direction = (direction != 0U) ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;
  if (speed > 100U)
  {
    speed = 100U;
  }
  motor_level = (speed == 0U) ? 0U : 1U;
  motor_duty = speed;
  motor_running = (speed != 0U) ? 1U : 0U;
  Motor_Apply(speed);
}

void Motor_Stop(void)
{
  motor_level = 0U;
  motor_duty = 0U;
  motor_running = 0U;
  Motor_Apply(0U);
}

void Motor_SetLevel(uint8_t level)
{
  if (level > 3U)
  {
    level = 3U;
  }
  motor_level = level;
  motor_duty = Motor_LevelToDuty(level);
  motor_running = (level != 0U) ? 1U : 0U;
  Motor_Apply(motor_duty);
}

void Motor_SetDirection(MotorDirection_t direction)
{
  motor_direction = direction;
  Motor_Apply(motor_duty);
}

void Motor_SetAutoReverse(uint8_t enable, uint32_t interval_ms)
{
  (void)enable;
  (void)interval_ms;
}

uint8_t Motor_GetLevel(void)
{
  return motor_level;
}

uint8_t Motor_GetDutyPercent(void)
{
  return motor_duty;
}

MotorDirection_t Motor_GetDirection(void)
{
  return motor_direction;
}

uint8_t Motor_IsRunning(void)
{
  return motor_running;
}

uint8_t Motor_HasFault(void)
{
  return motor_fault;
}

void Motor_ClearFault(void)
{
  motor_fault = 0U;
}

void Motor_TaskProcess(void)
{
  /*
   * 周期性刷新当前 PWM。
   * 这样即使后续增加故障恢复、定时器重新初始化或自动换向，
   * 也能统一放在本任务里处理。
   */
  if (motor_running != 0U)
  {
    Motor_Apply(motor_duty);
  }
}
