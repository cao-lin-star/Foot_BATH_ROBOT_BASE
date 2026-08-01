#include "system_monitor.h"
#include "color_light.h"
#include "log.h"
#include "main.h"
#include "motor_control.h"
#include <string.h>

#define BASE_BUCKET_STATUS_RUNNING 0x04U

#define BASE_CMD_OFF              0xB0U  /* 关机命令�?*/
#define BASE_CMD_STANDBY          0xB1U  /* 待机命令�?*/
#define BASE_CMD_AUTO_FILL        0xB2U  /* 自动注水命令�?*/
#define BASE_CMD_AUTO_CLEAN       0xB3U  /* 自动清洁命令�?*/
#define BASE_CMD_FORCE_DRAIN      0xB4U  /* 强制排水命令�?*/
#define BASE_CMD_CLEAN_SPRAY      0xB5U  /* 单独清洁液喷淋命令�?*/
#define BASE_CMD_CLEAR_SPRAY      0xB6U  /* 单独清水/热水喷淋命令�?*/
#define BASE_CMD_DRY              0xB7U  /* 单独烘干命令�?*/
#define BASE_CMD_SELF_CHECK       0xB8U  /* 自检命令�?*/
#define BASE_CMD_STOP_AUTO_CLEAN  0xB9U  /* 停止自动清洁命令�?*/

#define BASE_DEFAULT_DRAIN_MS     (60UL * 1000UL)
#define BASE_AUTO_FILL_PRIME_MS   (5UL * 1000UL)       /* 自动注水前先开进水阀 5s 排空管道 */
#define BASE_AUTO_FILL_TIMEOUT_MS (15UL * 60UL * 1000UL) /* 自动注水最长 15 分钟保护 */
#define BASE_DRAIN_AFTER_EMPTY_MS (10UL * 1000UL)       /* Drain 10s after bucket water reaches 0L. */
#define BASE_DRAIN_MAX_MS         (10UL * 60UL * 1000UL) /* Max drain time if bucket does not report 0L. */
#define BASE_AUTO_CLEAN_CLEANER_MS      (5UL * 1000UL)  /* B3 clean spray: cleaner + water. */
#define BASE_AUTO_CLEAN_WAIT_MS         (10UL * 1000UL) /* B3 clean spray: wait after cleaner. */
#define BASE_LEVEL_BOARD_TIMEOUT_MS 5000UL          /* 缺液控制板通信超时时间，单�?ms�?*/
#define BASE_LEVEL_MED_PUMP1_LOW    0x01U           /* 缺液位图 bit0：药液泵 1 缺液�?*/
#define BASE_LEVEL_MED_PUMP2_LOW    0x02U           /* 缺液位图 bit1：药液泵 2 缺液�?*/
#define BASE_LEVEL_CLEAN_LOW        0x04U           /* 缺液位图 bit2：清洁液缺液�?*/
#define BASE_LEVEL_VALID_MASK       0x07U           /* 当前参与基站错误判断的缺液位�?*/
#define BASE_LEVEL_FRAME_FIXED      0x50U           /* 缺液控制板上报字节高 5 位固定为 01010�?*/
#define BASE_LEVEL_FRAME_FIXED_MASK 0xF8U           /* 缺液控制板上报字节高 5 位掩码�?*/

/*
 * 内部动作状态�? * 这些状态比协议上报�?sub_status 更细，用来精确推进自动清洁流程�? */
typedef enum
{
  BASE_ACTION_IDLE = 0,           /* 无内部动作�?*/
  BASE_ACTION_AUTO_DRAIN1,        /* 自动清洁�?1 次排水�?*/
  BASE_ACTION_AUTO_CLEAN_SPRAY,   /* 自动清洁清洁液喷淋�?*/
  BASE_ACTION_AUTO_DRAIN2,        /* 自动清洁�?2 次排水�?*/
  BASE_ACTION_AUTO_HOT_SPRAY2,    /* 自动清洁�?2 次热水喷淋�?*/
  BASE_ACTION_AUTO_DRAIN3,        /* 自动清洁�?3 次排水�?*/
  BASE_ACTION_AUTO_DRY,           /* 自动清洁烘干�?*/
  BASE_ACTION_FORCE_DRAIN,        /* 强制排水�?*/
  BASE_ACTION_SINGLE_CLEAN_SPRAY, /* 单独清洁液喷淋�?*/
  BASE_ACTION_SINGLE_CLEAR_SPRAY, /* 单独清水/热水喷淋�?*/
  BASE_ACTION_SINGLE_DRY,         /* Single dry. */
  BASE_ACTION_SINGLE_CLEAN_DRAIN,
  BASE_ACTION_SINGLE_CLEAR_DRAIN,
  BASE_ACTION_AUTO_FILL           /* 自动注水�?*/
} BaseAction_t;

typedef enum
{
  BASE_POSITIONED_OUTPUT_NONE = 0,
  BASE_POSITIONED_OUTPUT_DRAIN,
  BASE_POSITIONED_OUTPUT_CLEAN_SPRAY,
  BASE_POSITIONED_OUTPUT_HOT_SPRAY,
  BASE_POSITIONED_OUTPUT_DRY,
  BASE_POSITIONED_OUTPUT_STANDBY,
  BASE_POSITIONED_OUTPUT_OFF,
  BASE_POSITIONED_OUTPUT_SELF_CHECK
} BasePositionedOutput_t;

/* 当前主状态，对应状态上�?frame[23]�?*/
static uint8_t base_main_status;
/* 当前子状态，对应状态上�?frame[24]�?*/
static uint8_t base_sub_status;
static uint32_t base_status_enter_tick;
/* 错误�?1，对应状态上�?frame[25]�?*/
static uint8_t base_err1;
/* 错误�?2，对应状态上�?frame[26]�?*/
static uint8_t base_err2;
/* 最近一次收到或设置的基站命令字�?*/
static uint8_t base_last_cmd;
/* 桶体链路状态，1 表示在线�?*/
static uint8_t base_link_status;
/* 软件复位请求标志�?*/
static uint8_t base_reset_requested;
/* 软件复位请求产生时的 tick�?*/
static uint32_t base_reset_tick;
/* 当前动作截止 tick�? 表示没有计时动作�?*/
static uint32_t base_action_deadline;
static uint32_t base_action_start_tick;
/* 清洁液喷淋时长，单位分钟，来自命令帧 frame[17]�?*/
static uint8_t base_clean_spray_min;
/* 清水/热水喷淋时长，单位分钟，来自命令�?frame[18]�?*/
static uint8_t base_clear_spray_min;
/* 烘干时长，单位分钟，来自命令�?frame[19]�?*/
static uint8_t base_dry_min;
/* 当前内部动作状态�?*/
static BaseAction_t base_action;
/* 桶体连接状态，1 表示在线�?*/
static uint8_t bucket_connected;
static uint8_t base_auto_fill_active;
static uint8_t base_auto_fill_target_water;
static uint8_t base_auto_fill_target_temp;
static uint8_t base_auto_fill_completed;
static uint32_t base_auto_fill_prime_deadline;
static uint32_t base_auto_fill_timeout_deadline;
static uint8_t base_auto_fill_med1_sec;
static uint8_t base_auto_fill_med2_sec;
static uint8_t base_bucket_current_water;
static uint8_t base_bucket_data_valid;
static uint8_t base_drain_empty_seen;
static uint32_t base_med_pump1_deadline;
static uint32_t base_med_pump2_deadline;
/* 请求桶体开启水泵内循环的标志�?*/
static uint8_t bucket_circulation_requested;
/* 缺液控制板最新缺液位图，内部统一使用 1 表示缺液�?*/
static uint8_t level_sensor_value;
/* 最近一次收到缺液控制板字节�?tick�?*/
static uint32_t level_sensor_last_rx_tick;
static BasePositionedOutput_t base_pending_output;
static uint32_t base_pending_duration_ms;

static void Base_SetMainStatus(uint8_t status)
{
  base_main_status = status;
  base_status_enter_tick = HAL_GetTick();
}

static uint8_t Base_GetElapsedSecByte(uint32_t now)
{
  uint32_t elapsed_sec;

  elapsed_sec = (now - base_status_enter_tick) / 1000UL;
  return (elapsed_sec > 255UL) ? 255U : (uint8_t)elapsed_sec;
}

static uint8_t Base_IsTimedProtocolStatus(uint8_t status)
{
  return (status >= BASE_STATUS_AUTO_FILL) ? 1U : 0U;
}
/* 将分钟数转换为毫秒�?*/
static uint32_t Base_MinToMs(uint8_t minutes)
{
  /* 协议中的喷淋时间按分钟下发�?*/
  return (uint32_t)minutes * 60UL * 1000UL;
}

/* 将烘干分钟数转换为毫秒�?*/
static uint32_t Base_DryMinToMs(uint8_t minutes)
{
  return Base_MinToMs(minutes);
}

/* 设置一�?GPIO 输出脚的开关状态�?*/
static void Base_SetOutput(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
  HAL_GPIO_WritePin(port, pin, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Base_GetRemainingSecByte(uint32_t deadline, uint32_t now)
{
  uint32_t remaining_sec;

  if ((deadline == 0UL) || ((int32_t)(deadline - now) <= 0))
  {
    return 0U;
  }

  remaining_sec = (deadline - now + 999UL) / 1000UL;
  return (remaining_sec > 255UL) ? 255U : (uint8_t)remaining_sec;
}

static void Base_StopMedicineDosing(void)
{
  base_med_pump1_deadline = 0UL;
  base_med_pump2_deadline = 0UL;
  Base_SetOutput(MED_PUMP1_GPIO_Port, MED_PUMP1_Pin, 0U);
  Base_SetOutput(MED_PUMP2_GPIO_Port, MED_PUMP2_Pin, 0U);
}

static void Base_StartMedicineDosing(void)
{
  uint32_t now;

  now = HAL_GetTick();
  base_med_pump1_deadline = 0UL;
  base_med_pump2_deadline = 0UL;

  if ((base_auto_fill_med1_sec != 0U) && ((level_sensor_value & BASE_LEVEL_MED_PUMP1_LOW) == 0U))
  {
    base_med_pump1_deadline = now + ((uint32_t)base_auto_fill_med1_sec * 1000UL);
    Base_SetOutput(MED_PUMP1_GPIO_Port, MED_PUMP1_Pin, 1U);
  }
  else
  {
    Base_SetOutput(MED_PUMP1_GPIO_Port, MED_PUMP1_Pin, 0U);
  }

  if ((base_auto_fill_med2_sec != 0U) && ((level_sensor_value & BASE_LEVEL_MED_PUMP2_LOW) == 0U))
  {
    base_med_pump2_deadline = now + ((uint32_t)base_auto_fill_med2_sec * 1000UL);
    Base_SetOutput(MED_PUMP2_GPIO_Port, MED_PUMP2_Pin, 1U);
  }
  else
  {
    Base_SetOutput(MED_PUMP2_GPIO_Port, MED_PUMP2_Pin, 0U);
  }
}

static void Base_UpdateMedicineDosing(uint32_t now)
{
  if ((base_med_pump1_deadline != 0UL) &&
      (((int32_t)(now - base_med_pump1_deadline) >= 0) ||
       ((level_sensor_value & BASE_LEVEL_MED_PUMP1_LOW) != 0U)))
  {
    base_med_pump1_deadline = 0UL;
    Base_SetOutput(MED_PUMP1_GPIO_Port, MED_PUMP1_Pin, 0U);
  }

  if ((base_med_pump2_deadline != 0UL) &&
      (((int32_t)(now - base_med_pump2_deadline) >= 0) ||
       ((level_sensor_value & BASE_LEVEL_MED_PUMP2_LOW) != 0U)))
  {
    base_med_pump2_deadline = 0UL;
    Base_SetOutput(MED_PUMP2_GPIO_Port, MED_PUMP2_Pin, 0U);
  }
}

/* 关闭基站所有可控输出�?*/
static void Base_AllOutputsOff(void)
{
  /*
   * 基站总停止函数�?   * 所有关机、待机、异常退出都从这里收口，避免某个输出遗漏关闭�?   */
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  Base_SetOutput(DRY_FAN_GPIO_Port, DRY_FAN_Pin, 0U);
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 0U);
  Base_SetOutput(WATER_OUT_GPIO_Port, WATER_OUT_Pin, 0U);
  Base_SetOutput(EN_IR_GPIO_Port, EN_IR_Pin, 0U);
  Base_SetOutput(MED_PUMP1_GPIO_Port, MED_PUMP1_Pin, 0U);
  Base_SetOutput(MED_PUMP2_GPIO_Port, MED_PUMP2_Pin, 0U);
  Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, 0U);
  Base_SetOutput(SPARE_SW_GPIO_Port, SPARE_SW_Pin, 0U);
  Motor_Stop();
  base_pending_output = BASE_POSITIONED_OUTPUT_NONE;
  base_pending_duration_ms = 0UL;
  base_auto_fill_active = 0U;
  base_auto_fill_target_water = 0U;
  base_auto_fill_target_temp = 0U;
  base_auto_fill_completed = 0U;
  base_auto_fill_prime_deadline = 0UL;
  base_auto_fill_timeout_deadline = 0UL;
  base_auto_fill_med1_sec = 0U;
  base_auto_fill_med2_sec = 0U;
  Base_StopMedicineDosing();
  ColorLight_Off();
  bucket_circulation_requested = 0U;
}

/* 启动一个带超时时间的动作�?*/
#if 0
static void Base_StartTimedAction(BaseAction_t action, uint8_t sub_status, uint32_t duration_ms)
{
  /*
   * 使用有符号差值比�?HAL_GetTick()，可以兼�?tick 溢出�?   * 截止时间只用于状态机推进，不阻塞当前任务�?   */
  base_action = action;
  Base_SetMainStatus(protocol_status);
  base_sub_status = 0U;
  base_action_deadline = HAL_GetTick() + duration_ms;
}

#endif

static void Base_StartPositionedAction(BaseAction_t action,
                                       uint8_t protocol_status,
                                       uint32_t duration_ms,
                                       BasePositionedOutput_t output,
                                       MotorPosition_t position)
{
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 0U);
  Base_SetOutput(WATER_OUT_GPIO_Port, WATER_OUT_Pin, 0U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  Base_SetOutput(DRY_FAN_GPIO_Port, DRY_FAN_Pin, 0U);
  Base_SetOutput(EN_IR_GPIO_Port, EN_IR_Pin, 0U);
  Base_SetOutput(MED_PUMP1_GPIO_Port, MED_PUMP1_Pin, 0U);
  Base_SetOutput(MED_PUMP2_GPIO_Port, MED_PUMP2_Pin, 0U);
  Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, 0U);
  Base_SetOutput(SPARE_SW_GPIO_Port, SPARE_SW_Pin, 0U);
  Base_StopMedicineDosing();
  bucket_circulation_requested = 0U;
  base_auto_fill_active = 0U;
  base_auto_fill_target_water = 0U;
  base_auto_fill_target_temp = 0U;
  base_auto_fill_completed = 0U;
  base_auto_fill_prime_deadline = 0UL;
  base_auto_fill_timeout_deadline = 0UL;
  base_auto_fill_med1_sec = 0U;
  base_auto_fill_med2_sec = 0U;

  base_action = action;
  Base_SetMainStatus(protocol_status);
  base_sub_status = 0U;
  base_action_deadline = 0UL;
  base_action_start_tick = 0UL;
  base_drain_empty_seen = 0U;
  base_pending_output = output;
  base_pending_duration_ms = duration_ms;
  Motor_MoveToPosition(position);
}

/* 应用排水阶段的硬件输出�?*/
static void Base_ApplyDrain(void)
{
  /* 关闭进水、加热、喷淋和清洁液，打开 WATER_OUT�?*/
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 0U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, 0U);
  Base_SetOutput(WATER_OUT_GPIO_Port, WATER_OUT_Pin, 1U);
  bucket_circulation_requested = 0U;
  ColorLight_SetRgbw(0U, 0U, 60U, 0U);
}

static uint8_t Base_IsLevelBoardTimeout(uint32_t now);

static void Base_SetCleanSprayOutputs(uint8_t water_on, uint8_t cleaner_on)
{
  uint8_t clean_pump_on;

  clean_pump_on = cleaner_on;
  if ((clean_pump_on != 0U) &&
      (((level_sensor_value & BASE_LEVEL_CLEAN_LOW) != 0U) ||
       (Base_IsLevelBoardTimeout(HAL_GetTick()) != 0U)))
  {
    clean_pump_on = 0U;
  }

  Base_SetOutput(WATER_OUT_GPIO_Port, WATER_OUT_Pin, 0U);
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, water_on);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, clean_pump_on);
}

/* Apply cleaner + clear water spray output. */
static void Base_ApplyCleanSpray(void)
{
  Base_SetCleanSprayOutputs(1U, 1U);
  ColorLight_SetRgbw(0U, 70U, 30U, 0U);
}

static void Base_ApplyCleanClearWater(void)
{
  Base_SetCleanSprayOutputs(1U, 0U);
  ColorLight_SetRgbw(0U, 40U, 80U, 0U);
}

static void Base_ApplyCleanSprayWait(void)
{
  Base_SetCleanSprayOutputs(0U, 0U);
  ColorLight_SetRgbw(0U, 20U, 20U, 0U);
}

static void Base_UpdateAutoCleanSprayPattern(uint32_t now)
{
  uint32_t elapsed;
  uint32_t phase_end;

  if ((base_action != BASE_ACTION_AUTO_CLEAN_SPRAY) ||
      (base_pending_output != BASE_POSITIONED_OUTPUT_NONE) ||
      (base_action_start_tick == 0UL))
  {
    return;
  }

  elapsed = now - base_action_start_tick;
  phase_end = BASE_AUTO_CLEAN_CLEANER_MS;
  if (elapsed < phase_end)
  {
    Base_ApplyCleanSpray();
    return;
  }

  phase_end += BASE_AUTO_CLEAN_WAIT_MS;
  if (elapsed < phase_end)
  {
    Base_ApplyCleanSprayWait();
    return;
  }

  Base_ApplyCleanClearWater();
}
/* 应用热水喷淋阶段的硬件输出�?*/
static void Base_ApplyHotSpray(void)
{
  /*
   * 热水喷淋阶段�?   *   WATER_IN 打开�?   *   EN_HEAT 打开�?   *   喷淋电机 70% 运行�?   *   CLEAN_PUMP 关闭�?   */
  Base_SetOutput(WATER_OUT_GPIO_Port, WATER_OUT_Pin, 0U);
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 1U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 1U);
  Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, 0U);
  ColorLight_SetRgbw(80U, 40U, 0U, 0U);
}

static uint8_t Base_IsDrainAction(void)
{
  return ((base_action == BASE_ACTION_AUTO_DRAIN1) ||
          (base_action == BASE_ACTION_AUTO_DRAIN2) ||
          (base_action == BASE_ACTION_AUTO_DRAIN3) ||
          (base_action == BASE_ACTION_FORCE_DRAIN) ||
          (base_action == BASE_ACTION_SINGLE_CLEAN_DRAIN) ||
          (base_action == BASE_ACTION_SINGLE_CLEAR_DRAIN)) ? 1U : 0U;
}

static void Base_UpdateDrainEmptyTimer(uint32_t now)
{
  if ((Base_IsDrainAction() == 0U) ||
      (base_bucket_data_valid == 0U) ||
      (base_pending_output != BASE_POSITIONED_OUTPUT_NONE) ||
      (HAL_GPIO_ReadPin(WATER_OUT_GPIO_Port, WATER_OUT_Pin) != GPIO_PIN_SET))
  {
    return;
  }

  if ((base_bucket_current_water == 0U) && (base_drain_empty_seen == 0U))
  {
    base_drain_empty_seen = 1U;
    base_action_deadline = now + BASE_DRAIN_AFTER_EMPTY_MS;
  }
}
static void Base_ApplyDry(void);

static void Base_CompletePositionedAction(void)
{
  BasePositionedOutput_t output;
  uint8_t start_timed_action;

  output = base_pending_output;
  base_pending_output = BASE_POSITIONED_OUTPUT_NONE;
  start_timed_action = 1U;

  switch (output)
  {
    case BASE_POSITIONED_OUTPUT_DRAIN:
      Base_ApplyDrain();
      base_pending_duration_ms = BASE_DRAIN_MAX_MS;
      start_timed_action = 1U;
      break;

    case BASE_POSITIONED_OUTPUT_CLEAN_SPRAY:
      if (base_action == BASE_ACTION_AUTO_CLEAN_SPRAY)
      {
        Base_ApplyCleanSpray();
        bucket_circulation_requested = 1U;
      }
      else
      {
        Base_ApplyCleanSpray();
      }
      break;

    case BASE_POSITIONED_OUTPUT_HOT_SPRAY:
      Base_ApplyHotSpray();
      if (base_action == BASE_ACTION_AUTO_HOT_SPRAY2)
      {
        bucket_circulation_requested = 1U;
      }
      break;

    case BASE_POSITIONED_OUTPUT_DRY:
      Base_ApplyDry();
      break;

    case BASE_POSITIONED_OUTPUT_STANDBY:
      Motor_Stop();
      ColorLight_Off();
      bucket_circulation_requested = 0U;
      base_action = BASE_ACTION_IDLE;
      Base_SetMainStatus(BASE_STATUS_STANDBY);
      base_sub_status = 0U;
      start_timed_action = 0U;
      break;

    case BASE_POSITIONED_OUTPUT_OFF:
      Motor_Stop();
      ColorLight_Off();
      bucket_circulation_requested = 0U;
      base_action = BASE_ACTION_IDLE;
      Base_SetMainStatus(BASE_STATUS_OFF);
      base_sub_status = 0U;
      start_timed_action = 0U;
      break;

    case BASE_POSITIONED_OUTPUT_SELF_CHECK:
      Motor_Stop();
      SystemMonitor_ClearErrors();
      base_action = BASE_ACTION_IDLE;
      Base_SetMainStatus(BASE_STATUS_SELF_CHECK);
      base_sub_status = 0U;
      ColorLight_SetRgbw(0U, 50U, 0U, 0U);
      start_timed_action = 0U;
      break;

    default:
      return;
  }

  if (start_timed_action != 0U)
  {
    base_action_start_tick = HAL_GetTick();
    base_action_deadline = base_action_start_tick + base_pending_duration_ms;
  }
  else
  {
    base_action_start_tick = 0UL;
    base_action_deadline = 0UL;
  }
  base_pending_duration_ms = 0UL;
  Base_UpdateDrainEmptyTimer(HAL_GetTick());
}

/* 应用烘干阶段的硬件输出�?*/
static void Base_ApplyDry(void)
{
  /* 关闭水路和喷淋电机，打开烘干风机�?*/
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 0U);
  Base_SetOutput(WATER_OUT_GPIO_Port, WATER_OUT_Pin, 0U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, 0U);
  Motor_Stop();
  Base_SetOutput(DRY_FAN_GPIO_Port, DRY_FAN_Pin, 1U);
  bucket_circulation_requested = 0U;
  ColorLight_SetRgbw(70U, 20U, 0U, 20U);
}

/* 关闭喷淋相关输出，不影响排水和烘干输出�?*/
static void Base_StopAutoFill(void)
{
  uint8_t keep_warm;

  keep_warm = (base_auto_fill_target_temp != 0U) ? 1U : 0U;
  base_auto_fill_active = 0U;
  base_auto_fill_target_temp = 0U;
  base_auto_fill_completed = 1U;
  base_auto_fill_prime_deadline = 0UL;
  base_auto_fill_timeout_deadline = 0UL;
  base_auto_fill_med1_sec = 0U;
  base_auto_fill_med2_sec = 0U;
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 0U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  if (base_action == BASE_ACTION_AUTO_FILL)
  {
    base_action = BASE_ACTION_IDLE;
    base_action_deadline = 0UL;
    Base_SetMainStatus((keep_warm != 0U) ? BASE_STATUS_FILL_KEEP_WARM : BASE_STATUS_FILL_WAIT_DRAIN);
    base_sub_status = 0U;
    ColorLight_Off();
  }
}

static void Base_CheckAutoFillTimeout(uint32_t now)
{
  if ((base_auto_fill_active == 0U) || (base_auto_fill_timeout_deadline == 0UL))
  {
    return;
  }

  if ((int32_t)(now - base_auto_fill_timeout_deadline) < 0)
  {
    return;
  }

  base_auto_fill_active = 0U;
  base_auto_fill_target_water = 0U;
  base_auto_fill_target_temp = 0U;
  base_auto_fill_completed = 0U;
  base_auto_fill_prime_deadline = 0UL;
  base_auto_fill_timeout_deadline = 0UL;
  base_auto_fill_med1_sec = 0U;
  base_auto_fill_med2_sec = 0U;
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 0U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  Base_StopMedicineDosing();

  if (base_action == BASE_ACTION_AUTO_FILL)
  {
    base_action = BASE_ACTION_IDLE;
    base_action_deadline = 0UL;
    Base_SetMainStatus(BASE_STATUS_STANDBY);
    base_sub_status = 0U;
    ColorLight_Off();
  }
}
static void Base_UpdateAutoFillPrime(uint32_t now)
{
  if ((base_auto_fill_active == 0U) || (base_auto_fill_prime_deadline == 0UL))
  {
    return;
  }

  if ((int32_t)(now - base_auto_fill_prime_deadline) < 0)
  {
    return;
  }

  base_auto_fill_prime_deadline = 0UL;
  Base_StartMedicineDosing();
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 1U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin,
                 (base_auto_fill_target_temp != 0U) ? 1U : 0U);
}
static void Base_UpdateAutoFill(uint8_t current_water, uint8_t current_temp)
{
  if (base_auto_fill_active == 0U)
  {
    return;
  }

  if (current_water >= base_auto_fill_target_water)
  {
    Base_StopAutoFill();
    return;
  }

  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 1U);
  if (base_auto_fill_prime_deadline != 0UL)
  {
    Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
    return;
  }
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin,
                 (current_temp < base_auto_fill_target_temp) ? 1U : 0U);
}

static void Base_StartAutoFill(const uint8_t *frame)
{
  if ((base_auto_fill_completed != 0U) && (frame[16] != 0U) &&
      (base_bucket_data_valid != 0U) && (base_bucket_current_water >= frame[16]))
  {
    Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 0U);
    Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
    base_auto_fill_prime_deadline = 0UL;
    base_auto_fill_timeout_deadline = 0UL;
    base_auto_fill_med1_sec = 0U;
    base_auto_fill_med2_sec = 0U;
    return;
  }

  Base_AllOutputsOff();
  base_auto_fill_active = 1U;
  base_auto_fill_target_water = frame[16];
  base_auto_fill_target_temp = frame[6];
  base_auto_fill_completed = 0U;
  base_auto_fill_prime_deadline = HAL_GetTick() + BASE_AUTO_FILL_PRIME_MS;
  base_auto_fill_timeout_deadline = HAL_GetTick() + BASE_AUTO_FILL_TIMEOUT_MS;
  base_auto_fill_med1_sec = frame[20];
  base_auto_fill_med2_sec = frame[21];
  base_action = BASE_ACTION_AUTO_FILL;
  Base_SetMainStatus(BASE_STATUS_AUTO_FILL);
  base_sub_status = 0U;
  base_action_deadline = 0UL;
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 1U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  ColorLight_SetRgbw(0U, 40U, 80U, 0U);
}

static void Base_HandleAutoFillFrame(const uint8_t *frame)
{
  if (base_auto_fill_active == 0U)
  {
    Base_StartAutoFill(frame);
    return;
  }

  base_auto_fill_target_water = frame[16];
  base_auto_fill_target_temp = frame[6];
  base_auto_fill_completed = 0U;
  if (base_auto_fill_prime_deadline != 0UL)
  {
    base_auto_fill_med1_sec = frame[20];
    base_auto_fill_med2_sec = frame[21];
  }
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 1U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin,
                 (base_auto_fill_prime_deadline == 0UL && base_auto_fill_target_temp != 0U) ? 1U : 0U);
}
/* 正常结束或中断动作后统一回到待机�?*/
static void Base_FinishToStandby(void)
{
  Base_StartPositionedAction(BASE_ACTION_IDLE,
                             BASE_STATUS_STANDBY,
                             0UL,
                             BASE_POSITIONED_OUTPUT_STANDBY,
                             MOTOR_POSITION_DRAIN_0);
}

/* 判断当前是否处于清洁液喷淋动作�?*/
/* 缺液控制板是否已经通信超时�?*/
static uint8_t Base_IsLevelBoardTimeout(uint32_t now)
{
  return ((now - level_sensor_last_rx_tick) >= BASE_LEVEL_BOARD_TIMEOUT_MS) ? 1U : 0U;
}

/* 根据缺液控制板位图刷�?err1�?*/
static void Base_UpdateLevelErrors(uint32_t now)
{
  uint8_t level_bits;

  level_bits = level_sensor_value & BASE_LEVEL_VALID_MASK;
  if ((level_bits & BASE_LEVEL_MED_PUMP1_LOW) != 0U)
  {
    base_err1 |= BASE_ERR1_MED_PUMP1_LOW;
  }
  if ((level_bits & BASE_LEVEL_MED_PUMP2_LOW) != 0U)
  {
    base_err1 |= BASE_ERR1_MED_PUMP2_LOW;
  }
  if ((level_bits & BASE_LEVEL_CLEAN_LOW) != 0U)
  {
    base_err1 |= BASE_ERR1_CLEAN_LOW;
  }
  if (Base_IsLevelBoardTimeout(now) != 0U)
  {
    base_err1 |= BASE_ERR1_LEVEL_BOARD;
  }
}

/* 执行缺液保护：只停止相关�?喷淋动作，不影响排水和烘干�?*/
static void Base_ApplyLevelProtection(void)
{
  if ((level_sensor_value & BASE_LEVEL_MED_PUMP1_LOW) != 0U)
  {
    Base_SetOutput(MED_PUMP1_GPIO_Port, MED_PUMP1_Pin, 0U);
  }
  if ((level_sensor_value & BASE_LEVEL_MED_PUMP2_LOW) != 0U)
  {
    Base_SetOutput(MED_PUMP2_GPIO_Port, MED_PUMP2_Pin, 0U);
  }
  if ((level_sensor_value & BASE_LEVEL_CLEAN_LOW) != 0U)
  {
    Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, 0U);
  }
}

/* 判断当前内部动作是否属于自动清洁流程�?*/
static uint8_t Base_IsAutoCleanAction(void)
{
  return ((base_action >= BASE_ACTION_AUTO_DRAIN1) && (base_action <= BASE_ACTION_AUTO_DRY)) ? 1U : 0U;
}

/* 停止正在执行的自动清洁流程�?*/
static void Base_StopAutoClean(void)
{
  if (Base_IsAutoCleanAction() != 0U)
  {
    Logging_Print("Auto clean stop\r\n");
    Base_FinishToStandby();
  }
}

/* 推进自动清洁流程到下一阶段�?*/
static void Base_AdvanceAutoClean(void)
{
  /*
   * 自动清洁流程推进�?   *
   * 1 排水
   * 2 清洁喷淋，同时请求桶体水泵内循环，持�?frame[17] 分钟
   * 3 排水
   * 4 热水喷淋，同时请求桶体水泵内循环，持�?frame[18] 分钟
   * 5 排水
   * 6 热风烘干，持�?frame[19] 分钟
   */
  switch (base_action)
  {
    case BASE_ACTION_AUTO_DRAIN1:
      Base_StartPositionedAction(BASE_ACTION_AUTO_CLEAN_SPRAY,
                                 BASE_STATUS_CLEAN_SPRAY,
                                 Base_MinToMs(base_clean_spray_min),
                                 BASE_POSITIONED_OUTPUT_CLEAN_SPRAY,
                                 MOTOR_POSITION_SPRAY_90);
      break;

    case BASE_ACTION_AUTO_CLEAN_SPRAY:
      Base_StartPositionedAction(BASE_ACTION_AUTO_DRAIN2,
                                 BASE_STATUS_CLEAN_DRAIN1,
                                 BASE_DEFAULT_DRAIN_MS,
                                 BASE_POSITIONED_OUTPUT_DRAIN,
                                 MOTOR_POSITION_DRAIN_0);
      break;

    case BASE_ACTION_AUTO_DRAIN2:
      Base_StartPositionedAction(BASE_ACTION_AUTO_HOT_SPRAY2,
                                 BASE_STATUS_CLEAR_SPRAY,
                                 Base_MinToMs(base_clear_spray_min),
                                 BASE_POSITIONED_OUTPUT_HOT_SPRAY,
                                 MOTOR_POSITION_SPRAY_90);
      break;

    case BASE_ACTION_AUTO_HOT_SPRAY2:
      Base_StartPositionedAction(BASE_ACTION_AUTO_DRAIN3,
                                 BASE_STATUS_CLEAR_DRAIN2,
                                 BASE_DEFAULT_DRAIN_MS,
                                 BASE_POSITIONED_OUTPUT_DRAIN,
                                 MOTOR_POSITION_DRAIN_0);
      break;

    case BASE_ACTION_AUTO_DRAIN3:
      Base_StartPositionedAction(BASE_ACTION_AUTO_DRY,
                                 BASE_STATUS_CLEAN_DRY,
                                 Base_DryMinToMs(base_dry_min),
                                 BASE_POSITIONED_OUTPUT_DRY,
                                 MOTOR_POSITION_DRAIN_0);
      break;

    case BASE_ACTION_AUTO_DRY:
      Base_FinishToStandby();
      break;

    default:
      Base_FinishToStandby();
      break;
  }
}

/* 启动自动清洁流程�?*/
static void Base_StartAutoClean(void)
{
  Base_StartPositionedAction(BASE_ACTION_AUTO_DRAIN1,
                             BASE_STATUS_FORCE_DRAIN,
                             BASE_DEFAULT_DRAIN_MS,
                             BASE_POSITIONED_OUTPUT_DRAIN,
                             MOTOR_POSITION_DRAIN_0);
}

/* 初始化基站状态机和所有硬件输出�?*/
void SystemMonitor_Init(void)
{
  base_main_status = BASE_STATUS_POWER_ON;
  base_sub_status = 0U;
  base_status_enter_tick = HAL_GetTick();
  base_err1 = 0U;
  base_err2 = 0U;
  base_last_cmd = BASE_CMD_STANDBY;
  base_link_status = 0U;
  base_reset_requested = 0U;
  base_reset_tick = 0UL;
  base_action_deadline = 0UL;
  base_action_start_tick = 0UL;
  base_drain_empty_seen = 0U;
  base_clean_spray_min = 0U;
  base_clear_spray_min = 0U;
  base_dry_min = 0U;
  base_action = BASE_ACTION_IDLE;
  base_auto_fill_prime_deadline = 0UL;
  base_auto_fill_timeout_deadline = 0UL;
  base_auto_fill_med1_sec = 0U;
  base_auto_fill_med2_sec = 0U;
  bucket_connected = 0U;
  bucket_circulation_requested = 0U;
  base_bucket_data_valid = 0U;
  level_sensor_value = 0U;
  level_sensor_last_rx_tick = HAL_GetTick();
  base_pending_output = BASE_POSITIONED_OUTPUT_NONE;
  base_pending_duration_ms = 0UL;
  Base_AllOutputsOff();
}

/* 周期推进基站状态机�?*/
void SystemMonitor_TaskProcess(void)
{
  uint32_t now;  /* 当前 HAL tick�?*/

  now = HAL_GetTick();
  base_err1 = 0U;
  base_err2 = 0U;
  if (Motor_HasFault() != 0U)
  {
    base_err2 |= BASE_ERR2_SPRAY_MOTOR;
  }
  Base_UpdateLevelErrors(now);
  Base_ApplyLevelProtection();
  Base_CheckAutoFillTimeout(now);
  Base_UpdateAutoFillPrime(now);
  Base_UpdateMedicineDosing(now);
  Base_UpdateDrainEmptyTimer(now);
  Base_UpdateAutoCleanSprayPattern(now);

  if (base_pending_output != BASE_POSITIONED_OUTPUT_NONE)
  {
    if (Motor_HasFault() != 0U)
    {
      base_pending_output = BASE_POSITIONED_OUTPUT_NONE;
      base_pending_duration_ms = 0UL;
      base_action = BASE_ACTION_IDLE;
      Base_SetMainStatus(BASE_STATUS_STANDBY);
      base_sub_status = 0U;
    }
    else if (Motor_IsBusy() == 0U)
    {
      Base_CompletePositionedAction();
    }
  }

  /* 当前动作到期后，自动清洁进入下一阶段；单项动作直接结束待机�?*/
  if ((base_action_deadline != 0UL) && ((int32_t)(now - base_action_deadline) >= 0))
  {
    if (Base_IsAutoCleanAction() != 0U)
    {
      Base_AdvanceAutoClean();
    }
    else if (base_action == BASE_ACTION_SINGLE_CLEAN_SPRAY)
    {
      Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAN_DRAIN,
                                 BASE_STATUS_FORCE_DRAIN,
                                 BASE_DEFAULT_DRAIN_MS,
                                 BASE_POSITIONED_OUTPUT_DRAIN,
                                 MOTOR_POSITION_DRAIN_0);
    }
    else if (base_action == BASE_ACTION_SINGLE_CLEAR_SPRAY)
    {
      Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAR_DRAIN,
                                 BASE_STATUS_FORCE_DRAIN,
                                 BASE_DEFAULT_DRAIN_MS,
                                 BASE_POSITIONED_OUTPUT_DRAIN,
                                 MOTOR_POSITION_DRAIN_0);
    }
    else
    {
      Base_FinishToStandby();
    }
  }

  if ((base_reset_requested != 0U) && ((now - base_reset_tick) > 200UL))
  {
    NVIC_SystemReset();
  }
}

/* 停止所有输出并回到待机�?*/
void SystemMonitor_StopAllOutputs(void)
{
  Base_FinishToStandby();
}

/* 直接设置主状态和子状态�?*/
void SystemMonitor_SetMainStatus(uint8_t main_status, uint8_t sub_status)
{
  Base_SetMainStatus(main_status);
  base_sub_status = sub_status;
}

/* 获取当前主状态�?*/
uint8_t SystemMonitor_GetMainStatus(void)
{
  return base_main_status;
}

/* 获取当前子状态�?*/
uint8_t SystemMonitor_GetSubStatus(void)
{
  if (base_sub_status != 0U)
  {
    return base_sub_status;
  }
  if (Base_IsTimedProtocolStatus(base_main_status) != 0U)
  {
    return SystemMonitor_GetTimerRemainingMin();
  }
  return Base_GetElapsedSecByte(HAL_GetTick());
}

/* Return full elapsed seconds since entering the current main status. */
uint32_t SystemMonitor_GetStatusElapsedSec(void)
{
  return (HAL_GetTick() - base_status_enter_tick) / 1000UL;
}

uint8_t SystemMonitor_GetErrCode1(void)
{
  return base_err1 & 0x7FU;
}

/* 获取错误�?2�?*/
uint8_t SystemMonitor_GetErrCode2(void)
{
  return base_err2 & 0x7FU;
}

/* 保存最近一次基站命令字�?*/
void SystemMonitor_SetCommand(uint8_t cmd)
{
  base_last_cmd = cmd;
}

/* 获取最近一次基站命令字�?*/
uint8_t SystemMonitor_GetCommand(void)
{
  return base_last_cmd;
}

/* 设置链路状态�?*/
void SystemMonitor_SetLinkStatus(uint8_t link_status)
{
  base_link_status = (link_status != 0U) ? 1U : 0U;
}

/* 获取链路状态�?*/
uint8_t SystemMonitor_GetLinkStatus(void)
{
  return base_link_status;
}

/* 设置泡脚定时编码，当前版本预留�?*/
void SystemMonitor_SetBathTimer(uint8_t timer_code)
{
  (void)timer_code;
}

/* 获取当前动作剩余分钟数�?*/
uint8_t SystemMonitor_GetTimerRemainingMin(void)
{
  uint32_t remaining_ms;  /* 当前动作剩余毫秒数�?*/

  if ((base_action_deadline == 0UL) || ((int32_t)(base_action_deadline - HAL_GetTick()) <= 0))
  {
    return 0U;
  }
  remaining_ms = base_action_deadline - HAL_GetTick();
  return (uint8_t)((remaining_ms + 59999UL) / 60000UL);
}

/* 获取当前动作剩余秒数�?*/
uint32_t SystemMonitor_GetTimerRemainingSec(void)
{
  if ((base_action_deadline == 0UL) || ((int32_t)(base_action_deadline - HAL_GetTick()) <= 0))
  {
    return 0UL;
  }
  return (base_action_deadline - HAL_GetTick() + 999UL) / 1000UL;
}

/* 请求软件复位�?*/
uint8_t SystemMonitor_GetMedicine1RemainingSec(void)
{
  return Base_GetRemainingSecByte(base_med_pump1_deadline, HAL_GetTick());
}

uint8_t SystemMonitor_GetMedicine2RemainingSec(void)
{
  return Base_GetRemainingSecByte(base_med_pump2_deadline, HAL_GetTick());
}

uint8_t SystemMonitor_GetBucketCurrentWater(void)
{
  return base_bucket_current_water;
}

uint8_t SystemMonitor_GetAutoFillTargetWater(void)
{
  return base_auto_fill_target_water;
}
void SystemMonitor_RequestReset(void)
{
  base_reset_requested = 1U;
  base_reset_tick = HAL_GetTick();
}

/* 查询是否已请求软件复位�?*/
uint8_t SystemMonitor_IsResetRequested(void)
{
  return base_reset_requested;
}

/* 清除错误码�?*/
void SystemMonitor_ClearErrors(void)
{
  base_err1 = 0U;
  base_err2 = 0U;
}

/* 设置桶体连接状态�?*/
void Base_SetBucketConnected(uint8_t connected)
{
  uint8_t was_connected;  /* 更新前的桶体连接状态�?*/

  was_connected = bucket_connected;
  bucket_connected = (connected != 0U) ? 1U : 0U;
  base_link_status = bucket_connected;
  /*
   * 只有在需要桶体协同的阶段掉线，才立即停止流程�?   * 例如单独排水不依赖桶体，断开时仍可按本机状态继续处理�?   */
  if ((was_connected != 0U) && (bucket_connected == 0U) &&
      ((bucket_circulation_requested != 0U) || (base_auto_fill_active != 0U)))
  {
    Base_FinishToStandby();
  }
}

/* 返回桶体是否在线�?*/
uint8_t Base_IsBucketConnected(void)
{
  return bucket_connected;
}

/* 返回是否请求桶体水泵内循环�?*/
uint8_t Base_IsBucketCirculationRequested(void)
{
  return bucket_circulation_requested;
}

/* 校验并缓存缺液控制板上报字节；协议低 3 位为 1 表示有液�? 表示缺液�?*/
void Base_SetLevelSensorValue(uint8_t level)
{
  if ((level & BASE_LEVEL_FRAME_FIXED_MASK) != BASE_LEVEL_FRAME_FIXED)
  {
    return;
  }

  level_sensor_value = (uint8_t)((~level) & BASE_LEVEL_VALID_MASK);
  level_sensor_last_rx_tick = HAL_GetTick();
}

/* Update bucket realtime water/temp from data[3] == 0x00 frame. */
void Base_UpdateBucketRealtimeData(const uint8_t *frame)
{
  if ((frame == NULL) || (frame[3] != 0x00U))
  {
    return;
  }

  base_bucket_current_water = frame[5];
  base_bucket_data_valid = 1U;
  Base_UpdateAutoFill(frame[5], frame[6]);
  Base_UpdateDrainEmptyTimer(HAL_GetTick());
}

/* Handle base command frames from bucket. */
void Base_HandleCommand(const uint8_t *frame)
{
  uint8_t cmd;  /* 当前命令字，来自 frame[3]�?*/

  if (frame == NULL)
  {
    return;
  }

  /*
   * 协议字段�?   *   frame[3]  = 基站命令 0xB0-0xBF
   *   frame[17] = 清洁液喷淋时间，单位分钟
   *   frame[18] = 清水/热水喷淋时间，单位分�?   *   frame[19] = 烘干时间，单位分�?   */
  cmd = frame[3];
  base_last_cmd = cmd;
  base_clean_spray_min = frame[17];
  base_clear_spray_min = frame[18];
  base_dry_min = frame[19];

  switch (cmd)
  {
    case BASE_CMD_OFF:
      Base_StartPositionedAction(BASE_ACTION_IDLE,
                                 BASE_STATUS_OFF,
                                 0UL,
                                 BASE_POSITIONED_OUTPUT_OFF,
                                 MOTOR_POSITION_DRAIN_0);
      break;

    case BASE_CMD_STANDBY:
      Base_FinishToStandby();
      break;

    case BASE_CMD_AUTO_FILL:
      Base_HandleAutoFillFrame(frame);
      break;

    case BASE_CMD_AUTO_CLEAN:
      Logging_Print("Auto clean start\r\n");
      Base_StartAutoClean();
      break;

    case BASE_CMD_FORCE_DRAIN:
      Base_StartPositionedAction(BASE_ACTION_FORCE_DRAIN,
                                 BASE_STATUS_FORCE_DRAIN,
                                 BASE_DEFAULT_DRAIN_MS,
                                 BASE_POSITIONED_OUTPUT_DRAIN,
                                 MOTOR_POSITION_DRAIN_0);
      break;

    case BASE_CMD_CLEAN_SPRAY:
      Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAN_SPRAY,
                                 BASE_STATUS_SINGLE_CLEAN_SPRAY,
                                 Base_MinToMs(base_clean_spray_min),
                                 BASE_POSITIONED_OUTPUT_CLEAN_SPRAY,
                                 MOTOR_POSITION_SPRAY_90);
      break;

    case BASE_CMD_CLEAR_SPRAY:
      Base_StartPositionedAction(BASE_ACTION_SINGLE_CLEAR_SPRAY,
                                 BASE_STATUS_SINGLE_CLEAR_SPRAY,
                                 Base_MinToMs(base_clear_spray_min),
                                 BASE_POSITIONED_OUTPUT_HOT_SPRAY,
                                 MOTOR_POSITION_SPRAY_90);
      break;

    case BASE_CMD_DRY:
      Base_StartPositionedAction(BASE_ACTION_SINGLE_DRY,
                                 BASE_STATUS_SINGLE_DRY,
                                 Base_DryMinToMs(base_dry_min),
                                 BASE_POSITIONED_OUTPUT_DRY,
                                 MOTOR_POSITION_DRAIN_0);
      break;

    case BASE_CMD_SELF_CHECK:
      Base_StartPositionedAction(BASE_ACTION_IDLE,
                                 BASE_STATUS_SELF_CHECK,
                                 0UL,
                                 BASE_POSITIONED_OUTPUT_SELF_CHECK,
                                 MOTOR_POSITION_DRAIN_0);
      break;

    case BASE_CMD_STOP_AUTO_CLEAN:
      Base_StopAutoClean();
      break;

    default:
      break;
  }
}

/* 填充基站状态上报帧中的业务数据字段�?*/
void Base_BuildStatusData(uint8_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  /*
   * 基站状态上报数据：
   *   frame[16] 上水控制状�?   *   frame[17] 清洁液喷淋时�?   *   frame[18] 清水/热水喷淋时间
   *   frame[19] 烘干时间，单位分�?   *   frame[20] 药泵 1 状�?   *   frame[21] 药泵 2 状�?   *   frame[22] 清洁液泵状�?   *   frame[23] 主状�?   *   frame[24] 子状�?   *   frame[25] 错误�?1
   *   frame[26] 错误�?2
   *   frame[27] 氛围灯状�?   */
  frame[4] = bucket_connected;
  frame[16] = (HAL_GPIO_ReadPin(WATER_IN_GPIO_Port, WATER_IN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  frame[17] = base_clean_spray_min;
  frame[18] = base_clear_spray_min;
  frame[19] = base_dry_min;
  frame[20] = SystemMonitor_GetMedicine1RemainingSec();
  frame[21] = SystemMonitor_GetMedicine2RemainingSec();
  frame[22] = (HAL_GPIO_ReadPin(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  frame[23] = base_main_status;
  frame[24] = SystemMonitor_GetSubStatus();
  frame[25] = SystemMonitor_GetErrCode1();
  frame[26] = SystemMonitor_GetErrCode2();

  if ((frame[25] != 0U) || (frame[26] != 0U))
  {
    frame[27] = 0x04U;
  }
  else if ((base_main_status == BASE_STATUS_CLEAN_DRY) ||
           (base_main_status == BASE_STATUS_SINGLE_DRY))
  {
    frame[27] = 0x03U;
  }
  else if (Base_IsTimedProtocolStatus(base_main_status) != 0U)
  {
    frame[27] = 0x02U;
  }
  else
  {
    frame[27] = 0x00U;
  }
}
