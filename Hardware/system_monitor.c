#include "system_monitor.h"
#include "color_light.h"
#include "log.h"
#include "main.h"
#include "motor_control.h"
#include <string.h>

#define BASE_CMD_OFF              0xB0U
#define BASE_CMD_STANDBY          0xB1U
#define BASE_CMD_AUTO_CLEAN       0xB3U
#define BASE_CMD_FORCE_DRAIN      0xB4U
#define BASE_CMD_CLEAN_SPRAY      0xB5U
#define BASE_CMD_CLEAR_SPRAY      0xB6U
#define BASE_CMD_DRY              0xB7U
#define BASE_CMD_SELF_CHECK       0xB8U

#define BASE_DEFAULT_DRAIN_MS     (60UL * 1000UL)
#define BASE_WATER_READY_LEVEL    1U

/*
 * 内部动作状态。
 * 这些状态比协议上报的 sub_status 更细，用来精确推进自动清洁流程。
 */
typedef enum
{
  BASE_ACTION_IDLE = 0,
  BASE_ACTION_AUTO_DRAIN1,
  BASE_ACTION_AUTO_HOT_SPRAY1,
  BASE_ACTION_AUTO_CLEAN_SPRAY,
  BASE_ACTION_AUTO_DRAIN2,
  BASE_ACTION_AUTO_HOT_SPRAY2,
  BASE_ACTION_AUTO_DRAIN3,
  BASE_ACTION_AUTO_DRY,
  BASE_ACTION_FORCE_DRAIN,
  BASE_ACTION_SINGLE_CLEAN_SPRAY,
  BASE_ACTION_SINGLE_CLEAR_SPRAY,
  BASE_ACTION_SINGLE_DRY
} BaseAction_t;

static uint8_t base_main_status;
static uint8_t base_sub_status;
static uint8_t base_err1;
static uint8_t base_err2;
static uint8_t base_last_cmd;
static uint8_t base_link_status;
static uint8_t base_reset_requested;
static uint32_t base_reset_tick;
static uint32_t base_action_deadline;
static uint8_t base_clean_spray_min;
static uint8_t base_clear_spray_min;
static uint8_t base_dry_code;
static BaseAction_t base_action;
static uint8_t bucket_connected;
static uint8_t bucket_circulation_requested;
static uint8_t level_sensor_value;

static uint32_t Base_MinToMs(uint8_t minutes)
{
  /* 协议中的喷淋时间按分钟下发。 */
  return (uint32_t)minutes * 60UL * 1000UL;
}

static uint32_t Base_DryCodeToMs(uint8_t code)
{
  /* 协议规定烘干时间 1-9 表示 code * 10 分钟。 */
  if (code > 9U)
  {
    code = 9U;
  }
  return (uint32_t)code * 10UL * 60UL * 1000UL;
}

static void Base_SetOutput(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
  HAL_GPIO_WritePin(port, pin, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Base_AllOutputsOff(void)
{
  /*
   * 基站总停止函数。
   * 所有关机、待机、异常退出都从这里收口，避免某个输出遗漏关闭。
   */
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
  ColorLight_Off();
  bucket_circulation_requested = 0U;
}

static void Base_StartTimedAction(BaseAction_t action, uint8_t sub_status, uint32_t duration_ms)
{
  /*
   * 启动一个带超时时间的动作。
   * 使用有符号差值比较 HAL_GetTick()，可以兼容 tick 溢出。
   */
  base_action = action;
  base_main_status = BASE_STATUS_RUNNING;
  base_sub_status = sub_status;
  base_action_deadline = HAL_GetTick() + duration_ms;
}

static void Base_ApplyDrain(void)
{
  /* 排水阶段：关闭进水/加热/喷淋/清洁液，打开 WATER_OUT。 */
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 0U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, 0U);
  Motor_Stop();
  Base_SetOutput(WATER_OUT_GPIO_Port, WATER_OUT_Pin, 1U);
  bucket_circulation_requested = 0U;
  ColorLight_SetRgbw(0U, 0U, 60U, 0U);
}

static void Base_ApplyHotSpray(uint8_t with_cleaner)
{
  /*
   * 热水喷淋阶段：
   *   WATER_IN 打开；
   *   EN_HEAT 打开；
   *   喷淋电机 70% 运行；
   *   with_cleaner=1 时同步打开清洁液泵。
   */
  Base_SetOutput(WATER_OUT_GPIO_Port, WATER_OUT_Pin, 0U);
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 1U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 1U);
  Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, with_cleaner);
  Motor_Run(70U, 0U);
  ColorLight_SetRgbw(80U, 40U, 0U, 0U);
}

static void Base_ApplyDry(void)
{
  /* 烘干阶段：关闭水路和喷淋电机，打开风机。 */
  Base_SetOutput(WATER_IN_GPIO_Port, WATER_IN_Pin, 0U);
  Base_SetOutput(WATER_OUT_GPIO_Port, WATER_OUT_Pin, 0U);
  Base_SetOutput(EN_HEAT_GPIO_Port, EN_HEAT_Pin, 0U);
  Base_SetOutput(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin, 0U);
  Motor_Stop();
  Base_SetOutput(DRY_FAN_GPIO_Port, DRY_FAN_Pin, 1U);
  bucket_circulation_requested = 0U;
  ColorLight_SetRgbw(70U, 20U, 0U, 20U);
}

static void Base_FinishToStandby(void)
{
  /* 动作正常结束或被待机命令打断后，统一回到待机状态。 */
  Base_AllOutputsOff();
  base_action = BASE_ACTION_IDLE;
  base_action_deadline = 0UL;
  base_main_status = BASE_STATUS_STANDBY;
  base_sub_status = BASE_SUB_IDLE;
}

static void Base_AdvanceAutoClean(void)
{
  /*
   * 自动清洁流程推进。
   * 用户给定步骤为 1、2、3、5、6、7、8，编号 4 按需求跳过。
   *
   * 1 排水
   * 2 热水喷淋
   * 3 清洁液喷淋，同时请求桶体水泵内循环
   * 5 关闭喷淋/内循环，开启排水
   * 6 热水喷淋，到达水位后请求桶体水泵内循环
   * 7 排水
   * 8 烘干
   */
  switch (base_action)
  {
    case BASE_ACTION_AUTO_DRAIN1:
      Base_ApplyHotSpray(0U);
      Base_StartTimedAction(BASE_ACTION_AUTO_HOT_SPRAY1, BASE_SUB_CLEAR_SPRAY, Base_MinToMs(base_clear_spray_min));
      break;

    case BASE_ACTION_AUTO_HOT_SPRAY1:
      Base_ApplyHotSpray(1U);
      /* 通信层会把该请求写入状态帧，桶体收到后开启水泵内循环。 */
      bucket_circulation_requested = 1U;
      Base_StartTimedAction(BASE_ACTION_AUTO_CLEAN_SPRAY, BASE_SUB_CLEAN_SPRAY, Base_MinToMs(base_clean_spray_min));
      break;

    case BASE_ACTION_AUTO_CLEAN_SPRAY:
      Base_ApplyDrain();
      Base_StartTimedAction(BASE_ACTION_AUTO_DRAIN2, BASE_SUB_CLEAN_DRAIN1, BASE_DEFAULT_DRAIN_MS);
      break;

    case BASE_ACTION_AUTO_DRAIN2:
      Base_ApplyHotSpray(0U);
      Base_StartTimedAction(BASE_ACTION_AUTO_HOT_SPRAY2, BASE_SUB_CLEAR_SPRAY, Base_MinToMs(base_clear_spray_min));
      break;

    case BASE_ACTION_AUTO_HOT_SPRAY2:
      Base_ApplyDrain();
      Base_StartTimedAction(BASE_ACTION_AUTO_DRAIN3, BASE_SUB_CLEAN_DRAIN2, BASE_DEFAULT_DRAIN_MS);
      break;

    case BASE_ACTION_AUTO_DRAIN3:
      Base_ApplyDry();
      Base_StartTimedAction(BASE_ACTION_AUTO_DRY, BASE_SUB_DRY, Base_DryCodeToMs(base_dry_code));
      break;

    case BASE_ACTION_AUTO_DRY:
      Base_FinishToStandby();
      break;

    default:
      Base_FinishToStandby();
      break;
  }
}

static void Base_StartAutoClean(void)
{
  Base_ApplyDrain();
  Base_StartTimedAction(BASE_ACTION_AUTO_DRAIN1, BASE_SUB_CLEAN_DRAIN1, BASE_DEFAULT_DRAIN_MS);
}

void SystemMonitor_Init(void)
{
  base_main_status = BASE_STATUS_POWER_ON;
  base_sub_status = BASE_SUB_IDLE;
  base_err1 = 0U;
  base_err2 = 0U;
  base_last_cmd = BASE_CMD_STANDBY;
  base_link_status = 0U;
  base_reset_requested = 0U;
  base_reset_tick = 0UL;
  base_action_deadline = 0UL;
  base_clean_spray_min = 0U;
  base_clear_spray_min = 0U;
  base_dry_code = 0U;
  base_action = BASE_ACTION_IDLE;
  bucket_connected = 0U;
  bucket_circulation_requested = 0U;
  level_sensor_value = 0U;
  Base_AllOutputsOff();
}

void SystemMonitor_TaskProcess(void)
{
  uint32_t now;

  now = HAL_GetTick();
  base_err1 = 0U;
  base_err2 = 0U;

  /* 当前动作到期后，自动清洁进入下一阶段；单项动作直接结束待机。 */
  if ((base_action_deadline != 0UL) && ((int32_t)(now - base_action_deadline) >= 0))
  {
    if ((base_action >= BASE_ACTION_AUTO_DRAIN1) && (base_action <= BASE_ACTION_AUTO_DRY))
    {
      Base_AdvanceAutoClean();
    }
    else
    {
      Base_FinishToStandby();
    }
  }

  /*
   * 自动清洁第 6 步：热水喷淋达到设定液位后，才请求桶体开启内循环。
   * 当前液位协议先按单字节缓存，后续接入完整液位帧时只需改 UART 层解析。
   */
  if ((base_action == BASE_ACTION_AUTO_HOT_SPRAY2) && (level_sensor_value >= BASE_WATER_READY_LEVEL))
  {
    bucket_circulation_requested = 1U;
  }

  if ((base_reset_requested != 0U) && ((now - base_reset_tick) > 200UL))
  {
    NVIC_SystemReset();
  }
}

void SystemMonitor_StopAllOutputs(void)
{
  Base_FinishToStandby();
}

void SystemMonitor_SetMainStatus(uint8_t main_status, uint8_t sub_status)
{
  base_main_status = main_status;
  base_sub_status = sub_status;
}

uint8_t SystemMonitor_GetMainStatus(void)
{
  return base_main_status;
}

uint8_t SystemMonitor_GetSubStatus(void)
{
  return base_sub_status;
}

uint8_t SystemMonitor_GetErrCode1(void)
{
  return base_err1 & 0x7FU;
}

uint8_t SystemMonitor_GetErrCode2(void)
{
  return base_err2 & 0x7FU;
}

void SystemMonitor_SetCommand(uint8_t cmd)
{
  base_last_cmd = cmd;
}

uint8_t SystemMonitor_GetCommand(void)
{
  return base_last_cmd;
}

void SystemMonitor_SetLinkStatus(uint8_t link_status)
{
  base_link_status = (link_status != 0U) ? 1U : 0U;
}

uint8_t SystemMonitor_GetLinkStatus(void)
{
  return base_link_status;
}

void SystemMonitor_SetBathTimer(uint8_t timer_code)
{
  (void)timer_code;
}

uint8_t SystemMonitor_GetTimerRemainingMin(void)
{
  uint32_t remaining_ms;

  if ((base_action_deadline == 0UL) || ((int32_t)(base_action_deadline - HAL_GetTick()) <= 0))
  {
    return 0U;
  }
  remaining_ms = base_action_deadline - HAL_GetTick();
  return (uint8_t)((remaining_ms + 59999UL) / 60000UL);
}

uint32_t SystemMonitor_GetTimerRemainingSec(void)
{
  if ((base_action_deadline == 0UL) || ((int32_t)(base_action_deadline - HAL_GetTick()) <= 0))
  {
    return 0UL;
  }
  return (base_action_deadline - HAL_GetTick() + 999UL) / 1000UL;
}

void SystemMonitor_RequestReset(void)
{
  base_reset_requested = 1U;
  base_reset_tick = HAL_GetTick();
}

uint8_t SystemMonitor_IsResetRequested(void)
{
  return base_reset_requested;
}

void SystemMonitor_ClearErrors(void)
{
  base_err1 = 0U;
  base_err2 = 0U;
}

void Base_SetBucketConnected(uint8_t connected)
{
  uint8_t was_connected;

  was_connected = bucket_connected;
  bucket_connected = (connected != 0U) ? 1U : 0U;
  base_link_status = bucket_connected;
  /*
   * 只有在需要桶体协同的阶段掉线，才立即停止流程。
   * 例如单独排水不依赖桶体，断开时仍可按本机状态继续处理。
   */
  if ((was_connected != 0U) && (bucket_connected == 0U) && (bucket_circulation_requested != 0U))
  {
    Base_AllOutputsOff();
    base_action = BASE_ACTION_IDLE;
    base_action_deadline = 0UL;
    base_main_status = BASE_STATUS_STANDBY;
    base_sub_status = BASE_SUB_IDLE;
  }
}

uint8_t Base_IsBucketConnected(void)
{
  return bucket_connected;
}

uint8_t Base_IsBucketCirculationRequested(void)
{
  return bucket_circulation_requested;
}

void Base_SetLevelSensorValue(uint8_t level)
{
  level_sensor_value = level;
}

void Base_HandleCommand(const uint8_t *frame)
{
  uint8_t cmd;

  if (frame == NULL)
  {
    return;
  }

  /*
   * 协议字段：
   *   frame[3]  = 基站命令 0xB0-0xBF
   *   frame[17] = 清洁液喷淋时间，单位分钟
   *   frame[18] = 清水/热水喷淋时间，单位分钟
   *   frame[19] = 烘干时间编码，1-9 表示 10-90 分钟
   */
  cmd = frame[3];
  base_last_cmd = cmd;
  base_clean_spray_min = frame[17];
  base_clear_spray_min = frame[18];
  base_dry_code = frame[19];

  switch (cmd)
  {
    case BASE_CMD_OFF:
      Base_AllOutputsOff();
      base_action = BASE_ACTION_IDLE;
      base_main_status = BASE_STATUS_OFF;
      base_sub_status = BASE_SUB_IDLE;
      break;

    case BASE_CMD_STANDBY:
      Base_FinishToStandby();
      break;

    case BASE_CMD_AUTO_CLEAN:
      Logging_Print("Auto clean start\r\n");
      Base_StartAutoClean();
      break;

    case BASE_CMD_FORCE_DRAIN:
      Base_ApplyDrain();
      Base_StartTimedAction(BASE_ACTION_FORCE_DRAIN, BASE_SUB_FORCE_DRAIN, BASE_DEFAULT_DRAIN_MS);
      break;

    case BASE_CMD_CLEAN_SPRAY:
      Base_ApplyHotSpray(1U);
      Base_StartTimedAction(BASE_ACTION_SINGLE_CLEAN_SPRAY, BASE_SUB_SINGLE_CLEAN_SPRAY, Base_MinToMs(base_clean_spray_min));
      break;

    case BASE_CMD_CLEAR_SPRAY:
      Base_ApplyHotSpray(0U);
      Base_StartTimedAction(BASE_ACTION_SINGLE_CLEAR_SPRAY, BASE_SUB_SINGLE_CLEAR_SPRAY, Base_MinToMs(base_clear_spray_min));
      break;

    case BASE_CMD_DRY:
      Base_ApplyDry();
      Base_StartTimedAction(BASE_ACTION_SINGLE_DRY, BASE_SUB_SINGLE_DRY, Base_DryCodeToMs(base_dry_code));
      break;

    case BASE_CMD_SELF_CHECK:
      SystemMonitor_ClearErrors();
      base_main_status = BASE_STATUS_SELF_CHECK;
      base_sub_status = BASE_SUB_IDLE;
      ColorLight_SetRgbw(0U, 50U, 0U, 0U);
      break;

    default:
      break;
  }
}

void Base_BuildStatusData(uint8_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  /*
   * 基站状态上报数据：
   *   frame[16] 上水控制状态
   *   frame[17] 清洁液喷淋时间
   *   frame[18] 清水/热水喷淋时间
   *   frame[19] 烘干时间编码
   *   frame[20] 药泵1状态
   *   frame[21] 药泵2状态
   *   frame[22] 清洁液泵状态
   *   frame[23] 主状态
   *   frame[24] 子状态
   *   frame[25] 错误码1
   *   frame[26] 错误码2
   *   frame[27] 氛围灯状态
   */
  frame[4] = bucket_connected;
  frame[16] = (HAL_GPIO_ReadPin(WATER_IN_GPIO_Port, WATER_IN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  frame[17] = base_clean_spray_min;
  frame[18] = base_clear_spray_min;
  frame[19] = base_dry_code;
  frame[20] = (HAL_GPIO_ReadPin(MED_PUMP1_GPIO_Port, MED_PUMP1_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  frame[21] = (HAL_GPIO_ReadPin(MED_PUMP2_GPIO_Port, MED_PUMP2_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  frame[22] = (HAL_GPIO_ReadPin(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  frame[23] = base_main_status;
  frame[24] = base_sub_status;
  frame[25] = SystemMonitor_GetErrCode1();
  frame[26] = SystemMonitor_GetErrCode2();

  if (base_main_status == BASE_STATUS_ERROR)
  {
    frame[27] = 0x04U;
  }
  else if (base_sub_status == BASE_SUB_DRY || base_sub_status == BASE_SUB_SINGLE_DRY)
  {
    frame[27] = 0x03U;
  }
  else if (base_sub_status != BASE_SUB_IDLE)
  {
    frame[27] = 0x02U;
  }
  else
  {
    frame[27] = 0x00U;
  }
}
