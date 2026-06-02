#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 基站错误码定义。
 * err1 偏向药液/液位/泵类错误，err2 偏向执行器错误。
 * 当前第一版先保留位定义，具体检测输入后续硬件联调时补充。
 */
#define BASE_ERR1_MED_PUMP1_LOW      0x01U
#define BASE_ERR1_MED_PUMP2_LOW      0x02U
#define BASE_ERR1_CLEAN_LOW          0x04U
#define BASE_ERR1_LEVEL_BOARD        0x08U
#define BASE_ERR1_MED_PUMP1          0x10U
#define BASE_ERR1_MED_PUMP2          0x20U
#define BASE_ERR1_CLEAN_PUMP         0x40U

#define BASE_ERR2_WATER_IN           0x01U
#define BASE_ERR2_HEAT               0x02U
#define BASE_ERR2_SPRAY_MOTOR        0x04U
#define BASE_ERR2_DRAIN              0x08U
#define BASE_ERR2_DRY                0x10U
#define BASE_ERR2_LIGHT              0x20U
#define BASE_ERR2_IR                 0x40U

typedef enum
{
  /* 上电初始化完成但还未进入业务状态。 */
  BASE_STATUS_POWER_ON = 0,
  /* 基站自检命令执行中。 */
  BASE_STATUS_SELF_CHECK = 1,
  /* 关机状态，所有输出关闭。 */
  BASE_STATUS_OFF = 2,
  /* 待机状态，等待桶体命令。 */
  BASE_STATUS_STANDBY = 3,
  /* 自动清洁/单项动作/排水等正在执行。 */
  BASE_STATUS_RUNNING = 4,
  /* 错误状态，错误码通过 data[25]/data[26] 上报。 */
  BASE_STATUS_ERROR = 5
} BaseMainStatus_t;

typedef enum
{
  /* 无动作。 */
  BASE_SUB_IDLE = 0,
  /* 保留状态：等待排水或进水条件。 */
  BASE_SUB_WATER_IN_WAIT_DRAIN = 1,
  /* 自动清洁中的清洁液喷淋阶段。 */
  BASE_SUB_CLEAN_SPRAY = 2,
  /* 自动清洁第一次排水。 */
  BASE_SUB_CLEAN_DRAIN1 = 3,
  /* 清水/热水喷淋阶段。 */
  BASE_SUB_CLEAR_SPRAY = 4,
  /* 自动清洁第二次排水。 */
  BASE_SUB_CLEAN_DRAIN2 = 5,
  /* 烘干阶段。 */
  BASE_SUB_DRY = 6,
  /* 强制排水命令。 */
  BASE_SUB_FORCE_DRAIN = 7,
  /* 单独清洁液喷淋命令。 */
  BASE_SUB_SINGLE_CLEAN_SPRAY = 8,
  /* 单独清水/热水喷淋命令。 */
  BASE_SUB_SINGLE_CLEAR_SPRAY = 9,
  /* 单独热风烘干命令。 */
  BASE_SUB_SINGLE_DRY = 10
} BaseSubStatus_t;

/* 初始化基站状态机、错误码和所有硬件输出。 */
void SystemMonitor_Init(void);

/* 状态机周期处理：动作超时推进、液位触发内循环、软复位延时执行。 */
void SystemMonitor_TaskProcess(void);

/* 停止所有输出并回到待机状态。 */
void SystemMonitor_StopAllOutputs(void);

/* 直接设置主/子状态，保留给调试或特殊业务流程使用。 */
void SystemMonitor_SetMainStatus(uint8_t main_status, uint8_t sub_status);
uint8_t SystemMonitor_GetMainStatus(void);
uint8_t SystemMonitor_GetSubStatus(void);
uint8_t SystemMonitor_GetErrCode1(void);
uint8_t SystemMonitor_GetErrCode2(void);
void SystemMonitor_SetCommand(uint8_t cmd);
uint8_t SystemMonitor_GetCommand(void);
void SystemMonitor_SetLinkStatus(uint8_t link_status);
uint8_t SystemMonitor_GetLinkStatus(void);
void SystemMonitor_SetBathTimer(uint8_t timer_code);
uint8_t SystemMonitor_GetTimerRemainingMin(void);
uint32_t SystemMonitor_GetTimerRemainingSec(void);
void SystemMonitor_RequestReset(void);
uint8_t SystemMonitor_IsResetRequested(void);
void SystemMonitor_ClearErrors(void);

/* 设置桶体连接状态；若协同阶段掉线，会停止输出并回待机。 */
void Base_SetBucketConnected(uint8_t connected);
uint8_t Base_IsBucketConnected(void);

/* 处理桶体下发的 0xB0-0xBF 基站命令帧。 */
void Base_HandleCommand(const uint8_t *frame);

/* 按协议填充基站状态上报数据，主要写入 frame[16] 到 frame[27]。 */
void Base_BuildStatusData(uint8_t *frame);

/* 自动清洁需要桶体水泵内循环时返回 1，由通信层写入状态帧。 */
uint8_t Base_IsBucketCirculationRequested(void);

/* 更新液位传感器状态；当前先按单字节缓存，后续可替换为完整解析。 */
void Base_SetLevelSensorValue(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif
