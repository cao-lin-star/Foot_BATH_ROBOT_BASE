#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 基站错误码定义�? * err1 偏向药液、液位、泵类错误，err2 偏向执行器错误�? * 当前版本先保留位定义，具体检测输入后续硬件联调时补充�? */
#define BASE_ERR1_MED_PUMP1_LOW      0x01U  /* 药液�?1 液位低�?*/
#define BASE_ERR1_MED_PUMP2_LOW      0x02U  /* 药液�?2 液位低�?*/
#define BASE_ERR1_CLEAN_LOW          0x04U  /* 清洁液液位低�?*/
#define BASE_ERR1_LEVEL_BOARD        0x08U  /* 液位板通信或检测异常�?*/
#define BASE_ERR1_MED_PUMP1          0x10U  /* 药液�?1 故障�?*/
#define BASE_ERR1_MED_PUMP2          0x20U  /* 药液�?2 故障�?*/
#define BASE_ERR1_CLEAN_PUMP         0x40U  /* 清洁泵故障�?*/

#define BASE_ERR2_WATER_IN           0x01U  /* 上水阀或进水通道故障�?*/
#define BASE_ERR2_HEAT               0x02U  /* 加热输出故障�?*/
#define BASE_ERR2_SPRAY_MOTOR        0x04U  /* 喷淋电机故障�?*/
#define BASE_ERR2_DRAIN              0x08U  /* 排水输出故障�?*/
#define BASE_ERR2_DRY                0x10U  /* 烘干风机故障�?*/
#define BASE_ERR2_LIGHT              0x20U  /* 氛围灯故障�?*/
#define BASE_ERR2_IR                 0x40U  /* 红外输出故障�?*/

/* Base protocol work status, reported in status frame data[23]. */
typedef enum
{
  BASE_STATUS_POWER_ON = 0x00U,
  BASE_STATUS_SELF_CHECK = 0x01U,
  BASE_STATUS_OFF = 0x02U,
  BASE_STATUS_STANDBY = 0x03U,
  BASE_STATUS_AUTO_FILL = 0x04U,
  BASE_STATUS_FILL_KEEP_WARM = 0x05U,
  BASE_STATUS_FILL_WAIT_DRAIN = 0x06U,
  BASE_STATUS_CLEAN_SPRAY = 0x07U,
  BASE_STATUS_CLEAN_DRAIN1 = 0x08U,
  BASE_STATUS_CLEAR_SPRAY = 0x09U,
  BASE_STATUS_CLEAR_DRAIN2 = 0x0AU,
  BASE_STATUS_CLEAN_DRY = 0x0BU,
  BASE_STATUS_FORCE_DRAIN = 0x0CU,
  BASE_STATUS_SINGLE_CLEAN_SPRAY = 0x0DU,
  BASE_STATUS_SINGLE_CLEAR_SPRAY = 0x0EU,
  BASE_STATUS_SINGLE_DRY = 0x0FU
} BaseMainStatus_t;
/* 初始化基站状态机、错误码和所有硬件输出�?*/
void SystemMonitor_Init(void);

/* 状态机周期处理：动作超时推进、液位触发内循环、软复位延时执行�?*/
void SystemMonitor_TaskProcess(void);

/* 停止所有输出并回到待机状态�?*/
void SystemMonitor_StopAllOutputs(void);

/* 直接设置�?子状态，保留给调试或特殊业务流程使用�?*/
void SystemMonitor_SetMainStatus(uint8_t main_status, uint8_t time_value);

/* 获取当前主状态�?*/
uint8_t SystemMonitor_GetMainStatus(void);

/* Return status frame data[24]: elapsed seconds or remaining minutes. */
uint8_t SystemMonitor_GetSubStatus(void);

/* Return full elapsed seconds since entering the current main status. */
uint32_t SystemMonitor_GetStatusElapsedSec(void);

/* 获取错误�?1，返回值只保留�?7 位�?*/
uint8_t SystemMonitor_GetErrCode1(void);

/* 获取错误�?2，返回值只保留�?7 位�?*/
uint8_t SystemMonitor_GetErrCode2(void);

/* 记录最近一次基站命令字�?*/
void SystemMonitor_SetCommand(uint8_t cmd);

/* 获取最近一次基站命令字�?*/
uint8_t SystemMonitor_GetCommand(void);

/* 设置链路状态，�?0 表示在线�?*/
void SystemMonitor_SetLinkStatus(uint8_t link_status);

/* 获取链路状态，1 表示在线�? 表示离线�?*/
uint8_t SystemMonitor_GetLinkStatus(void);

/* 设置泡脚定时编码，当前版本预留�?*/
void SystemMonitor_SetBathTimer(uint8_t timer_code);

/* 获取当前动作剩余分钟数，向上取整�?*/
uint8_t SystemMonitor_GetTimerRemainingMin(void);

/* 获取当前动作剩余秒数，向上取整�?*/
uint32_t SystemMonitor_GetTimerRemainingSec(void);
uint8_t SystemMonitor_GetMedicine1RemainingSec(void);
uint8_t SystemMonitor_GetMedicine2RemainingSec(void);
uint8_t SystemMonitor_GetBucketCurrentWater(void);
uint8_t SystemMonitor_GetAutoFillTargetWater(void);

/* 请求软件复位，状态机将在短延时后调用 NVIC_SystemReset�?*/
void SystemMonitor_RequestReset(void);

/* 查询是否已经请求软件复位�?*/
uint8_t SystemMonitor_IsResetRequested(void);

/* 清除当前错误码�?*/
void SystemMonitor_ClearErrors(void);

/* 设置桶体连接状态；若协同阶段掉线，会停止输出并回待机�?*/
void Base_SetBucketConnected(uint8_t connected);

/* 查询桶体是否在线�?*/
uint8_t Base_IsBucketConnected(void);

/* 处理桶体下发�?0xB0-0xBF 基站命令帧�?*/
void Base_HandleCommand(const uint8_t *frame);

/* Update bucket realtime water/temp from data[3] == 0x00 frame. */
void Base_UpdateBucketRealtimeData(const uint8_t *frame);

/* 按协议填充基站状态上报数据，主要写入 frame[16] �?frame[27]�?*/
void Base_BuildStatusData(uint8_t *frame);

/* 自动清洁需要桶体水泵内循环时返�?1，由通信层写入状态帧�?*/
uint8_t Base_IsBucketCirculationRequested(void);

/* 更新缺液控制板上报字节：01010xxx，低 3 位中 1 表示有液�? 表示缺液�?*/
void Base_SetLevelSensorValue(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif
