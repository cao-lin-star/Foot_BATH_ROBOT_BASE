#ifndef NTC_SENSOR_H
#define NTC_SENSOR_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NTC_SENSOR_CHANNEL_COUNT 2U
#define NTC_SENSOR_TEMP_INVALID_X10 (-1000)

typedef enum
{
  NTC_SENSOR_INLET = 0,
  NTC_SENSOR_OUTLET = 1
} NtcSensorChannel_t;

typedef struct
{
  uint16_t raw[NTC_SENSOR_CHANNEL_COUNT];
  uint16_t millivolt[NTC_SENSOR_CHANNEL_COUNT];
  int16_t temperature_x10[NTC_SENSOR_CHANNEL_COUNT];
  uint8_t valid[NTC_SENSOR_CHANNEL_COUNT];
} NtcSensorSnapshot_t;

/* 校准 ADC1 并启动循环 DMA 扫描；NTC 分压电路的供电由基站主状态统一管理。 */
void NTC_Sensor_Init(void);

/* 关机和待机时关闭分压电路，其余状态保持使能，并按 20 ms 周期执行双级滤波。 */
void NTC_Sensor_TaskProcess(void);

/* 原子复制最近一次诊断快照；当前温度数据只写入日志，不参与基站业务控制。 */
void NTC_Sensor_GetSnapshot(NtcSensorSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* NTC_SENSOR_H */
