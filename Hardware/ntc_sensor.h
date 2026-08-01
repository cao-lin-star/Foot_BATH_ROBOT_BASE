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

/* Calibrate ADC1 and start circular DMA scans; divider power follows main state. */
void NTC_Sensor_Init(void);

/* Gate divider power in OFF/STANDBY and apply the V1 20ms filters otherwise. */
void NTC_Sensor_TaskProcess(void);

/* Copy the diagnostic snapshot; values are not used by base business logic. */
void NTC_Sensor_GetSnapshot(NtcSensorSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* NTC_SENSOR_H */
