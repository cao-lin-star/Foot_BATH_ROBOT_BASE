#include "ntc_sensor.h"
#include "adc.h"
#include "main.h"
#include "system_monitor.h"
#include <math.h>
#include <string.h>

/*
 * NTC 参数与桶体 Foot_bath_robot_V1 当前实现保持一致。
 * 两级一阶低通滤波分别作用于 ADC 原始值和换算后的温度，可降低水泵、加热器
 * 等负载切换引入的瞬态干扰；滤波除数越大，输出越平稳，但响应速度也越慢。
 */
#define NTC_FILTER_DIV          8U          /* ADC 原始采样值低通滤波除数。 */
#define NTC_TEMP_FILTER_DIV     16U         /* 温度值二级低通滤波除数。 */
#define NTC_R0_OHMS             10000.0f    /* NTC 在 25 ℃时的标称阻值：10 kΩ。 */
#define NTC_BETA                3950.0f     /* NTC 的 B 值，用于 Beta 方程换算温度。 */
#define NTC_T0_K                298.15f     /* 标称温度 25 ℃对应的开尔文温度。 */
#define NTC_FIXED_OHMS          100000.0f   /* 分压电路固定电阻阻值：100 kΩ。 */
#define NTC_ADC_VREF_MV         3300U       /* ADC 参考电压：3.3 V。 */

/* DMA 数组顺序必须与 MX_ADC1_Init() 的通道顺序一致：先记录进水口，再记录出水口。 */
static volatile uint16_t ntc_adc_dma[NTC_SENSOR_CHANNEL_COUNT];
static uint32_t ntc_raw_filter_acc[NTC_SENSOR_CHANNEL_COUNT];
static float ntc_temp_filter_c[NTC_SENSOR_CHANNEL_COUNT];
static uint8_t ntc_temp_filter_ready[NTC_SENSOR_CHANNEL_COUNT];
static uint8_t ntc_raw_filter_ready;
static uint8_t ntc_sampling_active;
static uint8_t ntc_power_enabled;
static NtcSensorSnapshot_t ntc_snapshot;

static void NTC_ResetMeasurements(void)
{
  uint8_t index;

  memset(ntc_raw_filter_acc, 0, sizeof(ntc_raw_filter_acc));
  memset(ntc_temp_filter_c, 0, sizeof(ntc_temp_filter_c));
  memset(ntc_temp_filter_ready, 0, sizeof(ntc_temp_filter_ready));
  memset(&ntc_snapshot, 0, sizeof(ntc_snapshot));
  ntc_raw_filter_ready = 0U;

  for (index = 0U; index < NTC_SENSOR_CHANNEL_COUNT; index++)
  {
    ntc_snapshot.temperature_x10[index] = NTC_SENSOR_TEMP_INVALID_X10;
  }
}

static void NTC_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint16_t NTC_RawToMv(uint16_t raw)
{
  uint32_t millivolt;

  millivolt = ((uint32_t)raw * NTC_ADC_VREF_MV + 2047U) / 4095U;
  return (uint16_t)millivolt;
}

static float NTC_CalcTemperature(uint16_t raw)
{
  float ratio;
  float ntc_ohms;
  float inverse_kelvin;

  if ((raw <= 5U) || (raw >= 4090U))
  {
    return -100.0f;
  }

  /* 使用与桶体 V1 相同的 Beta 方程，先由分压值计算 NTC 电阻，再换算摄氏温度。 */
  ratio = (float)raw / 4095.0f;
  ntc_ohms = NTC_FIXED_OHMS * (1.0f - ratio) / ratio;
  if (ntc_ohms <= 1.0f)
  {
    return -100.0f;
  }

  inverse_kelvin = (1.0f / NTC_T0_K) +
                   (logf(ntc_ohms / NTC_R0_OHMS) / NTC_BETA);
  if (inverse_kelvin <= 0.0f)
  {
    return -100.0f;
  }

  return (1.0f / inverse_kelvin) - 273.15f;
}

static void NTC_ProcessTemperature(uint8_t index, float temperature_c)
{
  if ((temperature_c > 0.0f) && (temperature_c < 85.0f))
  {
    if (ntc_temp_filter_ready[index] == 0U)
    {
      ntc_temp_filter_c[index] = temperature_c;
      ntc_temp_filter_ready[index] = 1U;
    }
    else
    {
      ntc_temp_filter_c[index] +=
        (temperature_c - ntc_temp_filter_c[index]) / (float)NTC_TEMP_FILTER_DIV;
    }

    ntc_snapshot.temperature_x10[index] =
      (int16_t)(ntc_temp_filter_c[index] * 10.0f);
    ntc_snapshot.valid[index] = 1U;
  }
  else
  {
    ntc_temp_filter_c[index] = -100.0f;
    ntc_temp_filter_ready[index] = 0U;
    ntc_snapshot.temperature_x10[index] = NTC_SENSOR_TEMP_INVALID_X10;
    ntc_snapshot.valid[index] = 0U;
  }
}

void NTC_Sensor_Init(void)
{
  memset((void *)ntc_adc_dma, 0, sizeof(ntc_adc_dma));
  NTC_ResetMeasurements();
  ntc_sampling_active = 0U;
  ntc_power_enabled = 0U;

  /* POWER_ON is applied by the periodic task after the state machine is ready. */
  HAL_GPIO_WritePin(INLET_NTC_EN_GPIO_Port, INLET_NTC_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUTLET_NTC_EN_GPIO_Port, OUTLET_NTC_EN_Pin, GPIO_PIN_RESET);

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    HAL_GPIO_WritePin(INLET_NTC_EN_GPIO_Port, INLET_NTC_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUTLET_NTC_EN_GPIO_Port, OUTLET_NTC_EN_Pin, GPIO_PIN_RESET);
    return;
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ntc_adc_dma,
                        NTC_SENSOR_CHANNEL_COUNT) != HAL_OK)
  {
    HAL_GPIO_WritePin(INLET_NTC_EN_GPIO_Port, INLET_NTC_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUTLET_NTC_EN_GPIO_Port, OUTLET_NTC_EN_Pin, GPIO_PIN_RESET);
    return;
  }

  /* Circular DMA keeps the latest pair; unused high-rate HT/TC notifications are disabled. */
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC);
  ntc_sampling_active = 1U;
}

void NTC_Sensor_TaskProcess(void)
{
  uint8_t index;
  uint8_t should_enable;
  uint8_t main_status;
  uint16_t raw;
  float temperature_c;

  main_status = SystemMonitor_GetMainStatus();
  should_enable = ((ntc_sampling_active != 0U) &&
                   (main_status != BASE_STATUS_OFF) &&
                   (main_status != BASE_STATUS_STANDBY)) ? 1U : 0U;

  if (should_enable != ntc_power_enabled)
  {
    HAL_GPIO_WritePin(INLET_NTC_EN_GPIO_Port, INLET_NTC_EN_Pin,
                      (should_enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUTLET_NTC_EN_GPIO_Port, OUTLET_NTC_EN_Pin,
                      (should_enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    ntc_power_enabled = should_enable;
    NTC_ResetMeasurements();

    /* 分压电路上电后等待一个 20 ms 任务周期，避开使能瞬间尚未稳定的 ADC 数据。 */
    return;
  }

  /* NTC 当前只用于诊断日志；即使 ADC 启动失败，也不能阻塞原有基站业务逻辑。 */
  if ((ntc_sampling_active == 0U) || (ntc_power_enabled == 0U))
  {
    return;
  }

  for (index = 0U; index < NTC_SENSOR_CHANNEL_COUNT; index++)
  {
    raw = ntc_adc_dma[index];
    if (ntc_raw_filter_ready == 0U)
    {
      ntc_raw_filter_acc[index] = (uint32_t)raw * NTC_FILTER_DIV;
    }
    else
    {
      ntc_raw_filter_acc[index] = ntc_raw_filter_acc[index] -
                                  (ntc_raw_filter_acc[index] / NTC_FILTER_DIV) +
                                  raw;
    }

    ntc_snapshot.raw[index] =
      (uint16_t)(ntc_raw_filter_acc[index] / NTC_FILTER_DIV);
    ntc_snapshot.millivolt[index] = NTC_RawToMv(ntc_snapshot.raw[index]);

    temperature_c = NTC_CalcTemperature(ntc_snapshot.raw[index]);
    NTC_ProcessTemperature(index, temperature_c);
  }

  ntc_raw_filter_ready = 1U;
}

void NTC_Sensor_GetSnapshot(NtcSensorSnapshot_t *snapshot)
{
  uint32_t primask;

  if (snapshot == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *snapshot = ntc_snapshot;
  NTC_RestoreIrq(primask);
}
