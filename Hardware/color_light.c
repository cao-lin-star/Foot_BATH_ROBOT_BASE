#include "color_light.h"
#include "tim.h"

/* 红色通道当前亮度缓存，单位为百分比 0-100。 */
static uint8_t color_light_r;
/* 绿色通道当前亮度缓存，单位为百分比 0-100。 */
static uint8_t color_light_g;
/* 蓝色通道当前亮度缓存，单位为百分比 0-100。 */
static uint8_t color_light_b;
/* 白色通道当前亮度缓存，单位为百分比 0-100。 */
static uint8_t color_light_w;
/* 灯带 PWM 是否已经初始化，避免未启动定时器时写比较值。 */
static uint8_t color_light_initialized;

/* 限制亮度范围，避免上层传入超过 100 的百分比。 */
static uint8_t ColorLight_ClampValue(uint8_t value)
{
  if (value > 100U)
  {
    return 100U;
  }
  return value;
}

/* 根据 TIM4 的 Period 将百分比亮度转换为 CCR 比较值。 */
static uint32_t ColorLight_DutyToPulse(uint8_t duty_percent)
{
  uint32_t period;  /* TIM4 自动重装载值。 */
  uint32_t pulse;   /* 换算后的 PWM 比较值。 */

  period = htim4.Init.Period;
  pulse = ((period + 1U) * (uint32_t)duty_percent) / 100U;
  if (pulse > period)
  {
    pulse = period;
  }
  return pulse;
}

/* 写入指定 TIM4 通道的 PWM 比较值。 */
static void ColorLight_SetTimerChannel(uint32_t channel, uint8_t value)
{
  __HAL_TIM_SET_COMPARE(&htim4, channel, ColorLight_DutyToPulse(value));
}

/* 将 RGBW 亮度缓存应用到实际硬件通道。 */
static void ColorLight_Apply(void)
{
  if (color_light_initialized == 0U)
  {
    return;
  }

  /*
   * 原理图通道与颜色不是 R/G/B/W 顺序：
   *   PB6 TIM4_CH1 = LED_W
   *   PB7 TIM4_CH2 = LED_B
   *   PB8 TIM4_CH3 = LED_G
   *   PB9 TIM4_CH4 = LED_R
   */
  ColorLight_SetTimerChannel(TIM_CHANNEL_1, color_light_w);
  ColorLight_SetTimerChannel(TIM_CHANNEL_2, color_light_b);
  ColorLight_SetTimerChannel(TIM_CHANNEL_3, color_light_g);
  ColorLight_SetTimerChannel(TIM_CHANNEL_4, color_light_r);
}

/* 初始化 RGBW 灯带 PWM 输出。 */
void ColorLight_Init(void)
{
  /* 亮度缓存默认是 0，因此初始化后灯带保持关闭。 */
  color_light_initialized = 1U;

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  ColorLight_Apply();
}

/* 关闭 RGBW 灯带 PWM 输出。 */
void ColorLight_DeInit(void)
{
  ColorLight_Off();

  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
  color_light_initialized = 0U;
}

/* 关闭全部灯光通道。 */
void ColorLight_Off(void)
{
  ColorLight_SetRgbw(0U, 0U, 0U, 0U);
}

/* 设置 RGB 三色亮度，并关闭白光通道。 */
void ColorLight_SetRgb(uint8_t r, uint8_t g, uint8_t b)
{
  ColorLight_SetRgbw(r, g, b, 0U);
}

/* 设置 RGBW 四通道亮度并立即刷新输出。 */
void ColorLight_SetRgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
  color_light_r = ColorLight_ClampValue(r);
  color_light_g = ColorLight_ClampValue(g);
  color_light_b = ColorLight_ClampValue(b);
  color_light_w = ColorLight_ClampValue(w);
  ColorLight_Apply();
}

/* 设置单个颜色通道的亮度。 */
void ColorLight_SetChannel(uint8_t channel, uint8_t value)
{
  value = ColorLight_ClampValue(value);

  switch (channel)
  {
    case COLOR_LIGHT_CHANNEL_R:
      color_light_r = value;
      break;

    case COLOR_LIGHT_CHANNEL_G:
      color_light_g = value;
      break;

    case COLOR_LIGHT_CHANNEL_B:
      color_light_b = value;
      break;

    case COLOR_LIGHT_CHANNEL_W:
      color_light_w = value;
      break;

    default:
      return;
  }

  ColorLight_Apply();
}

/* 读取当前 RGBW 亮度缓存。 */
void ColorLight_GetRgbw(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w)
{
  if (r != NULL)
  {
    *r = color_light_r;
  }
  if (g != NULL)
  {
    *g = color_light_g;
  }
  if (b != NULL)
  {
    *b = color_light_b;
  }
  if (w != NULL)
  {
    *w = color_light_w;
  }
}
