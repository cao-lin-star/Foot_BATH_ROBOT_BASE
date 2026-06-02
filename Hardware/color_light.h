#ifndef COLOR_LIGHT_H
#define COLOR_LIGHT_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * RGBW 彩色灯带通道编号。
 * 代码层使用 R/G/B/W 的自然顺序；硬件实际 PWM 映射在 color_light.c 中完成：
 *   TIM4_CH1 -> LED_W(PB6)
 *   TIM4_CH2 -> LED_B(PB7)
 *   TIM4_CH3 -> LED_G(PB8)
 *   TIM4_CH4 -> LED_R(PB9)
 */
#define COLOR_LIGHT_CHANNEL_R    0U
#define COLOR_LIGHT_CHANNEL_G    1U
#define COLOR_LIGHT_CHANNEL_B    2U
#define COLOR_LIGHT_CHANNEL_W    3U

/* 启动 TIM4 四路 PWM，默认保持当前缓存亮度。 */
void ColorLight_Init(void);

/* 关闭 TIM4 PWM 输出，通常只在外设反初始化时使用。 */
void ColorLight_DeInit(void);

/* 四个通道亮度全部置 0。 */
void ColorLight_Off(void);

/* 设置 RGB 亮度，范围 0-100，白光通道自动置 0。 */
void ColorLight_SetRgb(uint8_t r, uint8_t g, uint8_t b);

/* 设置 RGBW 亮度，范围 0-100，超过 100 会被钳位。 */
void ColorLight_SetRgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/* 单独设置某个通道亮度，channel 使用 COLOR_LIGHT_CHANNEL_x。 */
void ColorLight_SetChannel(uint8_t channel, uint8_t value);

/* 读取当前缓存的 RGBW 亮度；不需要的输出参数可传 NULL。 */
void ColorLight_GetRgbw(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w);

#ifdef __cplusplus
}
#endif

#endif
