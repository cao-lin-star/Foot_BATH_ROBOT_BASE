/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration of all used GPIO pins.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "gpio.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 基站使用 GPIOA/GPIOB 输出，同时开启 AFIO 以支持 PB3/PB4/JTAG 重映射后的 GPIO。 */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /*
   * 上电默认关闭所有执行器。
   * 这样在 FreeRTOS 和业务状态机启动前，不会误开加热、风机、水阀或药泵。
   */
  HAL_GPIO_WritePin(GPIOA, DRY_FAN_Pin|EN_HEAT_Pin|RESERVED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, EN_IR_Pin|WATER_OUT_Pin|WATER_IN_Pin|MED_PUMP1_Pin|
                          MED_PUMP2_Pin|CLEAN_PUMP_Pin|SPARE_SW_Pin, GPIO_PIN_RESET);

  /* GPIOA 输出：烘干风机、加热、PA15 预留。 */
  GPIO_InitStruct.Pin = DRY_FAN_Pin|EN_HEAT_Pin|RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIOB 输出：红外引导灯、水阀、药泵、清洁液泵和备用开关。 */
  GPIO_InitStruct.Pin = EN_IR_Pin|WATER_OUT_Pin|WATER_IN_Pin|MED_PUMP1_Pin|
                        MED_PUMP2_Pin|CLEAN_PUMP_Pin|SPARE_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
