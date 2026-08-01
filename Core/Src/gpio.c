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
  HAL_GPIO_WritePin(GPIOA, MOTOR_IN4_Pin|MOTOR_IN3_Pin|INLET_NTC_EN_Pin|
                           EN_HEAT_Pin|EN_FAN_Pin|RESERVED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN2_Pin|MOTOR_IN1_Pin|EN_IR_Pin|WATER_OUT_Pin|
                           WATER_IN_Pin|MED_PUMP1_Pin|MED_PUMP2_Pin|CLEAN_PUMP_Pin|
                           OUTLET_NTC_EN_Pin, GPIO_PIN_RESET);

  /* ULN2003 四相输入：推挽、无上下拉、低速，上电保持全低避免线圈误励磁。 */
  GPIO_InitStruct.Pin = MOTOR_IN4_Pin|MOTOR_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOTOR_IN2_Pin|MOTOR_IN1_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* GPIOA 输出：进水口 NTC 使能、加热、烘干风机和 PA15 预留。 */
  GPIO_InitStruct.Pin = INLET_NTC_EN_Pin|EN_HEAT_Pin|EN_FAN_Pin|RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIOB 输出：红外、水阀、药泵、清洁液泵和出水口 NTC 使能。 */
  GPIO_InitStruct.Pin = EN_IR_Pin|WATER_OUT_Pin|WATER_IN_Pin|MED_PUMP1_Pin|
                        MED_PUMP2_Pin|CLEAN_PUMP_Pin|OUTLET_NTC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
