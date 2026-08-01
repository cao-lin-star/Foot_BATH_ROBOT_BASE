/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM configuration for the ULN2003 step tick and RGBW PWM outputs.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tim.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

static void TIM_ConfigPwmChannel(TIM_HandleTypeDef *htim, uint32_t channel)
{
  /*
   * 所有 PWM 通道统一使用 PWM1 模式，初始 Pulse=0。
   * 具体占空比由 color_light.c 在运行时设置。
   */
  TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_TIM3_Init(void)
{
  /*
   * TIM3 仅作为 ULN2003 相序节拍：72MHz / 72 / 3000 = 333.3Hz，
   * 即每 3ms 进入一次更新中断。四相输出均为普通 GPIO。
   */
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_TIM4_Init(void)
{
  /*
   * TIM4 用于 RGBW 彩色灯带：
   *   CH1 PB6 = LED_W
   *   CH2 PB7 = LED_B
   *   CH3 PB8 = LED_G
   *   CH4 PB9 = LED_R
   * Period=499，对应约 2kHz PWM，减少灯带可见闪烁。
   */
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_ConfigPwmChannel(&htim4, TIM_CHANNEL_1);
  TIM_ConfigPwmChannel(&htim4, TIM_CHANNEL_2);
  TIM_ConfigPwmChannel(&htim4, TIM_CHANNEL_3);
  TIM_ConfigPwmChannel(&htim4, TIM_CHANNEL_4);
  HAL_TIM_MspPostInit(&htim4);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
  if(tim_pwmHandle->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(timHandle->Instance==TIM4)
  {
    /* 彩色灯带 RGBW 四路 PWM 输出。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = LED_W_Pin|LED_B_Pin|LED_G_Pin|LED_R_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{
  if(tim_pwmHandle->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM3)
  {
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
}
