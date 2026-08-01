/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/*
 * 基站 GPIO/外设引脚定义。
 * 命名优先使用原理图网络名或业务含义：
 *   LinuxRX1/LinuxTX1 -> USART1 日志串口
 *   LOG_RX1/LOG_TX1   -> USART3 桶体通信串口
 *   JZ_TX1/JZ_RX1     -> USART2 液位传感器串口
 */
#define INLET_NTC_ADC_Pin GPIO_PIN_0
#define INLET_NTC_ADC_GPIO_Port GPIOA
#define OUTLET_NTC_ADC_Pin GPIO_PIN_1
#define OUTLET_NTC_ADC_GPIO_Port GPIOA
#define JZ_TX1_Pin GPIO_PIN_2
#define JZ_TX1_GPIO_Port GPIOA
#define JZ_RX1_Pin GPIO_PIN_3
#define JZ_RX1_GPIO_Port GPIOA
#define MOTOR_IN4_Pin GPIO_PIN_5
#define MOTOR_IN4_GPIO_Port GPIOA
#define MOTOR_IN3_Pin GPIO_PIN_7
#define MOTOR_IN3_GPIO_Port GPIOA
#define INLET_NTC_EN_Pin GPIO_PIN_8
#define INLET_NTC_EN_GPIO_Port GPIOA
#define LinuxRX1_Pin GPIO_PIN_9
#define LinuxRX1_GPIO_Port GPIOA
#define LinuxTX1_Pin GPIO_PIN_10
#define LinuxTX1_GPIO_Port GPIOA
#define EN_HEAT_Pin GPIO_PIN_11
#define EN_HEAT_GPIO_Port GPIOA
#define EN_FAN_Pin GPIO_PIN_12
#define EN_FAN_GPIO_Port GPIOA
#define RESERVED_Pin GPIO_PIN_15
#define RESERVED_GPIO_Port GPIOA
#define MOTOR_IN2_Pin GPIO_PIN_0
#define MOTOR_IN2_GPIO_Port GPIOB
#define MOTOR_IN1_Pin GPIO_PIN_1
#define MOTOR_IN1_GPIO_Port GPIOB
#define EN_IR_Pin GPIO_PIN_3
#define EN_IR_GPIO_Port GPIOB
#define WATER_OUT_Pin GPIO_PIN_4
#define WATER_OUT_GPIO_Port GPIOB
#define WATER_IN_Pin GPIO_PIN_5
#define WATER_IN_GPIO_Port GPIOB
#define LED_W_Pin GPIO_PIN_6
#define LED_W_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_7
#define LED_B_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_9
#define LED_R_GPIO_Port GPIOB
#define LOG_RX1_Pin GPIO_PIN_10
#define LOG_RX1_GPIO_Port GPIOB
#define LOG_TX1_Pin GPIO_PIN_11
#define LOG_TX1_GPIO_Port GPIOB
#define MED_PUMP1_Pin GPIO_PIN_12
#define MED_PUMP1_GPIO_Port GPIOB
#define MED_PUMP2_Pin GPIO_PIN_13
#define MED_PUMP2_GPIO_Port GPIOB
#define CLEAN_PUMP_Pin GPIO_PIN_14
#define CLEAN_PUMP_GPIO_Port GPIOB
#define OUTLET_NTC_EN_Pin GPIO_PIN_15
#define OUTLET_NTC_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
