/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "color_light.h"
#include "motor_control.h"
#include "uart_comm.h"
#include "log.h"
#include "system_monitor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for BaseMotorTask */
osThreadId_t BaseMotorTaskHandle;
const osThreadAttr_t BaseMotorTask_attributes = {
  .name = "BaseMotor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BaseCommTask */
osThreadId_t BaseCommTaskHandle;
const osThreadAttr_t BaseCommTask_attributes = {
  .name = "BaseComm",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BaseLogTask */
osThreadId_t BaseLogTaskHandle;
const osThreadAttr_t BaseLogTask_attributes = {
  .name = "BaseLog",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BaseStateTask */
osThreadId_t BaseStateTaskHandle;
const osThreadAttr_t BaseStateTask_attributes = {  
  .name = "BaseState",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void BaseMotor_Task(void *argument);
void BaseComm_Task(void *argument);
void BaseLog_Task(void *argument);
void BaseState_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /*
   * 基站业务模块初始化顺序：
   *   1. 灯带和喷淋电机先初始化，确保 PWM 输出处于可控状态；
   *   2. 状态机关闭所有执行器，建立待机/上电状态；
   *   3. 日志和串口通信最后启动，便于打印后续运行状态。
   */
  ColorLight_Init();
  Motor_Init();
  SystemMonitor_Init();
  Logging_Init();
  UART_Comm_Init();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BaseMotorTask */
  BaseMotorTaskHandle = osThreadNew(BaseMotor_Task, NULL, &BaseMotorTask_attributes);

  /* creation of BaseCommTask */
  BaseCommTaskHandle = osThreadNew(BaseComm_Task, NULL, &BaseCommTask_attributes);

  /* creation of BaseLogTask */
  BaseLogTaskHandle = osThreadNew(BaseLog_Task, NULL, &BaseLogTask_attributes);

  /* creation of BaseStateTask */
  BaseStateTaskHandle = osThreadNew(BaseState_Task, NULL, &BaseStateTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_BaseMotor_Task */
/**
* @brief Function implementing the BaseMotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BaseMotor_Task */
void BaseMotor_Task(void *argument)
{
  /* USER CODE BEGIN BaseMotor_Task */
  /* 喷淋电机周期任务，预留给 PWM 刷新、故障检测和后续自动换向逻辑。 */
  for(;;)
  {
    Motor_TaskProcess();
    osDelay(20);
  }
  /* USER CODE END BaseMotor_Task */
}

/* USER CODE BEGIN Header_BaseComm_Task */
/**
* @brief Function implementing the BaseCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BaseComm_Task */
void BaseComm_Task(void *argument)
{
  /* USER CODE BEGIN BaseComm_Task */
  /* 桶体协议解析、液位接收和状态帧上报。 */
  for(;;)
  {
    UART_Comm_TaskProcess();
    osDelay(50);
  }
  /* USER CODE END BaseComm_Task */
}

/* USER CODE BEGIN Header_BaseLog_Task */
/**
* @brief Function implementing the BaseLogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BaseLog_Task */
void BaseLog_Task(void *argument)
{
  /* USER CODE BEGIN BaseLog_Task */
  /* 周期性输出基站状态日志到 USART1。 */
  for(;;)
  {
    Logging_TaskProcess();
    osDelay(1000);
  }
  /* USER CODE END BaseLog_Task */
}

/* USER CODE BEGIN Header_BaseState_Task */
/**
* @brief Function implementing the BaseStateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BaseState_Task */
void BaseState_Task(void *argument)
{
  /* USER CODE BEGIN BaseState_Task */
  /* 基站自动清洁/排水/喷淋/烘干状态机。 */
  for(;;)
  {
    SystemMonitor_TaskProcess();
    osDelay(50);
  }
  /* USER CODE END BaseState_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
/* USER CODE END Application */
