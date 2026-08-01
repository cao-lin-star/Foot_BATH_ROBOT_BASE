#include "log.h"
#include "color_light.h"
#include "main.h"
#include "motor_control.h"
#include "ntc_sensor.h"
#include "system_monitor.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef LOGGING_TX_BUFFER_LEN
#define LOGGING_TX_BUFFER_LEN       512U  /* 日志软件环形缓冲区长度�?*/
#endif

#ifndef LOGGING_TX_DMA_CHUNK_LEN
#define LOGGING_TX_DMA_CHUNK_LEN    64U   /* 单次 DMA 最大发送分块长度�?*/
#endif

#ifndef LOGGING_PRINTF_BUFFER_LEN
#define LOGGING_PRINTF_BUFFER_LEN   192U  /* 单条格式化日志缓冲区；长状态日志拆成两行输出�?*/
#endif

#if defined(__CC_ARM)
#pragma import(__use_no_semihosting)
/* Keil 无半主机模式下的 FILE 占位结构�?*/
struct __FILE
{
  int handle;  /* 标准库要求的文件句柄字段，占位使用�?*/
};
FILE __stdout;  /* printf 标准输出重定向目标�?*/

/* Keil 标准库退出钩子，避免链接半主机依赖�?*/
void _sys_exit(int x)
{
  (void)x;
}
#endif

/* 最近一次日志发送状态，HAL_OK 表示成功，HAL_BUSY/HAL_ERROR 表示异常�?*/
static volatile HAL_StatusTypeDef logging_last_status = HAL_OK;
/* 软件环形缓冲区：业务任务写入日志，DMA 任务分块搬运到串口�?*/
static uint8_t logging_tx_buffer[LOGGING_TX_BUFFER_LEN];
/* DMA 专用发送缓冲区，避免发送期间环形缓冲区 head/tail 改变�?*/
static uint8_t logging_tx_dma_buffer[LOGGING_TX_DMA_CHUNK_LEN];
/* 环形缓冲区写入位置�?*/
static volatile uint16_t logging_tx_head;
/* 环形缓冲区读取位置�?*/
static volatile uint16_t logging_tx_tail;
/* DMA 发送忙标志�? 表示当前已有分块在发送�?*/
static volatile uint8_t logging_dma_busy;

/* 按进入临界区前的中断状态恢复中断�?*/
static void Logging_RestoreIrq(uint32_t primask)
{
  /* 只在进入临界区前中断为开启状态时恢复，避免破坏外层临界区�?*/
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/* 计算环形缓冲区的下一个索引�?*/
static uint16_t Logging_NextIndex(uint16_t index)
{
  index++;
  if (index >= LOGGING_TX_BUFFER_LEN)
  {
    index = 0U;
  }
  return index;
}

/* 如果当前空闲，则从环形缓冲区取一段日志并启动 DMA 发送�?*/
static void Logging_StartTxDma(void)
{
  HAL_StatusTypeDef status;  /* HAL_UART_Transmit_DMA 的返回状态�?*/
  uint32_t primask;          /* 进入临界区前的中断屏蔽状态�?*/
  uint16_t length = 0U;      /* 本次准备发送的字节数�?*/
  uint16_t next_tail;        /* 本次发送成功后 tail 应移动到的位置�?*/

  /*
   * 从环形缓冲区复制一小段�?DMA 专用缓冲区�?   * DMA 正在发送时不能直接使用环形缓冲区，因为 head/tail 会继续变化�?   */
  primask = __get_PRIMASK();
  __disable_irq();
  if ((logging_dma_busy != 0U) || (logging_tx_head == logging_tx_tail))
  {
    Logging_RestoreIrq(primask);
    return;
  }

  logging_dma_busy = 1U;
  next_tail = logging_tx_tail;
  while ((next_tail != logging_tx_head) && (length < LOGGING_TX_DMA_CHUNK_LEN))
  {
    logging_tx_dma_buffer[length] = logging_tx_buffer[next_tail];
    next_tail = Logging_NextIndex(next_tail);
    length++;
  }
  Logging_RestoreIrq(primask);

  if (length == 0U)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    logging_dma_busy = 0U;
    Logging_RestoreIrq(primask);
    return;
  }

  status = HAL_UART_Transmit_DMA(&huart1, logging_tx_dma_buffer, length);
  logging_last_status = status;

  primask = __get_PRIMASK();
  __disable_irq();
  if (status == HAL_OK)
  {
    logging_tx_tail = next_tail;
  }
  else
  {
    logging_dma_busy = 0U;
  }
  Logging_RestoreIrq(primask);
}

/* 将一段字节写入日志环形缓冲区，并尝试启动 DMA 发送�?*/
static void Logging_WriteBuffer(const uint8_t *data, uint16_t length)
{
  uint16_t next_head;  /* 写入一个字节后的候�?head�?*/
  uint16_t index;      /* data 遍历索引�?*/
  uint32_t primask;    /* 进入临界区前的中断屏蔽状态�?*/
  uint8_t overflow = 0U; /* 环形缓冲区满标志�?*/

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  /* 写环形缓冲区时短暂关中断，防�?DMA 完成回调同时移动 tail�?*/
  primask = __get_PRIMASK();
  __disable_irq();
  for (index = 0U; index < length; index++)
  {
    next_head = Logging_NextIndex(logging_tx_head);
    if (next_head == logging_tx_tail)
    {
      overflow = 1U;
      break;
    }

    logging_tx_buffer[logging_tx_head] = data[index];
    logging_tx_head = next_head;
  }
  logging_last_status = (overflow == 0U) ? HAL_OK : HAL_BUSY;
  Logging_RestoreIrq(primask);

  Logging_StartTxDma();
}

/* printf/fputc 重定向入口，把单字符写入日志缓冲区�?*/
int fputc(int ch, FILE *f)
{
  uint8_t data = (uint8_t)ch;  /* 待发送字符�?*/

  (void)f;
  Logging_WriteBuffer(&data, 1U);
  return ch;
}

/* 初始化日志环形缓冲区和发送状态�?*/
void Logging_Init(void)
{
  uint32_t primask;  /* 进入临界区前的中断屏蔽状态�?*/

  primask = __get_PRIMASK();
  __disable_irq();
  logging_tx_head = 0U;
  logging_tx_tail = 0U;
  logging_dma_busy = 0U;
  logging_last_status = HAL_OK;
  Logging_RestoreIrq(primask);

  Logging_Print("BASE LOG is ready\r\n");
}

/* 写入一个以 '\0' 结尾的字符串日志�?*/
void Logging_Print(const char *msg)
{
  size_t length;  /* 剩余待写入长度�?*/
  uint16_t chunk; /* 单次写入长度，受 uint16_t 参数限制�?*/

  if (msg == NULL)
  {
    return;
  }

  length = strlen(msg);
  while (length > 0U)
  {
    chunk = (length > 0xFFFFU) ? 0xFFFFU : (uint16_t)length;
    Logging_WriteBuffer((const uint8_t *)msg, chunk);
    msg += chunk;
    length -= chunk;
  }
}

/* 格式化并写入一�?printf 风格日志�?*/
void Logging_Printf(const char *fmt, ...)
{
  va_list args;                         /* 可变参数列表�?*/
  char buffer[LOGGING_PRINTF_BUFFER_LEN]; /* 格式化临时缓冲区�?*/
  int length;                           /* vsnprintf 返回长度�?*/
  uint8_t truncated = 0U;               /* 日志是否被截断�?*/

  if (fmt == NULL)
  {
    return;
  }

  va_start(args, fmt);
  length = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (length < 0)
  {
    logging_last_status = HAL_ERROR;
    return;
  }

  if ((size_t)length >= sizeof(buffer))
  {
    length = (int)(sizeof(buffer) - 1U);
    truncated = 1U;
  }

  Logging_WriteBuffer((const uint8_t *)buffer, (uint16_t)length);
  if (truncated != 0U)
  {
    logging_last_status = HAL_BUSY;
  }
}

/* 周期性打印基站核心状态�?*/
#if 0
void Logging_TaskProcess(void)
{
  uint8_t err1;
  uint8_t err2;

  err1 = SystemMonitor_GetErrCode1();
  err2 = SystemMonitor_GetErrCode2();

  /* 每秒打印一次核心状态，便于通过 LinuxTX1/LinuxRX1 日志口观察流程�?*/
  Logging_Printf("[BASE] CMD=%02X | LINK=%u | BST=%02X T=%u EL=%lus TS=%lus | CW=%u TW=%u WIN=%u | ERR=%02X/%02X LOW=%u/%u/%u LB=%u\r\n",
                 SystemMonitor_GetCommand(),
                 SystemMonitor_GetLinkStatus(),
                 SystemMonitor_GetMainStatus(),
                 SystemMonitor_GetSubStatus(),
                 SystemMonitor_GetStatusElapsedSec(),
                 SystemMonitor_GetTimerRemainingSec(),
                 SystemMonitor_GetBucketCurrentWater(),
                 SystemMonitor_GetAutoFillTargetWater(),
                 (HAL_GPIO_ReadPin(WATER_IN_GPIO_Port, WATER_IN_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 err1,
                 err2,
                 ((err1 & BASE_ERR1_MED_PUMP1_LOW) != 0U) ? 1U : 0U,
                 ((err1 & BASE_ERR1_MED_PUMP2_LOW) != 0U) ? 1U : 0U,
                 ((err1 & BASE_ERR1_CLEAN_LOW) != 0U) ? 1U : 0U,
                 ((err1 & BASE_ERR1_LEVEL_BOARD) != 0U) ? 1U : 0U);
}

/* 返回最近一次日志发送状态�?*/
#endif

void Logging_TaskProcess(void)
{
  uint8_t err1;
  uint8_t err2;
  uint8_t light_r;
  uint8_t light_g;
  uint8_t light_b;
  uint8_t light_w;
  NtcSensorSnapshot_t ntc;
  int16_t inlet_temp_x10;
  int16_t outlet_temp_x10;
  uint16_t inlet_temp_abs;
  uint16_t outlet_temp_abs;
  char inlet_temp_sign;
  char outlet_temp_sign;

  err1 = SystemMonitor_GetErrCode1();
  err2 = SystemMonitor_GetErrCode2();
  ColorLight_GetRgbw(&light_r, &light_g, &light_b, &light_w);
  NTC_Sensor_GetSnapshot(&ntc);

  inlet_temp_x10 = ntc.temperature_x10[NTC_SENSOR_INLET];
  inlet_temp_sign = '+';
  if (inlet_temp_x10 < 0)
  {
    inlet_temp_sign = '-';
    inlet_temp_x10 = (int16_t)(-inlet_temp_x10);
  }
  inlet_temp_abs = (uint16_t)inlet_temp_x10;

  outlet_temp_x10 = ntc.temperature_x10[NTC_SENSOR_OUTLET];
  outlet_temp_sign = '+';
  if (outlet_temp_x10 < 0)
  {
    outlet_temp_sign = '-';
    outlet_temp_x10 = (int16_t)(-outlet_temp_x10);
  }
  outlet_temp_abs = (uint16_t)outlet_temp_x10;

  Logging_Printf("[BASE] CMD=%02X | LINK=%u | BST=%02X T=%u EL=%lus TS=%lus | CW=%u TW=%u WIN=%u | ERR=%02X/%02X LOW=%u/%u/%u LB=%u\r\n",
                 SystemMonitor_GetCommand(),
                 SystemMonitor_GetLinkStatus(),
                 SystemMonitor_GetMainStatus(),
                 SystemMonitor_GetSubStatus(),
                 SystemMonitor_GetStatusElapsedSec(),
                 SystemMonitor_GetTimerRemainingSec(),
                 SystemMonitor_GetBucketCurrentWater(),
                 SystemMonitor_GetAutoFillTargetWater(),
                 (HAL_GPIO_ReadPin(WATER_IN_GPIO_Port, WATER_IN_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 err1,
                 err2,
                 ((err1 & BASE_ERR1_MED_PUMP1_LOW) != 0U) ? 1U : 0U,
                 ((err1 & BASE_ERR1_MED_PUMP2_LOW) != 0U) ? 1U : 0U,
                 ((err1 & BASE_ERR1_CLEAN_LOW) != 0U) ? 1U : 0U,
                 ((err1 & BASE_ERR1_LEVEL_BOARD) != 0U) ? 1U : 0U);

  Logging_Printf("[ OUT] WIN=%u WOUT=%u | MED1=%u MED2=%u CLEAN=%u | HEAT=%u FAN=%u IR=%u | "
                 "MOT POS=%u BUSY=%u F=%u | LED=%u/%u/%u/%u\r\n",
                 (HAL_GPIO_ReadPin(WATER_IN_GPIO_Port, WATER_IN_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 (HAL_GPIO_ReadPin(WATER_OUT_GPIO_Port, WATER_OUT_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 (HAL_GPIO_ReadPin(MED_PUMP1_GPIO_Port, MED_PUMP1_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 (HAL_GPIO_ReadPin(MED_PUMP2_GPIO_Port, MED_PUMP2_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 (HAL_GPIO_ReadPin(CLEAN_PUMP_GPIO_Port, CLEAN_PUMP_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 (HAL_GPIO_ReadPin(EN_HEAT_GPIO_Port, EN_HEAT_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 (HAL_GPIO_ReadPin(EN_FAN_GPIO_Port, EN_FAN_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 (HAL_GPIO_ReadPin(EN_IR_GPIO_Port, EN_IR_Pin) == GPIO_PIN_SET) ? 1U : 0U,
                 (uint8_t)Motor_GetPosition(),
                 Motor_IsBusy(),
                 Motor_HasFault(),
                 light_r,
                 light_g,
                 light_b,
                 light_w);

  Logging_Printf("[ NTC] IN=%c%u.%uC OK=%u RAW=%u MV=%u | OUT=%c%u.%uC OK=%u RAW=%u MV=%u\r\n",
                 inlet_temp_sign,
                 inlet_temp_abs / 10U,
                 inlet_temp_abs % 10U,
                 ntc.valid[NTC_SENSOR_INLET],
                 ntc.raw[NTC_SENSOR_INLET],
                 ntc.millivolt[NTC_SENSOR_INLET],
                 outlet_temp_sign,
                 outlet_temp_abs / 10U,
                 outlet_temp_abs % 10U,
                 ntc.valid[NTC_SENSOR_OUTLET],
                 ntc.raw[NTC_SENSOR_OUTLET],
                 ntc.millivolt[NTC_SENSOR_OUTLET]);
}

HAL_StatusTypeDef Logging_GetLastStatus(void)
{
  return logging_last_status;
}

/* USART1 TX DMA 发送完成后继续发送下一段日志�?*/
void Logging_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;  /* 进入临界区前的中断屏蔽状态�?*/

  if (huart != &huart1)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  logging_dma_busy = 0U;
  Logging_RestoreIrq(primask);

  Logging_StartTxDma();
}

/* USART1 �?DMA 发送错误后恢复状态并尝试继续发送�?*/
void Logging_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;  /* 进入临界区前的中断屏蔽状态�?*/

  if (huart != &huart1)
  {
    return;
  }

  logging_last_status = HAL_ERROR;
  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);
  if (huart->hdmatx != NULL)
  {
    (void)HAL_DMA_Abort(huart->hdmatx);
  }
  huart->gState = HAL_UART_STATE_READY;

  primask = __get_PRIMASK();
  __disable_irq();
  logging_dma_busy = 0U;
  Logging_RestoreIrq(primask);

  Logging_StartTxDma();
}
