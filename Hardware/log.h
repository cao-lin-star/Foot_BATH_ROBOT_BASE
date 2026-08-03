#ifndef LOGGING_H
#define LOGGING_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 基站日志模块。
 *
 * 硬件端口：
 *   USART1 PA9/PA10，对应原理图 LinuxRX1/LinuxTX1。
 *
 * 发送方式：
 *   日志先写入软件环形缓冲区，再由 USART1 TX DMA 分块发送。
 *   这样业务任务调用 Logging_Printf() 时不会长时间阻塞。
 */

/* 初始化日志缓冲区，并输出启动提示。 */
void Logging_Init(void);

/* 发送普通字符串，字符串必须以 '\0' 结尾。 */
void Logging_Print(const char *msg);

/* printf 风格日志输出，内部有固定格式化缓冲区，过长内容会被截断。 */
void Logging_Printf(const char *fmt, ...);

/* 周期性打印基站运行状态，由 BaseLog FreeRTOS 任务调用。 */
void Logging_TaskProcess(void);

/* USART TX DMA 完成回调，继续发送环形缓冲区内的剩余日志。 */
void Logging_TxCpltCallback(UART_HandleTypeDef *huart);

/* USART/DMA 错误回调，尝试恢复 DMA 日志发送状态。 */
void Logging_ErrorCallback(UART_HandleTypeDef *huart);

/* 获取最近一次日志发送状态，便于调试串口或 DMA 异常。 */
HAL_StatusTypeDef Logging_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif
