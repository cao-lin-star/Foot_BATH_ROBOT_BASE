#ifndef UART_COMM_H
#define UART_COMM_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 统一串口通信协议，固定 30 字节一帧：
 *   byte[0]     = 0x55，帧头高字节。
 *   byte[1]     = 0xAA，帧头低字节。
 *   byte[2]     = 链路模式。
 *   byte[3]     = 命令字。
 *   byte[4..28] = 数据区。
 *   byte[29]    = 校验和，累加 byte[2] 到 byte[28] 后取低 8 位。
 */
#define UART_COMM_FRAME_LEN          30U    /* 通信协议固定帧长。 */
#define UART_COMM_HEAD1              0x55U  /* 帧头第 1 字节。 */
#define UART_COMM_HEAD2              0xAAU  /* 帧头第 2 字节。 */

/* 协议链路模式。基站默认按桶体链路处理，同时保留透传/直连兼容值。 */
#define UART_COMM_LINK_MODE_BUCKET   0x01U  /* 桶体链路模式。 */
#define UART_COMM_LINK_MODE_TRANSIT  0x02U  /* 透传链路模式。 */
#define UART_COMM_LINK_MODE_DIRECT   0x03U  /* 直连链路模式。 */
#define UART_COMM_DEFAULT_LINK_MODE  UART_COMM_LINK_MODE_BUCKET  /* 默认链路模式。 */

/* 兼容旧接口的端口编号，当前基站主动使用 USART3 与桶体通信。 */
typedef enum
{
  UART_COMM_PORT_LINUX = 0,  /* Linux/上位机日志或调试端口。 */
  UART_COMM_PORT_BASE = 1    /* 基站与桶体通信端口。 */
} UartCommPort_t;

/* 初始化 USART3 桶体 DMA 接收、USART2 液位传感器 DMA 接收和发送状态。 */
void UART_Comm_Init(void);

/* 通信任务周期入口：解析桶体帧、发送状态帧、检查 5s 断开超时。 */
void UART_Comm_TaskProcess(void);

/* HAL_UARTEx_RxEventCallback 转发入口，用于处理 ReceiveToIdle DMA 新数据。 */
void UART_Comm_RxEventCallback(UART_HandleTypeDef *huart, uint16_t pos);

/* HAL_UART_TxCpltCallback 转发入口，用于继续发送排队中的状态帧。 */
void UART_Comm_TxCpltCallback(UART_HandleTypeDef *huart);

/* HAL_UART_ErrorCallback 转发入口，用于恢复 DMA 接收或发送状态。 */
void UART_Comm_ErrorCallback(UART_HandleTypeDef *huart);

/* 直接解析一帧 30 字节数据，主要用于单元测试或兼容旧调用。 */
void UART_ParseFrame(uint8_t *data, uint8_t len);

/* 构造基站状态上报帧，返回帧长度；frame 至少需要 30 字节空间。 */
uint8_t UART_Comm_BuildStatusFrame(uint8_t *frame);

/* 校验帧头、data[2]=0x02 和校验和，返回 1 表示基站可解析。 */
uint8_t UART_Comm_IsFrameValid(const uint8_t *frame);

/* 计算 byte[2] 到 byte[28] 的协议校验和。 */
uint8_t UART_Comm_Checksum(const uint8_t *frame);

/* 返回当前桶体链路是否在线；5s 未收到有效帧会变为 0。 */
uint8_t UART_Comm_IsBaseConnected(void);

#ifdef __cplusplus
}
#endif

#endif
