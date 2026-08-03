#include "uart_comm.h"
#include "log.h"
#include "system_monitor.h"
#include "usart.h"
#include <string.h>

#ifndef UART_COMM_STATUS_PERIOD_MS
#define UART_COMM_STATUS_PERIOD_MS       1000UL  /* 主动上报基站状态帧的周期。 */
#endif

#ifndef UART_COMM_BUCKET_TIMEOUT_MS
#define UART_COMM_BUCKET_TIMEOUT_MS      5000UL  /* 桶体链路无有效帧超时时间。 */
#endif

#ifndef UART_COMM_RX_DMA_BUFFER_LEN
#define UART_COMM_RX_DMA_BUFFER_LEN      128U    /* 串口 ReceiveToIdle DMA 环形缓冲区长度。 */
#endif

/* 固定帧流式解析器。 */
typedef struct
{
  /* DMA 收到的是字节流，需要按帧头重新组包。 */
  uint8_t buffer[UART_COMM_FRAME_LEN];  /* 当前正在组装的帧缓存。 */
  uint8_t index;                        /* 下一字节写入位置。 */
} UartParser_t;

/* 中断到任务之间传递完整帧的单帧槽。 */
typedef struct
{
  /* 中断/DMA 回调写入完整帧，任务上下文再取走处理。 */
  uint8_t frame[UART_COMM_FRAME_LEN];  /* 已校验通过的完整帧。 */
  volatile uint8_t ready;              /* 完整帧就绪标志。 */
} UartFrameSlot_t;

/* 单帧发送队列。 */
typedef struct
{
  /*
   * active_frame 交给 DMA 使用，pending_frame 保存下一帧，
   * 避免发送中覆盖 DMA 正在读取的数据。   */
  UART_HandleTypeDef *huart;                 /* 该发送槽绑定的串口句柄。 */
  uint8_t active_frame[UART_COMM_FRAME_LEN];  /* DMA 当前发送帧。 */
  uint8_t pending_frame[UART_COMM_FRAME_LEN]; /* 等待发送的下一帧。 */
  volatile uint8_t busy;                      /* DMA 发送忙标志。 */
  volatile uint8_t pending;                   /* 是否存在待发送帧。 */
} UartTxSlot_t;

/* 桶体协议流式解析器。 */
static UartParser_t bucket_parser;
/* 桶体接收完整帧槽。 */
static UartFrameSlot_t bucket_rx_slot;
/* 桶体状态帧发送槽。 */
static UartTxSlot_t bucket_tx_slot;

/* USART3 桶体 ReceiveToIdle DMA 接收缓冲区。 */
static uint8_t bucket_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
/* USART2 液位传感器 ReceiveToIdle DMA 接收缓冲区。 */
static uint8_t level_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
/* 桶体 DMA 上一次处理到的位置。 */
static uint16_t bucket_rx_dma_pos;
/* 液位 DMA 上一次处理到的位置。 */
static uint16_t level_rx_dma_pos;

/* 最近一次收到桶体有效帧。tick。 */
static uint32_t bucket_last_rx_tick;
/* 最近一次主动发送状态帧。tick。 */
static uint32_t last_status_tx_tick;
/* 最近一次收到或使用的链路模式。 */
static uint8_t last_link_mode;
/* 桶体掉线日志是否已经打印，避免超时期间重复刷屏。 */
static uint8_t bucket_timeout_logged;
/* 最近一帧桶体有效数据，用于在线时透传部分桶体状态。 */
static uint8_t last_bucket_frame[UART_COMM_FRAME_LEN];

/* 按进入临界区前的中断状态恢复中断。 */
static void UART_Comm_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/* 计算协议校验和。 */
uint8_t UART_Comm_Checksum(const uint8_t *frame)
{
  uint16_t sum = 0U; /* 累加和，保留 16 位避免中间溢出影响表达。 */
  uint8_t index;     /* 协议数据区遍历索引。 */

  if (frame == NULL)
  {
    return 0U;
  }

  /* 协议校验不包含帧头，也不包含最后的校验字节。 */
  for (index = 2U; index <= 28U; index++)
  {
    sum = (uint16_t)(sum + frame[index]);
  }
  return (uint8_t)(sum & 0xFFU);
}

/* 校验基站接收的 30 字节协议帧是否有效。 */
uint8_t UART_Comm_IsFrameValid(const uint8_t *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }
  if ((frame[0] != UART_COMM_HEAD1) || (frame[1] != UART_COMM_HEAD2))
  {
    return 0U;
  }
  /* 基站只解析透传模式帧，data[2] 不是 0x02 时直接丢弃。 */
  if (frame[2] != UART_COMM_LINK_MODE_TRANSIT)
  {
    return 0U;
  }
  return (UART_Comm_Checksum(frame) == frame[29]) ? 1U : 0U;
}

/* 向流式解析器推入一个字节。 */
static void UART_Comm_ParserPush(UartParser_t *parser, UartFrameSlot_t *slot, uint8_t byte)
{
  if ((parser == NULL) || (slot == NULL))
  {
    return;
  }

  /* 第 1 个字节必须是 0x55，否则继续丢弃直到找到帧头。 */
  if (parser->index == 0U)
  {
    if (byte == UART_COMM_HEAD1)
    {
      parser->buffer[parser->index++] = byte;
    }
    return;
  }

  /* 第 2 个字节必须是 0xAA；不匹配则重新找帧头。 */
  if (parser->index == 1U)
  {
    if (byte != UART_COMM_HEAD2)
    {
      parser->index = 0U;
      return;
    }
    parser->buffer[parser->index++] = byte;
    return;
  }

  /* 后续固定收满 30 字节，再做一次完整校验。 */
  parser->buffer[parser->index++] = byte;
  if (parser->index >= UART_COMM_FRAME_LEN)
  {
    if (UART_Comm_IsFrameValid(parser->buffer) != 0U)
    {
      memcpy(slot->frame, parser->buffer, UART_COMM_FRAME_LEN);
      slot->ready = 1U;
    }
    parser->index = 0U;
  }
}

/* 从中断写入的帧槽中取出一帧。 */
static uint8_t UART_Comm_FetchFrame(UartFrameSlot_t *slot, uint8_t *out_frame)
{
  uint8_t ready;     /* 进入临界区时读取到的就绪状态。 */
  uint32_t primask;  /* 进入临界区前的中断屏蔽状态。 */

  if ((slot == NULL) || (out_frame == NULL))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  ready = slot->ready;
  if (ready != 0U)
  {
    memcpy(out_frame, slot->frame, UART_COMM_FRAME_LEN);
    slot->ready = 0U;
  }
  UART_Comm_RestoreIrq(primask);
  return ready;
}

/* 启动一个串口的 ReceiveToIdle DMA 接收。 */
static HAL_StatusTypeDef UART_Comm_StartReceiveOne(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t *old_pos)
{
  HAL_StatusTypeDef status; /* HAL 启动 DMA 接收的返回状态。 */

  *old_pos = 0U;
  /*
   * 使用 ReceiveToIdle DMA。   *   串口空闲。DMA 缓冲区位置变化时进入 RxEvent 回调。   *   适合协议帧长度固定但到达间隔不固定的串口通信。   */
  status = HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, UART_COMM_RX_DMA_BUFFER_LEN);
  if ((status == HAL_OK) && (huart->hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  return status;
}

/* 停止一个串口的 DMA 接收并恢复 HAL 状态。 */
static void UART_Comm_StopReceiveOne(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
  if (huart->hdmarx != NULL)
  {
    (void)HAL_DMA_Abort(huart->hdmarx);
  }
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE | USART_CR1_IDLEIE));
  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
  huart->RxState = HAL_UART_STATE_READY;
  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
}

/* 处理桶体 DMA 缓冲区中的一段新数据。 */
static void UART_Comm_ProcessBucketRxRange(uint16_t start, uint16_t end)
{
  uint16_t index; /* DMA 缓冲区遍历索引。 */

  for (index = start; index < end; index++)
  {
    UART_Comm_ParserPush(&bucket_parser, &bucket_rx_slot, bucket_rx_dma_buffer[index]);
  }
}

/* 处理液位传感器 DMA 缓冲区中的一段新数据。 */
static void UART_Comm_ProcessLevelRxRange(uint16_t start, uint16_t end)
{
  uint16_t index; /* DMA 缓冲区遍历索引。 */

  for (index = start; index < end; index++)
  {
    /*
     * 缺液控制板每秒主动上报 1 字节。1010xxx。     * 低 3 位中 1 表示有液，0 表示缺液。     *   bit0/K1 = 药液 1
     *   bit1/K2 = 药液 2
     *   bit2/K3 = 清洁液     * 状态机负责校验高 5 位并转换成内部缺液位图。     */
    Base_SetLevelSensorValue(level_rx_dma_buffer[index]);
  }
}

/* 根据 DMA 新位置处理新增接收数据，自动兼容环形缓冲区回卷。 */
static void UART_Comm_ProcessDmaRx(uint8_t is_bucket, uint16_t *old_pos, uint16_t pos)
{
  if ((old_pos == NULL) || (pos > UART_COMM_RX_DMA_BUFFER_LEN) || (pos == *old_pos))
  {
    return;
  }

  /*
   * DMA 环形缓冲区位置可能正向推进，也可能回卷。   * 分两段处理回卷数据，保证不漏字节。   */
  if (pos > *old_pos)
  {
    if (is_bucket != 0U)
    {
      UART_Comm_ProcessBucketRxRange(*old_pos, pos);
    }
    else
    {
      UART_Comm_ProcessLevelRxRange(*old_pos, pos);
    }
  }
  else
  {
    if (is_bucket != 0U)
    {
      UART_Comm_ProcessBucketRxRange(*old_pos, UART_COMM_RX_DMA_BUFFER_LEN);
      UART_Comm_ProcessBucketRxRange(0U, pos);
    }
    else
    {
      UART_Comm_ProcessLevelRxRange(*old_pos, UART_COMM_RX_DMA_BUFFER_LEN);
      UART_Comm_ProcessLevelRxRange(0U, pos);
    }
  }
  *old_pos = pos;
}

/* 如果发送槽空闲且存在待发送帧，则启动 DMA 发送。 */
static void UART_Comm_TryStartTx(UartTxSlot_t *slot)
{
  uint8_t should_start = 0U; /* 是否需要在退出临界区后启。DMA。 */
  uint32_t primask;          /* 进入临界区前的中断屏蔽状态。 */

  if ((slot == NULL) || (slot->huart == NULL))
  {
    return;
  }

  /* 发送状态在任务与 DMA 完成回调间共享，需要短临界区保护。 */
  primask = __get_PRIMASK();
  __disable_irq();
  if ((slot->busy == 0U) && (slot->pending != 0U))
  {
    memcpy(slot->active_frame, slot->pending_frame, UART_COMM_FRAME_LEN);
    slot->pending = 0U;
    slot->busy = 1U;
    should_start = 1U;
  }
  UART_Comm_RestoreIrq(primask);

  if (should_start != 0U)
  {
    if (HAL_UART_Transmit_DMA(slot->huart, slot->active_frame, UART_COMM_FRAME_LEN) != HAL_OK)
    {
      primask = __get_PRIMASK();
      __disable_irq();
      memcpy(slot->pending_frame, slot->active_frame, UART_COMM_FRAME_LEN);
      slot->pending = 1U;
      slot->busy = 0U;
      UART_Comm_RestoreIrq(primask);
    }
  }
}

/* 将一帧桶体方向数据放入发送槽。 */
static void UART_Comm_SendBucketFrame(const uint8_t *frame)
{
  uint32_t primask; /* 进入临界区前的中断屏蔽状态。 */

  if (frame == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(bucket_tx_slot.pending_frame, frame, UART_COMM_FRAME_LEN);
  bucket_tx_slot.pending = 1U;
  UART_Comm_RestoreIrq(primask);

  UART_Comm_TryStartTx(&bucket_tx_slot);
}

/* 处理一帧已校验通过的桶体协议帧。 */
static void UART_Comm_ProcessBucketFrame(const uint8_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  /*
   * 只有通过帧头和校验的桶体帧才会到这里。   * 收到有效帧即刷新 5s 在线计时，并按命令字驱动基站状态机。   */
  memcpy(last_bucket_frame, frame, UART_COMM_FRAME_LEN);
  bucket_last_rx_tick = HAL_GetTick();
  bucket_timeout_logged = 0U;
  Base_SetBucketConnected(1U);
  last_link_mode = frame[2];

  if ((frame[3] >= 0xB0U) && (frame[3] <= 0xBFU))
  {
    Base_HandleCommand(frame);
  }
  else if (frame[3] == 0x00U)
  {
    Base_UpdateBucketRealtimeData(frame);
  }
}

/* 检查桶体链路是否超时离线。 */
static void UART_Comm_CheckBucketTimeout(uint32_t now)
{
  if (bucket_last_rx_tick == 0UL)
  {
    Base_SetBucketConnected(0U);
    return;
  }

  /* 5s 未收到桶体有效帧则判定断开，并只打印一次断开日志。 */
  if ((now - bucket_last_rx_tick) >= UART_COMM_BUCKET_TIMEOUT_MS)
  {
    Base_SetBucketConnected(0U);
    if (bucket_timeout_logged == 0U)
    {
      bucket_timeout_logged = 1U;
      Logging_Print("Bucket link timeout\r\n");
    }
  }
}

/* 构造基站状态上报帧。 */
uint8_t UART_Comm_BuildStatusFrame(uint8_t *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }

  /*
   * 状态帧由通信层统一填帧头、链路和校验。   * 业务数据由 Base_BuildStatusData() 填入 frame[16]-frame[27]。   */
  memset(frame, 0, UART_COMM_FRAME_LEN);
  frame[0] = UART_COMM_HEAD1;
  frame[1] = UART_COMM_HEAD2;
  frame[2] = last_link_mode;
  frame[3] = SystemMonitor_GetCommand();
  /*
   * 连接在线时保留桶体上一帧的部分数据，便于上位侧看到桶体状态透传。   * 断开时保持为 0，避免上报过期桶体数据。   */
  if (Base_IsBucketConnected() != 0U)
  {
    memcpy(&frame[5], &last_bucket_frame[5], 11U);
  }
  Base_BuildStatusData(frame);
  frame[28] = Base_IsBucketCirculationRequested();
  frame[29] = UART_Comm_Checksum(frame);
  return UART_COMM_FRAME_LEN;
}

/* 返回桶体链路是否在线。 */
uint8_t UART_Comm_IsBaseConnected(void)
{
  return Base_IsBucketConnected();
}

/* 兼容旧接口：直接解析一帧完整数据。 */
void UART_ParseFrame(uint8_t *data, uint8_t len)
{
  if ((data == NULL) || (len != UART_COMM_FRAME_LEN) || (UART_Comm_IsFrameValid(data) == 0U))
  {
    return;
  }
  UART_Comm_ProcessBucketFrame(data);
}

/* 初始化通信模块状态和 DMA 接收。 */
void UART_Comm_Init(void)
{
  memset(&bucket_parser, 0, sizeof(bucket_parser));
  memset(&bucket_rx_slot, 0, sizeof(bucket_rx_slot));
  memset(&bucket_tx_slot, 0, sizeof(bucket_tx_slot));
  memset(bucket_rx_dma_buffer, 0, sizeof(bucket_rx_dma_buffer));
  memset(level_rx_dma_buffer, 0, sizeof(level_rx_dma_buffer));
  memset(last_bucket_frame, 0, sizeof(last_bucket_frame));

  bucket_tx_slot.huart = &huart3;
  bucket_last_rx_tick = 0UL;
  last_status_tx_tick = 0UL;
  last_link_mode = UART_COMM_LINK_MODE_TRANSIT;
  bucket_timeout_logged = 0U;

  (void)UART_Comm_StartReceiveOne(&huart3, bucket_rx_dma_buffer, &bucket_rx_dma_pos);
  (void)UART_Comm_StartReceiveOne(&huart2, level_rx_dma_buffer, &level_rx_dma_pos);
}

/* 通信周期任务入口。 */
void UART_Comm_TaskProcess(void)
{
  uint8_t frame[UART_COMM_FRAME_LEN]; /* 临时收发帧缓存。 */
  uint32_t now;                       /* 当前 HAL tick。 */

  /* 优先处理桶体最新命令帧，并立即回一帧基站状态。 */
  if (UART_Comm_FetchFrame(&bucket_rx_slot, frame) != 0U)
  {
    UART_Comm_ProcessBucketFrame(frame);
    UART_Comm_BuildStatusFrame(frame);
    UART_Comm_SendBucketFrame(frame);
  }

  now = HAL_GetTick();
  UART_Comm_CheckBucketTimeout(now);
  /* 即使没有新命令，也每秒主动上报一次基站状态。 */
  if ((now - last_status_tx_tick) >= UART_COMM_STATUS_PERIOD_MS)
  {
    last_status_tx_tick = now;
    UART_Comm_BuildStatusFrame(frame);
    UART_Comm_SendBucketFrame(frame);
  }
}

/* 串口 ReceiveToIdle DMA 事件回调转发入口。 */
void UART_Comm_RxEventCallback(UART_HandleTypeDef *huart, uint16_t pos)
{
  if (huart == &huart3)
  {
    UART_Comm_ProcessDmaRx(1U, &bucket_rx_dma_pos, pos);
  }
  else if (huart == &huart2)
  {
    UART_Comm_ProcessDmaRx(0U, &level_rx_dma_pos, pos);
  }
}

/* 串口 DMA 发送完成回调转发入口。 */
void UART_Comm_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask; /* 进入临界区前的中断屏蔽状态。 */

  if (huart != &huart3)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  bucket_tx_slot.busy = 0U;
  UART_Comm_RestoreIrq(primask);
  UART_Comm_TryStartTx(&bucket_tx_slot);
}

/* 串口错误回调转发入口。 */
void UART_Comm_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    UART_Comm_StopReceiveOne(&huart3);
    (void)UART_Comm_StartReceiveOne(&huart3, bucket_rx_dma_buffer, &bucket_rx_dma_pos);
  }
  else if (huart == &huart2)
  {
    UART_Comm_StopReceiveOne(&huart2);
    (void)UART_Comm_StartReceiveOne(&huart2, level_rx_dma_buffer, &level_rx_dma_pos);
  }

  if ((huart == &huart3) && (huart->gState == HAL_UART_STATE_READY))
  {
    bucket_tx_slot.busy = 0U;
    UART_Comm_TryStartTx(&bucket_tx_slot);
  }
}
