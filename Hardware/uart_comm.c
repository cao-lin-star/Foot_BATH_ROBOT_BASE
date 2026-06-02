#include "uart_comm.h"
#include "log.h"
#include "system_monitor.h"
#include "usart.h"
#include <string.h>

#ifndef UART_COMM_STATUS_PERIOD_MS
#define UART_COMM_STATUS_PERIOD_MS       1000UL
#endif

#ifndef UART_COMM_BUCKET_TIMEOUT_MS
#define UART_COMM_BUCKET_TIMEOUT_MS      5000UL
#endif

#ifndef UART_COMM_RX_DMA_BUFFER_LEN
#define UART_COMM_RX_DMA_BUFFER_LEN      128U
#endif

typedef struct
{
  /* 流式解析缓存。DMA 收到的是字节流，需按帧头重新组包。 */
  uint8_t buffer[UART_COMM_FRAME_LEN];
  uint8_t index;
} UartParser_t;

typedef struct
{
  /* 中断/DMA 回调写入完整帧，任务上下文再取走处理。 */
  uint8_t frame[UART_COMM_FRAME_LEN];
  volatile uint8_t ready;
} UartFrameSlot_t;

typedef struct
{
  /*
   * 单帧发送队列。
   * active_frame 交给 DMA 使用，pending_frame 保存下一帧，避免发送中覆盖。
   */
  UART_HandleTypeDef *huart;
  uint8_t active_frame[UART_COMM_FRAME_LEN];
  uint8_t pending_frame[UART_COMM_FRAME_LEN];
  volatile uint8_t busy;
  volatile uint8_t pending;
} UartTxSlot_t;

static UartParser_t bucket_parser;
static UartFrameSlot_t bucket_rx_slot;
static UartTxSlot_t bucket_tx_slot;

static uint8_t bucket_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
static uint8_t level_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
static uint16_t bucket_rx_dma_pos;
static uint16_t level_rx_dma_pos;

static uint32_t bucket_last_rx_tick;
static uint32_t last_status_tx_tick;
static uint8_t last_link_mode;
static uint8_t bucket_timeout_logged;
static uint8_t last_bucket_frame[UART_COMM_FRAME_LEN];

static void UART_Comm_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

uint8_t UART_Comm_Checksum(const uint8_t *frame)
{
  uint16_t sum = 0U;
  uint8_t index;

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
  return (UART_Comm_Checksum(frame) == frame[29]) ? 1U : 0U;
}

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

static uint8_t UART_Comm_FetchFrame(UartFrameSlot_t *slot, uint8_t *out_frame)
{
  uint8_t ready;
  uint32_t primask;

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

static HAL_StatusTypeDef UART_Comm_StartReceiveOne(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t *old_pos)
{
  HAL_StatusTypeDef status;

  *old_pos = 0U;
  /*
   * 使用 ReceiveToIdle DMA：
   *   串口空闲或 DMA 缓冲区位置变化时进入 RxEvent 回调；
   *   适合协议帧长度固定但到达间隔不固定的串口通信。
   */
  status = HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, UART_COMM_RX_DMA_BUFFER_LEN);
  if ((status == HAL_OK) && (huart->hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  return status;
}

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

static void UART_Comm_ProcessBucketRxRange(uint16_t start, uint16_t end)
{
  uint16_t index;

  for (index = start; index < end; index++)
  {
    UART_Comm_ParserPush(&bucket_parser, &bucket_rx_slot, bucket_rx_dma_buffer[index]);
  }
}

static void UART_Comm_ProcessLevelRxRange(uint16_t start, uint16_t end)
{
  uint16_t index;

  for (index = start; index < end; index++)
  {
    /*
     * 液位传感器协议暂未展开解析，先将收到的最新字节作为液位状态缓存。
     * 后续如果液位板有完整帧格式，只需要替换这里的解析逻辑。
     */
    Base_SetLevelSensorValue(level_rx_dma_buffer[index]);
  }
}

static void UART_Comm_ProcessDmaRx(uint8_t is_bucket, uint16_t *old_pos, uint16_t pos)
{
  if ((old_pos == NULL) || (pos > UART_COMM_RX_DMA_BUFFER_LEN) || (pos == *old_pos))
  {
    return;
  }

  /*
   * DMA 环形缓冲区位置可能正向推进，也可能回卷。
   * 分两段处理回卷数据，保证不漏字节。
   */
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

static void UART_Comm_TryStartTx(UartTxSlot_t *slot)
{
  uint8_t should_start = 0U;
  uint32_t primask;

  if ((slot == NULL) || (slot->huart == NULL))
  {
    return;
  }

  /* 发送状态在任务和 DMA 完成回调间共享，需要短临界区保护。 */
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

static void UART_Comm_SendBucketFrame(const uint8_t *frame)
{
  uint32_t primask;

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

static void UART_Comm_ProcessBucketFrame(const uint8_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  /*
   * 只有通过帧头和校验的桶体帧才会到这里。
   * 收到有效帧即刷新 5s 在线计时，并按命令字驱动基站状态机。
   */
  memcpy(last_bucket_frame, frame, UART_COMM_FRAME_LEN);
  bucket_last_rx_tick = HAL_GetTick();
  bucket_timeout_logged = 0U;
  Base_SetBucketConnected(1U);
  last_link_mode = frame[2];

  if ((frame[3] >= 0xB0U) && (frame[3] <= 0xBFU))
  {
    Base_HandleCommand(frame);
  }
}

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

uint8_t UART_Comm_BuildStatusFrame(uint8_t *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }

  /*
   * 状态帧由通信层统一填帧头/链路/校验，
   * 业务数据由 Base_BuildStatusData() 填入 frame[16]-frame[27]。
   */
  memset(frame, 0, UART_COMM_FRAME_LEN);
  frame[0] = UART_COMM_HEAD1;
  frame[1] = UART_COMM_HEAD2;
  frame[2] = last_link_mode;
  frame[3] = SystemMonitor_GetCommand();
  /*
   * 连接在线时保留桶体上一帧的部分数据，便于上位侧看到桶体状态透传。
   * 断开时保持为 0，避免上报过期桶体数据。
   */
  if (Base_IsBucketConnected() != 0U)
  {
    memcpy(&frame[5], &last_bucket_frame[5], 11U);
  }
  Base_BuildStatusData(frame);
  frame[28] = Base_IsBucketCirculationRequested();
  frame[29] = UART_Comm_Checksum(frame);
  return UART_COMM_FRAME_LEN;
}

uint8_t UART_Comm_IsBaseConnected(void)
{
  return Base_IsBucketConnected();
}

void UART_ParseFrame(uint8_t *data, uint8_t len)
{
  if ((data == NULL) || (len != UART_COMM_FRAME_LEN) || (UART_Comm_IsFrameValid(data) == 0U))
  {
    return;
  }
  UART_Comm_ProcessBucketFrame(data);
}

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

void UART_Comm_TaskProcess(void)
{
  uint8_t frame[UART_COMM_FRAME_LEN];
  uint32_t now;

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

void UART_Comm_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

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
