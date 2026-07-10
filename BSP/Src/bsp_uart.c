#include "bsp_uart.h"

#include "ring_buffer.h"        /* RingBuffer 通用服务 */
#include "stm32g4xx_hal.h"      /* HAL UART / DMA 接口 */
#include "usart.h"              /* huart2, huart3 句柄 */

#include <string.h>             /* strlen */

/* ────────────────────────────────────────────────────────────
 * UART 通信驱动实现（DMA 收发）
 *
 * 接收路线：
 *   硬件 DMA 循环写 → dma_rx_buf（256 字节）
 *   Task 轮询 DMA 当前写指针 → 增量搬运到 RingBuffer
 *   Task → BspUart_ReadByte → RingBuffer_Pop
 *
 * 发送路线：
 *   Task → BspUart_WriteBuffer → HAL_UART_Transmit_DMA → 立即返回
 *   DMA TC 中断 → UART TC 中断 → HAL_UART_TxCpltCallback → 置 tx_done
 *
 * 注意：HAL_UART_RxHalfCpltCallback / RxCpltCallback / TxCpltCallback
 *       是全局唯一的 weak 函数，由本文件实现。
 *       通过 hUART->Instance 判断是哪一个 UART。
 * ──────────────────────────────────────────────────────────── */

#define DMA_RX_BUF_SIZE  256U          /* DMA 循环缓冲大小（必须是偶数） */
#define DMA_RX_HALF_SIZE (DMA_RX_BUF_SIZE / 2U)   /* 半满阈值 */

#define RING_BUF_SIZE    128U          /* RingBuffer 大小（任务侧读）   */
#define TX_BUF_SIZE      128U          /* DMA TX 内部拷贝缓冲大小       */

/* ── DMA TX 完成标志 ── */
typedef enum { TX_IDLE = 0, TX_BUSY } TxState;

/* 单个 UART 的完整 DMA 上下文 */
typedef struct
{
    UART_HandleTypeDef *huart;              /* HAL UART 句柄指针          */

    /* ── DMA RX ── */
    uint8_t    dma_rx_buf[DMA_RX_BUF_SIZE]; /* DMA 循环写缓冲区           */
    uint8_t    ring_storage[RING_BUF_SIZE];  /* RingBuffer 存储空间        */
    RingBuffer rx_ring;                     /* ISR Push → Task Pop         */
    uint16_t   rx_dma_read_pos;              /* Task 已搬运到的位置         */

    /* ── DMA TX ── */
    uint8_t        tx_storage[TX_BUF_SIZE];  /* DMA 发送内部缓冲            */
    const uint8_t *tx_data;                 /* 指向 tx_storage             */
    uint16_t       tx_len;                  /* 待发送数据长度              */
    volatile TxState tx_state;              /* 发送状态（ISR 中修改）      */

    bool initialized;
} BspUartCtx;

static BspUartCtx s_ctx[BSP_UART_COUNT];

/* 逻辑 ID → HAL 句柄指针 */
static UART_HandleTypeDef *get_huart(BspUartId id)
{
    if      (id == BSP_UART_K230) { return &huart2; }
    else if (id == BSP_UART_BT)   { return &huart3; }
    return 0;
}

/* ID → huart->Instance 的快速反向查找（给回调用） */
static BspUartId id_from_huart(USART_TypeDef *instance)
{
    if      (instance == USART2) { return BSP_UART_K230; }
    else if (instance == USART3) { return BSP_UART_BT;   }
    return BSP_UART_K230;   /* 兜底 */
}

/* ── 初始化 ── */

void BspUart_Init(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT) { return; }

    BspUartCtx *ctx = &s_ctx[id];
    UART_HandleTypeDef *huart = get_huart(id);
    if (huart == 0) { return; }

    ctx->huart       = huart;
    ctx->tx_data     = 0;
    ctx->tx_len      = 0U;
    ctx->tx_state    = TX_IDLE;
    ctx->rx_dma_read_pos = 0U;
    ctx->initialized = true;

    /* 初始化 RingBuffer */
    RingBuffer_Init(&ctx->rx_ring, ctx->ring_storage, RING_BUF_SIZE);

    /* 启动 DMA 循环接收：DMA 自动把收到的字节写入 dma_rx_buf
     * HT 中断 → 上半部满 → 回调搬数据
     * TC 中断 → 下半部满 → 回调搬数据
     * DMA CIRCULAR 模式永不停止，自动回绕 */
    HAL_UART_Receive_DMA(huart, ctx->dma_rx_buf, DMA_RX_BUF_SIZE);
}

static void poll_dma_rx(BspUartCtx *ctx)
{
    if ((ctx == 0) || !ctx->initialized || (ctx->huart == 0) || (ctx->huart->hdmarx == 0))
    {
        return;
    }

    uint16_t write_pos = (uint16_t)(DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(ctx->huart->hdmarx));
    if (write_pos >= DMA_RX_BUF_SIZE)
    {
        write_pos = 0U;
    }

    while (ctx->rx_dma_read_pos != write_pos)
    {
        (void)RingBuffer_PushFromIsr(&ctx->rx_ring, ctx->dma_rx_buf[ctx->rx_dma_read_pos]);
        ++ctx->rx_dma_read_pos;
        if (ctx->rx_dma_read_pos >= DMA_RX_BUF_SIZE)
        {
            ctx->rx_dma_read_pos = 0U;
        }
    }
}

/* ── 接收（从 RingBuffer 读，非阻塞） ── */

bool BspUart_ReadByte(BspUartId id, uint8_t *byte)
{
    if (((uint32_t)id >= (uint32_t)BSP_UART_COUNT) || (byte == 0)) { return false; }
    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized) { return false; }
    poll_dma_rx(ctx);
    return RingBuffer_Pop(&ctx->rx_ring, byte);
}

uint16_t BspUart_Available(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT) { return 0U; }
    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized) { return 0U; }
    poll_dma_rx(ctx);
    return RingBuffer_Available(&ctx->rx_ring);
}

/* ── 发送（DMA 异步） ── */

bool BspUart_WriteBuffer(BspUartId id, const uint8_t *data, uint16_t len)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT || (data == 0) || (len == 0U)) { return false; }

    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized || ctx->tx_state != TX_IDLE) { return false; }
    if (len > TX_BUF_SIZE) { return false; }

    memcpy(ctx->tx_storage, data, len);
    ctx->tx_data  = ctx->tx_storage;
    ctx->tx_len   = len;
    ctx->tx_state = TX_BUSY;

    if (HAL_UART_Transmit_DMA(ctx->huart, ctx->tx_storage, len) != HAL_OK)
    {
        ctx->tx_data  = 0;
        ctx->tx_len   = 0U;
        ctx->tx_state = TX_IDLE;
        return false;
    }
    return true;
}

bool BspUart_TxDone(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT) { return true; }
    BspUartCtx *ctx = &s_ctx[id];
    return !ctx->initialized || (ctx->tx_state == TX_IDLE);
}

void BspUart_WriteString(BspUartId id, const char *str)
{
    if (str == 0) { return; }

    (void)BspUart_WriteBuffer(id, (const uint8_t *)str,
                              (uint16_t)strlen((const char *)str));
}

/* ── DMA 接收数据搬运（ISR 回调中调用） ── */

void BspUart_DmaRxHalfCplt(BspUartId id)
{
    (void)id;
}

void BspUart_DmaRxFullCplt(BspUartId id)
{
    (void)id;
}

/* ────────────────────────────────────────────────────────────
 * HAL 全局回调（整个工程只有一个）
 * 通过 hUART->Instance 区分是哪个 USART
 * ──────────────────────────────────────────────────────────── */

/* DMA 循环接收 —— 半满中断 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    BspUart_DmaRxHalfCplt(id_from_huart(huart->Instance));
}

/* DMA 循环接收 —— 全满中断（缓冲区回绕，即收到 BUF_SIZE 字节） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BspUart_DmaRxFullCplt(id_from_huart(huart->Instance));
}

/* DMA 发送完成中断 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BspUartId id = id_from_huart(huart->Instance);
    BspUartCtx *ctx = &s_ctx[id];
    ctx->tx_data  = 0;
    ctx->tx_len   = 0U;
    ctx->tx_state = TX_IDLE;
}
