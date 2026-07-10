/* ────────────────────────────────────────────────────────────
 * UART 通信驱动实现（DMA 收发）
 *
 * 接收路线：DMA 循环模式 → dma_rx_buf（DMA_RX_BUF_SIZE 字节）
 *          Task 轮询 DMA 写指针 → 增量搬运到 RingBuffer
 *          Task → ReadByte → RingBuffer_Pop
 *
 * 发送路线：WriteBuffer → HAL_UART_Transmit_DMA → 立即返回
 *          WriteBuffer 会先复制到驱动内部 tx_buf，调用方缓冲可立即释放
 *          DMA TC → HAL_UART_TxCpltCallback → tx_state=IDLE
 *
 * 参数说明：
 *   DMA_RX_BUF_SIZE     = DMA 循环缓冲区大小（字节）。
 *                         256 是 UART 115200bps 吞吐量的合理值。
 *                         改大 → 更少中断但更占 RAM；
 *                         改小 → 更小 RAM 但可能溢出。
 *
 *   RING_BUF_SIZE       = Task 侧环形缓冲区大小（字节）。
 *                         256 给蓝牙摇杆这类突发文本命令留出余量。
 *                         如果 ReadByte 消费不及时会丢字节。
 *
 *   BSP_UART_RX_BUF_SIZE = 历史遗留兼容宏（指向 DMA_RX_BUF_SIZE）。
 *
 * HAL 回调注意：HAL_UART_RxCpltCallback / TxCpltCallback 是
 * 全局唯一的 weak 函数，由本文件实现。
 * ──────────────────────────────────────────────────────────── */

#include "bsp_uart.h"

#include "ring_buffer.h"
#include "stm32g4xx_hal.h"
#include "usart.h"

#include <string.h>

/* ════════════════════════════════════════════════════════════
 * 缓冲区大小参数
 * ════════════════════════════════════════════════════════════ */

#define DMA_RX_BUF_SIZE   256U            /* DMA 循环缓冲区大小（字节），改大→更少中断 */
#define RING_BUF_SIZE     256U            /* Task 侧环形缓冲（字节），过小→丢字节     */
#define DMA_TX_BUF_SIZE   256U            /* DMA 发送缓冲区大小（字节）               */

/* ── DMA TX 状态 ── */
typedef enum { TX_IDLE = 0, TX_BUSY } TxState;

/* 单个 UART 的 DMA 上下文 */
typedef struct
{
    UART_HandleTypeDef *huart;                              /* HAL UART 句柄指针            */
    uint8_t    dma_rx_buf[DMA_RX_BUF_SIZE];                 /* DMA 循环写缓冲区             */
    uint8_t    dma_tx_buf[DMA_TX_BUF_SIZE];                 /* DMA 发送缓冲，驱动内部持有   */
    uint8_t    ring_storage[RING_BUF_SIZE];                  /* RingBuffer 存储              */
    RingBuffer rx_ring;                                     /* ISR Push → Task Pop         */
    uint16_t   dma_rx_last_pos;                             /* 上次已搬运到 RingBuffer 的位置 */
    const uint8_t *tx_data;                                 /* 当前 DMA 发送缓冲指针         */
    uint16_t       tx_len;                                  /* 待发数据长度                  */
    volatile TxState tx_state;                              /* 发送状态（ISR 中修改）        */
    bool initialized;
} BspUartCtx;

static BspUartCtx s_ctx[BSP_UART_COUNT];

static UART_HandleTypeDef *get_huart(BspUartId id)
{
    if      (id == BSP_UART_K230) return &huart2;
    else if (id == BSP_UART_BT)   return &huart3;
    return 0;
}

static BspUartId id_from_huart(USART_TypeDef *inst)
{
    if      (inst == USART2) return BSP_UART_K230;
    else if (inst == USART3) return BSP_UART_BT;
    return BSP_UART_K230;
}

static uint16_t dma_rx_write_pos(const BspUartCtx *ctx)
{
    if ((ctx == 0) || (ctx->huart == 0) || (ctx->huart->hdmarx == 0))
    {
        return 0U;
    }

    uint32_t remaining = __HAL_DMA_GET_COUNTER(ctx->huart->hdmarx);
    if (remaining > DMA_RX_BUF_SIZE)
    {
        remaining = DMA_RX_BUF_SIZE;
    }
    return (uint16_t)(DMA_RX_BUF_SIZE - remaining);
}

static void poll_dma_rx(BspUartCtx *ctx)
{
    if ((ctx == 0) || !ctx->initialized) return;

    uint16_t pos = dma_rx_write_pos(ctx);
    uint16_t last = ctx->dma_rx_last_pos;

    if (pos == last) return;

    if (pos > last)
    {
        for (uint16_t i = last; i < pos; ++i)
        {
            RingBuffer_PushFromIsr(&ctx->rx_ring, ctx->dma_rx_buf[i]);
        }
    }
    else
    {
        for (uint16_t i = last; i < DMA_RX_BUF_SIZE; ++i)
        {
            RingBuffer_PushFromIsr(&ctx->rx_ring, ctx->dma_rx_buf[i]);
        }
        for (uint16_t i = 0U; i < pos; ++i)
        {
            RingBuffer_PushFromIsr(&ctx->rx_ring, ctx->dma_rx_buf[i]);
        }
    }

    ctx->dma_rx_last_pos = pos;
}

/* ── 初始化 ── */

void BspUart_Init(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT) return;
    BspUartCtx *ctx = &s_ctx[id];
    UART_HandleTypeDef *huart = get_huart(id);
    if (huart == 0) return;

    ctx->huart       = huart;
    ctx->tx_data     = 0;
    ctx->tx_len      = 0U;
    ctx->tx_state    = TX_IDLE;
    ctx->dma_rx_last_pos = 0U;
    ctx->initialized = true;
    RingBuffer_Init(&ctx->rx_ring, ctx->ring_storage, RING_BUF_SIZE);
    HAL_UART_Receive_DMA(huart, ctx->dma_rx_buf, DMA_RX_BUF_SIZE);
}

/* ── Task 侧接收 ── */

bool BspUart_ReadByte(BspUartId id, uint8_t *byte)
{
    if (((uint32_t)id >= (uint32_t)BSP_UART_COUNT) || (byte == 0)) return false;
    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized) return false;
    poll_dma_rx(ctx);
    return RingBuffer_Pop(&ctx->rx_ring, byte);
}

uint16_t BspUart_Available(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT) return 0U;
    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized) return 0U;
    poll_dma_rx(ctx);
    return RingBuffer_Available(&ctx->rx_ring);
}

/* ── Task 侧发送 ── */

bool BspUart_WriteBuffer(BspUartId id, const uint8_t *data, uint16_t len)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT || (data == 0) || (len == 0U)) return false;
    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized || ctx->tx_state != TX_IDLE) return false;

    if (len > DMA_TX_BUF_SIZE) return false;

    memcpy(ctx->dma_tx_buf, data, len);
    ctx->tx_data  = ctx->dma_tx_buf;
    ctx->tx_len   = len;
    ctx->tx_state = TX_BUSY;
    HAL_UART_Transmit_DMA(ctx->huart, ctx->dma_tx_buf, len);
    return true;
}

bool BspUart_TxDone(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT) return true;
    BspUartCtx *ctx = &s_ctx[id];
    return !ctx->initialized || (ctx->tx_state == TX_IDLE);
}

void BspUart_WriteString(BspUartId id, const char *str)
{
    if (str == 0) return;
    while (!BspUart_TxDone(id)) {}
    BspUart_WriteBuffer(id, (const uint8_t *)str, (uint16_t)strlen((const char *)str));
}

/* ── DMA 接收数据搬运（HT/TC 回调） ── */

void BspUart_DmaRxHalfCplt(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT) return;
    BspUartCtx *ctx = &s_ctx[id];
    poll_dma_rx(ctx);
}

void BspUart_DmaRxFullCplt(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT) return;
    BspUartCtx *ctx = &s_ctx[id];
    poll_dma_rx(ctx);
}

/* ── HAL 全局回调 ── */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) { BspUart_DmaRxHalfCplt(id_from_huart(huart->Instance)); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)     { BspUart_DmaRxFullCplt(id_from_huart(huart->Instance)); }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BspUartId id = id_from_huart(huart->Instance);
    BspUartCtx *ctx = &s_ctx[id];
    ctx->tx_data  = 0;
    ctx->tx_len   = 0U;
    ctx->tx_state = TX_IDLE;
}
