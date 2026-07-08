#include "bsp_uart.h"

#include "ring_buffer.h"
#include "stm32g4xx_hal.h"
#include "usart.h"

/* ── per-UART state ── */

#define BSP_UART_RX_BUF_SIZE 128U

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t rx_storage[BSP_UART_RX_BUF_SIZE];
    RingBuffer rx_ring;
    bool initialized;
} BspUartCtx;

static BspUartCtx s_ctx[BSP_UART_COUNT];

static UART_HandleTypeDef *get_huart(BspUartId id)
{
    /* map logical ID → CubeMX handle */
    if (id == BSP_UART_K230)
    {
        return &huart2;
    }
    if (id == BSP_UART_BT)
    {
        return &huart3;
    }
    return 0;
}

/* ── init ── */

void BspUart_Init(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT)
    {
        return;
    }

    BspUartCtx *ctx = &s_ctx[id];
    UART_HandleTypeDef *huart = get_huart(id);

    if (huart == 0)
    {
        return;
    }

    ctx->huart = huart;
    RingBuffer_Init(&ctx->rx_ring, ctx->rx_storage, BSP_UART_RX_BUF_SIZE);
    ctx->initialized = true;

    /* enable NVIC + RXNE interrupt */
    if (id == BSP_UART_K230)
    {
        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else
    {
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }

    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/* ── task-side RX ── */

bool BspUart_ReadByte(BspUartId id, uint8_t *byte)
{
    if (((uint32_t)id >= (uint32_t)BSP_UART_COUNT) || (byte == 0))
    {
        return false;
    }

    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized)
    {
        return false;
    }

    return RingBuffer_Pop(&ctx->rx_ring, byte);
}

uint16_t BspUart_Available(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT)
    {
        return 0U;
    }

    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized)
    {
        return 0U;
    }

    return RingBuffer_Available(&ctx->rx_ring);
}

/* ── task-side TX (blocking) ── */

void BspUart_WriteByte(BspUartId id, uint8_t byte)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT)
    {
        return;
    }

    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized || (ctx->huart == 0))
    {
        return;
    }

    /* poll until TX data register is empty */
    while (__HAL_UART_GET_FLAG(ctx->huart, UART_FLAG_TXE) == RESET)
    {
    }

    WRITE_REG(ctx->huart->Instance->TDR, byte);
}

void BspUart_WriteString(BspUartId id, const char *str)
{
    if (str == 0)
    {
        return;
    }

    while (*str != '\0')
    {
        BspUart_WriteByte(id, (uint8_t)*str);
        ++str;
    }
}

/* ── ISR hook ── */

void BspUart_RxIsrHook(BspUartId id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_COUNT)
    {
        return;
    }

    BspUartCtx *ctx = &s_ctx[id];
    if (!ctx->initialized || (ctx->huart == 0))
    {
        return;
    }

    /* check and clear RXNE */
    if (__HAL_UART_GET_FLAG(ctx->huart, UART_FLAG_RXNE) != RESET)
    {
        uint8_t byte = (uint8_t)READ_REG(ctx->huart->Instance->RDR);
        RingBuffer_PushFromIsr(&ctx->rx_ring, byte);
    }

    /* clear overrun error if any (prevents RX lockup) */
    if (__HAL_UART_GET_FLAG(ctx->huart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(ctx->huart);
    }
}
