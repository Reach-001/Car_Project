#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

/* ── board-level UART driver (platform-abstracted header) ──
 *
 * Each UART is backed by a static RingBuffer for RX.
 * ISR pushes bytes; task pops them.
 *
 * Wiring checklist for the user:
 *   1. Call BspUart_Init() after CubeMX MX_USARTx_UART_Init().
 *   2. Enable USART_RXNE interrupt: __HAL_UART_ENABLE_IT(&huartx, UART_IT_RXNE).
 *   3. In USARTx_IRQHandler, call BspUart_RxIsrHook(id).
 *   4. In the comm task, call BspUart_ReadByte() / BspUart_Available().
 */

typedef enum
{
    BSP_UART_K230 = 0,   /* USART2,  PA2 TX  PA3 RX */
    BSP_UART_BT           /* USART3, PC10 TX PC11 RX */
} BspUartId;

#define BSP_UART_COUNT 2

/* ── lifecycle ── */

void BspUart_Init(BspUartId id);

/* ── task-side RX (non-blocking) ── */

bool     BspUart_ReadByte(BspUartId id, uint8_t *byte);
uint16_t BspUart_Available(BspUartId id);

/* ── task-side TX (blocking, short strings / debug only) ── */

void BspUart_WriteByte(BspUartId id, uint8_t byte);
void BspUart_WriteString(BspUartId id, const char *str);

/* ── ISR hook ──
 * Call from USARTx_IRQHandler. Reads RDR + clears RXNE, pushes to ring buffer. */

void BspUart_RxIsrHook(BspUartId id);

#endif /* BSP_UART_H */
