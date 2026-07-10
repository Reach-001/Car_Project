#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * UART 通信驱动（DMA 收发）—— 硬件抽象层头文件
 *
 * 接收：DMA 循环模式，256 字节循环缓冲。
 *       Task 侧通过 ReadByte / Available 轮询 DMA 写指针，
 *       将增量数据搬到 RingBuffer，短包也能及时读到。
 *
 * 发送：DMA 异步，驱动内部缓冲
 *       WriteBuffer 启动 DMA 发送立即返回
 *       调用时会先复制数据，调用方缓冲可立即释放
 *       TxDone 查询是否发送完毕
 *       WriteString 内部用 WriteBuffer，不阻塞等待
 *
 * 硬件映射：
 *   BSP_UART_K230 = USART2  DMA1_CH1(RX) DMA1_CH2(TX)
 *   BSP_UART_BT   = USART3  DMA1_CH3(RX) DMA1_CH4(TX)
 * ──────────────────────────────────────────────────────────── */

typedef enum
{
    BSP_UART_K230 = 0,   /* USART2 — K230 视觉模块 */
    BSP_UART_BT           /* USART3 — 蓝牙遥控器   */
} BspUartId;

#define BSP_UART_COUNT 2

/* ── 初始化 ── */

/** 初始化 DMA 接收：启动 HAL_UART_Receive_DMA 循环模式
 *  调用时机：CubeMX 初始化 USART + DMA 之后 */
void BspUart_Init(BspUartId id);

/* ── 接收（非阻塞，Task 侧调用，从 RingBuffer 读） ── */

bool     BspUart_ReadByte(BspUartId id, uint8_t *byte);
uint16_t BspUart_Available(BspUartId id);

/* ── 发送（DMA 异步，立即返回） ── */

/** 启动 DMA 发送，数据会拷贝到驱动内部缓冲
 *  @return true = DMA 已启动；false = 上一次发送未完成 */
bool BspUart_WriteBuffer(BspUartId id, const uint8_t *data, uint16_t len);

/** 上一次 WriteBuffer 是否已完成（可发起下一次发送） */
bool BspUart_TxDone(BspUartId id);

/** 发送字符串，内部调用 WriteBuffer，不阻塞等待 */
void BspUart_WriteString(BspUartId id, const char *str);

/* ── DMA 接收管理（由 HAL 回调调用，用户不直接调） ── */

/** DMA 循环缓冲半满时 HAL 回调调用，把上半部数据搬到 RingBuffer */
void BspUart_DmaRxHalfCplt(BspUartId id);

/** DMA 循环缓冲全满时 HAL 回调调用，把下半部数据搬到 RingBuffer */
void BspUart_DmaRxFullCplt(BspUartId id);

#endif /* BSP_UART_H */
