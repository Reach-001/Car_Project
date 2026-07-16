#ifndef COMM_H
#define COMM_H

#include <stdbool.h>
#include <stdint.h>

#include "system_state_pool.h"

/* ────────────────────────────────────────────────────────────
 * Comm 域 —— 解析通信协议，只输出数据，不控制底盘
 *
 * 职责：通过 BspUart 收发数据，解析蓝牙/K230 协议，
 *       输出到 pool->comm 和 pool->event。
 *
 * 蓝牙子模块：bt_link.c（解析遥控器协议）
 * K230 子模块：k230_link.c（解析视觉识别结果）
 * 协议解析器：protocol.c（通用行解析器）
 *
 * 依赖：BSP/bsp_uart.h、协议工具
 * 禁止：调用任何 Motion/BSP Actuator 函数
 * ──────────────────────────────────────────────────────────── */

typedef struct
{
    BtCommand  bt_cmd;
    K230Result k230_res;
    bool       bt_connected;
    bool       k230_connected;
    uint32_t   bt_last_rx_ms;
    uint32_t   k230_last_rx_ms;
    uint32_t   bt_rx_bytes;
    uint32_t   k230_rx_bytes;
} CommState;

/* ── 生命周期 ── */

void Comm_Init(void);

/** 20ms 周期调用：收蓝牙 + K230 数据，解析后写入 pool */
void Comm_Task20ms(SystemStatePool *pool);

CommState Comm_GetState(void);

/* 2/3/4横杆触发后调用，授权K230上报一次最近识别结果。 */
bool Comm_RequestK230Detect(void);

/* ── 命令注入（App 层使用） ── */

typedef struct
{
    bool send_status;        /* 触发一次状态上报 */
    bool flush_rx;           /* 清空接收缓冲 */
} CommCommand;

void Comm_SetCommand(const CommCommand *cmd);

#endif /* COMM_H */
