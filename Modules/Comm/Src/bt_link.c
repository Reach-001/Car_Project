/* ────────────────────────────────────────────────────────────
 * 蓝牙通信子模块（Comm 域内部使用）
 *
 * 数据流：BT UART DMA → RingBuffer → 行协议解析 → BtCommand
 *
 * 手动控制协议：
 *   speed,angle
 *   speed: 协议量 -100~100，0 表示停车/刹车，负数后退
 *   angle: 轮子相对中位的角度 deg，负数左转，范围由 vehicle_config.h 配置
 * 控制命令：
 *   STOP
 *   DEBUG,0 / DEBUG,1
 *   TEST,0~11（0=退出运行时测试，10=倒车入库，11=侧方停车）
 *
 * 系统默认就是手动模式，发送 speed,angle 即可更新手动目标；
 * 未收到新的 speed,angle 时，Decision 会保持上一组目标。
 * ──────────────────────────────────────────────────────────── */

#include "comm_internal.h"

#include "bsp_uart.h"
#include "system_state_pool.h"
#include "vehicle_config.h"
#include "stm32g4xx_hal.h"     /* HAL_GetTick */

#include <ctype.h>
#include <stdlib.h>
#include <stddef.h>             /* size_t */

static ProtocolLineParser s_parser;
static BtCommand           s_pending;
static uint32_t            s_last_rx_ms;
static uint32_t            s_rx_count;

#define BT_MANUAL_SPEED_MIN        -100
#define BT_MANUAL_SPEED_MAX         100
#define BT_MANUAL_ANGLE_MIN_DEG    ((int32_t)(-VEHICLE_MANUAL_INPUT_MAX_DEG))
#define BT_MANUAL_ANGLE_MAX_DEG    ((int32_t)VEHICLE_MANUAL_INPUT_MAX_DEG)

static char to_upper_ascii(char c)
{
    if ((c >= 'a') && (c <= 'z'))
    {
        return (char)(c - ('a' - 'A'));
    }
    return c;
}

static bool text_equals_ignore_case(const char *lhs, const char *rhs)
{
    if ((lhs == 0) || (rhs == 0))
    {
        return false;
    }

    while ((*lhs != '\0') && (*rhs != '\0'))
    {
        if (to_upper_ascii(*lhs) != to_upper_ascii(*rhs))
        {
            return false;
        }
        ++lhs;
        ++rhs;
    }

    return (*lhs == '\0') && (*rhs == '\0');
}

static int16_t clamp_i16(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return (int16_t)min_value;
    if (value > max_value) return (int16_t)max_value;
    return (int16_t)value;
}

static bool parse_manual_move(const char *line, int16_t *speed, int16_t *angle)
{
    char *end;
    long parsed_speed;
    long parsed_angle;

    if ((line == 0) || (speed == 0) || (angle == 0))
    {
        return false;
    }

    parsed_speed = strtol(line, &end, 10);
    if ((end == line) || (*end != ','))
    {
        return false;
    }

    ++end;
    parsed_angle = strtol(end, &end, 10);
    while (*end != '\0' && isspace((unsigned char)*end))
    {
        ++end;
    }
    if (*end != '\0')
    {
        return false;
    }

    *speed = clamp_i16(parsed_speed,
                       BT_MANUAL_SPEED_MIN,
                       BT_MANUAL_SPEED_MAX);
    *angle = clamp_i16(parsed_angle,
                       BT_MANUAL_ANGLE_MIN_DEG,
                       BT_MANUAL_ANGLE_MAX_DEG);
    return true;
}

static bool parse_debug_command(const char *line, int16_t *enabled)
{
    char *end;
    long value;

    if ((line == 0) || (enabled == 0))
    {
        return false;
    }

    if (!((to_upper_ascii(line[0]) == 'D') &&
          (to_upper_ascii(line[1]) == 'E') &&
          (to_upper_ascii(line[2]) == 'B') &&
          (to_upper_ascii(line[3]) == 'U') &&
          (to_upper_ascii(line[4]) == 'G') &&
          (line[5] == ',')))
    {
        return false;
    }

    value = strtol(&line[6], &end, 10);
    while (*end != '\0' && isspace((unsigned char)*end))
    {
        ++end;
    }
    if (*end != '\0')
    {
        return false;
    }

    *enabled = (value != 0) ? 1 : 0;
    return true;
}

static bool parse_auto_command(const char *line, int16_t *task_id)
{
    char *end;
    long value;

    if ((line == 0) || (task_id == 0))
    {
        return false;
    }

    if (!((to_upper_ascii(line[0]) == 'A') &&
          (to_upper_ascii(line[1]) == 'U') &&
          (to_upper_ascii(line[2]) == 'T') &&
          (to_upper_ascii(line[3]) == 'O') &&
          (line[4] == ',')))
    {
        return false;
    }

    value = strtol(&line[5], &end, 10);
    while (*end != '\0' && isspace((unsigned char)*end))
    {
        ++end;
    }
    if (*end != '\0')
    {
        return false;
    }

    if ((value < 1) || (value > 3))
    {
        return false;
    }

    *task_id = (int16_t)value;
    return true;
}

static bool parse_test_command(const char *line, int16_t *test_id)
{
    char *end;
    long value;

    if ((line == 0) || (test_id == 0))
    {
        return false;
    }

    if (!((to_upper_ascii(line[0]) == 'T') &&
          (to_upper_ascii(line[1]) == 'E') &&
          (to_upper_ascii(line[2]) == 'S') &&
          (to_upper_ascii(line[3]) == 'T') &&
          (line[4] == ',')))
    {
        return false;
    }

    value = strtol(&line[5], &end, 10);
    while (*end != '\0' && isspace((unsigned char)*end))
    {
        ++end;
    }
    if (*end != '\0')
    {
        return false;
    }

    if ((value < 0) || (value > 99))
    {
        return false;
    }

    *test_id = (int16_t)value;
    return true;
}

static void set_pending_command(BtCommandType type, int16_t arg0, int16_t arg1, int16_t arg2)
{
    s_pending.type         = type;
    s_pending.arg0         = arg0;
    s_pending.arg1         = arg1;
    s_pending.arg2         = arg2;
    s_pending.timestamp_ms = HAL_GetTick();
    s_pending.valid        = true;
}

static void process_line(const char *line)
{
    int16_t speed;
    int16_t angle;
    int16_t debug_enabled;
    int16_t test_id;

    if (text_equals_ignore_case(line, "STOP"))
    {
        set_pending_command(BT_COMMAND_STOP, 0, 0, 0);
    }
    else if (parse_debug_command(line, &debug_enabled))
    {
        set_pending_command(BT_COMMAND_DEBUG_OUTPUT, debug_enabled, 0, 0);
    }
    else if (parse_test_command(line, &test_id))
    {
        set_pending_command(BT_COMMAND_START_TEST, test_id, 0, 0);
    }
    else if (parse_auto_command(line, &speed))
    {
        set_pending_command(BT_COMMAND_START_TASK, speed, 0, 0);
    }
    else if (parse_manual_move(line, &speed, &angle))
    {
        set_pending_command(BT_COMMAND_MANUAL_MOVE, speed, angle, 0);
    }
}

void BtLink_Task(void)
{
    uint8_t byte;
    char line[256];

    while (BspUart_ReadByte(BSP_UART_BT, &byte))
    {
        ++s_rx_count;
        s_last_rx_ms = HAL_GetTick();

        if (ProtocolLineParser_PushByte(&s_parser, byte) &&
            ProtocolLineParser_TakeLine(&s_parser, line, sizeof(line)))
        {
            process_line(line);
        }
    }
}

bool BtLink_TakeCommand(BtCommand *cmd)
{
    if ((cmd == 0) || !s_pending.valid) return false;
    *cmd = s_pending;
    s_pending.valid = false;
    return true;
}

void BtLink_GetStatus(bool *conn, uint32_t *last, uint32_t *cnt)
{
    if (conn)  *conn  = (s_last_rx_ms != 0U) &&
                        ((uint32_t)(HAL_GetTick() - s_last_rx_ms) < 2000U);
    if (last)  *last  = s_last_rx_ms;
    if (cnt)   *cnt   = s_rx_count;
}
