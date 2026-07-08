#ifndef BSP_GPIO_SENSOR_H
#define BSP_GPIO_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * GPIO 传感器驱动 —— 硬件抽象层头文件
 *
 * 封装所有通过 GPIO 直连的传感器原始读写操作。
 * 不在此层做滤波、单位换算或逻辑判断，这些属于 Modules 层职责。
 *
 * 传感器列表：
 *   五路循迹：PB3=TRACK_1, PB4=TRACK_2, PB5=TRACK_3,
 *             PB8=TRACK_4, PB9=TRACK_5（黑线 = 高电平）
 *   超声波：  PB2=HCSR04_TRIG（触发输出，10us 高脉冲）
 *             PB11=HCSR04_ECHO（回波输入，EXTI 双边沿中断）
 * ──────────────────────────────────────────────────────────── */

/* ── 循迹传感器（五路黑线检测） ── */

/** 五路循迹原始读数 */
typedef struct
{
    uint8_t bits;        /* bit[0..4] 对应传感器 1~5，1=检测到黑线 */
    bool    sensor[5];   /* sensor[0..4] 对应传感器 1~5              */
} BspTrackRaw;

/** 读取五路循迹当前电平 */
BspTrackRaw BspGpioSensor_ReadTrack(void);

/* ── 超声波 HC-SR04（TRIG 触发 + ECHO 回波） ── */

/** 设置 TRIG 引脚电平（高=触发脉冲中，低=空闲） */
void BspGpioSensor_TrigSet(bool high);

/** 读取 ECHO 引脚当前电平（供 ISR 回调中用） */
bool BspGpioSensor_EchoRead(void);

#endif /* BSP_GPIO_SENSOR_H */
