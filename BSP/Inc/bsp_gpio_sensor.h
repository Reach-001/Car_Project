#ifndef BSP_GPIO_SENSOR_H
#define BSP_GPIO_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/* ── GPIO-level sensor I/O (platform-abstracted header) ──
 *
 * Provides raw GPIO reads/writes for on-board sensors.
 * No filtering, debouncing, or unit conversion — that belongs in Modules.
 */

/* --- 5-way track sensor (PB3 TRACK_1, PB4 TRACK_2, PB5 TRACK_3,
 *                            PB8 TRACK_4, PB9 TRACK_5) --- */

typedef struct
{
    uint8_t bits;        /* bit[0..4] = sensor 1..5 */
    bool    sensor[5];   /* true = active (black line detected) */
} BspTrackRaw;

BspTrackRaw BspGpioSensor_ReadTrack(void);

/* --- HC-SR04 ultrasonic (PB2 TRIG output, PB11 ECHO input with EXTI) --- */

void BspGpioSensor_TrigSet(bool high);
bool BspGpioSensor_EchoRead(void);

#endif /* BSP_GPIO_SENSOR_H */
