#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ULTRASONIC_STATUS_IDLE = 0,
    ULTRASONIC_STATUS_WAIT_ECHO,
    ULTRASONIC_STATUS_READY,
    ULTRASONIC_STATUS_TIMEOUT
} UltrasonicStatus;

typedef struct
{
    UltrasonicStatus status;
    uint16_t distance_mm;
    uint32_t echo_us;
    uint32_t last_update_ms;
    bool valid;
} UltrasonicSample;

void Ultrasonic_Init(void);
void Ultrasonic_Task10ms(void);
void Ultrasonic_Trigger(void);
void Ultrasonic_OnEchoEdge(void);
UltrasonicSample Ultrasonic_GetSample(void);
bool Ultrasonic_IsObstacleNear(uint16_t threshold_mm);

#endif
