#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BUZZER_PATTERN_NONE = 0,
    BUZZER_PATTERN_OK,
    BUZZER_PATTERN_ERROR,
    BUZZER_PATTERN_START,
    BUZZER_PATTERN_OBSTACLE
} BuzzerPattern;

void BspBuzzer_Init(void);
void BspBuzzer_Set(bool on);
void BspBuzzer_Beep(uint16_t on_ms);
void BspBuzzer_Play(BuzzerPattern pattern);
void BspBuzzer_Task10ms(void);
bool BspBuzzer_IsActive(void);

#endif
