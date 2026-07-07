#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BSP_KEY_1 = 0,
    BSP_KEY_2,
    BSP_KEY_3,
    BSP_KEY_COUNT
} BspKeyId;

typedef struct
{
    bool pressed;
    bool pressed_event;
    bool released_event;
    bool clicked_event;
    uint32_t pressed_time_ms;
} BspKeyInfo;

void BspKey_Init(void);
void BspKey_Task10ms(void);
bool BspKey_IsPressed(BspKeyId key);
bool BspKey_TakePressedEvent(BspKeyId key);
bool BspKey_TakeReleasedEvent(BspKeyId key);
bool BspKey_TakeClickedEvent(BspKeyId key);
BspKeyInfo BspKey_GetInfo(BspKeyId key);

#endif
