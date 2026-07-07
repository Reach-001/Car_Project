#ifndef K230_LINK_H
#define K230_LINK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    K230_RESULT_NONE = 0,
    K230_RESULT_LINE,
    K230_RESULT_TARGET,
    K230_RESULT_MARKER,
    K230_RESULT_CUSTOM
} K230ResultType;

typedef struct
{
    K230ResultType type;
    int16_t value0;
    int16_t value1;
    int16_t value2;
    uint32_t timestamp_ms;
    bool valid;
} K230Result;

void K230Link_Init(void);
void K230Link_OnRxByte(uint8_t byte);
void K230Link_Task(void);
bool K230Link_GetLatestResult(K230Result *result);
void K230Link_SendStatus(void);

#endif
