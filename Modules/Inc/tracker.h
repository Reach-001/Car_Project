#ifndef TRACKER_H
#define TRACKER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t bits;
    uint8_t active_count;
    int16_t error;
    bool line_detected;
    bool crossroad;
} TrackerState;

void Tracker_Init(void);
void Tracker_Task10ms(void);
TrackerState Tracker_GetState(void);
void Tracker_SetActiveHigh(bool active_high);

#endif
