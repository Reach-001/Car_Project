#ifndef SENSOR_INTERNAL_H
#define SENSOR_INTERNAL_H

#include "system_state_pool.h"

void Tracker_Task10ms(void);
void Tracker_WriteToPool(SystemStatePool *pool);

void Ultrasonic_Task10ms(void);
void Ultrasonic_WriteToPool(SystemStatePool *pool);

#endif /* SENSOR_INTERNAL_H */
