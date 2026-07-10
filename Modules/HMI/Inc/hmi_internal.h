#ifndef HMI_INTERNAL_H
#define HMI_INTERNAL_H

#include "system_state_pool.h"

void KeyService_Process10ms(SystemStatePool *pool);
void BuzzerService_Update(SystemStatePool *pool);
void LedService_Update(SystemStatePool *pool);

#endif /* HMI_INTERNAL_H */
