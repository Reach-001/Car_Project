#ifndef DEBUG_TRACE_H
#define DEBUG_TRACE_H

#include "system_state_pool.h"

void DebugTrace_Init(void);
void DebugTrace_Task100ms(SystemStatePool *pool);

#endif /* DEBUG_TRACE_H */
