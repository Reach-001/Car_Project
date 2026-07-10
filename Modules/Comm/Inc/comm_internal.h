#ifndef COMM_INTERNAL_H
#define COMM_INTERNAL_H

#include "system_state_pool.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_LINE_MAX_LEN 96U

typedef struct
{
    char     data[PROTOCOL_LINE_MAX_LEN];
    uint16_t length;
    bool     ready;
} ProtocolLineParser;

void ProtocolLineParser_Init(ProtocolLineParser *parser);
bool ProtocolLineParser_PushByte(ProtocolLineParser *parser, uint8_t byte);
bool ProtocolLineParser_TakeLine(ProtocolLineParser *parser, char *buffer, size_t size);

void BtLink_Task(void);
bool BtLink_TakeCommand(BtCommand *command);
void BtLink_GetStatus(bool *connected, uint32_t *last_rx_ms, uint32_t *rx_count);

void K230Link_Task(void);
bool K230Link_GetLatestResult(K230Result *result);
void K230Link_GetStatus(bool *connected, uint32_t *last_rx_ms, uint32_t *rx_count);

#endif /* COMM_INTERNAL_H */
