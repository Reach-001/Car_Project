#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_LINE_MAX_LEN 96U

typedef struct
{
    char data[PROTOCOL_LINE_MAX_LEN];
    uint16_t length;
    bool ready;
} ProtocolLineParser;

void ProtocolLineParser_Init(ProtocolLineParser *parser);
bool ProtocolLineParser_PushByte(ProtocolLineParser *parser, uint8_t byte);
bool ProtocolLineParser_TakeLine(ProtocolLineParser *parser, char *buffer, size_t buffer_size);

#endif
