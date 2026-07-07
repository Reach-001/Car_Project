#include "protocol.h"

void ProtocolLineParser_Init(ProtocolLineParser *parser)
{
    if (parser == 0)
    {
        return;
    }

    parser->length = 0U;
    parser->ready = false;
    parser->data[0] = '\0';
}

bool ProtocolLineParser_PushByte(ProtocolLineParser *parser, uint8_t byte)
{
    if ((parser == 0) || parser->ready)
    {
        return false;
    }

    if ((byte == '\n') || (byte == '\r'))
    {
        if (parser->length > 0U)
        {
            parser->data[parser->length] = '\0';
            parser->ready = true;
            return true;
        }
        return false;
    }

    if (parser->length >= (PROTOCOL_LINE_MAX_LEN - 1U))
    {
        ProtocolLineParser_Init(parser);
        return false;
    }

    parser->data[parser->length++] = (char)byte;
    return false;
}

bool ProtocolLineParser_TakeLine(ProtocolLineParser *parser, char *buffer, size_t buffer_size)
{
    if ((parser == 0) || (buffer == 0) || (buffer_size == 0U) || !parser->ready)
    {
        return false;
    }

    size_t i = 0U;
    while ((i + 1U) < buffer_size && parser->data[i] != '\0')
    {
        buffer[i] = parser->data[i];
        ++i;
    }
    buffer[i] = '\0';

    ProtocolLineParser_Init(parser);
    return true;
}
