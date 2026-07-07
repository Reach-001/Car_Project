#ifndef BSP_IO_H
#define BSP_IO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool key1;
    bool key2;
    bool key3;
} BspKeyState;

typedef struct
{
    uint8_t bits;
    bool sensor[5];
} BspTrackState;

void BspIo_Init(void);
void BspIo_SetStateLed(bool on);
void BspIo_SetBuzzer(bool on);
BspKeyState BspIo_ReadKeys(void);
BspTrackState BspIo_ReadTrack(void);

#endif
