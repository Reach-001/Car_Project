#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

typedef struct
{
    int32_t left_count;
    int32_t right_count;
    int32_t left_delta;
    int32_t right_delta;
} BspEncoderSample;

void BspEncoder_Init(void);
BspEncoderSample BspEncoder_Read(void);

#endif
