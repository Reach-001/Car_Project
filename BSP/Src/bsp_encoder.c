#include "bsp_encoder.h"

#include "tim.h"

static int32_t s_last_left;
static int32_t s_last_right;

void BspEncoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    s_last_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_last_right = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

BspEncoderSample BspEncoder_Read(void)
{
    int32_t left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t right = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    BspEncoderSample sample;
    sample.left_count = left;
    sample.right_count = right;
    sample.left_delta = left - s_last_left;
    sample.right_delta = right - s_last_right;

    s_last_left = left;
    s_last_right = right;
    return sample;
}
