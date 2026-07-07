#include "tracker.h"

#include "bsp_io.h"

static TrackerState s_state;
static bool s_active_high = true;

void Tracker_Init(void)
{
    s_state.bits = 0U;
    s_state.active_count = 0U;
    s_state.error = 0;
    s_state.line_detected = false;
    s_state.crossroad = false;
}

void Tracker_SetActiveHigh(bool active_high)
{
    s_active_high = active_high;
}

void Tracker_Task10ms(void)
{
    static const int16_t weights[5] = {-2000, -1000, 0, 1000, 2000};
    BspTrackState raw = BspIo_ReadTrack();
    int32_t weighted_sum = 0;
    uint8_t active_count = 0U;
    uint8_t bits = 0U;

    for (uint32_t i = 0U; i < 5U; ++i)
    {
        bool active = s_active_high ? raw.sensor[i] : !raw.sensor[i];
        if (active)
        {
            bits |= (uint8_t)(1U << i);
            weighted_sum += weights[i];
            ++active_count;
        }
    }

    s_state.bits = bits;
    s_state.active_count = active_count;
    s_state.line_detected = active_count > 0U;
    s_state.crossroad = active_count >= 4U;

    if (active_count > 0U)
    {
        s_state.error = (int16_t)(weighted_sum / active_count);
    }
}

TrackerState Tracker_GetState(void)
{
    return s_state;
}
