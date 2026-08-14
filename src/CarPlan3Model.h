#ifndef CAR_PLAN_3_MODEL_H
#define CAR_PLAN_3_MODEL_H

#include "TelemetryProtocol.h"

#include <array>

class CarPlan3Model
{
public:
    CarPlan3Model();
    void reset();
    void process(TelemetryFrame* frame);

private:
    struct Track
    {
        bool valid = false;
        int gap = 0;
        int samples = 0;
        int farAge = 31;
        int suspectAge = 31;
        float x = 0.0f;
        float y = 0.0f;
    };

    std::array<std::array<Track, 4>, 3> m_tracks{};
};

#endif
