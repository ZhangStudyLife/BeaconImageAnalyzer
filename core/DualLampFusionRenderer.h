#ifndef DUAL_LAMP_FUSION_RENDERER_H
#define DUAL_LAMP_FUSION_RENDERER_H

#include "JustFloatLog.h"

#include <QImage>

class DualLampFusionRenderer
{
public:
    static constexpr int ViewCount = 4;

    static QImage render(const JustFloatDualLampFusionFrame& frame, int viewIndex);
};

#endif
