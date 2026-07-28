#ifndef HORIZON_LINE_GEOMETRY_H
#define HORIZON_LINE_GEOMETRY_H

#include <QLineF>
#include <QPointF>
#include <QSize>

#include <array>

namespace HorizonLineGeometry
{
bool clipThroughPoints(const QPointF& first,
                       const QPointF& second,
                       const QSize& imageSize,
                       QLineF* segment);
bool clipCoefficients(const std::array<double, 3>& line,
                      const QSize& imageSize,
                      QLineF* segment);
}

#endif
