#include "HorizonLineGeometry.h"

#include <QtGlobal>
#include <QVector>

#include <cmath>

namespace
{
constexpr double Epsilon = 1e-9;

void appendPoint(QVector<QPointF>* points, double x, double y, int width, int height)
{
    if (points == nullptr || x < -Epsilon || y < -Epsilon
        || x > width - 1.0 + Epsilon || y > height - 1.0 + Epsilon)
    {
        return;
    }

    const QPointF point(qBound(0.0, x, width - 1.0),
                        qBound(0.0, y, height - 1.0));
    for (const QPointF& existing : *points)
    {
        if (QLineF(existing, point).length() < 1e-6)
        {
            return;
        }
    }
    points->push_back(point);
}
}

bool HorizonLineGeometry::clipThroughPoints(const QPointF& first,
                                            const QPointF& second,
                                            const QSize& imageSize,
                                            QLineF* segment)
{
    const double dx = second.x() - first.x();
    const double dy = second.y() - first.y();
    if (std::hypot(dx, dy) < Epsilon)
    {
        return false;
    }
    return clipCoefficients({dy, -dx, dx * first.y() - dy * first.x()},
                            imageSize,
                            segment);
}

bool HorizonLineGeometry::clipCoefficients(const std::array<double, 3>& line,
                                           const QSize& imageSize,
                                           QLineF* segment)
{
    if (segment == nullptr || imageSize.width() <= 0 || imageSize.height() <= 0)
    {
        return false;
    }

    const double a = line[0];
    const double b = line[1];
    const double c = line[2];
    if (std::hypot(a, b) < Epsilon)
    {
        return false;
    }

    QVector<QPointF> points;
    if (std::abs(b) > Epsilon)
    {
        appendPoint(&points, 0.0, -c / b, imageSize.width(), imageSize.height());
        appendPoint(&points,
                    imageSize.width() - 1.0,
                    -(a * (imageSize.width() - 1.0) + c) / b,
                    imageSize.width(),
                    imageSize.height());
    }
    if (std::abs(a) > Epsilon)
    {
        appendPoint(&points, -c / a, 0.0, imageSize.width(), imageSize.height());
        appendPoint(&points,
                    -(b * (imageSize.height() - 1.0) + c) / a,
                    imageSize.height() - 1.0,
                    imageSize.width(),
                    imageSize.height());
    }
    if (points.size() < 2)
    {
        return false;
    }

    double longest = -1.0;
    QLineF result;
    for (int firstIndex = 0; firstIndex < points.size(); ++firstIndex)
    {
        for (int secondIndex = firstIndex + 1; secondIndex < points.size(); ++secondIndex)
        {
            const QLineF candidate(points[firstIndex], points[secondIndex]);
            if (candidate.length() > longest)
            {
                longest = candidate.length();
                result = candidate;
            }
        }
    }
    *segment = result;
    return longest > Epsilon;
}
