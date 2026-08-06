#include <QElapsedTimer>
#include <QtTest>

#include <algorithm>
#include <array>
#include <cmath>

extern "C"
{
#include "image_down_horizon.h"
}

namespace
{
constexpr int kWidth = 188;
constexpr int kHeight = 120;
constexpr int kPointCount = 360;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kCenterX = 86.7802501f;
constexpr float kCenterY = 55.0531910f;
constexpr float kScale = 93.5f;
constexpr float kThetaK1 = 1.26656119f;
constexpr float kThetaK3 = -0.0392416268f;
constexpr float kThetaK5 = 0.143348613f;
constexpr float kRangeMm = 7000.0f;
constexpr float kHeightBiasMm = 239.727462f;
constexpr float kEdgeRadius = 1.27147706f;
constexpr float kEdgeTheta = 2.00610176f;
constexpr float kEdgeSlope = 2.94949888f;
constexpr float kStepCos = 0.999847695f;
constexpr float kStepSin = 0.0174524064f;

constexpr std::array<std::array<float, 3>, 3> kBodyToCamera{{
    {{-0.0156029708f, 0.999826714f, 0.0101532740f}},
    {{-0.999776778f, -0.0157452587f, 0.0140883038f}},
    {{0.0142457284f, -0.00993118815f, 0.999849204f}}
}};

struct LegacyBoundary
{
    std::array<float, kPointCount> x{};
    std::array<float, kPointCount> y{};
    std::array<float, kWidth> top{};
    std::array<float, kWidth> bottom{};
    std::array<unsigned char, kWidth> columnValid{};
    std::array<unsigned char, kWidth> columnState{};
};

struct ImplicitBoundary
{
    std::array<float, kWidth> top{};
    std::array<float, kWidth> bottom{};
    std::array<unsigned char, kWidth> columnValid{};
    std::array<unsigned char, kWidth * kHeight> contains{};
};

ImplicitBoundary captureImplicitBoundary()
{
    ImplicitBoundary boundary;
    std::copy_n(g_image_down_horizon_top_y, kWidth, boundary.top.begin());
    std::copy_n(g_image_down_horizon_bottom_y, kWidth, boundary.bottom.begin());
    std::copy_n(g_image_down_horizon_column_valid,
                kWidth,
                boundary.columnValid.begin());
    for (int y = 0; y < kHeight; ++y)
    {
        for (int x = 0; x < kWidth; ++x)
        {
            boundary.contains[static_cast<std::size_t>(y * kWidth + x)] =
                image_down_horizon_contains(static_cast<float>(x),
                                            static_cast<float>(y),
                                            0.0f);
        }
    }
    return boundary;
}

float legacyRadius(float theta)
{
    if (theta > kEdgeTheta)
    {
        return kEdgeRadius + (theta - kEdgeTheta) / kEdgeSlope;
    }
    float low = 0.0f;
    float high = kEdgeRadius;
    for (int iteration = 0; iteration < 28; ++iteration)
    {
        const float radius = (low + high) * 0.5f;
        const float radius2 = radius * radius;
        const float mapped = radius
            * (kThetaK1 + radius2 * (kThetaK3 + radius2 * kThetaK5));
        if (mapped < theta)
        {
            low = radius;
        }
        else
        {
            high = radius;
        }
    }
    return (low + high) * 0.5f;
}

void addLegacyColumn(LegacyBoundary* boundary, int x, float y)
{
    if (boundary->columnValid[static_cast<std::size_t>(x)] == 0U)
    {
        boundary->top[static_cast<std::size_t>(x)] = y;
        boundary->bottom[static_cast<std::size_t>(x)] = y;
        boundary->columnValid[static_cast<std::size_t>(x)] = 1U;
        return;
    }
    boundary->top[static_cast<std::size_t>(x)] =
        std::min(boundary->top[static_cast<std::size_t>(x)], y);
    boundary->bottom[static_cast<std::size_t>(x)] =
        std::max(boundary->bottom[static_cast<std::size_t>(x)], y);
}

bool legacyPointInside(const LegacyBoundary& boundary, float x, float y)
{
    bool inside = false;
    for (int index = 0; index < kPointCount; ++index)
    {
        const int next = (index + 1) % kPointCount;
        const float x0 = boundary.x[static_cast<std::size_t>(index)];
        const float y0 = boundary.y[static_cast<std::size_t>(index)];
        const float x1 = boundary.x[static_cast<std::size_t>(next)];
        const float y1 = boundary.y[static_cast<std::size_t>(next)];
        if (((y0 > y) != (y1 > y))
            && x < (x1 - x0) * (y - y0) / (y1 - y0) + x0)
        {
            inside = !inside;
        }
    }
    return inside;
}

LegacyBoundary legacyBoundary(float rollDeg, float pitchDeg, float heightMm)
{
    LegacyBoundary boundary;
    const float roll = rollDeg * kPi / 180.0f;
    const float pitch = pitchDeg * kPi / 180.0f;
    const std::array<float, 3> gravity{{
        -std::sin(pitch),
        std::sin(roll) * std::cos(pitch),
        std::cos(roll) * std::cos(pitch)
    }};
    std::array<float, 3> u{{0.0f, gravity[2], -gravity[1]}};
    float uNorm = std::hypot(u[1], u[2]);
    if (uNorm < 1.0e-6f)
    {
        u = {{-gravity[2], 0.0f, gravity[0]}};
        uNorm = std::hypot(u[0], u[2]);
    }
    for (float& value : u)
    {
        value /= uNorm;
    }
    const std::array<float, 3> v{{
        gravity[1] * u[2] - gravity[2] * u[1],
        gravity[2] * u[0] - gravity[0] * u[2],
        gravity[0] * u[1] - gravity[1] * u[0]
    }};
    const float height = heightMm + kHeightBiasMm;
    float boundaryCos = 1.0f;
    float boundarySin = 0.0f;

    for (int index = 0; index < kPointCount; ++index)
    {
        std::array<float, 3> direction{};
        for (int axis = 0; axis < 3; ++axis)
        {
            direction[static_cast<std::size_t>(axis)] = height * gravity[axis]
                + kRangeMm * (boundaryCos * u[axis] + boundarySin * v[axis]);
        }
        const float norm = std::sqrt(direction[0] * direction[0]
                                     + direction[1] * direction[1]
                                     + direction[2] * direction[2]);
        for (float& value : direction)
        {
            value /= norm;
        }
        std::array<float, 3> camera{};
        for (int row = 0; row < 3; ++row)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                camera[static_cast<std::size_t>(row)] +=
                    kBodyToCamera[static_cast<std::size_t>(row)]
                                 [static_cast<std::size_t>(axis)]
                    * direction[static_cast<std::size_t>(axis)];
            }
        }
        camera[2] = std::clamp(camera[2], -1.0f, 1.0f);
        const float theta = std::acos(camera[2]);
        const float lateral = std::hypot(camera[0], camera[1]);
        const float radius = legacyRadius(theta);
        if (lateral <= 1.0e-6f)
        {
            boundary.x[static_cast<std::size_t>(index)] = kCenterX;
            boundary.y[static_cast<std::size_t>(index)] = kCenterY;
        }
        else
        {
            const float factor = kScale * radius / lateral;
            boundary.x[static_cast<std::size_t>(index)] = kCenterX + camera[0] * factor;
            boundary.y[static_cast<std::size_t>(index)] = kCenterY + camera[1] * factor;
        }
        const float nextCos = boundaryCos * kStepCos - boundarySin * kStepSin;
        boundarySin = boundarySin * kStepCos + boundaryCos * kStepSin;
        boundaryCos = nextCos;
    }

    for (int index = 0; index < kPointCount; ++index)
    {
        const int next = (index + 1) % kPointCount;
        const float x0 = boundary.x[static_cast<std::size_t>(index)];
        const float y0 = boundary.y[static_cast<std::size_t>(index)];
        const float x1 = boundary.x[static_cast<std::size_t>(next)];
        const float y1 = boundary.y[static_cast<std::size_t>(next)];
        const float dx = x1 - x0;
        if (std::abs(dx) < 1.0e-6f)
        {
            const int x = static_cast<int>(x0 + (x0 >= 0.0f ? 0.5f : -0.5f));
            if (x >= 0 && x < kWidth)
            {
                addLegacyColumn(&boundary, x, y0);
                addLegacyColumn(&boundary, x, y1);
            }
            continue;
        }
        const int first = std::max(0, static_cast<int>(std::ceil(std::min(x0, x1))));
        const int last = std::min(kWidth - 1,
                                  static_cast<int>(std::floor(std::max(x0, x1))));
        for (int x = first; x <= last; ++x)
        {
            const float ratio = (static_cast<float>(x) - x0) / dx;
            if (ratio >= 0.0f && ratio <= 1.0f)
            {
                addLegacyColumn(&boundary, x, y0 + ratio * (y1 - y0));
            }
        }
    }
    for (int x = 0; x < kWidth; ++x)
    {
        if (boundary.columnValid[static_cast<std::size_t>(x)] != 0U)
        {
            boundary.columnState[static_cast<std::size_t>(x)] = 1U;
        }
        else if (legacyPointInside(boundary, static_cast<float>(x), kHeight * 0.5f))
        {
            boundary.columnState[static_cast<std::size_t>(x)] = 2U;
        }
    }
    return boundary;
}

bool legacyContains(const LegacyBoundary& boundary, int x, int y)
{
    const unsigned char state = boundary.columnState[static_cast<std::size_t>(x)];
    if (state == 2U)
    {
        return true;
    }
    if (state != 1U)
    {
        return false;
    }
    return static_cast<float>(y) >= boundary.top[static_cast<std::size_t>(x)]
        && static_cast<float>(y) <= boundary.bottom[static_cast<std::size_t>(x)];
}

bool nearLegacyBoundary(const LegacyBoundary& boundary, int x, int y)
{
    if (boundary.columnValid[static_cast<std::size_t>(x)] == 0U)
    {
        return false;
    }
    return std::abs(static_cast<float>(y) - boundary.top[static_cast<std::size_t>(x)]) <= 0.25f
        || std::abs(static_cast<float>(y) - boundary.bottom[static_cast<std::size_t>(x)]) <= 0.25f;
}
}

class DownHorizonImplicitTests : public QObject
{
    Q_OBJECT

private slots:
    void trackedRootsMatchFullScan();
    void trackedRootsMatchFullScanOnDenseGrid();
    void matchesLegacyProjectionAcrossCalibrationRange();
    void coversColumnStatesAndValidityFlags();
    void resolvesTangentWithoutPolygonChordBias();
    void keepsLegacyPointInterfaceLazyAndCompatible();
    void isFasterThanLegacyProjection();
};

void DownHorizonImplicitTests::trackedRootsMatchFullScan()
{
    image_down_horizon_init();
    float maximumRootError = 0.0f;
    int comparedFrames = 0;
    int comparedRoots = 0;
    const std::array<float, 9> rolls{{
        -36.0f, -27.0f, -18.0f, -9.0f, 0.0f,
        9.0f, 18.0f, 27.0f, 34.0f
    }};
    const std::array<float, 9> pitches{{
        -34.0f, -25.0f, -16.0f, -8.0f, 0.0f,
        8.0f, 16.0f, 25.0f, 37.0f
    }};
    const std::array<float, 5> heights{{520.0f, 650.0f, 850.0f, 1050.0f, 1300.0f}};

    for (float roll : rolls)
    {
        for (float pitch : pitches)
        {
            for (float height : heights)
            {
                image_down_horizon_test_force_full_scan(1U);
                image_down_horizon_update(roll, pitch, height, 1U, 1U);
                const ImplicitBoundary full = captureImplicitBoundary();

                image_down_horizon_test_force_full_scan(0U);
                image_down_horizon_update(roll, pitch, height, 1U, 1U);
                const ImplicitBoundary tracked = captureImplicitBoundary();

                for (int x = 0; x < kWidth; ++x)
                {
                    const std::size_t column = static_cast<std::size_t>(x);
                    QVERIFY2(full.columnValid[column] == tracked.columnValid[column],
                             qPrintable(QStringLiteral(
                                 "column-valid roll=%1 pitch=%2 height=%3 x=%4")
                                 .arg(roll).arg(pitch).arg(height).arg(x)));
                    if (full.columnValid[column] != 0U)
                    {
                        const float topError = std::abs(full.top[column] - tracked.top[column]);
                        const float bottomError = std::abs(
                            full.bottom[column] - tracked.bottom[column]);
                        maximumRootError = std::max(
                            maximumRootError, std::max(topError, bottomError));
                        comparedRoots += 2;
                        QVERIFY2(topError <= 1.0e-4f && bottomError <= 1.0e-4f,
                                 qPrintable(QStringLiteral(
                                     "root roll=%1 pitch=%2 height=%3 x=%4 top=%5 bottom=%6")
                                     .arg(roll).arg(pitch).arg(height).arg(x)
                                     .arg(topError, 0, 'g', 8)
                                     .arg(bottomError, 0, 'g', 8)));
                    }
                }
                for (std::size_t index = 0; index < full.contains.size(); ++index)
                {
                    QVERIFY2(full.contains[index] == tracked.contains[index],
                             qPrintable(QStringLiteral(
                                 "contains roll=%1 pitch=%2 height=%3 x=%4 y=%5")
                                 .arg(roll).arg(pitch).arg(height)
                                 .arg(static_cast<int>(index % kWidth))
                                 .arg(static_cast<int>(index / kWidth))));
                }
                ++comparedFrames;
            }
        }
    }
    image_down_horizon_test_force_full_scan(0U);
    QCOMPARE(comparedFrames, 405);
    QVERIFY(comparedRoots > 10000);
    qInfo() << "tracked/full maximum root error" << maximumRootError << "pixels";
}

void DownHorizonImplicitTests::trackedRootsMatchFullScanOnDenseGrid()
{
    constexpr std::array<int, 3> sampleRows{{0, kHeight / 2, kHeight - 1}};
    std::array<float, kWidth> fullTop{};
    std::array<float, kWidth> fullBottom{};
    std::array<unsigned char, kWidth> fullValid{};
    std::array<unsigned char, kWidth * sampleRows.size()> fullContains{};
    int comparedFrames = 0;

    image_down_horizon_init();
    for (int roll = -37; roll <= 33; roll += 5)
    {
        for (int pitch = -34; pitch <= 36; pitch += 5)
        {
            for (int height = 520; height <= 1270; height += 150)
            {
                image_down_horizon_test_force_full_scan(1U);
                image_down_horizon_update(static_cast<float>(roll),
                                          static_cast<float>(pitch),
                                          static_cast<float>(height),
                                          1U,
                                          1U);
                std::copy_n(g_image_down_horizon_top_y, kWidth, fullTop.begin());
                std::copy_n(g_image_down_horizon_bottom_y, kWidth, fullBottom.begin());
                std::copy_n(g_image_down_horizon_column_valid,
                            kWidth,
                            fullValid.begin());
                for (std::size_t rowIndex = 0; rowIndex < sampleRows.size(); ++rowIndex)
                {
                    for (int x = 0; x < kWidth; ++x)
                    {
                        fullContains[rowIndex * kWidth + static_cast<std::size_t>(x)] =
                            image_down_horizon_contains(
                                static_cast<float>(x),
                                static_cast<float>(sampleRows[rowIndex]),
                                0.0f);
                    }
                }

                image_down_horizon_test_force_full_scan(0U);
                image_down_horizon_update(static_cast<float>(roll),
                                          static_cast<float>(pitch),
                                          static_cast<float>(height),
                                          1U,
                                          1U);
                for (int x = 0; x < kWidth; ++x)
                {
                    const std::size_t column = static_cast<std::size_t>(x);
                    QVERIFY2(fullValid[column] == g_image_down_horizon_column_valid[x],
                             qPrintable(QStringLiteral(
                                 "dense valid roll=%1 pitch=%2 height=%3 x=%4")
                                 .arg(roll).arg(pitch).arg(height).arg(x)));
                    if (fullValid[column] != 0U)
                    {
                        QVERIFY2(std::abs(fullTop[column] -
                                          g_image_down_horizon_top_y[x]) <= 1.0e-4f &&
                                     std::abs(fullBottom[column] -
                                              g_image_down_horizon_bottom_y[x]) <= 1.0e-4f,
                                 qPrintable(QStringLiteral(
                                     "dense root roll=%1 pitch=%2 height=%3 x=%4")
                                     .arg(roll).arg(pitch).arg(height).arg(x)));
                    }
                    for (std::size_t rowIndex = 0;
                         rowIndex < sampleRows.size();
                         ++rowIndex)
                    {
                        const unsigned char actual = image_down_horizon_contains(
                            static_cast<float>(x),
                            static_cast<float>(sampleRows[rowIndex]),
                            0.0f);
                        QVERIFY2(fullContains[rowIndex * kWidth + column] == actual,
                                 qPrintable(QStringLiteral(
                                     "dense contains roll=%1 pitch=%2 height=%3 x=%4 y=%5")
                                     .arg(roll).arg(pitch).arg(height).arg(x)
                                     .arg(sampleRows[rowIndex])));
                    }
                }
                ++comparedFrames;
            }
        }
    }
    image_down_horizon_test_force_full_scan(0U);
    QCOMPARE(comparedFrames, 1350);
}

void DownHorizonImplicitTests::matchesLegacyProjectionAcrossCalibrationRange()
{
    image_down_horizon_init();
    float maximumError = 0.0f;
    int comparedRoots = 0;
    const std::array<float, 5> rolls{{-30.0f, -15.0f, 0.0f, 15.0f, 30.0f}};
    const std::array<float, 5> pitches{{-30.0f, -15.0f, 0.0f, 15.0f, 30.0f}};
    const std::array<float, 3> heights{{550.0f, 900.0f, 1250.0f}};

    for (float roll : rolls)
    {
        for (float pitch : pitches)
        {
            for (float height : heights)
            {
                const LegacyBoundary legacy = legacyBoundary(roll, pitch, height);
                image_down_horizon_update(roll, pitch, height, 1U, 1U);
                QCOMPARE(g_image_down_horizon_valid, quint8(1U));

                for (int x = 0; x < kWidth; ++x)
                {
                    const float legacyTop = legacy.top[static_cast<std::size_t>(x)];
                    const float legacyBottom = legacy.bottom[static_cast<std::size_t>(x)];
                    const bool topVisible = legacy.columnValid[static_cast<std::size_t>(x)] != 0U
                        && legacyTop >= 0.0f && legacyTop <= static_cast<float>(kHeight - 1);
                    const bool bottomVisible = legacy.columnValid[static_cast<std::size_t>(x)] != 0U
                        && legacyBottom >= 0.0f && legacyBottom <= static_cast<float>(kHeight - 1);
                    if (topVisible || bottomVisible)
                    {
                        QVERIFY(g_image_down_horizon_column_valid[x] != 0U);
                    }
                    if (topVisible)
                    {
                        const float error = std::abs(g_image_down_horizon_top_y[x] - legacyTop);
                        maximumError = std::max(maximumError, error);
                        ++comparedRoots;
                        QVERIFY2(error <= 0.25f, qPrintable(QStringLiteral(
                            "top roll=%1 pitch=%2 height=%3 x=%4 error=%5")
                            .arg(roll).arg(pitch).arg(height).arg(x).arg(error)));
                    }
                    if (bottomVisible)
                    {
                        const float error = std::abs(g_image_down_horizon_bottom_y[x] - legacyBottom);
                        maximumError = std::max(maximumError, error);
                        ++comparedRoots;
                        QVERIFY2(error <= 0.25f, qPrintable(QStringLiteral(
                            "bottom roll=%1 pitch=%2 height=%3 x=%4 error=%5")
                            .arg(roll).arg(pitch).arg(height).arg(x).arg(error)));
                    }
                    for (int y = 0; y < kHeight; ++y)
                    {
                        const bool expected = legacyContains(legacy, x, y);
                        const bool actual = image_down_horizon_contains(
                            static_cast<float>(x), static_cast<float>(y), 0.0f) != 0U;
                        QVERIFY2(expected == actual || nearLegacyBoundary(legacy, x, y),
                                 qPrintable(QStringLiteral(
                                     "contains roll=%1 pitch=%2 height=%3 x=%4 y=%5")
                                     .arg(roll).arg(pitch).arg(height).arg(x).arg(y)));
                    }
                }
            }
        }
    }
    QVERIFY(comparedRoots > 1000);
    qInfo() << "implicit maximum visible-root error" << maximumError << "pixels";
}

void DownHorizonImplicitTests::coversColumnStatesAndValidityFlags()
{
    image_down_horizon_init();
    image_down_horizon_update(0.0f, 0.0f, 1000.0f, 0U, 1U);
    QCOMPARE(g_image_down_horizon_valid, quint8(0U));
    image_down_horizon_update(0.0f, 0.0f, 1000.0f, 1U, 0U);
    QCOMPARE(g_image_down_horizon_valid, quint8(0U));

    image_down_horizon_update(50.0f, 0.0f, 1000.0f, 1U, 1U);
    QCOMPARE(g_image_down_horizon_valid, quint8(1U));
    QCOMPARE(g_image_down_horizon_extrapolated, quint8(1U));
    QCOMPARE(image_down_horizon_contains(0.0f, 0.0f, 0.0f), quint8(1U));

    bool foundDoubleRoot = false;
    bool foundTopSingleRoot = false;
    bool foundBottomSingleRoot = false;
    bool foundInsideColumn = false;
    bool foundOutsideColumn = false;
    for (float roll : {-30.0f, 0.0f, 30.0f})
    {
        for (float pitch : {-30.0f, 0.0f, 30.0f})
        {
            image_down_horizon_update(roll, pitch, 900.0f, 1U, 1U);
            for (int x = 0; x < kWidth; ++x)
            {
                if (g_image_down_horizon_column_valid[x] != 0U)
                {
                    const float top = g_image_down_horizon_top_y[x];
                    const float bottom = g_image_down_horizon_bottom_y[x];
                    foundDoubleRoot |= top >= 0.0f && bottom <= static_cast<float>(kHeight - 1);
                    foundTopSingleRoot |= top < 0.0f && bottom >= 0.0f
                        && bottom <= static_cast<float>(kHeight - 1);
                    foundBottomSingleRoot |= top >= 0.0f
                        && top <= static_cast<float>(kHeight - 1)
                        && bottom > static_cast<float>(kHeight - 1);
                }
                else
                {
                    const bool inside = image_down_horizon_contains(
                        static_cast<float>(x), kHeight * 0.5f, 0.0f) != 0U;
                    foundInsideColumn |= inside;
                    foundOutsideColumn |= !inside;
                }
            }
        }
    }
    QVERIFY(foundDoubleRoot);
    QVERIFY(foundTopSingleRoot);
    QVERIFY(foundBottomSingleRoot);
    QVERIFY(foundInsideColumn);
    QVERIFY(foundOutsideColumn);
}

void DownHorizonImplicitTests::resolvesTangentWithoutPolygonChordBias()
{
    constexpr float roll = -1.68191147f;
    constexpr float pitch = 0.881824315f;
    constexpr float height = 1227.09424f;
    constexpr int tangentColumn = 179;
    constexpr float exactTop = 55.817f;
    constexpr float exactBottom = 57.367f;

    image_down_horizon_init();
    image_down_horizon_update(roll, pitch, height, 1U, 1U);
    QVERIFY(g_image_down_horizon_column_valid[tangentColumn] != 0U);
    QVERIFY(std::abs(g_image_down_horizon_top_y[tangentColumn] - exactTop) <= 0.25f);
    QVERIFY(std::abs(g_image_down_horizon_bottom_y[tangentColumn] - exactBottom) <= 0.25f);

    const LegacyBoundary legacy = legacyBoundary(roll, pitch, height);
    QVERIFY(std::abs(legacy.bottom[static_cast<std::size_t>(tangentColumn)]
                     - exactBottom) > 1.0f);

    image_down_horizon_update(-1.19218278f, 1.50740671f, 1147.8175f, 1U, 1U);
    QVERIFY(g_image_down_horizon_column_valid[180] != 0U);
    QVERIFY(std::abs(g_image_down_horizon_top_y[180] - 56.343f) <= 0.25f);
    QVERIFY(std::abs(g_image_down_horizon_bottom_y[180] - 57.875f) <= 0.25f);

    image_down_horizon_update(-5.30925035f, -0.385299623f, 1044.28308f, 1U, 1U);
    QVERIFY(g_image_down_horizon_column_valid[177] != 0U);
    QVERIFY(std::abs(g_image_down_horizon_top_y[177] - 55.152f) <= 0.25f);
    QVERIFY(std::abs(g_image_down_horizon_bottom_y[177] - 55.904f) <= 0.25f);
}

void DownHorizonImplicitTests::keepsLegacyPointInterfaceLazyAndCompatible()
{
    image_down_horizon_init();
    constexpr float roll = 12.0f;
    constexpr float pitch = -18.0f;
    constexpr float height = 980.0f;
    const LegacyBoundary legacy = legacyBoundary(roll, pitch, height);
    image_down_horizon_update(roll, pitch, height, 1U, 1U);

    for (int index = 0; index < kPointCount; ++index)
    {
        float x = 0.0f;
        float y = 0.0f;
        QCOMPARE(image_down_horizon_get_point(static_cast<uint16>(index), &x, &y),
                 quint8(1U));
        QVERIFY(std::abs(x - legacy.x[static_cast<std::size_t>(index)]) <= 1.0e-4f);
        QVERIFY(std::abs(y - legacy.y[static_cast<std::size_t>(index)]) <= 1.0e-4f);
    }
}

void DownHorizonImplicitTests::isFasterThanLegacyProjection()
{
    constexpr int iterations = 2000;
    volatile float checksum = 0.0f;
    QElapsedTimer timer;

    image_down_horizon_init();
    for (int index = 0; index < 20; ++index)
    {
        image_down_horizon_update(static_cast<float>(index % 21 - 10),
                                  static_cast<float>(index % 17 - 8),
                                  900.0f + static_cast<float>(index % 7) * 20.0f,
                                  1U,
                                  1U);
        const LegacyBoundary legacy = legacyBoundary(
            static_cast<float>(index % 21 - 10),
            static_cast<float>(index % 17 - 8),
            900.0f + static_cast<float>(index % 7) * 20.0f);
        checksum = checksum + legacy.top[static_cast<std::size_t>(index % kWidth)];
    }

    timer.start();
    image_down_horizon_test_force_full_scan(0U);
    for (int index = 0; index < iterations; ++index)
    {
        image_down_horizon_update(static_cast<float>(index % 61 - 30),
                                  static_cast<float>(index % 57 - 28),
                                  600.0f + static_cast<float>(index % 36) * 20.0f,
                                  1U,
                                  1U);
        checksum = checksum + g_image_down_horizon_top_y[index % kWidth];
    }
    const qint64 trackedNs = timer.nsecsElapsed();

    timer.restart();
    image_down_horizon_test_force_full_scan(1U);
    for (int index = 0; index < iterations; ++index)
    {
        image_down_horizon_update(static_cast<float>(index % 61 - 30),
                                  static_cast<float>(index % 57 - 28),
                                  600.0f + static_cast<float>(index % 36) * 20.0f,
                                  1U,
                                  1U);
        checksum = checksum + g_image_down_horizon_top_y[index % kWidth];
    }
    const qint64 fullScanNs = timer.nsecsElapsed();
    image_down_horizon_test_force_full_scan(0U);

    timer.restart();
    for (int index = 0; index < iterations; ++index)
    {
        const LegacyBoundary legacy = legacyBoundary(
            static_cast<float>(index % 61 - 30),
            static_cast<float>(index % 57 - 28),
            600.0f + static_cast<float>(index % 36) * 20.0f);
        checksum = checksum + legacy.top[static_cast<std::size_t>(index % kWidth)];
    }
    const qint64 legacyNs = timer.nsecsElapsed();
    QVERIFY(std::isfinite(checksum));
    qInfo() << "tracked average ns" << trackedNs / iterations
            << "full-scan average ns" << fullScanNs / iterations
            << "legacy average ns" << legacyNs / iterations
            << "tracked/full ratio"
            << static_cast<double>(trackedNs) / static_cast<double>(fullScanNs)
            << "tracked/legacy ratio"
            << static_cast<double>(trackedNs) / static_cast<double>(legacyNs);
    QVERIFY2(trackedNs * 5 <= fullScanNs * 4,
             qPrintable(QStringLiteral("tracked=%1ns full-scan=%2ns")
                            .arg(trackedNs / iterations)
                            .arg(fullScanNs / iterations)));
    QVERIFY2(trackedNs * 2 <= legacyNs,
             qPrintable(QStringLiteral("tracked=%1ns legacy=%2ns")
                            .arg(trackedNs / iterations)
                            .arg(legacyNs / iterations)));
}

QTEST_MAIN(DownHorizonImplicitTests)
#include "DownHorizonImplicitTests.moc"
