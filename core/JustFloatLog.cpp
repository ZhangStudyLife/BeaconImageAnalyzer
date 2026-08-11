#include "JustFloatLog.h"

#include <QFile>
#include <QFileInfo>
#include <QLocale>
#include <QStringList>
#include <QTextStream>
#include <QtEndian>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
constexpr int SingleLampRoiPayloadValueCount = JustFloatLog::SingleLampRoiChannelCount - 1;
constexpr int LegacyPayloadValueCount = JustFloatLog::LegacyChannelCount - 1;
constexpr int MotionPayloadValueCount = JustFloatLog::MotionChannelCount - 1;
constexpr int FusedPayloadValueCount = JustFloatLog::LegacyFusedChannelCount - 1;
constexpr int DualLampFusionPayloadValueCount = JustFloatLog::DualLampFusionChannelCount - 1;
constexpr double InvalidSentinel = -900.0;
constexpr double SchemaTolerance = 0.5;
constexpr quint32 Packed24Maximum = 0x00ffffffU;
constexpr float SingleLampWidthScale = 4.0f;
constexpr float SingleLampLengthScale = 2.0f;
constexpr float SingleLampAngleStepDeg = 2.0f;
constexpr float SingleLampBeaconDistanceStep = 4.0f;

using ValueArray = std::array<double, JustFloatLog::ChannelCount>;
using HeaderMap = std::array<int, JustFloatLog::ChannelCount>;

struct ParsedValues
{
    ValueArray values{};
    JustFloatLogLayout layout = JustFloatLogLayout::Legacy;
    bool hasMotionData = false;
    bool hasFusedCarLampData = false;
};

bool isInvalidValue(double value)
{
    return !std::isfinite(value) || value <= InvalidSentinel;
}

bool isSchemaId(double value)
{
    return std::isfinite(value) &&
           std::abs(value - JustFloatLog::DualLampFusionSchemaId) <= SchemaTolerance;
}

QStringList splitCsvLine(const QString& line)
{
    QStringList cells = line.split(QLatin1Char(','), Qt::KeepEmptyParts);
    for (QString& cell : cells)
    {
        cell = cell.trimmed();
        if (cell.startsWith(QChar(0xfeff)))
        {
            cell.remove(0, 1);
        }
    }
    return cells;
}

bool parseDoubleCell(const QString& text, double* value)
{
    if (value == nullptr)
    {
        return false;
    }
    QString normalized = text.trimmed().toLower();
    const int equalIndex = normalized.indexOf(QLatin1Char('='));
    if (equalIndex >= 0)
    {
        normalized = normalized.mid(equalIndex + 1).trimmed();
    }
    if (normalized.isEmpty())
    {
        return false;
    }
    if (normalized == QStringLiteral("nan") || normalized == QStringLiteral("+nan") ||
        normalized == QStringLiteral("-nan"))
    {
        *value = std::numeric_limits<double>::quiet_NaN();
        return true;
    }
    if (normalized == QStringLiteral("inf") || normalized == QStringLiteral("+inf") ||
        normalized == QStringLiteral("infinity") || normalized == QStringLiteral("+infinity"))
    {
        *value = std::numeric_limits<double>::infinity();
        return true;
    }
    if (normalized == QStringLiteral("-inf") || normalized == QStringLiteral("-infinity"))
    {
        *value = -std::numeric_limits<double>::infinity();
        return true;
    }
    bool ok = false;
    const double parsed = QLocale::c().toDouble(normalized, &ok);
    if (!ok)
    {
        return false;
    }
    *value = parsed;
    return true;
}

QString formatDouble(double value, int precision)
{
    if (std::isnan(value))
    {
        return QStringLiteral("nan");
    }
    if (std::isinf(value))
    {
        return value < 0.0 ? QStringLiteral("-inf") : QStringLiteral("inf");
    }
    return QString::number(value, 'g', precision);
}

bool looksLikeHeader(const QStringList& cells)
{
    for (const QString& cell : cells)
    {
        if (!cell.startsWith(QLatin1Char('I')))
        {
            continue;
        }
        bool ok = false;
        cell.mid(1).section(QLatin1Char(' '), 0, 0).toInt(&ok);
        if (ok)
        {
            return true;
        }
    }
    return false;
}

bool parseHeader(const QStringList& cells, HeaderMap* map, int* declaredCount, QString* error)
{
    if (map == nullptr || declaredCount == nullptr)
    {
        return false;
    }
    map->fill(-1);
    for (int cellIndex = 0; cellIndex < cells.size(); ++cellIndex)
    {
        const QString cell = cells[cellIndex].trimmed();
        if (!cell.startsWith(QLatin1Char('I')))
        {
            continue;
        }
        QString number = cell.mid(1);
        const int space = number.indexOf(QChar(' '));
        if (space >= 0)
        {
            number = number.left(space);
        }
        bool ok = false;
        const int index = number.toInt(&ok);
        if (!ok || index < 0 || index >= JustFloatLog::ChannelCount)
        {
            continue;
        }
        if ((*map)[index] >= 0)
        {
            if (error != nullptr)
            {
                *error = QStringLiteral("duplicate CSV column I%1").arg(index);
            }
            return false;
        }
        (*map)[index] = cellIndex;
    }

    int highestIndex = -1;
    for (int index = 0; index < JustFloatLog::ChannelCount; ++index)
    {
        if ((*map)[index] >= 0)
        {
            highestIndex = index;
        }
    }
    const int count = highestIndex + 1;
    for (int index = 0; index < count; ++index)
    {
        if ((*map)[index] < 0)
        {
            if (error != nullptr)
            {
                *error = QStringLiteral("CSV header is missing I%1").arg(index);
            }
            return false;
        }
    }
    if (count != JustFloatLog::SingleLampRoiChannelCount &&
        count != JustFloatLog::LegacyChannelCount &&
        count != JustFloatLog::MotionChannelCount &&
        count != JustFloatLog::LegacyFusedChannelCount &&
        count != JustFloatLog::DualLampFusionChannelCount)
    {
        if (error != nullptr)
        {
            if (count > JustFloatLog::LegacyChannelCount &&
                count < JustFloatLog::MotionChannelCount)
            {
                *error = QStringLiteral("CSV header requires complete I0-I37 and I38-I42 groups");
            }
            else if (count > JustFloatLog::MotionChannelCount &&
                     count < JustFloatLog::LegacyFusedChannelCount)
            {
                *error = QStringLiteral("CSV header requires complete I43-I46 group");
            }
            else
            {
                *error = QStringLiteral("unsupported CSV channel count %1").arg(count);
            }
        }
        return false;
    }
    *declaredCount = count;
    return true;
}

bool parseNumberAt(const QStringList& cells,
                   const HeaderMap* map,
                   int index,
                   int lineNumber,
                   double* value,
                   QString* error)
{
    const int cellIndex = map == nullptr ? index : (*map)[index];
    if (cellIndex < 0 || cellIndex >= cells.size() ||
        !parseDoubleCell(cells[cellIndex], value))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("line %1 has invalid I%2").arg(lineNumber).arg(index);
        }
        return false;
    }
    return true;
}

bool parseOptionalGroup(const QStringList& cells,
                        const HeaderMap* map,
                        int first,
                        int last,
                        int lineNumber,
                        ValueArray* values,
                        bool* present,
                        QString* error)
{
    bool allEmpty = true;
    bool anyEmpty = false;
    for (int index = first; index <= last; ++index)
    {
        const int cellIndex = map == nullptr ? index : (*map)[index];
        if (cellIndex < 0 || cellIndex >= cells.size())
        {
            allEmpty = true;
            anyEmpty = true;
            continue;
        }
        const bool empty = cells[cellIndex].trimmed().isEmpty();
        allEmpty = allEmpty && empty;
        anyEmpty = anyEmpty || empty;
    }
    if (allEmpty)
    {
        *present = false;
        return true;
    }
    if (anyEmpty)
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("line %1 contains a partial optional group I%2-I%3")
                         .arg(lineNumber)
                         .arg(first)
                         .arg(last);
        }
        return false;
    }
    for (int index = first; index <= last; ++index)
    {
        if (!parseNumberAt(cells, map, index, lineNumber, &(*values)[index], error))
        {
            return false;
        }
    }
    *present = true;
    return true;
}

bool parseSingleLampRoiValues(const QStringList& cells,
                              const HeaderMap* map,
                              int lineNumber,
                              ParsedValues* parsed,
                              QString* error)
{
    parsed->layout = JustFloatLogLayout::SingleLampRoiV1;
    parsed->values.fill(0.0);
    for (int index = 0; index < JustFloatLog::SingleLampRoiChannelCount; ++index)
    {
        if (!parseNumberAt(cells, map, index, lineNumber, &parsed->values[index], error))
        {
            return false;
        }
    }
    return true;
}

bool parseLegacyValues(const QStringList& cells,
                       const HeaderMap* map,
                       int declaredCount,
                       int lineNumber,
                       ParsedValues* parsed,
                       QString* error)
{
    parsed->layout = JustFloatLogLayout::Legacy;
    parsed->values.fill(0.0);
    for (int index = 0; index < JustFloatLog::LegacyChannelCount; ++index)
    {
        if (declaredCount <= index)
        {
            if (index < JustFloatLog::LegacyChannelCount)
            {
                if (error != nullptr)
                {
                    *error = QStringLiteral("line %1 is missing I%2").arg(lineNumber).arg(index);
                }
                return false;
            }
            break;
        }
        if ((index >= JustFloatLog::LegacyChannelCount) ||
            !parseNumberAt(cells, map, index, lineNumber, &parsed->values[index], error))
        {
            return false;
        }
    }
    if (!parseOptionalGroup(cells,
                            map,
                            JustFloatLog::LegacyChannelCount,
                            JustFloatLog::MotionChannelCount - 1,
                            lineNumber,
                            &parsed->values,
                            &parsed->hasMotionData,
                            error))
    {
        return false;
    }
    if (!parseOptionalGroup(cells,
                            map,
                            JustFloatLog::MotionChannelCount,
                            JustFloatLog::LegacyFusedChannelCount - 1,
                            lineNumber,
                            &parsed->values,
                            &parsed->hasFusedCarLampData,
                            error))
    {
        return false;
    }
    if (parsed->hasFusedCarLampData && !parsed->hasMotionData)
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("fused fields require motion fields");
        }
        return false;
    }
    return true;
}

bool parseDualValues(const QStringList& cells,
                     const HeaderMap* map,
                     int lineNumber,
                     ParsedValues* parsed,
                     QString* error)
{
    parsed->layout = JustFloatLogLayout::DualLampFusionV1;
    parsed->values.fill(0.0);
    for (int index = 0; index < JustFloatLog::DualLampFusionChannelCount; ++index)
    {
        if (!parseNumberAt(cells, map, index, lineNumber, &parsed->values[index], error))
        {
            return false;
        }
    }
    if (!isSchemaId(parsed->values[1]))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("invalid DualLampFusionV1 schema identifier");
        }
        return false;
    }
    return true;
}

bool parseCells(const QStringList& cells,
                const HeaderMap* map,
                int declaredCount,
                double fallbackTimestampMs,
                int lineNumber,
                ParsedValues* parsed,
                QString* error,
                bool allowLegacyFallback)
{
    if (declaredCount == JustFloatLog::SingleLampRoiChannelCount)
    {
        return parseSingleLampRoiValues(cells, map, lineNumber, parsed, error);
    }
    if (declaredCount == JustFloatLog::DualLampFusionChannelCount)
    {
        if (!parseDualValues(cells, map, lineNumber, parsed, error))
        {
            if (allowLegacyFallback && map != nullptr)
            {
                ParsedValues legacy;
                if (parseLegacyValues(cells,
                                       map,
                                       declaredCount,
                                       lineNumber,
                                       &legacy,
                                       nullptr))
                {
                    *parsed = legacy;
                    return true;
                }
            }
            return false;
        }
        return true;
    }
    if (!parseLegacyValues(cells, map, declaredCount, lineNumber, parsed, error))
    {
        return false;
    }
    if (declaredCount == LegacyPayloadValueCount ||
        declaredCount == MotionPayloadValueCount ||
        declaredCount == FusedPayloadValueCount)
    {
        parsed->values[0] = fallbackTimestampMs;
    }
    return true;
}

bool parseSequentialDatagramValues(const QStringList& cells,
                                   double fallbackTimestampMs,
                                   ParsedValues* parsed,
                                   QString* error)
{
    const int count = cells.size();
    if (count != SingleLampRoiPayloadValueCount &&
        count != JustFloatLog::SingleLampRoiChannelCount &&
        count != LegacyPayloadValueCount &&
        count != JustFloatLog::LegacyChannelCount &&
        count != MotionPayloadValueCount &&
        count != JustFloatLog::MotionChannelCount &&
        count != FusedPayloadValueCount &&
        count != JustFloatLog::LegacyFusedChannelCount &&
        count != JustFloatLog::DualLampFusionChannelCount)
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("unsupported JustFloat field count %1").arg(count);
        }
        return false;
    }
    if (count == SingleLampRoiPayloadValueCount)
    {
        parsed->layout = JustFloatLogLayout::SingleLampRoiV1;
        parsed->values.fill(0.0);
        parsed->values[0] = fallbackTimestampMs;
        for (int i = 0; i < count; ++i)
        {
            if (!parseDoubleCell(cells[i], &parsed->values[i + 1]))
            {
                if (error != nullptr)
                {
                    *error = QStringLiteral("invalid datagram field %1").arg(i);
                }
                return false;
            }
        }
        return true;
    }
    if (count == LegacyPayloadValueCount ||
        count == MotionPayloadValueCount ||
        count == FusedPayloadValueCount)
    {
        parsed->layout = JustFloatLogLayout::Legacy;
        parsed->values.fill(0.0);
        parsed->values[0] = fallbackTimestampMs;
        for (int i = 0; i < count; ++i)
        {
            if (!parseDoubleCell(cells[i], &parsed->values[i + 1]))
            {
                if (error != nullptr)
                {
                    *error = QStringLiteral("invalid datagram field %1").arg(i);
                }
                return false;
            }
        }
        parsed->hasMotionData = count >= MotionPayloadValueCount;
        parsed->hasFusedCarLampData = count >= FusedPayloadValueCount;
        return true;
    }

    HeaderMap identity;
    identity.fill(-1);
    for (int i = 0; i < count; ++i)
    {
        identity[i] = i;
    }
    return parseCells(cells,
                      &identity,
                      count,
                      fallbackTimestampMs,
                      1,
                      parsed,
                      error,
                      false);
}

bool hasVofaTail(const QByteArray& data)
{
    const int size = data.size();
    return size >= 4 &&
           static_cast<unsigned char>(data[size - 4]) == 0x00 &&
           static_cast<unsigned char>(data[size - 3]) == 0x00 &&
           static_cast<unsigned char>(data[size - 2]) == 0x80 &&
           static_cast<unsigned char>(data[size - 1]) == 0x7f;
}

QByteArray withoutVofaTail(const QByteArray& data)
{
    QByteArray payload = data;
    if (hasVofaTail(payload))
    {
        payload.chop(4);
    }
    return payload;
}

bool parseBinaryDatagram(const QByteArray& datagram,
                         double fallbackTimestampMs,
                         ParsedValues* parsed,
                         QString* error)
{
    const QByteArray payload = withoutVofaTail(datagram);
    if (payload.size() % static_cast<int>(sizeof(float)) != 0)
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("binary payload is not float aligned");
        }
        return false;
    }
    const int count = payload.size() / static_cast<int>(sizeof(float));
    if (count != SingleLampRoiPayloadValueCount &&
        count != JustFloatLog::SingleLampRoiChannelCount &&
        count != LegacyPayloadValueCount &&
        count != JustFloatLog::LegacyChannelCount &&
        count != MotionPayloadValueCount &&
        count != JustFloatLog::MotionChannelCount &&
        count != FusedPayloadValueCount &&
        count != JustFloatLog::LegacyFusedChannelCount &&
        count != JustFloatLog::DualLampFusionChannelCount)
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("unsupported binary float count %1").arg(count);
        }
        return false;
    }

    QStringList values;
    values.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        const uchar* source = reinterpret_cast<const uchar*>(payload.constData() +
                                                              i * sizeof(float));
        const quint32 bits = qFromLittleEndian<quint32>(source);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        values.push_back(QString::number(value, 'g', 9));
    }
    return parseSequentialDatagramValues(values, fallbackTimestampMs, parsed, error);
}

bool parseTextDatagram(const QByteArray& datagram,
                       double fallbackTimestampMs,
                       ParsedValues* parsed,
                       QString* error)
{
    const QString text = QString::fromUtf8(withoutVofaTail(datagram)).trimmed();
    if (text.isEmpty())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("empty text datagram");
        }
        return false;
    }
    return parseSequentialDatagramValues(splitCsvLine(text), fallbackTimestampMs, parsed, error);
}

JustFloatBeacon makeLegacyBeacon(const ValueArray& values, int offset)
{
    JustFloatBeacon beacon;
    beacon.x = static_cast<float>(values[offset]);
    beacon.y = static_cast<float>(values[offset + 1]);
    beacon.area = static_cast<float>(values[offset + 2]);
    beacon.valid = !isInvalidValue(values[offset]) &&
                   !isInvalidValue(values[offset + 1]) &&
                   values[offset + 2] > 0.0;
    return beacon;
}

JustFloatCarLamp makeLegacyLamp(const ValueArray& values, int offset)
{
    JustFloatCarLamp lamp;
    lamp.cx = static_cast<float>(values[offset]);
    lamp.cy = static_cast<float>(values[offset + 1]);
    lamp.angle = static_cast<float>(values[offset + 2]);
    lamp.width = static_cast<float>(values[offset + 3]);
    lamp.length = static_cast<float>(values[offset + 4]);
    lamp.valid = !isInvalidValue(values[offset]) &&
                 !isInvalidValue(values[offset + 1]) &&
                 !isInvalidValue(values[offset + 2]) &&
                 values[offset + 3] > 0.0 &&
                 values[offset + 4] > 0.0;
    return lamp;
}

JustFloatMappedPoint makeMappedPoint(const ValueArray& values, int offset)
{
    JustFloatMappedPoint point;
    point.x = static_cast<float>(values[offset]);
    point.y = static_cast<float>(values[offset + 1]);
    point.valid = !isInvalidValue(values[offset]) && !isInvalidValue(values[offset + 1]);
    return point;
}

JustFloatMappedCarLamp makeMappedLamp(const ValueArray& values, int offset, bool measured)
{
    JustFloatMappedCarLamp lamp;
    lamp.cx = static_cast<float>(values[offset]);
    lamp.cy = static_cast<float>(values[offset + 1]);
    lamp.angle = static_cast<float>(values[offset + 2]);
    lamp.valid = !isInvalidValue(values[offset]) &&
                 !isInvalidValue(values[offset + 1]) &&
                 !isInvalidValue(values[offset + 2]);
    lamp.measured = measured;
    return lamp;
}

quint32 packed24(double value)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        return 0U;
    }
    const double rounded = std::floor(value + 0.5);
    return static_cast<quint32>(qBound(0.0,
                                      rounded,
                                      static_cast<double>(Packed24Maximum)));
}

JustFloatSingleLampShape decodeSingleLampShape(double value)
{
    JustFloatSingleLampShape shape;
    shape.packed = packed24(value);
    shape.widthCode = static_cast<quint8>(shape.packed & 0x3fU);
    shape.lengthCode = static_cast<quint8>((shape.packed >> 6U) & 0x7fU);
    shape.angleCode = static_cast<quint8>((shape.packed >> 13U) & 0x7fU);
    shape.nearestBeaconDistanceCode =
        static_cast<quint8>((shape.packed >> 20U) & 0x0fU);
    shape.valid = shape.packed != 0U;
    if (shape.valid)
    {
        shape.width = static_cast<float>(shape.widthCode) / SingleLampWidthScale;
        shape.length = static_cast<float>(shape.lengthCode) / SingleLampLengthScale;
        shape.angle = static_cast<float>(shape.angleCode) * SingleLampAngleStepDeg - 90.0f;
    }
    shape.nearestBeaconDistanceValid = shape.valid &&
                                       shape.nearestBeaconDistanceCode < 15U;
    if (shape.nearestBeaconDistanceValid)
    {
        shape.nearestBeaconDistance =
            static_cast<float>(shape.nearestBeaconDistanceCode) *
            SingleLampBeaconDistanceStep;
    }
    return shape;
}

JustFloatSingleLampTrackGeometry decodeSingleLampTrackGeometry(double value)
{
    JustFloatSingleLampTrackGeometry geometry;
    geometry.packed = packed24(value);
    geometry.valid = (geometry.packed & (1U << 23U)) != 0U;
    if (geometry.valid)
    {
        geometry.centerX = static_cast<float>(geometry.packed & 0x1ffU) - 140.0f;
        geometry.centerY = static_cast<float>((geometry.packed >> 9U) & 0xffU) - 110.0f;
        geometry.centerRoiHalfSize =
            static_cast<float>((geometry.packed >> 17U) & 0x3fU);
    }
    return geometry;
}

JustFloatSingleLampCrossCheck decodeSingleLampCrossCheck(double value)
{
    JustFloatSingleLampCrossCheck crossCheck;
    crossCheck.packed = packed24(value);
    crossCheck.state = static_cast<quint8>(crossCheck.packed & 0x07U);
    crossCheck.supportCameraMask = static_cast<quint8>((crossCheck.packed >> 3U) & 0x07U);
    crossCheck.roiValidMask = static_cast<quint8>((crossCheck.packed >> 6U) & 0x07U);
    crossCheck.roiHitMask = static_cast<quint8>((crossCheck.packed >> 9U) & 0x07U);
    crossCheck.conflictCameraMask = static_cast<quint8>((crossCheck.packed >> 12U) & 0x07U);
    crossCheck.projectionEnabled = ((crossCheck.packed >> 15U) & 0x01U) != 0U;
    crossCheck.fullFrameFallbackMask =
        static_cast<quint8>((crossCheck.packed >> 16U) & 0x07U);
    crossCheck.manuallyMarked = ((crossCheck.packed >> 19U) & 0x01U) != 0U;
    crossCheck.measuredCameraMask =
        static_cast<quint8>((crossCheck.packed >> 20U) & 0x07U);
    crossCheck.roiMode = ((crossCheck.packed >> 23U) & 0x01U) != 0U;
    return crossCheck;
}

void decodeSingleLampSourceFrames(double value, JustFloatSingleLampRoiFrame* frame)
{
    if (frame == nullptr)
    {
        return;
    }
    frame->frameSequencePacked = packed24(value);
    for (int camera = 0; camera < 3; ++camera)
    {
        const quint8 packedFrame = static_cast<quint8>(
            (frame->frameSequencePacked >> (camera * 8U)) & 0xffU);
        frame->sourceFrames[camera].sequenceLow7 = packedFrame & 0x7fU;
        frame->sourceFrames[camera].valid = (packedFrame & 0x80U) != 0U;
    }
}

JustFloatLogRow makeSingleLampRoiRow(const ParsedValues& parsed)
{
    JustFloatLogRow row;
    row.layout = JustFloatLogLayout::SingleLampRoiV1;
    const ValueArray& values = parsed.values;
    row.rowTime = values[0];
    row.roll = static_cast<float>(values[7]);
    row.pitch = static_cast<float>(values[8]);
    row.singleLampRoi.heightMm = static_cast<float>(values[9]);
    row.singleLampRoi.heightValid = !isInvalidValue(values[9]);

    for (int camera = 0; camera < 3; ++camera)
    {
        JustFloatCarLamp& lamp = row.cameras[camera].carLamp;
        lamp.cx = static_cast<float>(values[1 + camera * 2]);
        lamp.cy = static_cast<float>(values[2 + camera * 2]);
        row.singleLampRoi.lampShapes[camera] = decodeSingleLampShape(values[10 + camera]);
        const JustFloatSingleLampShape& shape = row.singleLampRoi.lampShapes[camera];
        lamp.angle = shape.angle;
        lamp.width = shape.width;
        lamp.length = shape.length;
        lamp.valid = !isInvalidValue(lamp.cx) && !isInvalidValue(lamp.cy) && shape.valid;

        for (int beacon = 0; beacon < 2; ++beacon)
        {
            const int offset = 13 + camera * 6 + beacon * 3;
            row.cameras[camera].beacons[beacon] = makeLegacyBeacon(values, offset);
        }
    }

    row.singleLampRoi.trackGeometry = decodeSingleLampTrackGeometry(values[31]);
    row.singleLampRoi.crossCheck = decodeSingleLampCrossCheck(values[32]);
    decodeSingleLampSourceFrames(values[33], &row.singleLampRoi);
    row.singleLampRoi.relativeYawDeg = static_cast<float>(values[34]);
    row.singleLampRoi.relativeYawValid = !isInvalidValue(values[34]);
    row.singleLampRoi.maxSkewMs = static_cast<float>(values[35]);
    row.singleLampRoi.maxSkewValid = !isInvalidValue(values[35]) && values[35] >= 0.0;
    return row;
}

JustFloatLogRow makeRow(const ParsedValues& parsed)
{
    if (parsed.layout == JustFloatLogLayout::SingleLampRoiV1)
    {
        return makeSingleLampRoiRow(parsed);
    }
    JustFloatLogRow row;
    row.layout = parsed.layout;
    const ValueArray& values = parsed.values;
    row.rowTime = values[0];

    if (parsed.layout == JustFloatLogLayout::DualLampFusionV1)
    {
        row.dualLampFusion.schemaId = static_cast<float>(values[1]);
        row.dualLampFusion.imageSequence = static_cast<quint32>(qMax(0.0, values[2]));
        for (int camera = 0; camera < 3; ++camera)
        {
            const int base = 3 + camera * 11;
            JustFloatMappedCameraFrame& target = row.dualLampFusion.cameras[camera];
            target.measuredMask = static_cast<quint8>(qBound(0,
                                                              static_cast<int>(std::lround(values[base])),
                                                              255));
            target.carLamps[0] = makeMappedLamp(values, base + 1,
                                                (target.measuredMask & 0x01U) != 0U);
            target.carLamps[1] = makeMappedLamp(values, base + 4,
                                                (target.measuredMask & 0x02U) != 0U);
            target.beacons[0] = makeMappedPoint(values, base + 7);
            target.beacons[1] = makeMappedPoint(values, base + 9);

            row.cameras[camera].carLamp.cx = target.carLamps[0].cx;
            row.cameras[camera].carLamp.cy = target.carLamps[0].cy;
            row.cameras[camera].carLamp.angle = target.carLamps[0].angle;
            row.cameras[camera].carLamp.width = 1.0f;
            row.cameras[camera].carLamp.length = 1.0f;
            row.cameras[camera].carLamp.valid = target.carLamps[0].valid;
            for (int beacon = 0; beacon < 2; ++beacon)
            {
                row.cameras[camera].beacons[beacon].x = target.beacons[beacon].x;
                row.cameras[camera].beacons[beacon].y = target.beacons[beacon].y;
                row.cameras[camera].beacons[beacon].area = target.beacons[beacon].valid ? 1.0f : 0.0f;
                row.cameras[camera].beacons[beacon].valid = target.beacons[beacon].valid;
            }
        }

        row.dualLampFusion.control.planMode =
            static_cast<int>(std::lround(values[36]));
        row.dualLampFusion.control.car.cx = static_cast<float>(values[37]);
        row.dualLampFusion.control.car.cy = static_cast<float>(values[38]);
        row.dualLampFusion.control.car.angle = static_cast<float>(values[39]);
        row.dualLampFusion.control.car.valid =
            !isInvalidValue(values[37]) && !isInvalidValue(values[38]) &&
            !isInvalidValue(values[39]);
        row.dualLampFusion.control.beacon.x = static_cast<float>(values[40]);
        row.dualLampFusion.control.beacon.y = static_cast<float>(values[41]);
        row.dualLampFusion.control.beacon.valid =
            !isInvalidValue(values[40]) && !isInvalidValue(values[41]);
        row.dualLampFusion.shadow.confidence =
            static_cast<int>(std::lround(values[42]));
        row.dualLampFusion.shadow.cx = static_cast<float>(values[43]);
        row.dualLampFusion.shadow.cy = static_cast<float>(values[44]);
        row.dualLampFusion.shadow.axisAngle = static_cast<float>(values[45]);
        row.dualLampFusion.shadow.valid =
            !isInvalidValue(values[43]) && !isInvalidValue(values[44]) &&
            !isInvalidValue(values[45]);
        row.dualLampFusion.shadow.lamps[0] = makeMappedPoint(values, 46);
        row.dualLampFusion.shadow.lamps[1] = makeMappedPoint(values, 48);
        return row;
    }

    row.cameras[0].beacons[0] = makeLegacyBeacon(values, 1);
    row.cameras[0].beacons[1] = makeLegacyBeacon(values, 4);
    row.cameras[1].beacons[0] = makeLegacyBeacon(values, 7);
    row.cameras[1].beacons[1] = makeLegacyBeacon(values, 10);
    row.cameras[2].beacons[0] = makeLegacyBeacon(values, 13);
    row.cameras[2].beacons[1] = makeLegacyBeacon(values, 16);
    row.cameras[0].carLamp = makeLegacyLamp(values, 19);
    row.cameras[1].carLamp = makeLegacyLamp(values, 24);
    row.cameras[2].carLamp = makeLegacyLamp(values, 29);
    row.pitch = static_cast<float>(values[34]);
    row.roll = static_cast<float>(values[35]);
    row.yaw = static_cast<float>(values[36]);
    row.syncTimeMs = values[37];
    row.hasMotionData = parsed.hasMotionData;
    if (row.hasMotionData)
    {
        row.actualVelocityX = static_cast<float>(values[38]);
        row.actualVelocityY = static_cast<float>(values[39]);
        row.vehicleYawDeg = static_cast<float>(values[40]);
        row.targetForwardMps = static_cast<float>(values[41]);
        row.targetStrafeMps = static_cast<float>(values[42]);
    }
    row.hasFusedCarLampData = parsed.hasFusedCarLampData;
    if (row.hasFusedCarLampData)
    {
        row.fusedCarLamp.valid = !isInvalidValue(values[43]);
        row.fusedCarLamp.cx = static_cast<float>(values[44]);
        row.fusedCarLamp.cy = static_cast<float>(values[45]);
        row.fusedCarLamp.angle = static_cast<float>(values[46]);
    }
    return row;
}

QVector<JustFloatChannelDescriptor> makeLegacyDescriptors()
{
    QVector<JustFloatChannelDescriptor> descriptors;
    descriptors.reserve(JustFloatLog::LegacyFusedChannelCount);
    descriptors.push_back({0, QStringLiteral("Time"), QStringLiteral("I0 Timestamp"), QStringLiteral("ms")});
    const QStringList cameraNames = {QStringLiteral("Front"), QStringLiteral("Center"), QStringLiteral("Back")};
    int index = 1;
    for (const QString& camera : cameraNames)
    {
        for (int beacon = 0; beacon < 2; ++beacon)
        {
            descriptors.push_back({index++, camera, QStringLiteral("Beacon %1 X").arg(beacon), QStringLiteral("px")});
            descriptors.push_back({index++, camera, QStringLiteral("Beacon %1 Y").arg(beacon), QStringLiteral("px")});
            descriptors.push_back({index++, camera, QStringLiteral("Beacon %1 Area").arg(beacon), QStringLiteral("px^2")});
        }
    }
    for (const QString& camera : cameraNames)
    {
        descriptors.push_back({index++, camera, QStringLiteral("Car lamp X"), QStringLiteral("px")});
        descriptors.push_back({index++, camera, QStringLiteral("Car lamp Y"), QStringLiteral("px")});
        descriptors.push_back({index++, camera, QStringLiteral("Car lamp angle"), QStringLiteral("deg")});
        descriptors.push_back({index++, camera, QStringLiteral("Car lamp width"), QStringLiteral("px")});
        descriptors.push_back({index++, camera, QStringLiteral("Car lamp length"), QStringLiteral("px")});
    }
    descriptors.push_back({34, QStringLiteral("Attitude"), QStringLiteral("Pitch"), QStringLiteral("deg")});
    descriptors.push_back({35, QStringLiteral("Attitude"), QStringLiteral("Roll"), QStringLiteral("deg")});
    descriptors.push_back({36, QStringLiteral("Attitude"), QStringLiteral("Yaw"), QStringLiteral("deg")});
    descriptors.push_back({37, QStringLiteral("Time"), QStringLiteral("Sync time"), QStringLiteral("ms")});
    descriptors.push_back({38, QStringLiteral("Motion"), QStringLiteral("Actual velocity X"), QStringLiteral("m/s")});
    descriptors.push_back({39, QStringLiteral("Motion"), QStringLiteral("Actual velocity Y"), QStringLiteral("m/s")});
    descriptors.push_back({40, QStringLiteral("Motion"), QStringLiteral("Vehicle Yaw"), QStringLiteral("deg")});
    descriptors.push_back({41, QStringLiteral("Motion"), QStringLiteral("Target forward"), QStringLiteral("m/s")});
    descriptors.push_back({42, QStringLiteral("Motion"), QStringLiteral("Target strafe"), QStringLiteral("m/s")});
    descriptors.push_back({43,
                           QStringLiteral("Fused lamp"),
                           QString::fromUtf8("\xE6\x9C\x89\xE6\x95\x88"),
                           QString()});
    descriptors.push_back({44, QStringLiteral("Fused lamp"), QStringLiteral("Center X"), QStringLiteral("px")});
    descriptors.push_back({45, QStringLiteral("Fused lamp"), QStringLiteral("Center Y"), QStringLiteral("px")});
    descriptors.push_back({46, QStringLiteral("Fused lamp"), QStringLiteral("Angle"), QStringLiteral("deg")});
    descriptors.push_back({47, QStringLiteral("Reserved"), QStringLiteral("I47"), QString()});
    descriptors.push_back({48, QStringLiteral("Reserved"), QStringLiteral("I48"), QString()});
    descriptors.push_back({49, QStringLiteral("Reserved"), QStringLiteral("I49"), QString()});
    return descriptors;
}

QVector<JustFloatChannelDescriptor> makeSingleLampRoiDescriptors()
{
    QVector<JustFloatChannelDescriptor> descriptors;
    descriptors.reserve(JustFloatLog::SingleLampRoiChannelCount);
    descriptors.push_back({0, QStringLiteral("Time"), QStringLiteral("Timestamp"), QStringLiteral("ms")});
    const QStringList cameras = {QStringLiteral("Front"), QStringLiteral("Center"), QStringLiteral("Back")};
    int index = 1;
    for (const QString& camera : cameras)
    {
        descriptors.push_back({index++, camera, QStringLiteral("Car lamp X"), QStringLiteral("px")});
        descriptors.push_back({index++, camera, QStringLiteral("Car lamp Y"), QStringLiteral("px")});
    }
    descriptors.push_back({7, QStringLiteral("Attitude"), QStringLiteral("Roll"), QStringLiteral("deg")});
    descriptors.push_back({8, QStringLiteral("Attitude"), QStringLiteral("Pitch"), QStringLiteral("deg")});
    descriptors.push_back({9, QStringLiteral("Attitude"), QStringLiteral("Height"), QStringLiteral("mm")});
    for (int camera = 0; camera < cameras.size(); ++camera)
    {
        descriptors.push_back({10 + camera,
                               cameras[camera],
                               QStringLiteral("Car lamp shape packed"),
                               QString()});
    }
    index = 13;
    for (const QString& camera : cameras)
    {
        for (int beacon = 0; beacon < 2; ++beacon)
        {
            descriptors.push_back({index++, camera, QStringLiteral("Beacon %1 X").arg(beacon), QStringLiteral("px")});
            descriptors.push_back({index++, camera, QStringLiteral("Beacon %1 Y").arg(beacon), QStringLiteral("px")});
            descriptors.push_back({index++, camera, QStringLiteral("Beacon %1 Area").arg(beacon), QStringLiteral("px^2")});
        }
    }
    descriptors.push_back({31, QStringLiteral("ROI track"), QStringLiteral("Track geometry packed"), QString()});
    descriptors.push_back({32, QStringLiteral("ROI track"), QStringLiteral("Cross-check packed"), QString()});
    descriptors.push_back({33, QStringLiteral("Frame sync"), QStringLiteral("Source frames packed"), QString()});
    descriptors.push_back({34, QStringLiteral("Attitude"), QStringLiteral("Relative yaw"), QStringLiteral("deg")});
    descriptors.push_back({35, QStringLiteral("Frame sync"), QStringLiteral("Maximum skew"), QStringLiteral("ms")});
    return descriptors;
}

QVector<JustFloatChannelDescriptor> makeDualDescriptors()
{
    QVector<JustFloatChannelDescriptor> descriptors;
    descriptors.reserve(JustFloatLog::DualLampFusionChannelCount);
    descriptors.push_back({0, QStringLiteral("DualLampFusionV1"), QStringLiteral("Timestamp"), QStringLiteral("ms")});
    descriptors.push_back({1, QStringLiteral("DualLampFusionV1"), QStringLiteral("Schema"), QString()});
    descriptors.push_back({2, QStringLiteral("DualLampFusionV1"), QStringLiteral("Image sequence"), QString()});
    const QStringList cameras = {QStringLiteral("Front mapped"), QStringLiteral("Center observed"), QStringLiteral("Back mapped")};
    int index = 3;
    for (const QString& camera : cameras)
    {
        descriptors.push_back({index++, camera, QStringLiteral("Measured mask"), QString()});
        for (int slot = 0; slot < 2; ++slot)
        {
            descriptors.push_back({index++, camera, QStringLiteral("Car lamp %1 X").arg(slot), QStringLiteral("px")});
            descriptors.push_back({index++, camera, QStringLiteral("Car lamp %1 Y").arg(slot), QStringLiteral("px")});
            descriptors.push_back({index++, camera, QStringLiteral("Car lamp %1 angle").arg(slot), QStringLiteral("deg")});
        }
        for (int beacon = 0; beacon < 2; ++beacon)
        {
            descriptors.push_back({index++, camera, QStringLiteral("Beacon %1 X").arg(beacon), QStringLiteral("px")});
            descriptors.push_back({index++, camera, QStringLiteral("Beacon %1 Y").arg(beacon), QStringLiteral("px")});
        }
    }
    descriptors.push_back({36, QStringLiteral("Control"), QStringLiteral("Plan mode"), QString()});
    descriptors.push_back({37, QStringLiteral("Control"), QStringLiteral("Car X"), QStringLiteral("px")});
    descriptors.push_back({38, QStringLiteral("Control"), QStringLiteral("Car Y"), QStringLiteral("px")});
    descriptors.push_back({39, QStringLiteral("Control"), QStringLiteral("Car angle"), QStringLiteral("deg")});
    descriptors.push_back({40, QStringLiteral("Control"), QStringLiteral("Beacon X"), QStringLiteral("px")});
    descriptors.push_back({41, QStringLiteral("Control"), QStringLiteral("Beacon Y"), QStringLiteral("px")});
    descriptors.push_back({42, QStringLiteral("Shadow fusion"), QStringLiteral("Confidence"), QString()});
    descriptors.push_back({43, QStringLiteral("Shadow fusion"), QStringLiteral("Center X"), QStringLiteral("px")});
    descriptors.push_back({44, QStringLiteral("Shadow fusion"), QStringLiteral("Center Y"), QStringLiteral("px")});
    descriptors.push_back({45, QStringLiteral("Shadow fusion"), QStringLiteral("Axis angle"), QStringLiteral("deg")});
    descriptors.push_back({46, QStringLiteral("Shadow fusion"), QStringLiteral("Lamp 0 X"), QStringLiteral("px")});
    descriptors.push_back({47, QStringLiteral("Shadow fusion"), QStringLiteral("Lamp 0 Y"), QStringLiteral("px")});
    descriptors.push_back({48, QStringLiteral("Shadow fusion"), QStringLiteral("Lamp 1 X"), QStringLiteral("px")});
    descriptors.push_back({49, QStringLiteral("Shadow fusion"), QStringLiteral("Lamp 1 Y"), QStringLiteral("px")});
    return descriptors;
}

bool parseDatagramValues(const QByteArray& datagram,
                         double fallbackTimestampMs,
                         ParsedValues* parsed,
                         QString* error)
{
    const QByteArray payload = withoutVofaTail(datagram);
    QString textError;
    QString binaryError;
    if (payload.contains(','))
    {
        if (parseTextDatagram(datagram, fallbackTimestampMs, parsed, &textError))
        {
            return true;
        }
        if (parseBinaryDatagram(datagram, fallbackTimestampMs, parsed, &binaryError))
        {
            return true;
        }
    }
    else
    {
        if (parseBinaryDatagram(datagram, fallbackTimestampMs, parsed, &binaryError))
        {
            return true;
        }
        if (parseTextDatagram(datagram, fallbackTimestampMs, parsed, &textError))
        {
            return true;
        }
    }
    if (error != nullptr)
    {
        *error = QStringLiteral("%1; %2").arg(binaryError, textError);
    }
    return false;
}
}

bool JustFloatLog::loadCsv(const QString& path, JustFloatLog* output, QString* errorMessage)
{
    if (output == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("output is null");
        }
        return false;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = file.errorString();
        }
        return false;
    }
    QTextStream stream(&file);
    QStringList lines;
    while (!stream.atEnd())
    {
        const QString line = stream.readLine();
        if (!line.trimmed().isEmpty())
        {
            lines.push_back(line);
        }
    }
    if (lines.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CSV is empty");
        }
        return false;
    }

    const QStringList firstCells = splitCsvLine(lines.front());
    HeaderMap headerMap;
    int declaredCount = 0;
    const bool hasHeader = looksLikeHeader(firstCells);
    const bool headerRequiresDual = hasHeader && firstCells.size() > 1 &&
                                    firstCells[1].contains(QStringLiteral("Schema"),
                                                           Qt::CaseInsensitive);
    if (hasHeader && !parseHeader(firstCells, &headerMap, &declaredCount, errorMessage))
    {
        return false;
    }

    QVector<JustFloatLogRow> rows;
    rows.reserve(lines.size());
    JustFloatLogLayout detectedLayout = JustFloatLogLayout::Legacy;
    bool layoutSet = false;
    const int firstDataLine = hasHeader ? 1 : 0;
    for (int lineIndex = firstDataLine; lineIndex < lines.size(); ++lineIndex)
    {
        const QStringList cells = splitCsvLine(lines[lineIndex]);
        const int count = hasHeader ? declaredCount : cells.size();
        if (!hasHeader && count != JustFloatLog::SingleLampRoiChannelCount &&
            count != JustFloatLog::LegacyChannelCount &&
            count != JustFloatLog::MotionChannelCount &&
            count != JustFloatLog::LegacyFusedChannelCount &&
            count != JustFloatLog::DualLampFusionChannelCount)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("line %1 has unsupported field count %2")
                                    .arg(lineIndex + 1)
                                    .arg(count);
            }
            return false;
        }
        if (hasHeader && cells.size() < declaredCount)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("line %1 has fewer cells than its header")
                                    .arg(lineIndex + 1);
            }
            return false;
        }
        ParsedValues parsed;
        HeaderMap identity;
        identity.fill(-1);
        for (int i = 0; i < count; ++i)
        {
            identity[i] = i;
        }
        const HeaderMap* map = hasHeader ? &headerMap : &identity;
        if (!parseCells(cells,
                        map,
                        count,
                        0.0,
                        lineIndex + 1,
                        &parsed,
                        errorMessage,
                        hasHeader && !headerRequiresDual &&
                            count == JustFloatLog::DualLampFusionChannelCount))
        {
            return false;
        }
        if (!layoutSet)
        {
            detectedLayout = parsed.layout;
            layoutSet = true;
        }
        else if (detectedLayout != parsed.layout)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("CSV cannot mix JustFloat layouts");
            }
            return false;
        }
        rows.push_back(makeRow(parsed));
    }
    if (rows.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CSV has no data rows");
        }
        return false;
    }
    output->m_sourcePath = QFileInfo(path).absoluteFilePath();
    output->m_rows = std::move(rows);
    output->m_layout = detectedLayout;
    return true;
}

bool JustFloatLog::parseDatagram(const QByteArray& datagram,
                                 double fallbackTimestampMs,
                                 JustFloatLogRow* output,
                                 QString* errorMessage)
{
    if (output == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("output is null");
        }
        return false;
    }
    ParsedValues parsed;
    QString error;
    if (!parseDatagramValues(datagram, fallbackTimestampMs, &parsed, &error))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = error;
        }
        return false;
    }
    *output = makeRow(parsed);
    return true;
}

const QVector<JustFloatChannelDescriptor>& JustFloatLog::channelDescriptors()
{
    return channelDescriptors(JustFloatLogLayout::Legacy);
}

const QVector<JustFloatChannelDescriptor>& JustFloatLog::channelDescriptors(JustFloatLogLayout layout)
{
    static const QVector<JustFloatChannelDescriptor> legacy = makeLegacyDescriptors();
    static const QVector<JustFloatChannelDescriptor> singleLampRoi = makeSingleLampRoiDescriptors();
    static const QVector<JustFloatChannelDescriptor> dual = makeDualDescriptors();
    if (layout == JustFloatLogLayout::SingleLampRoiV1)
    {
        return singleLampRoi;
    }
    return layout == JustFloatLogLayout::DualLampFusionV1 ? dual : legacy;
}

int JustFloatLog::channelCount(JustFloatLogLayout layout)
{
    return layout == JustFloatLogLayout::SingleLampRoiV1
               ? SingleLampRoiChannelCount
               : ChannelCount;
}

bool JustFloatLog::channelValue(const JustFloatLogRow& row, int channelIndex, double* value)
{
    if (value == nullptr || channelIndex < 0 || channelIndex >= channelCount(row.layout))
    {
        return false;
    }
    if (row.layout == JustFloatLogLayout::SingleLampRoiV1)
    {
        const JustFloatSingleLampRoiFrame& frame = row.singleLampRoi;
        if (channelIndex == 0) { *value = row.rowTime; return true; }
        if (channelIndex >= 1 && channelIndex <= 6)
        {
            const int offset = channelIndex - 1;
            const JustFloatCarLamp& lamp = row.cameras[offset / 2].carLamp;
            *value = (offset % 2 == 0) ? lamp.cx : lamp.cy;
            return true;
        }
        if (channelIndex == 7) { *value = row.roll; return true; }
        if (channelIndex == 8) { *value = row.pitch; return true; }
        if (channelIndex == 9) { *value = frame.heightMm; return true; }
        if (channelIndex >= 10 && channelIndex <= 12)
        {
            *value = frame.lampShapes[channelIndex - 10].packed;
            return true;
        }
        if (channelIndex >= 13 && channelIndex <= 30)
        {
            const int offset = channelIndex - 13;
            const JustFloatBeacon& beacon = row.cameras[offset / 6].beacons[(offset % 6) / 3];
            const int component = offset % 3;
            *value = component == 0 ? beacon.x : (component == 1 ? beacon.y : beacon.area);
            return true;
        }
        switch (channelIndex)
        {
        case 31: *value = frame.trackGeometry.packed; return true;
        case 32: *value = frame.crossCheck.packed; return true;
        case 33: *value = frame.frameSequencePacked; return true;
        case 34: *value = frame.relativeYawDeg; return true;
        case 35: *value = frame.maxSkewMs; return true;
        default: return false;
        }
    }
    if (row.layout == JustFloatLogLayout::DualLampFusionV1)
    {
        const JustFloatDualLampFusionFrame& frame = row.dualLampFusion;
        if (channelIndex == 0) { *value = row.rowTime; return true; }
        if (channelIndex == 1) { *value = frame.schemaId; return true; }
        if (channelIndex == 2) { *value = frame.imageSequence; return true; }
        if (channelIndex >= 3 && channelIndex <= 35)
        {
            const int offset = channelIndex - 3;
            const int camera = offset / 11;
            const int field = offset % 11;
            const JustFloatMappedCameraFrame& cameraFrame = frame.cameras[camera];
            if (field == 0) { *value = cameraFrame.measuredMask; return true; }
            if (field >= 1 && field <= 6)
            {
                const JustFloatMappedCarLamp& lamp = cameraFrame.carLamps[(field - 1) / 3];
                const int component = (field - 1) % 3;
                *value = component == 0 ? lamp.cx : (component == 1 ? lamp.cy : lamp.angle);
                return true;
            }
            const JustFloatMappedPoint& beacon = cameraFrame.beacons[(field - 7) / 2];
            *value = ((field - 7) % 2 == 0) ? beacon.x : beacon.y;
            return true;
        }
        switch (channelIndex)
        {
        case 36: *value = frame.control.planMode; return true;
        case 37: *value = frame.control.car.cx; return true;
        case 38: *value = frame.control.car.cy; return true;
        case 39: *value = frame.control.car.angle; return true;
        case 40: *value = frame.control.beacon.x; return true;
        case 41: *value = frame.control.beacon.y; return true;
        case 42: *value = frame.shadow.confidence; return true;
        case 43: *value = frame.shadow.cx; return true;
        case 44: *value = frame.shadow.cy; return true;
        case 45: *value = frame.shadow.axisAngle; return true;
        case 46: *value = frame.shadow.lamps[0].x; return true;
        case 47: *value = frame.shadow.lamps[0].y; return true;
        case 48: *value = frame.shadow.lamps[1].x; return true;
        case 49: *value = frame.shadow.lamps[1].y; return true;
        default: return false;
        }
    }

    if (channelIndex == 0) { *value = row.rowTime; return true; }
    if (channelIndex >= 1 && channelIndex <= 18)
    {
        const int offset = channelIndex - 1;
        const JustFloatBeacon& beacon = row.cameras[offset / 6].beacons[(offset % 6) / 3];
        const int component = offset % 3;
        *value = component == 0 ? beacon.x : (component == 1 ? beacon.y : beacon.area);
        return true;
    }
    if (channelIndex >= 19 && channelIndex <= 33)
    {
        const int offset = channelIndex - 19;
        const JustFloatCarLamp& lamp = row.cameras[offset / 5].carLamp;
        const int component = offset % 5;
        *value = component == 0 ? lamp.cx :
                 (component == 1 ? lamp.cy :
                  (component == 2 ? lamp.angle :
                   (component == 3 ? lamp.width : lamp.length)));
        return true;
    }
    switch (channelIndex)
    {
    case 34: *value = row.pitch; return true;
    case 35: *value = row.roll; return true;
    case 36: *value = row.yaw; return true;
    case 37: *value = row.syncTimeMs; return true;
    case 38: if (!row.hasMotionData) return false; *value = row.actualVelocityX; return true;
    case 39: if (!row.hasMotionData) return false; *value = row.actualVelocityY; return true;
    case 40: if (!row.hasMotionData) return false; *value = row.vehicleYawDeg; return true;
    case 41: if (!row.hasMotionData) return false; *value = row.targetForwardMps; return true;
    case 42: if (!row.hasMotionData) return false; *value = row.targetStrafeMps; return true;
    case 43: if (!row.hasFusedCarLampData) return false; *value = row.fusedCarLamp.valid ? 1.0 : 0.0; return true;
    case 44: if (!row.hasFusedCarLampData) return false; *value = row.fusedCarLamp.cx; return true;
    case 45: if (!row.hasFusedCarLampData) return false; *value = row.fusedCarLamp.cy; return true;
    case 46: if (!row.hasFusedCarLampData) return false; *value = row.fusedCarLamp.angle; return true;
    default: return false;
    }
}

QString JustFloatLog::csvHeader()
{
    return csvHeader(JustFloatLogLayout::Legacy);
}

QString JustFloatLog::csvHeader(JustFloatLogLayout layout)
{
    QStringList fields;
    const int count = channelCount(layout);
    fields.reserve(count);
    const auto& descriptors = channelDescriptors(layout);
    for (int i = 0; i < count; ++i)
    {
        fields.push_back(layout != JustFloatLogLayout::Legacy
                             ? QStringLiteral("I%1 %2").arg(i).arg(descriptors[i].name)
                             : QStringLiteral("I%1").arg(i));
    }
    return fields.join(QLatin1Char(','));
}

QString JustFloatLog::csvRow(const JustFloatLogRow& row)
{
    QStringList fields;
    const int count = channelCount(row.layout);
    fields.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        double value = 0.0;
        if (!channelValue(row, i, &value))
        {
            fields.push_back(QString());
        }
        else
        {
            fields.push_back(formatDouble(value, (i == 0) ? 17 : 9));
        }
    }
    return fields.join(QLatin1Char(','));
}

QString JustFloatLog::layoutName(JustFloatLogLayout layout)
{
    if (layout == JustFloatLogLayout::SingleLampRoiV1)
    {
        return QStringLiteral("SingleLampRoiV1");
    }
    return layout == JustFloatLogLayout::DualLampFusionV1
               ? QStringLiteral("DualLampFusionV1")
               : QStringLiteral("Legacy");
}

QString JustFloatLog::sourcePath() const { return m_sourcePath; }
int JustFloatLog::rowCount() const { return m_rows.size(); }
const JustFloatLogRow& JustFloatLog::rowAt(int index) const { return m_rows[index]; }
JustFloatLogLayout JustFloatLog::layout() const { return m_layout; }
