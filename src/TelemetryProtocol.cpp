#include "TelemetryProtocol.h"

#include "CarPlan3Model.h"

#include <QFile>
#include <QTextStream>
#include <QtEndian>

#include <cmath>
#include <cstring>
#include <limits>

namespace
{
constexpr float InvalidSentinel = -900.0f;

bool hasJustFloatTail(const QByteArray& data)
{
    const int size = data.size();
    return size >= TelemetryProtocol::TailBytes &&
           static_cast<unsigned char>(data[size - 4]) == 0x00 &&
           static_cast<unsigned char>(data[size - 3]) == 0x00 &&
           static_cast<unsigned char>(data[size - 2]) == 0x80 &&
           static_cast<unsigned char>(data[size - 1]) == 0x7f;
}

bool isInvalid(float value)
{
    return !std::isfinite(value) || value <= InvalidSentinel;
}

bool parseCsvValue(const QString& text, float* value)
{
    const QString normalized = text.trimmed().toLower();
    if (normalized == QStringLiteral("nan"))
    {
        *value = std::numeric_limits<float>::quiet_NaN();
        return true;
    }
    if (normalized == QStringLiteral("inf") || normalized == QStringLiteral("+inf"))
    {
        *value = std::numeric_limits<float>::infinity();
        return true;
    }
    if (normalized == QStringLiteral("-inf"))
    {
        *value = -std::numeric_limits<float>::infinity();
        return true;
    }
    bool ok = false;
    const double parsed = normalized.toDouble(&ok);
    if (!ok || parsed < -std::numeric_limits<float>::max() ||
        parsed > std::numeric_limits<float>::max())
    {
        return false;
    }
    *value = static_cast<float>(parsed);
    return true;
}

BeaconSample makeBeacon(const std::array<float, TelemetryProtocol::ChannelCount>& values,
                        int offset)
{
    BeaconSample beacon;
    beacon.x = values[offset];
    beacon.y = values[offset + 1];
    beacon.area = values[offset + 2];
    beacon.valid = !isInvalid(beacon.x) && !isInvalid(beacon.y) &&
                   std::isfinite(beacon.area) && beacon.area > 0.0f;
    return beacon;
}

CarLampSample makeLamp(const std::array<float, TelemetryProtocol::ChannelCount>& values,
                       int offset)
{
    CarLampSample lamp;
    lamp.cx = values[offset];
    lamp.cy = values[offset + 1];
    lamp.angle = values[offset + 2];
    lamp.length = values[offset + 3];
    lamp.valid = !isInvalid(lamp.cx) && !isInvalid(lamp.cy) &&
                 !isInvalid(lamp.angle) && std::isfinite(lamp.length) && lamp.length > 0.0f;
    return lamp;
}

int discreteValue(float value, int minimum, int maximum, int fallback)
{
    if (!std::isfinite(value))
    {
        return fallback;
    }
    const int rounded = static_cast<int>(std::lround(value));
    return std::abs(value - static_cast<float>(rounded)) <= 0.001f &&
                   rounded >= minimum && rounded <= maximum
               ? rounded
               : fallback;
}
}

bool TelemetryProtocol::parseDatagram(const QByteArray& datagram,
                                      TelemetryFrame* frame,
                                      QString* errorMessage)
{
    if (frame == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("输出帧为空");
        }
        return false;
    }

    int channelCount = 0;
    bool hasTail = false;
    if (datagram.size() == LegacyPayloadBytes ||
        datagram.size() == LegacyPayloadBytes + TailBytes)
    {
        channelCount = LegacyChannelCount;
        hasTail = datagram.size() == LegacyPayloadBytes + TailBytes;
    }
    else if (datagram.size() == CarPlan3PayloadBytes ||
             datagram.size() == CarPlan3PayloadBytes + TailBytes)
    {
        channelCount = CarPlan3ChannelCount;
        hasTail = datagram.size() == CarPlan3PayloadBytes + TailBytes;
    }
    else
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("数据包长度应为 %1/%2 或 %3/%4 字节，实际为 %5 字节")
                                .arg(CarPlan3PayloadBytes)
                                .arg(CarPlan3PayloadBytes + TailBytes)
                                .arg(LegacyPayloadBytes)
                                .arg(LegacyPayloadBytes + TailBytes)
                                .arg(datagram.size());
        }
        return false;
    }
    if (hasTail && !hasJustFloatTail(datagram))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("JustFloat 数据包缺少尾标 00 00 80 7F");
        }
        return false;
    }

    std::array<float, LegacyChannelCount> values{};
    values.fill(InvalidSentinel);
    for (int index = 0; index < channelCount; ++index)
    {
        const auto* source = reinterpret_cast<const uchar*>(datagram.constData() +
                                                             index * sizeof(float));
        const quint32 bits = qFromLittleEndian<quint32>(source);
        std::memcpy(&values[index], &bits, sizeof(float));
    }
    return makeFrame(values, channelCount, frame, errorMessage);
}

bool TelemetryProtocol::makeFrame(const std::array<float, LegacyChannelCount>& values,
                                  int channelCount,
                                  TelemetryFrame* frame,
                                  QString* errorMessage)
{
    TelemetryFrame parsed;
    parsed.channels = values;
    parsed.channelCount = channelCount;
    parsed.timestampMs = values[0];

    for (int camera = 0; camera < 3; ++camera)
    {
        const int offset = 1 + camera * 10;
        parsed.cameras[camera].beacons[0] = makeBeacon(values, offset);
        parsed.cameras[camera].beacons[1] = makeBeacon(values, offset + 3);
        parsed.cameras[camera].carLamp = makeLamp(values, offset + 6);
    }

    if (channelCount == CarPlan3ChannelCount)
    {
        if (discreteValue(values[63], 3, 3, 0) != 3)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("64列数据缺少CarPlan3 V2协议版本");
            }
            return false;
        }
        for (int slot = 0; slot < 4; ++slot)
        {
            const int offset = 31 + slot * 4;
            GlobalBeaconSample& beacon = parsed.globalBeacons[slot];
            beacon.x = values[offset];
            beacon.y = values[offset + 1];
            beacon.area = values[offset + 2];
            beacon.cameraMask = discreteValue(values[offset + 3], 1, 7, 0);
            beacon.valid = !isInvalid(beacon.x) && !isInvalid(beacon.y) &&
                           std::isfinite(beacon.area) && beacon.area > 0.0f &&
                           beacon.cameraMask != 0;
        }
        parsed.globalCarLamp.x = values[47];
        parsed.globalCarLamp.y = values[48];
        parsed.globalCarLamp.angleDeg = values[49];
        parsed.globalCarLamp.cameraMask = discreteValue(values[50], 1, 7, 0);
        parsed.globalCarLamp.valid = !isInvalid(parsed.globalCarLamp.x) &&
                                     !isInvalid(parsed.globalCarLamp.y) &&
                                     !isInvalid(parsed.globalCarLamp.angleDeg) &&
                                     parsed.globalCarLamp.cameraMask != 0;
        parsed.carPlan3Valid = std::isfinite(values[51]) && values[51] >= 0.5f;
        parsed.carYawDeg = values[52];
        parsed.carActualVelocityX = values[53];
        parsed.carActualVelocityY = values[54];
        parsed.carTargetVelocityX = values[55];
        parsed.carTargetVelocityY = values[56];
        parsed.aircraftHeightMm = values[57];
        parsed.aircraftRollDeg = values[58];
        parsed.aircraftPitchDeg = values[59];
        parsed.aircraftYawDeg = values[60];
        parsed.selectedTargetId = discreteValue(values[61], -1, 3, -1);
        parsed.markerActive = std::isfinite(values[62]) && values[62] >= 0.5f;
        parsed.carPlan3Direct = true;
        *frame = parsed;
        return true;
    }

    for (int slot = 0; slot < 3; ++slot)
    {
        const int offset = 31 + slot * 4;
        FusedBeaconSample& beacon = parsed.centerBeacons[slot];
        beacon.x = values[offset];
        beacon.y = values[offset + 1];
        beacon.area = values[offset + 2];
        beacon.cameraMask = discreteValue(values[offset + 3], 1, 7, 0);
        beacon.valid = !isInvalid(beacon.x) && !isInvalid(beacon.y) &&
                       std::isfinite(beacon.area) && beacon.area > 0.0f &&
                       beacon.cameraMask != 0;
    }
    parsed.centerCarLamp = makeLamp(values, 43);

    for (int slot = 0; slot < 3; ++slot)
    {
        parsed.modelBeacons[slot] = makeBeacon(values, 47 + slot * 3);
    }
    parsed.modelCarLamp = makeLamp(values, 56);

    parsed.carYawDeg = values[60];
    parsed.carActualVelocityX = values[61];
    parsed.carActualVelocityY = values[62];
    parsed.carTargetVelocityX = values[63];
    parsed.carTargetVelocityY = values[64];
    parsed.aircraftHeightMm = values[65];
    parsed.aircraftRollDeg = values[66];
    parsed.aircraftPitchDeg = values[67];
    parsed.aircraftYawDeg = values[68];
    parsed.selectedTargetId = discreteValue(values[69], 0, 2, -1);
    parsed.markerActive = std::isfinite(values[70]) && values[70] >= 0.5f;
    *frame = parsed;
    return true;
}

QString TelemetryProtocol::csvHeader(int channelCount)
{
    QStringList columns;
    columns.reserve(channelCount);
    for (int index = 0; index < channelCount; ++index)
    {
        columns.push_back(QStringLiteral("I%1").arg(index));
    }
    return columns.join(QLatin1Char(','));
}

QString TelemetryProtocol::csvRow(const TelemetryFrame& frame)
{
    QStringList columns;
    columns.reserve(frame.channelCount);
    for (int index = 0; index < frame.channelCount; ++index)
    {
        columns.push_back(QString::number(frame.channels[index], 'g', 9));
    }
    return columns.join(QLatin1Char(','));
}

bool TelemetryProtocol::saveCsv(const QString& path,
                                const QVector<TelemetryFrame>& frames,
                                QString* errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = file.errorString();
        }
        return false;
    }
    QTextStream stream(&file);
    const int channelCount = frames.isEmpty() ? LegacyChannelCount : frames.front().channelCount;
    stream << csvHeader(channelCount) << Qt::endl;
    for (const TelemetryFrame& frame : frames)
    {
        if (frame.channelCount != channelCount)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("不能在同一CSV中混合64列和71列协议");
            }
            return false;
        }
        stream << csvRow(frame) << Qt::endl;
    }
    if (stream.status() != QTextStream::Ok)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = file.errorString();
        }
        return false;
    }
    return true;
}

bool TelemetryProtocol::loadCsv(const QString& path,
                                QVector<TelemetryFrame>* frames,
                                QString* errorMessage)
{
    if (frames == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("输出日志为空");
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
    QVector<TelemetryFrame> loaded;
    int lineNumber = 0;
    bool firstNonEmptyLine = true;
    int channelCount = 0;
    while (!stream.atEnd())
    {
        const QString line = stream.readLine().trimmed();
        ++lineNumber;
        if (line.isEmpty())
        {
            continue;
        }
        if (firstNonEmptyLine && line.compare(csvHeader(LegacyChannelCount), Qt::CaseInsensitive) == 0)
        {
            channelCount = LegacyChannelCount;
            firstNonEmptyLine = false;
            continue;
        }
        if (firstNonEmptyLine && line.compare(csvHeader(CarPlan3ChannelCount), Qt::CaseInsensitive) == 0)
        {
            channelCount = CarPlan3ChannelCount;
            firstNonEmptyLine = false;
            continue;
        }
        firstNonEmptyLine = false;

        const QStringList cells = line.split(QLatin1Char(','), Qt::KeepEmptyParts);
        if (channelCount == 0 &&
            (cells.size() == LegacyChannelCount || cells.size() == CarPlan3ChannelCount))
        {
            channelCount = cells.size();
        }
        if (cells.size() != channelCount)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 行应有64列或71列，实际为 %2 列")
                                    .arg(lineNumber)
                                    .arg(cells.size());
            }
            return false;
        }
        std::array<float, LegacyChannelCount> values{};
        values.fill(InvalidSentinel);
        for (int index = 0; index < channelCount; ++index)
        {
            if (!parseCsvValue(cells[index], &values[index]))
            {
                if (errorMessage != nullptr)
                {
                    *errorMessage = QStringLiteral("第 %1 行 I%2 不是有效浮点数")
                                        .arg(lineNumber)
                                        .arg(index);
                }
                return false;
            }
        }
        TelemetryFrame frame;
        if (!makeFrame(values, channelCount, &frame, errorMessage))
        {
            return false;
        }
        loaded.push_back(frame);
    }
    if (loaded.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CSV 没有数据行");
        }
        return false;
    }
    CarPlan3Model carPlan3;
    for (TelemetryFrame& frame : loaded)
    {
        carPlan3.process(&frame);
    }
    *frames = std::move(loaded);
    return true;
}

double TelemetryProtocol::playbackTimestampMs(const TelemetryFrame& frame)
{
    return frame.timestampMs;
}
