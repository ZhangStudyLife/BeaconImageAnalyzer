#include "TelemetryProtocol.h"

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

    const bool hasTail = datagram.size() == PayloadBytes + TailBytes;
    if (datagram.size() != PayloadBytes && !hasTail)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("数据包长度应为 %1 或 %2 字节，实际为 %3 字节")
                                .arg(PayloadBytes)
                                .arg(PayloadBytes + TailBytes)
                                .arg(datagram.size());
        }
        return false;
    }
    if (hasTail && !hasJustFloatTail(datagram))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("288 字节数据包缺少 JustFloat 尾标 00 00 80 7F");
        }
        return false;
    }

    std::array<float, ChannelCount> values{};
    for (int index = 0; index < ChannelCount; ++index)
    {
        const auto* source = reinterpret_cast<const uchar*>(datagram.constData() +
                                                             index * sizeof(float));
        const quint32 bits = qFromLittleEndian<quint32>(source);
        std::memcpy(&values[index], &bits, sizeof(float));
    }
    return makeFrame(values, frame, errorMessage);
}

bool TelemetryProtocol::makeFrame(const std::array<float, ChannelCount>& values,
                                  TelemetryFrame* frame,
                                  QString* errorMessage)
{
    TelemetryFrame parsed;
    parsed.channels = values;
    parsed.timestampMs = values[0];

    for (int camera = 0; camera < 3; ++camera)
    {
        const int offset = 1 + camera * 10;
        parsed.cameras[camera].beacons[0] = makeBeacon(values, offset);
        parsed.cameras[camera].beacons[1] = makeBeacon(values, offset + 3);
        parsed.cameras[camera].carLamp = makeLamp(values, offset + 6);
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

QString TelemetryProtocol::csvHeader()
{
    QStringList columns;
    columns.reserve(ChannelCount);
    for (int index = 0; index < ChannelCount; ++index)
    {
        columns.push_back(QStringLiteral("I%1").arg(index));
    }
    return columns.join(QLatin1Char(','));
}

QString TelemetryProtocol::csvRow(const TelemetryFrame& frame)
{
    QStringList columns;
    columns.reserve(ChannelCount);
    for (float value : frame.channels)
    {
        columns.push_back(QString::number(value, 'g', 9));
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
    stream << csvHeader() << Qt::endl;
    for (const TelemetryFrame& frame : frames)
    {
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
    while (!stream.atEnd())
    {
        const QString line = stream.readLine().trimmed();
        ++lineNumber;
        if (line.isEmpty())
        {
            continue;
        }
        if (firstNonEmptyLine && line.compare(csvHeader(), Qt::CaseInsensitive) == 0)
        {
            firstNonEmptyLine = false;
            continue;
        }
        firstNonEmptyLine = false;

        const QStringList cells = line.split(QLatin1Char(','), Qt::KeepEmptyParts);
        if (cells.size() != ChannelCount)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 行应有 71 列，实际为 %2 列")
                                    .arg(lineNumber)
                                    .arg(cells.size());
            }
            return false;
        }
        std::array<float, ChannelCount> values{};
        for (int index = 0; index < ChannelCount; ++index)
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
        if (!makeFrame(values, &frame, errorMessage))
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
    *frames = std::move(loaded);
    return true;
}

double TelemetryProtocol::playbackTimestampMs(const TelemetryFrame& frame)
{
    return frame.timestampMs;
}
