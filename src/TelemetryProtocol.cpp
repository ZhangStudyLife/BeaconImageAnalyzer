#include "TelemetryProtocol.h"

#include <QFile>
#include <QtEndian>
#include <QTextStream>

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
    bool ok = false;
    const double parsed = text.trimmed().toDouble(&ok);
    if (!ok || !std::isfinite(parsed) ||
        parsed < -std::numeric_limits<float>::max() ||
        parsed > std::numeric_limits<float>::max())
    {
        return false;
    }
    *value = static_cast<float>(parsed);
    return true;
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
            *errorMessage = QStringLiteral("176 字节数据包缺少 JustFloat 尾标 00 00 80 7F");
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
    for (int index = 0; index < ChannelCount; ++index)
    {
        if (!std::isfinite(values[index]))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("I%1 不是有限浮点数").arg(index);
            }
            return false;
        }
    }

    TelemetryFrame parsed;
    parsed.channels = values;
    int index = 1;
    for (CameraSample& camera : parsed.cameras)
    {
        for (BeaconSample& beacon : camera.beacons)
        {
            beacon.x = values[index++];
            beacon.y = values[index++];
            beacon.area = values[index++];
            beacon.valid = !isInvalid(beacon.x) && !isInvalid(beacon.y) && beacon.area > 0.0f;
        }
    }
    for (CameraSample& camera : parsed.cameras)
    {
        CarLampSample& lamp = camera.carLamp;
        lamp.cx = values[index++];
        lamp.cy = values[index++];
        lamp.angle = values[index++];
        lamp.width = values[index++];
        lamp.length = values[index++];
        lamp.valid = !isInvalid(lamp.cx) && !isInvalid(lamp.cy) &&
                     !isInvalid(lamp.angle) && lamp.width > 0.0f && lamp.length > 0.0f;
    }
    parsed.timestampMs = values[0];
    parsed.pitch = values[34];
    parsed.roll = values[35];
    parsed.yaw = values[36];
    parsed.syncTimestampMs = values[37];
    parsed.reserved = values[38];
    parsed.carForwardVelocity = values[39];
    parsed.carYaw = values[40];
    parsed.plannedForwardVelocity = values[41];
    parsed.plannedStrafeVelocity = values[42];
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
                *errorMessage = QStringLiteral("第 %1 行应有 43 列，实际为 %2 列")
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
    if (std::isfinite(frame.syncTimestampMs) && frame.syncTimestampMs > 0.0f)
    {
        return frame.syncTimestampMs;
    }
    return frame.timestampMs;
}
