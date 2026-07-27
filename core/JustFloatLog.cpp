#include "JustFloatLog.h"

#include <QFile>
#include <QFileInfo>
#include <QLocale>
#include <QTextStream>
#include <QtEndian>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
constexpr int LegacyPayloadValueCount = JustFloatLog::LegacyChannelCount - 1;
constexpr int MotionPayloadValueCount = JustFloatLog::MotionChannelCount - 1;
constexpr int FusedPayloadValueCount = JustFloatLog::ChannelCount - 1;
constexpr double InvalidSentinel = -900.0;

using ValueArray = std::array<double, JustFloatLog::ChannelCount>;
using HeaderMap = std::array<int, JustFloatLog::ChannelCount>;

struct ParsedValues
{
    ValueArray values{};
    bool hasMotionData = false;
    bool hasFusedCarLampData = false;
};

bool isInvalidValue(double value)
{
    return !std::isfinite(value) || value <= InvalidSentinel;
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

    const QString normalized = text.trimmed().toLower();
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

bool looksLikeJustFloatHeader(const QStringList& cells)
{
    for (const QString& cell : cells)
    {
        if (!cell.startsWith(QLatin1Char('I')))
        {
            continue;
        }

        bool ok = false;
        const int index = cell.mid(1).toInt(&ok);
        if (ok && index >= 0 && index < JustFloatLog::ChannelCount)
        {
            return true;
        }
    }
    return false;
}

bool buildHeaderMap(const QStringList& cells,
                    HeaderMap* map,
                    bool* hasMotionColumns,
                    bool* hasFusedCarLampColumns,
                    QString* errorMessage)
{
    map->fill(-1);
    for (int i = 0; i < cells.size(); ++i)
    {
        const QString name = cells[i].trimmed();
        if (!name.startsWith(QLatin1Char('I')))
        {
            continue;
        }

        bool ok = false;
        const int index = name.mid(1).toInt(&ok);
        if (!ok || index < 0 || index >= JustFloatLog::ChannelCount)
        {
            continue;
        }
        if ((*map)[index] >= 0)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("CSV 表头重复定义 I%1。").arg(index);
            }
            return false;
        }
        (*map)[index] = i;
    }

    for (int i = 0; i < JustFloatLog::LegacyChannelCount; ++i)
    {
        if ((*map)[i] < 0)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("CSV 表头缺少 I%1。").arg(i);
            }
            return false;
        }
    }

    int motionColumnCount = 0;
    for (int i = JustFloatLog::LegacyChannelCount; i < JustFloatLog::MotionChannelCount; ++i)
    {
        motionColumnCount += ((*map)[i] >= 0) ? 1 : 0;
    }
    if (motionColumnCount != 0 &&
        motionColumnCount != JustFloatLog::MotionChannelCount - JustFloatLog::LegacyChannelCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CSV 表头中的 I38 至 I42 必须同时存在。");
        }
        return false;
    }

    *hasMotionColumns = motionColumnCount > 0;

    int fusedColumnCount = 0;
    for (int i = JustFloatLog::MotionChannelCount; i < JustFloatLog::ChannelCount; ++i)
    {
        fusedColumnCount += ((*map)[i] >= 0) ? 1 : 0;
    }
    if (fusedColumnCount != 0 &&
        fusedColumnCount != JustFloatLog::ChannelCount - JustFloatLog::MotionChannelCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CSV 表头中的 I43 至 I46 必须同时存在。");
        }
        return false;
    }
    if (fusedColumnCount > 0 && !*hasMotionColumns)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CSV 表头包含 I43 至 I46 时也必须包含 I38 至 I42。");
        }
        return false;
    }
    *hasFusedCarLampColumns = fusedColumnCount > 0;
    return true;
}

bool parseRequiredValues(const QStringList& cells,
                         const HeaderMap& map,
                         int lineNumber,
                         ParsedValues* parsed,
                         QString* errorMessage)
{
    for (int i = 0; i < JustFloatLog::LegacyChannelCount; ++i)
    {
        const int cellIndex = map[i];
        if (cellIndex < 0 || cellIndex >= cells.size())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 行缺少 I%2。").arg(lineNumber).arg(i);
            }
            return false;
        }
        if (!parseDoubleCell(cells[cellIndex], &parsed->values[i]))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 行 I%2 不是有效浮点数。").arg(lineNumber).arg(i);
            }
            return false;
        }
    }
    return true;
}

bool parseMotionValues(const QStringList& cells,
                       const HeaderMap& map,
                       int lineNumber,
                       ParsedValues* parsed,
                       QString* errorMessage)
{
    bool allEmpty = true;
    bool anyEmpty = false;
    for (int channel = JustFloatLog::LegacyChannelCount;
         channel < JustFloatLog::MotionChannelCount;
         ++channel)
    {
        const int cellIndex = map[channel];
        if (cellIndex < 0 || cellIndex >= cells.size())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 行缺少 I%2。").arg(lineNumber).arg(channel);
            }
            return false;
        }
        const bool empty = cells[cellIndex].trimmed().isEmpty();
        allEmpty = allEmpty && empty;
        anyEmpty = anyEmpty || empty;
    }
    if (allEmpty)
    {
        parsed->hasMotionData = false;
        return true;
    }
    if (anyEmpty)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("第 %1 行的 I38 至 I42 必须同时有值或同时留空。").arg(lineNumber);
        }
        return false;
    }

    for (int channel = JustFloatLog::LegacyChannelCount;
         channel < JustFloatLog::MotionChannelCount;
         ++channel)
    {
        if (!parseDoubleCell(cells[map[channel]], &parsed->values[channel]))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 行的 I%2 不是有效浮点数。")
                                    .arg(lineNumber)
                                    .arg(channel);
            }
            return false;
        }
    }

    parsed->hasMotionData = true;
    return true;
}

bool parseFusedCarLampValues(const QStringList& cells,
                             const HeaderMap& map,
                             int lineNumber,
                             ParsedValues* parsed,
                             QString* errorMessage)
{
    bool allEmpty = true;
    bool anyEmpty = false;
    for (int channel = JustFloatLog::MotionChannelCount;
         channel < JustFloatLog::ChannelCount;
         ++channel)
    {
        const int cellIndex = map[channel];
        if (cellIndex < 0 || cellIndex >= cells.size())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 行缺少 I%2。").arg(lineNumber).arg(channel);
            }
            return false;
        }
        const bool empty = cells[cellIndex].trimmed().isEmpty();
        allEmpty = allEmpty && empty;
        anyEmpty = anyEmpty || empty;
    }
    if (allEmpty)
    {
        return true;
    }
    if (anyEmpty)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("第 %1 行的 I43 至 I46 必须同时有值或同时留空。").arg(lineNumber);
        }
        return false;
    }

    for (int channel = JustFloatLog::MotionChannelCount;
         channel < JustFloatLog::ChannelCount;
         ++channel)
    {
        if (!parseDoubleCell(cells[map[channel]], &parsed->values[channel]))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 行的 I%2 不是有效浮点数。")
                                    .arg(lineNumber)
                                    .arg(channel);
            }
            return false;
        }
    }
    parsed->hasFusedCarLampData = true;
    return true;
}

bool parseMappedValues(const QStringList& cells,
                       const HeaderMap& map,
                       bool hasMotionColumns,
                       bool hasFusedCarLampColumns,
                       int lineNumber,
                       ParsedValues* parsed,
                       QString* errorMessage)
{
    *parsed = ParsedValues{};
    if (!parseRequiredValues(cells, map, lineNumber, parsed, errorMessage))
    {
        return false;
    }
    if (hasMotionColumns &&
        !parseMotionValues(cells, map, lineNumber, parsed, errorMessage))
    {
        return false;
    }
    if (hasFusedCarLampColumns &&
        !parseFusedCarLampValues(cells, map, lineNumber, parsed, errorMessage))
    {
        return false;
    }
    if (parsed->hasFusedCarLampData && !parsed->hasMotionData)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("第 %1 行包含 I43 至 I46 时，I38 至 I42 不能留空。")
                                .arg(lineNumber);
        }
        return false;
    }
    return true;
}

bool parseSequentialCsvValues(const QStringList& cells,
                              int lineNumber,
                              ParsedValues* parsed,
                              QString* errorMessage)
{
    if (cells.size() != JustFloatLog::LegacyChannelCount &&
        cells.size() != JustFloatLog::MotionChannelCount &&
        cells.size() != JustFloatLog::ChannelCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("第 %1 行字段数量应为 38、43 或 47，实际为 %2。")
                                .arg(lineNumber)
                                .arg(cells.size());
        }
        return false;
    }

    HeaderMap map;
    map.fill(-1);
    for (int i = 0; i < cells.size(); ++i)
    {
        map[i] = i;
    }
    return parseMappedValues(cells,
                             map,
                             cells.size() >= JustFloatLog::MotionChannelCount,
                             cells.size() == JustFloatLog::ChannelCount,
                             lineNumber,
                             parsed,
                             errorMessage);
}

bool parseSequentialDatagramValues(const QStringList& cells,
                                   double fallbackTimestampMs,
                                   ParsedValues* parsed,
                                   QString* errorMessage)
{
    const int count = cells.size();
    if (count != LegacyPayloadValueCount &&
        count != JustFloatLog::LegacyChannelCount &&
        count != MotionPayloadValueCount &&
        count != JustFloatLog::MotionChannelCount &&
        count != FusedPayloadValueCount &&
        count != JustFloatLog::ChannelCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("字段数量应为 37、38、42、43、46 或 47，实际为 %1。").arg(count);
        }
        return false;
    }

    *parsed = ParsedValues{};
    const bool payloadWithoutRowTime = count == LegacyPayloadValueCount ||
                                       count == MotionPayloadValueCount ||
                                       count == FusedPayloadValueCount;
    const int shift = payloadWithoutRowTime ? 1 : 0;
    if (payloadWithoutRowTime)
    {
        parsed->values[0] = fallbackTimestampMs;
    }

    for (int i = 0; i < count; ++i)
    {
        QString cell = cells[i].trimmed();
        const int equalIndex = cell.indexOf(QLatin1Char('='));
        if (equalIndex >= 0)
        {
            cell = cell.mid(equalIndex + 1).trimmed();
        }
        if (!parseDoubleCell(cell, &parsed->values[i + shift]))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 个字段不是有效浮点数。").arg(i);
            }
            return false;
        }
    }

    parsed->hasMotionData = count >= MotionPayloadValueCount;
    parsed->hasFusedCarLampData = count >= FusedPayloadValueCount;
    return true;
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
                         QString* errorMessage)
{
    const QByteArray payload = withoutVofaTail(datagram);
    const int floatCount = payload.size() / static_cast<int>(sizeof(float));
    if (payload.size() % static_cast<int>(sizeof(float)) != 0 ||
        (floatCount != LegacyPayloadValueCount &&
         floatCount != JustFloatLog::LegacyChannelCount &&
         floatCount != MotionPayloadValueCount &&
         floatCount != JustFloatLog::MotionChannelCount &&
         floatCount != FusedPayloadValueCount &&
         floatCount != JustFloatLog::ChannelCount))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("二进制长度不匹配：%1 字节。").arg(datagram.size());
        }
        return false;
    }

    *parsed = ParsedValues{};
    const bool payloadWithoutRowTime = floatCount == LegacyPayloadValueCount ||
                                       floatCount == MotionPayloadValueCount ||
                                       floatCount == FusedPayloadValueCount;
    const int shift = payloadWithoutRowTime ? 1 : 0;
    if (payloadWithoutRowTime)
    {
        parsed->values[0] = fallbackTimestampMs;
    }

    for (int i = 0; i < floatCount; ++i)
    {
        const uchar* source = reinterpret_cast<const uchar*>(payload.constData() + i * sizeof(float));
        const quint32 bits = qFromLittleEndian<quint32>(source);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        parsed->values[i + shift] = static_cast<double>(value);
    }
    parsed->hasMotionData = floatCount >= MotionPayloadValueCount;
    parsed->hasFusedCarLampData = floatCount >= FusedPayloadValueCount;
    return true;
}

bool parseTextDatagram(const QByteArray& datagram,
                       double fallbackTimestampMs,
                       ParsedValues* parsed,
                       QString* errorMessage)
{
    const QString text = QString::fromUtf8(withoutVofaTail(datagram)).trimmed();
    if (text.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("文本 UDP 数据为空。");
        }
        return false;
    }
    return parseSequentialDatagramValues(splitCsvLine(text), fallbackTimestampMs, parsed, errorMessage);
}

JustFloatBeacon makeBeacon(const ValueArray& values, int offset)
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

JustFloatCarLamp makeCarLamp(const ValueArray& values, int offset)
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

JustFloatLogRow makeRow(const ParsedValues& parsed)
{
    const ValueArray& values = parsed.values;
    JustFloatLogRow row;
    row.rowTime = values[0];
    row.cameras[0].beacons[0] = makeBeacon(values, 1);
    row.cameras[0].beacons[1] = makeBeacon(values, 4);
    row.cameras[1].beacons[0] = makeBeacon(values, 7);
    row.cameras[1].beacons[1] = makeBeacon(values, 10);
    row.cameras[2].beacons[0] = makeBeacon(values, 13);
    row.cameras[2].beacons[1] = makeBeacon(values, 16);
    row.cameras[0].carLamp = makeCarLamp(values, 19);
    row.cameras[1].carLamp = makeCarLamp(values, 24);
    row.cameras[2].carLamp = makeCarLamp(values, 29);
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
        row.fusedCarLamp.valid = std::isfinite(values[43]) && values[43] != 0.0;
        row.fusedCarLamp.cx = static_cast<float>(values[44]);
        row.fusedCarLamp.cy = static_cast<float>(values[45]);
        row.fusedCarLamp.angle = static_cast<float>(values[46]);
    }
    return row;
}

QVector<JustFloatChannelDescriptor> makeChannelDescriptors()
{
    QVector<JustFloatChannelDescriptor> descriptors;
    descriptors.reserve(JustFloatLog::ChannelCount);
    descriptors.push_back({0, QStringLiteral("时间"), QStringLiteral("I0 时间戳"), QStringLiteral("ms")});

    const QStringList cameraNames = {
        QStringLiteral("Front"),
        QStringLiteral("Center"),
        QStringLiteral("Back")
    };
    int index = 1;
    for (int camera = 0; camera < 3; ++camera)
    {
        const QString& group = cameraNames[camera];
        for (int beacon = 0; beacon < 2; ++beacon)
        {
            descriptors.push_back({index++, group, QStringLiteral("信标 %1 X").arg(beacon), QStringLiteral("px")});
            descriptors.push_back({index++, group, QStringLiteral("信标 %1 Y").arg(beacon), QStringLiteral("px")});
            descriptors.push_back({index++, group, QStringLiteral("信标 %1 面积").arg(beacon), QStringLiteral("px^2")});
        }
    }
    for (int camera = 0; camera < 3; ++camera)
    {
        const QString& group = cameraNames[camera];
        descriptors.push_back({index++, group, QStringLiteral("车灯中心 X"), QStringLiteral("px")});
        descriptors.push_back({index++, group, QStringLiteral("车灯中心 Y"), QStringLiteral("px")});
        descriptors.push_back({index++, group, QStringLiteral("车灯角度"), QStringLiteral("deg")});
        descriptors.push_back({index++, group, QStringLiteral("车灯宽度"), QStringLiteral("px")});
        descriptors.push_back({index++, group, QStringLiteral("车灯长度"), QStringLiteral("px")});
    }
    descriptors.push_back({index++, QStringLiteral("姿态"), QStringLiteral("Pitch"), QStringLiteral("deg")});
    descriptors.push_back({index++, QStringLiteral("姿态"), QStringLiteral("Roll"), QStringLiteral("deg")});
    descriptors.push_back({index++, QStringLiteral("姿态"), QStringLiteral("Yaw"), QStringLiteral("deg")});
    descriptors.push_back({index++, QStringLiteral("时间"), QStringLiteral("同步时间"), QStringLiteral("ms")});
    descriptors.push_back({index++, QStringLiteral("车辆运动"), QStringLiteral("实际横移速度 X"), QStringLiteral("m/s")});
    descriptors.push_back({index++, QStringLiteral("车辆运动"), QStringLiteral("实际前进速度 Y"), QStringLiteral("m/s")});
    descriptors.push_back({index++, QStringLiteral("车辆运动"), QStringLiteral("车辆 Yaw"), QStringLiteral("deg")});
    descriptors.push_back({index++, QStringLiteral("车辆运动"), QStringLiteral("目标前进速度"), QStringLiteral("m/s")});
    descriptors.push_back({index++, QStringLiteral("车辆运动"), QStringLiteral("目标横移速度"), QStringLiteral("m/s")});
    descriptors.push_back({index++, QStringLiteral("融合车灯"), QStringLiteral("有效"), QString()});
    descriptors.push_back({index++, QStringLiteral("融合车灯"), QStringLiteral("中心 X"), QStringLiteral("px")});
    descriptors.push_back({index++, QStringLiteral("融合车灯"), QStringLiteral("中心 Y"), QStringLiteral("px")});
    descriptors.push_back({index, QStringLiteral("融合车灯"), QStringLiteral("角度"), QStringLiteral("deg")});
    return descriptors;
}
}

bool JustFloatLog::loadCsv(const QString& path, JustFloatLog* output, QString* errorMessage)
{
    if (output == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("输出对象为空。");
        }
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法打开 CSV：%1").arg(file.errorString());
        }
        return false;
    }

    QTextStream stream(&file);
    QString firstLine;
    int lineNumber = 0;
    while (!stream.atEnd())
    {
        firstLine = stream.readLine();
        ++lineNumber;
        if (!firstLine.trimmed().isEmpty())
        {
            break;
        }
    }

    if (firstLine.trimmed().isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CSV 为空。");
        }
        return false;
    }

    const QStringList firstCells = splitCsvLine(firstLine);
    HeaderMap map;
    bool hasMotionColumns = false;
    bool hasFusedCarLampColumns = false;
    const bool hasHeader = looksLikeJustFloatHeader(firstCells);
    QVector<JustFloatLogRow> rows;

    if (hasHeader)
    {
        if (!buildHeaderMap(firstCells,
                            &map,
                            &hasMotionColumns,
                            &hasFusedCarLampColumns,
                            errorMessage))
        {
            return false;
        }
    }
    else
    {
        ParsedValues parsed;
        if (!parseSequentialCsvValues(firstCells, lineNumber, &parsed, errorMessage))
        {
            return false;
        }
        rows.push_back(makeRow(parsed));
    }

    while (!stream.atEnd())
    {
        const QString line = stream.readLine();
        ++lineNumber;
        if (line.trimmed().isEmpty())
        {
            continue;
        }

        ParsedValues parsed;
        const QStringList cells = splitCsvLine(line);
        const bool ok = hasHeader
                            ? parseMappedValues(cells,
                                                map,
                                                hasMotionColumns,
                                                hasFusedCarLampColumns,
                                                lineNumber,
                                                &parsed,
                                                errorMessage)
                            : parseSequentialCsvValues(cells, lineNumber, &parsed, errorMessage);
        if (!ok)
        {
            return false;
        }
        rows.push_back(makeRow(parsed));
    }

    if (rows.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CSV 没有数据行。");
        }
        return false;
    }

    output->m_sourcePath = QFileInfo(path).absoluteFilePath();
    output->m_rows = rows;
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
            *errorMessage = QStringLiteral("输出对象为空。");
        }
        return false;
    }

    ParsedValues parsed;
    QString binaryError;
    QString textError;
    const bool likelyText = withoutVofaTail(datagram).contains(',');
    if (likelyText && parseTextDatagram(datagram, fallbackTimestampMs, &parsed, &textError))
    {
        *output = makeRow(parsed);
        return true;
    }
    if (parseBinaryDatagram(datagram, fallbackTimestampMs, &parsed, &binaryError))
    {
        *output = makeRow(parsed);
        return true;
    }
    if (!likelyText && parseTextDatagram(datagram, fallbackTimestampMs, &parsed, &textError))
    {
        *output = makeRow(parsed);
        return true;
    }

    if (errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("%1；%2").arg(binaryError, textError);
    }
    return false;
}

const QVector<JustFloatChannelDescriptor>& JustFloatLog::channelDescriptors()
{
    static const QVector<JustFloatChannelDescriptor> descriptors = makeChannelDescriptors();
    return descriptors;
}

bool JustFloatLog::channelValue(const JustFloatLogRow& row, int channelIndex, double* value)
{
    if (value == nullptr || channelIndex < 0 || channelIndex >= ChannelCount)
    {
        return false;
    }

    if (channelIndex == 0)
    {
        *value = row.rowTime;
        return true;
    }
    if (channelIndex >= 1 && channelIndex <= 18)
    {
        const int offset = channelIndex - 1;
        const int camera = offset / 6;
        const int beaconOffset = offset % 6;
        const JustFloatBeacon& beacon = row.cameras[camera].beacons[beaconOffset / 3];
        switch (beaconOffset % 3)
        {
        case 0:
            *value = beacon.x;
            return true;
        case 1:
            *value = beacon.y;
            return true;
        default:
            *value = beacon.area;
            return true;
        }
    }
    if (channelIndex >= 19 && channelIndex <= 33)
    {
        const int offset = channelIndex - 19;
        const JustFloatCarLamp& lamp = row.cameras[offset / 5].carLamp;
        switch (offset % 5)
        {
        case 0:
            *value = lamp.cx;
            return true;
        case 1:
            *value = lamp.cy;
            return true;
        case 2:
            *value = lamp.angle;
            return true;
        case 3:
            *value = lamp.width;
            return true;
        default:
            *value = lamp.length;
            return true;
        }
    }
    switch (channelIndex)
    {
    case 34:
        *value = row.pitch;
        return true;
    case 35:
        *value = row.roll;
        return true;
    case 36:
        *value = row.yaw;
        return true;
    case 37:
        *value = row.syncTimeMs;
        return true;
    case 38:
        if (!row.hasMotionData)
        {
            return false;
        }
        *value = row.actualVelocityX;
        return true;
    case 39:
        if (!row.hasMotionData)
        {
            return false;
        }
        *value = row.actualVelocityY;
        return true;
    case 40:
        if (!row.hasMotionData)
        {
            return false;
        }
        *value = row.vehicleYawDeg;
        return true;
    case 41:
        if (!row.hasMotionData)
        {
            return false;
        }
        *value = row.targetForwardMps;
        return true;
    case 42:
        if (!row.hasMotionData)
        {
            return false;
        }
        *value = row.targetStrafeMps;
        return true;
    case 43:
        if (!row.hasFusedCarLampData)
        {
            return false;
        }
        *value = row.fusedCarLamp.valid ? 1.0 : 0.0;
        return true;
    case 44:
        if (!row.hasFusedCarLampData)
        {
            return false;
        }
        *value = row.fusedCarLamp.cx;
        return true;
    case 45:
        if (!row.hasFusedCarLampData)
        {
            return false;
        }
        *value = row.fusedCarLamp.cy;
        return true;
    case 46:
        if (!row.hasFusedCarLampData)
        {
            return false;
        }
        *value = row.fusedCarLamp.angle;
        return true;
    default:
        return false;
    }
}

QString JustFloatLog::csvHeader()
{
    QStringList fields;
    fields.reserve(ChannelCount);
    for (int i = 0; i < ChannelCount; ++i)
    {
        fields.push_back(QStringLiteral("I%1").arg(i));
    }
    return fields.join(QLatin1Char(','));
}

QString JustFloatLog::csvRow(const JustFloatLogRow& row)
{
    QStringList fields;
    fields.reserve(ChannelCount);
    for (int i = 0; i < ChannelCount; ++i)
    {
        double value = 0.0;
        if (!channelValue(row, i, &value))
        {
            fields.push_back(QString());
            continue;
        }
        const bool doublePrecision = i == 0 || i == 37;
        fields.push_back(formatDouble(value, doublePrecision ? 17 : 9));
    }
    return fields.join(QLatin1Char(','));
}

QString JustFloatLog::sourcePath() const
{
    return m_sourcePath;
}

int JustFloatLog::rowCount() const
{
    return m_rows.size();
}

const JustFloatLogRow& JustFloatLog::rowAt(int index) const
{
    return m_rows[index];
}
