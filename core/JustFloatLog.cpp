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
constexpr int PayloadValueCount = JustFloatLog::ChannelCount - 1;
constexpr double InvalidSentinel = -900.0;

using ValueArray = std::array<double, JustFloatLog::ChannelCount>;
using HeaderMap = std::array<int, JustFloatLog::ChannelCount>;

struct ParsedValues
{
    ValueArray values{};
    bool hasProjectionDistance = false;
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
                    bool* hasProjectionColumns,
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

    const bool hasX = (*map)[38] >= 0;
    const bool hasY = (*map)[39] >= 0;
    if (hasX != hasY)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CSV 表头中的 I38 和 I39 必须同时存在。");
        }
        return false;
    }

    *hasProjectionColumns = hasX;
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

bool parseProjectionValues(const QStringList& cells,
                           const HeaderMap& map,
                           int lineNumber,
                           ParsedValues* parsed,
                           QString* errorMessage)
{
    const int xIndex = map[38];
    const int yIndex = map[39];
    if (xIndex < 0 && yIndex < 0)
    {
        parsed->hasProjectionDistance = false;
        return true;
    }
    if (xIndex >= cells.size() || yIndex >= cells.size())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("第 %1 行缺少 I38 或 I39。").arg(lineNumber);
        }
        return false;
    }

    const QString xText = cells[xIndex].trimmed();
    const QString yText = cells[yIndex].trimmed();
    if (xText.isEmpty() && yText.isEmpty())
    {
        parsed->hasProjectionDistance = false;
        return true;
    }
    if (xText.isEmpty() || yText.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("第 %1 行的 I38 和 I39 必须同时有值或同时留空。").arg(lineNumber);
        }
        return false;
    }
    if (!parseDoubleCell(xText, &parsed->values[38]) ||
        !parseDoubleCell(yText, &parsed->values[39]))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("第 %1 行的 I38 或 I39 不是有效浮点数。").arg(lineNumber);
        }
        return false;
    }

    parsed->hasProjectionDistance = true;
    return true;
}

bool parseMappedValues(const QStringList& cells,
                       const HeaderMap& map,
                       bool hasProjectionColumns,
                       int lineNumber,
                       ParsedValues* parsed,
                       QString* errorMessage)
{
    *parsed = ParsedValues{};
    if (!parseRequiredValues(cells, map, lineNumber, parsed, errorMessage))
    {
        return false;
    }
    if (!hasProjectionColumns)
    {
        return true;
    }
    return parseProjectionValues(cells, map, lineNumber, parsed, errorMessage);
}

bool parseSequentialCsvValues(const QStringList& cells,
                              int lineNumber,
                              ParsedValues* parsed,
                              QString* errorMessage)
{
    if (cells.size() != JustFloatLog::LegacyChannelCount &&
        cells.size() != JustFloatLog::ChannelCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("第 %1 行字段数量应为 38 或 40，实际为 %2。")
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
                             cells.size() == JustFloatLog::ChannelCount,
                             lineNumber,
                             parsed,
                             errorMessage);
}

bool parseSequentialDatagramValues(const QStringList& cells,
                                   quint64 sequence,
                                   ParsedValues* parsed,
                                   QString* errorMessage)
{
    const int count = cells.size();
    if (count != LegacyPayloadValueCount &&
        count != JustFloatLog::LegacyChannelCount &&
        count != PayloadValueCount &&
        count != JustFloatLog::ChannelCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("字段数量应为 37、38、39 或 40，实际为 %1。").arg(count);
        }
        return false;
    }

    *parsed = ParsedValues{};
    const bool payloadWithoutRowTime = count == LegacyPayloadValueCount || count == PayloadValueCount;
    const int shift = payloadWithoutRowTime ? 1 : 0;
    if (payloadWithoutRowTime)
    {
        parsed->values[0] = static_cast<double>(sequence);
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

    parsed->hasProjectionDistance = count == PayloadValueCount || count == JustFloatLog::ChannelCount;
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
                         quint64 sequence,
                         ParsedValues* parsed,
                         QString* errorMessage)
{
    const QByteArray payload = withoutVofaTail(datagram);
    const int floatCount = payload.size() / static_cast<int>(sizeof(float));
    if (payload.size() % static_cast<int>(sizeof(float)) != 0 ||
        (floatCount != LegacyPayloadValueCount &&
         floatCount != JustFloatLog::LegacyChannelCount &&
         floatCount != PayloadValueCount &&
         floatCount != JustFloatLog::ChannelCount))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("二进制长度不匹配：%1 字节。").arg(datagram.size());
        }
        return false;
    }

    *parsed = ParsedValues{};
    const bool payloadWithoutRowTime = floatCount == LegacyPayloadValueCount || floatCount == PayloadValueCount;
    const int shift = payloadWithoutRowTime ? 1 : 0;
    if (payloadWithoutRowTime)
    {
        parsed->values[0] = static_cast<double>(sequence);
    }

    for (int i = 0; i < floatCount; ++i)
    {
        const uchar* source = reinterpret_cast<const uchar*>(payload.constData() + i * sizeof(float));
        const quint32 bits = qFromLittleEndian<quint32>(source);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        parsed->values[i + shift] = static_cast<double>(value);
    }
    parsed->hasProjectionDistance = floatCount == PayloadValueCount ||
                                    floatCount == JustFloatLog::ChannelCount;
    return true;
}

bool parseTextDatagram(const QByteArray& datagram,
                       quint64 sequence,
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
    return parseSequentialDatagramValues(splitCsvLine(text), sequence, parsed, errorMessage);
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
    row.hasProjectionDistance = parsed.hasProjectionDistance;
    if (row.hasProjectionDistance)
    {
        row.projectionXcm = static_cast<float>(values[38]);
        row.projectionYcm = static_cast<float>(values[39]);
    }
    return row;
}

QVector<JustFloatChannelDescriptor> makeChannelDescriptors()
{
    QVector<JustFloatChannelDescriptor> descriptors;
    descriptors.reserve(JustFloatLog::ChannelCount);
    descriptors.push_back({0, QStringLiteral("时间"), QStringLiteral("设备时间"), QStringLiteral("ms")});

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
    descriptors.push_back({index++, QStringLiteral("投影距离"), QStringLiteral("X"), QStringLiteral("cm")});
    descriptors.push_back({index, QStringLiteral("投影距离"), QStringLiteral("Y"), QStringLiteral("cm")});
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
    bool hasProjectionColumns = false;
    const bool hasHeader = looksLikeJustFloatHeader(firstCells);
    QVector<JustFloatLogRow> rows;

    if (hasHeader)
    {
        if (!buildHeaderMap(firstCells, &map, &hasProjectionColumns, errorMessage))
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
                                                hasProjectionColumns,
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
                                 quint64 sequence,
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
    if (likelyText && parseTextDatagram(datagram, sequence, &parsed, &textError))
    {
        *output = makeRow(parsed);
        return true;
    }
    if (parseBinaryDatagram(datagram, sequence, &parsed, &binaryError))
    {
        *output = makeRow(parsed);
        return true;
    }
    if (!likelyText && parseTextDatagram(datagram, sequence, &parsed, &textError))
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
        if (!row.hasProjectionDistance)
        {
            return false;
        }
        *value = row.projectionXcm;
        return true;
    case 39:
        if (!row.hasProjectionDistance)
        {
            return false;
        }
        *value = row.projectionYcm;
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
