#include "JustFloatLog.h"

#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QtEndian>

#include <array>
#include <cmath>
#include <cstring>

namespace
{
constexpr int ValueCount = 38;
constexpr int PayloadValueCount = 37;
constexpr double InvalidSentinel = -900.0;

bool isInvalidValue(double value)
{
    return !std::isfinite(value) || value <= InvalidSentinel;
}

QStringList splitCsvLine(const QString& line)
{
    QStringList cells = line.split(',');
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
    bool ok = false;
    const double parsed = text.toDouble(&ok);
    if (!ok)
    {
        return false;
    }
    *value = parsed;
    return true;
}

bool hasJustFloatHeader(const QStringList& cells)
{
    return cells.contains(QStringLiteral("I0")) &&
           cells.contains(QStringLiteral("I1")) &&
           cells.contains(QStringLiteral("I37"));
}

bool buildHeaderMap(const QStringList& cells, std::array<int, ValueCount>* map, QString* errorMessage)
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
        if (ok && index >= 0 && index < ValueCount)
        {
            (*map)[index] = i;
        }
    }

    for (int i = 0; i < ValueCount; ++i)
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
    return true;
}

bool parseValues(const QStringList& cells,
                 const std::array<int, ValueCount>& map,
                 int lineNumber,
                 std::array<double, ValueCount>* values,
                 QString* errorMessage)
{
    for (int i = 0; i < ValueCount; ++i)
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
        if (!parseDoubleCell(cells[cellIndex], &(*values)[i]))
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

bool parseSequentialValues(const QStringList& cells,
                           quint64 sequence,
                           std::array<double, ValueCount>* values,
                           QString* errorMessage)
{
    if (cells.size() != PayloadValueCount && cells.size() != ValueCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("字段数量应为 37 或 38，实际为 %1。").arg(cells.size());
        }
        return false;
    }

    values->fill(0.0);
    const int shift = cells.size() == PayloadValueCount ? 1 : 0;
    if (shift != 0)
    {
        (*values)[0] = (double)sequence;
    }

    for (int i = 0; i < cells.size(); ++i)
    {
        double parsed = 0.0;
        QString cell = cells[i].trimmed();
        const int equalIndex = cell.indexOf(QLatin1Char('='));
        if (equalIndex >= 0)
        {
            cell = cell.mid(equalIndex + 1).trimmed();
        }
        if (!parseDoubleCell(cell, &parsed))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("第 %1 个字段不是有效浮点数。").arg(i);
            }
            return false;
        }
        (*values)[i + shift] = parsed;
    }
    return true;
}

bool hasVofaTail(const QByteArray& data)
{
    const int size = data.size();
    return size >= 4 &&
           (unsigned char)data[size - 4] == 0x00 &&
           (unsigned char)data[size - 3] == 0x00 &&
           (unsigned char)data[size - 2] == 0x80 &&
           (unsigned char)data[size - 1] == 0x7f;
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
                         std::array<double, ValueCount>* values,
                         QString* errorMessage)
{
    const QByteArray payload = withoutVofaTail(datagram);
    const int floatCount = payload.size() / 4;
    if (payload.size() % 4 != 0 ||
        (floatCount != PayloadValueCount && floatCount != ValueCount))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("二进制长度不匹配：%1 字节。").arg(datagram.size());
        }
        return false;
    }

    values->fill(0.0);
    const int shift = floatCount == PayloadValueCount ? 1 : 0;
    if (shift != 0)
    {
        (*values)[0] = (double)sequence;
    }

    for (int i = 0; i < floatCount; ++i)
    {
        const uchar* source = reinterpret_cast<const uchar*>(payload.constData() + i * 4);
        const quint32 bits = qFromLittleEndian<quint32>(source);
        float value = 0.0f;
        memcpy(&value, &bits, sizeof(value));
        (*values)[i + shift] = (double)value;
    }
    return true;
}

bool parseTextDatagram(const QByteArray& datagram,
                       quint64 sequence,
                       std::array<double, ValueCount>* values,
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
    return parseSequentialValues(splitCsvLine(text), sequence, values, errorMessage);
}

JustFloatBeacon makeBeacon(const std::array<double, ValueCount>& values, int offset)
{
    JustFloatBeacon beacon;
    beacon.x = (float)values[offset];
    beacon.y = (float)values[offset + 1];
    beacon.area = (float)values[offset + 2];
    beacon.valid = !isInvalidValue(values[offset]) &&
                   !isInvalidValue(values[offset + 1]) &&
                   values[offset + 2] > 0.0;
    return beacon;
}

JustFloatCarLamp makeCarLamp(const std::array<double, ValueCount>& values, int offset)
{
    JustFloatCarLamp lamp;
    lamp.cx = (float)values[offset];
    lamp.cy = (float)values[offset + 1];
    lamp.angle = (float)values[offset + 2];
    lamp.width = (float)values[offset + 3];
    lamp.length = (float)values[offset + 4];
    lamp.valid = !isInvalidValue(values[offset]) &&
                 !isInvalidValue(values[offset + 1]) &&
                 !isInvalidValue(values[offset + 2]) &&
                 values[offset + 3] > 0.0 &&
                 values[offset + 4] > 0.0;
    return lamp;
}

JustFloatLogRow makeRow(const std::array<double, ValueCount>& values)
{
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
    row.pitch = (float)values[34];
    row.roll = (float)values[35];
    row.yaw = (float)values[36];
    row.syncTimeMs = values[37];
    return row;
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
    std::array<int, ValueCount> map;
    QVector<JustFloatLogRow> rows;

    if (hasJustFloatHeader(firstCells))
    {
        if (!buildHeaderMap(firstCells, &map, errorMessage))
        {
            return false;
        }
    }
    else
    {
        map.fill(-1);
        for (int i = 0; i < ValueCount; ++i)
        {
            map[i] = i;
        }

        std::array<double, ValueCount> values;
        if (!parseValues(firstCells, map, lineNumber, &values, errorMessage))
        {
            return false;
        }
        rows.push_back(makeRow(values));
    }

    while (!stream.atEnd())
    {
        const QString line = stream.readLine();
        ++lineNumber;
        if (line.trimmed().isEmpty())
        {
            continue;
        }

        std::array<double, ValueCount> values;
        if (!parseValues(splitCsvLine(line), map, lineNumber, &values, errorMessage))
        {
            return false;
        }
        rows.push_back(makeRow(values));
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

    std::array<double, ValueCount> values;
    QString binaryError;
    if (parseBinaryDatagram(datagram, sequence, &values, &binaryError))
    {
        *output = makeRow(values);
        return true;
    }

    QString textError;
    if (parseTextDatagram(datagram, sequence, &values, &textError))
    {
        *output = makeRow(values);
        return true;
    }

    if (errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("%1；%2").arg(binaryError, textError);
    }
    return false;
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
