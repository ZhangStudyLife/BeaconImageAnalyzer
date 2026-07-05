#include "JustFloatLog.h"

#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <array>
#include <cmath>

namespace
{
constexpr int ValueCount = 38;
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
