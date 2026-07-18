#ifndef JUST_FLOAT_LOG_H
#define JUST_FLOAT_LOG_H

#include <QByteArray>
#include <QString>
#include <QVector>

struct JustFloatBeacon
{
    float x = 0.0f;
    float y = 0.0f;
    float area = 0.0f;
    bool valid = false;
};

struct JustFloatCarLamp
{
    float cx = 0.0f;
    float cy = 0.0f;
    float angle = 0.0f;
    float width = 0.0f;
    float length = 0.0f;
    bool valid = false;
};

struct JustFloatCameraFrame
{
    JustFloatBeacon beacons[2];
    JustFloatCarLamp carLamp;
};

struct JustFloatLogRow
{
    double rowTime = 0.0;
    double syncTimeMs = 0.0;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    JustFloatCameraFrame cameras[3];
    float projectionXcm = 0.0f;
    float projectionYcm = 0.0f;
    bool hasProjectionDistance = false;
};

struct JustFloatChannelDescriptor
{
    int index = -1;
    QString group;
    QString name;
    QString unit;
};

class JustFloatLog
{
public:
    static constexpr int LegacyChannelCount = 38;
    static constexpr int ChannelCount = 40;

    static bool loadCsv(const QString& path, JustFloatLog* output, QString* errorMessage = nullptr);
    static bool parseDatagram(const QByteArray& datagram,
                              quint64 sequence,
                              JustFloatLogRow* output,
                              QString* errorMessage = nullptr);
    static const QVector<JustFloatChannelDescriptor>& channelDescriptors();
    static bool channelValue(const JustFloatLogRow& row, int channelIndex, double* value);
    static QString csvHeader();
    static QString csvRow(const JustFloatLogRow& row);

    QString sourcePath() const;
    int rowCount() const;
    const JustFloatLogRow& rowAt(int index) const;

private:
    QString m_sourcePath;
    QVector<JustFloatLogRow> m_rows;
};

#endif
