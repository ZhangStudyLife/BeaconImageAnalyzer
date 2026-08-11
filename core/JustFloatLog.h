#ifndef JUST_FLOAT_LOG_H
#define JUST_FLOAT_LOG_H

#include <QByteArray>
#include <QString>
#include <QVector>

enum class JustFloatLogLayout
{
    Legacy,
    SingleLampRoiV1,
    DualLampFusionV1
};

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

struct JustFloatFusedCarLamp
{
    bool valid = false;
    float cx = 0.0f;
    float cy = 0.0f;
    float angle = 0.0f;
};

struct JustFloatMappedPoint
{
    float x = 0.0f;
    float y = 0.0f;
    bool valid = false;
};

struct JustFloatMappedCarLamp
{
    float cx = 0.0f;
    float cy = 0.0f;
    float angle = 0.0f;
    bool valid = false;
    bool measured = false;
};

struct JustFloatMappedCameraFrame
{
    JustFloatMappedCarLamp carLamps[2];
    JustFloatMappedPoint beacons[2];
    quint8 measuredMask = 0;
};

struct JustFloatControlGeometry
{
    int planMode = 1;
    JustFloatMappedCarLamp car;
    JustFloatMappedPoint beacon;
};

struct JustFloatShadowCarCenter
{
    int confidence = 0;
    float cx = 0.0f;
    float cy = 0.0f;
    float axisAngle = 0.0f;
    bool valid = false;
    JustFloatMappedPoint lamps[2];
};

struct JustFloatDualLampFusionFrame
{
    float schemaId = 0.0f;
    quint32 imageSequence = 0;
    JustFloatMappedCameraFrame cameras[3];
    JustFloatControlGeometry control;
    JustFloatShadowCarCenter shadow;
};

struct JustFloatSingleLampShape
{
    quint32 packed = 0;
    quint8 widthCode = 0;
    quint8 lengthCode = 0;
    quint8 angleCode = 0;
    quint8 nearestBeaconDistanceCode = 15;
    float width = 0.0f;
    float length = 0.0f;
    float angle = 0.0f;
    float nearestBeaconDistance = 0.0f;
    bool valid = false;
    bool nearestBeaconDistanceValid = false;
};

struct JustFloatSingleLampTrackGeometry
{
    quint32 packed = 0;
    float centerX = 0.0f;
    float centerY = 0.0f;
    float centerRoiHalfSize = 0.0f;
    bool valid = false;
};

struct JustFloatSingleLampCrossCheck
{
    quint32 packed = 0;
    quint8 state = 0;
    quint8 supportCameraMask = 0;
    quint8 roiValidMask = 0;
    quint8 roiHitMask = 0;
    quint8 conflictCameraMask = 0;
    quint8 fullFrameFallbackMask = 0;
    quint8 measuredCameraMask = 0;
    bool projectionEnabled = false;
    bool manuallyMarked = false;
    bool roiMode = false;
};

struct JustFloatSingleLampSourceFrame
{
    quint8 sequenceLow7 = 0;
    bool valid = false;
};

struct JustFloatSingleLampRoiFrame
{
    JustFloatSingleLampShape lampShapes[3];
    JustFloatSingleLampTrackGeometry trackGeometry;
    JustFloatSingleLampCrossCheck crossCheck;
    JustFloatSingleLampSourceFrame sourceFrames[3];
    quint32 frameSequencePacked = 0;
    float heightMm = 0.0f;
    float relativeYawDeg = 0.0f;
    float maxSkewMs = 0.0f;
    bool heightValid = false;
    bool relativeYawValid = false;
    bool maxSkewValid = false;
};

struct JustFloatLogRow
{
    JustFloatLogLayout layout = JustFloatLogLayout::Legacy;
    double rowTime = 0.0;
    double syncTimeMs = 0.0;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    JustFloatCameraFrame cameras[3];
    float actualVelocityX = 0.0f;
    float actualVelocityY = 0.0f;
    float vehicleYawDeg = 0.0f;
    float targetForwardMps = 0.0f;
    float targetStrafeMps = 0.0f;
    bool hasMotionData = false;
    JustFloatFusedCarLamp fusedCarLamp;
    bool hasFusedCarLampData = false;
    JustFloatSingleLampRoiFrame singleLampRoi;
    JustFloatDualLampFusionFrame dualLampFusion;
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
    static constexpr int SingleLampRoiChannelCount = 36;
    static constexpr int LegacyChannelCount = 38;
    static constexpr int MotionChannelCount = 43;
    static constexpr int LegacyFusedChannelCount = 47;
    static constexpr int DualLampFusionChannelCount = 50;
    static constexpr int ChannelCount = DualLampFusionChannelCount;
    static constexpr float DualLampFusionSchemaId = 260808.0f;

    static bool loadCsv(const QString& path, JustFloatLog* output, QString* errorMessage = nullptr);
    static bool parseDatagram(const QByteArray& datagram,
                              double fallbackTimestampMs,
                              JustFloatLogRow* output,
                              QString* errorMessage = nullptr);
    static const QVector<JustFloatChannelDescriptor>& channelDescriptors();
    static const QVector<JustFloatChannelDescriptor>& channelDescriptors(JustFloatLogLayout layout);
    static int channelCount(JustFloatLogLayout layout);
    static bool channelValue(const JustFloatLogRow& row, int channelIndex, double* value);
    static QString csvHeader();
    static QString csvHeader(JustFloatLogLayout layout);
    static QString csvRow(const JustFloatLogRow& row);
    static QString layoutName(JustFloatLogLayout layout);

    QString sourcePath() const;
    int rowCount() const;
    const JustFloatLogRow& rowAt(int index) const;
    JustFloatLogLayout layout() const;

private:
    QString m_sourcePath;
    QVector<JustFloatLogRow> m_rows;
    JustFloatLogLayout m_layout = JustFloatLogLayout::Legacy;
};

#endif
