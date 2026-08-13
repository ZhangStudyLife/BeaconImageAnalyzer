#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <QByteArray>
#include <QString>
#include <QVector>

#include <array>

struct BeaconSample
{
    float x = 0.0f;
    float y = 0.0f;
    float area = 0.0f;
    bool valid = false;
};

struct CarLampSample
{
    float cx = 0.0f;
    float cy = 0.0f;
    float angle = 0.0f;
    float width = 0.0f;
    float length = 0.0f;
    bool valid = false;
};

struct CameraSample
{
    std::array<BeaconSample, 2> beacons;
    CarLampSample carLamp;
};

struct TelemetryFrame
{
    std::array<float, 43> channels{};
    std::array<CameraSample, 3> cameras;
    float timestampMs = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    float syncTimestampMs = 0.0f;
    float reserved = 0.0f;
    float carForwardVelocity = 0.0f;
    float carYaw = 0.0f;
    float plannedForwardVelocity = 0.0f;
    float plannedStrafeVelocity = 0.0f;
};

class TelemetryProtocol
{
public:
    static constexpr int ChannelCount = 43;
    static constexpr int PayloadBytes = ChannelCount * static_cast<int>(sizeof(float));
    static constexpr int TailBytes = 4;

    static bool parseDatagram(const QByteArray& datagram,
                              TelemetryFrame* frame,
                              QString* errorMessage = nullptr);
    static QString csvHeader();
    static QString csvRow(const TelemetryFrame& frame);
    static bool loadCsv(const QString& path,
                        QVector<TelemetryFrame>* frames,
                        QString* errorMessage = nullptr);
    static double playbackTimestampMs(const TelemetryFrame& frame);

private:
    static bool makeFrame(const std::array<float, ChannelCount>& values,
                          TelemetryFrame* frame,
                          QString* errorMessage);
};

#endif
