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

struct FusedBeaconSample
{
    float x = 0.0f;
    float y = 0.0f;
    float area = 0.0f;
    int cameraMask = 0;
    bool valid = false;
};

struct GlobalBeaconSample
{
    float x = 0.0f;
    float y = 0.0f;
    float area = 0.0f;
    int cameraMask = 0;
    bool valid = false;
};

struct GlobalLampSample
{
    float x = 0.0f;
    float y = 0.0f;
    float angleDeg = 0.0f;
    int cameraMask = 0;
    bool valid = false;
};

struct CarLampSample
{
    float cx = 0.0f;
    float cy = 0.0f;
    float angle = 0.0f;
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
    std::array<float, 71> channels{};
    int channelCount = 71;
    float timestampMs = 0.0f;
    std::array<CameraSample, 3> cameras;
    std::array<FusedBeaconSample, 3> centerBeacons;
    CarLampSample centerCarLamp;
    std::array<BeaconSample, 3> modelBeacons;
    CarLampSample modelCarLamp;
    std::array<std::array<BeaconSample, 2>, 3> globalCandidates;
    std::array<GlobalBeaconSample, 4> globalBeacons;
    GlobalLampSample globalCarLamp;
    bool carPlan3Valid = false;
    bool carPlan3Direct = false;
    float carYawDeg = 0.0f;
    float carActualVelocityX = 0.0f;
    float carActualVelocityY = 0.0f;
    float carTargetVelocityX = 0.0f;
    float carTargetVelocityY = 0.0f;
    float aircraftHeightMm = 0.0f;
    float aircraftRollDeg = 0.0f;
    float aircraftPitchDeg = 0.0f;
    float aircraftYawDeg = 0.0f;
    int selectedTargetId = -1;
    bool markerActive = false;
};

class TelemetryProtocol
{
public:
    static constexpr int LegacyChannelCount = 71;
    static constexpr int CarPlan3ChannelCount = 64;
    static constexpr int ChannelCount = LegacyChannelCount;
    static constexpr int LegacyPayloadBytes = LegacyChannelCount * static_cast<int>(sizeof(float));
    static constexpr int CarPlan3PayloadBytes = CarPlan3ChannelCount * static_cast<int>(sizeof(float));
    static constexpr int TailBytes = 4;

    static bool parseDatagram(const QByteArray& datagram,
                              TelemetryFrame* frame,
                              QString* errorMessage = nullptr);
    static QString csvHeader(int channelCount = LegacyChannelCount);
    static QString csvRow(const TelemetryFrame& frame);
    static bool saveCsv(const QString& path,
                        const QVector<TelemetryFrame>& frames,
                        QString* errorMessage = nullptr);
    static bool loadCsv(const QString& path,
                        QVector<TelemetryFrame>* frames,
                        QString* errorMessage = nullptr);
    static double playbackTimestampMs(const TelemetryFrame& frame);

private:
    static bool makeFrame(const std::array<float, LegacyChannelCount>& values,
                          int channelCount,
                          TelemetryFrame* frame,
                          QString* errorMessage);
};

#endif
