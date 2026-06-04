#ifndef CROSS_CAMERA_BEACON_TRACKER_H
#define CROSS_CAMERA_BEACON_TRACKER_H

#include "ImageResult.h"

#include <QPointF>
#include <QString>
#include <QVector>

#include <array>

struct TrackedBeaconPoint
{
    int cameraIndex = -1;
    int frame = 0;
    QPointF imagePoint;
    float radius = 0.0f;
    float area = 0.0f;
    bool valid = false;
};

class CrossCameraBeaconTracker
{
public:
    static constexpr int CameraCount = 3;

    void reset();
    bool start(const std::array<beacon_result_t, CameraCount>& results, int timelineFrame, QString* errorMessage);
    bool update(const std::array<beacon_result_t, CameraCount>& results, int timelineFrame, QString* errorMessage);
    bool active() const;
    TrackedBeaconPoint currentPointForCamera(int cameraIndex) const;
    QString statusText() const;

private:
    struct Candidate
    {
        int cameraIndex = -1;
        QPointF imagePoint;
        float radius = 0.0f;
        float area = 0.0f;
        bool valid = false;
    };

    QVector<Candidate> candidatesForCamera(const beacon_result_t& result, int cameraIndex) const;
    Candidate findLargestCandidate(const std::array<beacon_result_t, CameraCount>& results) const;
    bool updateInCurrentCamera(const QVector<Candidate>& candidates,
                               int timelineFrame,
                               TrackedBeaconPoint* nextPoint) const;
    bool updateAcrossCamera(const std::array<beacon_result_t, CameraCount>& results,
                            int timelineFrame,
                            TrackedBeaconPoint* nextPoint) const;
    QPointF predictedImagePoint(int timelineFrame) const;
    void appendPoint(const TrackedBeaconPoint& point);

    QVector<TrackedBeaconPoint> m_history;
    TrackedBeaconPoint m_predictedPoint;
    bool m_active = false;
    int m_missingFrameCount = 0;
};

#endif
