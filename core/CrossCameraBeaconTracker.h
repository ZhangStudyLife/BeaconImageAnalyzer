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

struct ExpectedBeaconSearchArea
{
    int cameraIndex = -1;
    int sourceCameraIndex = -1;
    QPointF center;
    float radius = 0.0f;
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
    ExpectedBeaconSearchArea expectedSearchAreaForCamera(int cameraIndex) const;
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
                            const TrackedBeaconPoint& sourcePoint,
                            int timelineFrame,
                            TrackedBeaconPoint* nextPoint) const;
    bool isNearCropBoundary(int cameraIndex, const QPointF& point, QPointF* boundaryPoint) const;
    Candidate findNearestCandidate(const QVector<Candidate>& candidates,
                                   const QPointF& center,
                                   float radius) const;
    void appendPoint(const TrackedBeaconPoint& point);

    QVector<TrackedBeaconPoint> m_history;
    bool m_active = false;
    int m_missingFrameCount = 0;
};

#endif
