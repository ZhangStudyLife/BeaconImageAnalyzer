#include "CrossCameraBeaconTracker.h"

#include "CameraBoundaryMapping.h"

#include <QtGlobal>
#include <cmath>

#define TRACKER_HISTORY_FRAME_LIMIT          5
#define TRACKER_MIN_TARGET_AREA              1.0f
#define TRACKER_BASE_MATCH_RADIUS            14.0f
#define TRACKER_RADIUS_MATCH_SCALE           3.0f
#define TRACKER_CROSS_CAMERA_MATCH_RADIUS    30.0f
#define TRACKER_MAX_MISSING_FRAMES           3
#define TRACKER_FRONT_CAMERA_INDEX           0
#define TRACKER_CENTER_CAMERA_INDEX          1
#define TRACKER_REAR_CAMERA_INDEX            2

namespace
{
float square(float value)
{
    return value * value;
}

float distanceSquared(const QPointF& lhs, const QPointF& rhs)
{
    const float dx = (float)(lhs.x() - rhs.x());
    const float dy = (float)(lhs.y() - rhs.y());
    return dx * dx + dy * dy;
}

float candidateScore(float area, float radius)
{
    if (area > 0.0f)
    {
        return area;
    }
    return radius * radius;
}

QPointF algorithmToImagePoint(float x, float y)
{
    return QPointF(BEACON_IMAGE_TARGET_PIXEL_X - x,
                   BEACON_IMAGE_TARGET_PIXEL_Y + y);
}

bool mapBoundaryToCenter(int sourceCameraIndex, const QPointF& sourcePoint, QPointF* centerPoint)
{
    if (centerPoint == nullptr)
    {
        return false;
    }

    if (sourceCameraIndex == TRACKER_FRONT_CAMERA_INDEX)
    {
        const float sourceX = (float)sourcePoint.x();
        *centerPoint = CameraBoundaryMapping::frontBoundaryToCenter(sourceX);
        return true;
    }
    if (sourceCameraIndex == TRACKER_REAR_CAMERA_INDEX)
    {
        const float sourceX = (float)sourcePoint.x();
        *centerPoint = CameraBoundaryMapping::rearBoundaryToCenter(sourceX);
        return true;
    }
    return false;
}
}

void CrossCameraBeaconTracker::reset()
{
    m_history.clear();
    m_predictedPoint = TrackedBeaconPoint();
    m_active = false;
    m_missingFrameCount = 0;
}

bool CrossCameraBeaconTracker::start(const std::array<beacon_result_t, CameraCount>& results,
                                     int timelineFrame,
                                     QString* errorMessage)
{
    reset();
    const Candidate candidate = findLargestCandidate(results);
    if (!candidate.valid)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("当前帧没有可追踪的信标灯。");
        }
        return false;
    }

    TrackedBeaconPoint point;
    point.cameraIndex = candidate.cameraIndex;
    point.frame = timelineFrame;
    point.imagePoint = candidate.imagePoint;
    point.radius = candidate.radius;
    point.area = candidate.area;
    point.valid = true;
    appendPoint(point);
    m_predictedPoint = TrackedBeaconPoint();
    m_missingFrameCount = 0;
    m_active = true;
    return true;
}

bool CrossCameraBeaconTracker::update(const std::array<beacon_result_t, CameraCount>& results,
                                      int timelineFrame,
                                      QString* errorMessage)
{
    if (!m_active || m_history.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("追踪尚未启动。");
        }
        return false;
    }

    TrackedBeaconPoint nextPoint;
    const int currentCameraIndex = m_history.back().cameraIndex;
    if (currentCameraIndex >= 0 && currentCameraIndex < CameraCount &&
        updateInCurrentCamera(candidatesForCamera(results[(size_t)currentCameraIndex], currentCameraIndex),
                              timelineFrame,
                              &nextPoint))
    {
        appendPoint(nextPoint);
        m_predictedPoint = TrackedBeaconPoint();
        m_missingFrameCount = 0;
        return true;
    }

    if (updateAcrossCamera(results, timelineFrame, &nextPoint))
    {
        appendPoint(nextPoint);
        m_predictedPoint = TrackedBeaconPoint();
        m_missingFrameCount = 0;
        return true;
    }

    if (m_missingFrameCount < TRACKER_MAX_MISSING_FRAMES)
    {
        const TrackedBeaconPoint& lastPoint = m_history.back();
        m_predictedPoint = lastPoint;
        m_predictedPoint.frame = timelineFrame;
        m_predictedPoint.imagePoint = predictedImagePoint(timelineFrame);
        m_predictedPoint.valid = true;
        ++m_missingFrameCount;
        return true;
    }

    m_active = false;
    if (errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("追踪失败：当前帧未找到可匹配的同一信标灯。");
    }
    return false;
}

bool CrossCameraBeaconTracker::active() const
{
    return m_active;
}

TrackedBeaconPoint CrossCameraBeaconTracker::currentPointForCamera(int cameraIndex) const
{
    if (m_active && m_predictedPoint.valid && m_predictedPoint.cameraIndex == cameraIndex)
    {
        return m_predictedPoint;
    }
    if (!m_active || m_history.isEmpty() || m_history.back().cameraIndex != cameraIndex)
    {
        return TrackedBeaconPoint();
    }
    return m_history.back();
}

QString CrossCameraBeaconTracker::statusText() const
{
    if (!m_active || m_history.isEmpty())
    {
        return QStringLiteral("追踪：未启动");
    }
    const TrackedBeaconPoint& point = m_history.back();
    return QStringLiteral("追踪：摄像头 %1  X=%2  Y=%3")
        .arg(point.cameraIndex + 1)
        .arg(point.imagePoint.x(), 0, 'f', 1)
        .arg(point.imagePoint.y(), 0, 'f', 1);
}

QVector<CrossCameraBeaconTracker::Candidate> CrossCameraBeaconTracker::candidatesForCamera(
    const beacon_result_t& result,
    int cameraIndex) const
{
    QVector<Candidate> candidates;
    const int count = qMin((int)result.beacon_count, BEACON_MAX_CIRCLE_COUNT);
    for (int i = 0; i < count; ++i)
    {
        const beacon_circle_t& beacon = result.beacons[i];
        if (beacon.valid == 0)
        {
            continue;
        }

        Candidate candidate;
        candidate.cameraIndex = cameraIndex;
        candidate.imagePoint = algorithmToImagePoint(beacon.x, beacon.y);
        candidate.radius = beacon.radius;
        candidate.area = beacon.area;
        candidate.valid = true;
        if (candidateScore(candidate.area, candidate.radius) < TRACKER_MIN_TARGET_AREA)
        {
            continue;
        }
        candidates.push_back(candidate);
    }
    return candidates;
}

CrossCameraBeaconTracker::Candidate CrossCameraBeaconTracker::findLargestCandidate(
    const std::array<beacon_result_t, CameraCount>& results) const
{
    Candidate best;
    float bestScore = 0.0f;
    for (int cameraIndex = 0; cameraIndex < CameraCount; ++cameraIndex)
    {
        const QVector<Candidate> candidates = candidatesForCamera(results[(size_t)cameraIndex], cameraIndex);
        for (const Candidate& candidate : candidates)
        {
            const float score = candidateScore(candidate.area, candidate.radius);
            if (!best.valid || score > bestScore)
            {
                best = candidate;
                bestScore = score;
            }
        }
    }
    return best;
}

bool CrossCameraBeaconTracker::updateInCurrentCamera(const QVector<Candidate>& candidates,
                                                     int timelineFrame,
                                                     TrackedBeaconPoint* nextPoint) const
{
    if (nextPoint == nullptr || candidates.isEmpty() || m_history.isEmpty())
    {
        return false;
    }

    const TrackedBeaconPoint& lastPoint = m_history.back();
    const QPointF prediction = predictedImagePoint(timelineFrame);
    const float matchRadius = qMax(TRACKER_BASE_MATCH_RADIUS, lastPoint.radius * TRACKER_RADIUS_MATCH_SCALE);
    float bestDistance = 0.0f;
    Candidate best;

    for (const Candidate& candidate : candidates)
    {
        const float dist2 = distanceSquared(candidate.imagePoint, prediction);
        if (dist2 > square(matchRadius))
        {
            continue;
        }
        if (!best.valid || dist2 < bestDistance)
        {
            best = candidate;
            bestDistance = dist2;
        }
    }

    if (!best.valid)
    {
        return false;
    }

    nextPoint->cameraIndex = best.cameraIndex;
    nextPoint->frame = timelineFrame;
    nextPoint->imagePoint = best.imagePoint;
    nextPoint->radius = best.radius;
    nextPoint->area = best.area;
    nextPoint->valid = true;
    return true;
}

bool CrossCameraBeaconTracker::updateAcrossCamera(const std::array<beacon_result_t, CameraCount>& results,
                                                  int timelineFrame,
                                                  TrackedBeaconPoint* nextPoint) const
{
    if (nextPoint == nullptr || m_history.isEmpty())
    {
        return false;
    }

    const TrackedBeaconPoint& lastPoint = m_history.back();
    QPointF mappedCenter;
    if (!mapBoundaryToCenter(lastPoint.cameraIndex, lastPoint.imagePoint, &mappedCenter))
    {
        return false;
    }

    const QVector<Candidate> centerCandidates =
        candidatesForCamera(results[(size_t)TRACKER_CENTER_CAMERA_INDEX], TRACKER_CENTER_CAMERA_INDEX);
    Candidate best;
    float bestDistance = 0.0f;
    for (const Candidate& candidate : centerCandidates)
    {
        const float dist2 = distanceSquared(candidate.imagePoint, mappedCenter);
        if (dist2 > square(TRACKER_CROSS_CAMERA_MATCH_RADIUS))
        {
            continue;
        }
        if (!best.valid || dist2 < bestDistance)
        {
            best = candidate;
            bestDistance = dist2;
        }
    }

    if (!best.valid)
    {
        return false;
    }

    nextPoint->cameraIndex = best.cameraIndex;
    nextPoint->frame = timelineFrame;
    nextPoint->imagePoint = best.imagePoint;
    nextPoint->radius = best.radius;
    nextPoint->area = best.area;
    nextPoint->valid = true;
    return true;
}

QPointF CrossCameraBeaconTracker::predictedImagePoint(int timelineFrame) const
{
    if (m_history.isEmpty())
    {
        return QPointF();
    }
    const TrackedBeaconPoint& lastPoint = m_history.back();
    QPointF velocity(0.0, 0.0);
    int velocityCount = 0;
    const int firstIndex = qMax(1, m_history.size() - TRACKER_HISTORY_FRAME_LIMIT);
    for (int i = firstIndex; i < m_history.size(); ++i)
    {
        const TrackedBeaconPoint& previous = m_history[i - 1];
        const TrackedBeaconPoint& current = m_history[i];
        if (previous.cameraIndex != current.cameraIndex)
        {
            continue;
        }
        const int frameDelta = current.frame - previous.frame;
        if (frameDelta <= 0)
        {
            continue;
        }
        velocity += (current.imagePoint - previous.imagePoint) / (qreal)frameDelta;
        ++velocityCount;
    }
    if (velocityCount > 0)
    {
        velocity /= (qreal)velocityCount;
    }
    return lastPoint.imagePoint + velocity * (qreal)(timelineFrame - lastPoint.frame);
}

void CrossCameraBeaconTracker::appendPoint(const TrackedBeaconPoint& point)
{
    if (!point.valid)
    {
        return;
    }
    m_history.push_back(point);
    while (m_history.size() > TRACKER_HISTORY_FRAME_LIMIT)
    {
        m_history.pop_front();
    }
}
