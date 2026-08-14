#include "CarPlan3Model.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double DegToRad = 0.017453292519943295;
constexpr double BeaconHeightM = 0.0233670966;
constexpr double LampHeightM = 0.2490776486;
constexpr double YawBiasRad = 0.4068566800;
constexpr double MergeDistanceM = 0.35;
constexpr double MaxGroundDistanceM = 15.0;
constexpr double NearLampPx = 3.0;
constexpr double MatchPx = 15.0;
constexpr double FarLampPx = 10.0;
constexpr int HistoryTicks = 30;
constexpr int GapTicks = 2;
constexpr int ConfirmTicks = 3;

struct CameraModel
{
    double f;
    double cx;
    double cy;
    double k1;
    double cameraToBody[3][3];
};

constexpr CameraModel Models[3] = {
    {77.2632115, 10.0431456, 7.1929172, 0.7,
     {{-0.053257901, -0.823901336, 0.564225295},
      {0.997565442, -0.018423355, 0.067258975},
      {-0.045019836, 0.566433728, 0.822876690}}},
    {81.0801880, 5.4973460, 1.2174406, 0.7,
     {{-0.032443015, -0.995479870, -0.089259618},
      {0.996951194, -0.038572645, 0.067826744},
      {-0.070963138, -0.086786978, 0.993696258}}},
    {82.3499934, -1.1395418, -5.2939230, 0.7,
     {{0.156584158, 0.727257310, -0.668265072},
      {-0.985382340, 0.069061905, -0.155730851},
      {-0.067104741, 0.682881584, 0.727440510}}}
};

struct Candidate
{
    int camera = 0;
    double x = 0.0;
    double y = 0.0;
    float area = 0.0f;
};

void worldRotation(const TelemetryFrame& frame, double out[3][3])
{
    const double roll = frame.aircraftRollDeg * DegToRad;
    const double pitch = frame.aircraftPitchDeg * DegToRad;
    const double yaw = frame.aircraftYawDeg * DegToRad + YawBiasRad;
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    out[0][0] = cp * cy;
    out[0][1] = sr * sp * cy - cr * sy;
    out[0][2] = cr * sp * cy + sr * sy;
    out[1][0] = cp * sy;
    out[1][1] = sr * sp * sy + cr * cy;
    out[1][2] = cr * sp * sy - sr * cy;
    out[2][0] = -sp;
    out[2][1] = sr * cp;
    out[2][2] = cr * cp;
}

bool projectPoint(int camera,
                  double x,
                  double y,
                  double targetHeight,
                  double aircraftHeight,
                  const double world[3][3],
                  double* outX,
                  double* outY)
{
    const CameraModel& model = Models[camera];
    double rayX = (x - model.cx) / model.f;
    double rayY = (y - model.cy) / model.f;
    const double gain = 1.0 + model.k1 * (rayX * rayX + rayY * rayY);
    rayX *= gain;
    rayY *= gain;
    double body[3]{};
    double projected[3]{};
    for (int row = 0; row < 3; ++row)
    {
        body[row] = model.cameraToBody[row][0] * rayX +
                    model.cameraToBody[row][1] * rayY +
                    model.cameraToBody[row][2];
    }
    for (int row = 0; row < 3; ++row)
    {
        projected[row] = world[row][0] * body[0] +
                         world[row][1] * body[1] +
                         world[row][2] * body[2];
    }
    if (projected[2] <= 0.0001)
    {
        return false;
    }
    const double distance = (aircraftHeight - targetHeight) / projected[2];
    if (distance <= 0.0 || distance > MaxGroundDistanceM)
    {
        return false;
    }
    *outX = distance * projected[0];
    *outY = distance * projected[1];
    return true;
}

bool projectLampAngle(int camera,
                      const CarLampSample& lamp,
                      double aircraftHeight,
                      const double world[3][3],
                      double* angleDeg)
{
    const double angle = lamp.angle * DegToRad;
    const double halfLength = lamp.length * 0.5;
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
    if (halfLength <= 0.0 ||
        !projectPoint(camera, lamp.cx - halfLength * std::cos(angle),
                      lamp.cy - halfLength * std::sin(angle), LampHeightM,
                      aircraftHeight, world, &x1, &y1) ||
        !projectPoint(camera, lamp.cx + halfLength * std::cos(angle),
                      lamp.cy + halfLength * std::sin(angle), LampHeightM,
                      aircraftHeight, world, &x2, &y2))
    {
        return false;
    }
    *angleDeg = std::atan2(y2 - y1, x2 - x1) / DegToRad;
    return true;
}
}

CarPlan3Model::CarPlan3Model()
{
    reset();
}

void CarPlan3Model::reset()
{
    m_tracks = {};
    for (auto& camera : m_tracks)
    {
        for (Track& track : camera)
        {
            track.farAge = HistoryTicks + 1;
            track.suspectAge = HistoryTicks + 1;
        }
    }
}

void CarPlan3Model::process(TelemetryFrame* frame)
{
    if (frame == nullptr)
    {
        return;
    }
    for (auto& camera : frame->globalCandidates)
    {
        camera = {};
    }
    if (!std::isfinite(frame->aircraftHeightMm) || frame->aircraftHeightMm <= 0.0f)
    {
        return;
    }

    const double aircraftHeight = frame->aircraftHeightMm * 0.001;
    double world[3][3]{};
    worldRotation(*frame, world);
    for (int camera = 0; camera < 3; ++camera)
    {
        for (int slot = 0; slot < 2; ++slot)
        {
            const BeaconSample& raw = frame->cameras[camera].beacons[slot];
            BeaconSample& projected = frame->globalCandidates[camera][slot];
            double x = 0.0;
            double y = 0.0;
            if (raw.valid && projectPoint(camera, raw.x, raw.y, BeaconHeightM,
                                          aircraftHeight, world, &x, &y))
            {
                projected.x = static_cast<float>(x);
                projected.y = static_cast<float>(y);
                projected.area = raw.area;
                projected.valid = true;
            }
        }
    }
    if (frame->carPlan3Direct)
    {
        return;
    }

    frame->globalBeacons = {};
    frame->globalCarLamp = {};
    frame->carPlan3Valid = false;
    frame->selectedTargetId = -1;
    bool filtered[3][2]{};

    for (int camera = 0; camera < 3; ++camera)
    {
        bool used[4]{};
        const CarLampSample& lamp = frame->cameras[camera].carLamp;
        for (Track& track : m_tracks[camera])
        {
            if (track.valid && track.farAge <= HistoryTicks) ++track.farAge;
            if (track.valid && track.suspectAge <= HistoryTicks) ++track.suspectAge;
        }
        for (int slot = 0; slot < 2; ++slot)
        {
            const BeaconSample& beacon = frame->cameras[camera].beacons[slot];
            if (!beacon.valid)
            {
                continue;
            }
            int best = -1;
            bool matched = false;
            double bestDistance = MatchPx * MatchPx;
            for (int index = 0; index < 4; ++index)
            {
                Track& track = m_tracks[camera][index];
                if (!track.valid || used[index] || track.gap > GapTicks) continue;
                const double dx = beacon.x - track.x;
                const double dy = beacon.y - track.y;
                const double distance = dx * dx + dy * dy;
                if (distance < bestDistance)
                {
                    best = index;
                    matched = true;
                    bestDistance = distance;
                }
            }
            if (best < 0)
            {
                for (int index = 0; index < 4; ++index)
                {
                    if (!used[index])
                    {
                        best = index;
                        break;
                    }
                }
            }
            Track& track = m_tracks[camera][best];
            double lampDistance = FarLampPx;
            if (lamp.valid)
            {
                lampDistance = std::hypot(beacon.x - lamp.cx, beacon.y - lamp.cy);
            }
            used[best] = true;
            track.valid = true;
            track.gap = 0;
            track.samples = matched ? std::min(255, track.samples + 1) : 1;
            if (!matched)
            {
                track.farAge = HistoryTicks + 1;
                track.suspectAge = lamp.valid && lampDistance < MatchPx
                                       ? 0
                                       : HistoryTicks + 1;
            }
            track.x = beacon.x;
            track.y = beacon.y;
            if (lamp.valid && lampDistance >= FarLampPx)
            {
                track.farAge = 0;
            }
            const bool removed = track.suspectAge <= HistoryTicks ||
                                 (lamp.valid && lampDistance < NearLampPx &&
                                  (track.samples < ConfirmTicks ||
                                   track.farAge > HistoryTicks));
            filtered[camera][slot] = !removed;
        }
        for (int index = 0; index < 4; ++index)
        {
            Track& track = m_tracks[camera][index];
            if (track.valid && !used[index] && track.gap < 255) ++track.gap;
            if (track.gap > GapTicks)
            {
                track.valid = false;
                track.samples = 0;
                track.farAge = HistoryTicks + 1;
                track.suspectAge = HistoryTicks + 1;
            }
        }
    }

    double lampX = 0.0;
    double lampY = 0.0;
    double lampCos2 = 0.0;
    double lampSin2 = 0.0;
    int lampCount = 0;
    for (int camera = 0; camera < 3; ++camera)
    {
        const CarLampSample& lamp = frame->cameras[camera].carLamp;
        double x = 0.0;
        double y = 0.0;
        double angle = 0.0;
        if (!lamp.valid ||
            !projectPoint(camera, lamp.cx, lamp.cy, LampHeightM, aircraftHeight,
                          world, &x, &y) ||
            !projectLampAngle(camera, lamp, aircraftHeight, world, &angle))
        {
            continue;
        }
        lampX += x;
        lampY += y;
        lampCos2 += std::cos(2.0 * angle * DegToRad);
        lampSin2 += std::sin(2.0 * angle * DegToRad);
        frame->globalCarLamp.cameraMask |= 1 << camera;
        ++lampCount;
    }
    if (lampCount > 0)
    {
        frame->globalCarLamp.valid = true;
        frame->globalCarLamp.x = static_cast<float>(lampX / lampCount);
        frame->globalCarLamp.y = static_cast<float>(lampY / lampCount);
        frame->globalCarLamp.angleDeg = static_cast<float>(
            0.5 * std::atan2(lampSin2, lampCos2) / DegToRad);
    }

    Candidate candidates[6]{};
    int candidateCount = 0;
    for (int camera = 0; camera < 3; ++camera)
    {
        for (int slot = 0; slot < 2; ++slot)
        {
            const BeaconSample& projected = frame->globalCandidates[camera][slot];
            if (!filtered[camera][slot] || !projected.valid) continue;
            candidates[candidateCount++] = {camera, projected.x, projected.y,
                                            projected.area};
        }
    }

    double memberX[4][3]{};
    double memberY[4][3]{};
    int memberCount[4]{};
    int fusedCount = 0;
    for (int index = 0; index < candidateCount; ++index)
    {
        const Candidate& candidate = candidates[index];
        int best = -1;
        double bestDistance = MergeDistanceM * MergeDistanceM;
        for (int slot = 0; slot < fusedCount; ++slot)
        {
            const int cameraBit = 1 << candidate.camera;
            const GlobalBeaconSample& fused = frame->globalBeacons[slot];
            if ((fused.cameraMask & cameraBit) != 0 ||
                (candidate.camera != 1 && (fused.cameraMask & 2) == 0))
            {
                continue;
            }
            double distance = 0.0;
            if (candidate.camera != 1)
            {
                const double dx = candidate.x - memberX[slot][1];
                const double dy = candidate.y - memberY[slot][1];
                distance = dx * dx + dy * dy;
            }
            else
            {
                for (int camera = 0; camera < 3; ++camera)
                {
                    if ((fused.cameraMask & (1 << camera)) != 0)
                    {
                        const double dx = candidate.x - memberX[slot][camera];
                        const double dy = candidate.y - memberY[slot][camera];
                        distance = dx * dx + dy * dy;
                        break;
                    }
                }
            }
            if (distance < bestDistance)
            {
                best = slot;
                bestDistance = distance;
            }
        }
        if (best < 0)
        {
            if (fusedCount >= 4) continue;
            best = fusedCount++;
        }
        GlobalBeaconSample& fused = frame->globalBeacons[best];
        const double count = memberCount[best];
        fused.valid = true;
        fused.x = static_cast<float>((fused.x * count + candidate.x) / (count + 1.0));
        fused.y = static_cast<float>((fused.y * count + candidate.y) / (count + 1.0));
        fused.area = std::max(fused.area, candidate.area);
        fused.cameraMask |= 1 << candidate.camera;
        ++memberCount[best];
        memberX[best][candidate.camera] = candidate.x;
        memberY[best][candidate.camera] = candidate.y;
    }

    if (!frame->globalCarLamp.valid)
    {
        return;
    }
    int selected = -1;
    double selectedDistance = std::numeric_limits<double>::max();
    for (int slot = 0; slot < fusedCount; ++slot)
    {
        const GlobalBeaconSample& beacon = frame->globalBeacons[slot];
        const double distance = std::hypot(beacon.x - frame->globalCarLamp.x,
                                           beacon.y - frame->globalCarLamp.y);
        if (beacon.valid && distance < selectedDistance)
        {
            selected = slot;
            selectedDistance = distance;
        }
    }
    if (selected < 0 || selectedDistance <= 0.001)
    {
        return;
    }
    const GlobalBeaconSample& target = frame->globalBeacons[selected];
    const double dx = target.x - frame->globalCarLamp.x;
    const double dy = target.y - frame->globalCarLamp.y;
    const double angle = frame->globalCarLamp.angleDeg * DegToRad;
    double rightX = std::cos(angle);
    double rightY = std::sin(angle);
    if (std::cos((frame->globalCarLamp.angleDeg - frame->carYawDeg - 90.0) *
                 DegToRad) < 0.0)
    {
        rightX = -rightX;
        rightY = -rightY;
    }
    double speed = std::hypot(frame->carTargetVelocityX, frame->carTargetVelocityY);
    if (!std::isfinite(speed) || speed < 0.001) speed = 1.7;
    const double scale = speed / selectedDistance;
    frame->carTargetVelocityX = static_cast<float>((dx * rightX + dy * rightY) * scale);
    frame->carTargetVelocityY = static_cast<float>((dx * rightY - dy * rightX) * scale);
    frame->selectedTargetId = selected;
    frame->carPlan3Valid = true;
}
