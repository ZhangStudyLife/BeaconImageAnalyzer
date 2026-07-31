#ifndef BEACON_LABEL_SESSION_H
#define BEACON_LABEL_SESSION_H

#include <QMap>
#include <QPointF>
#include <QSize>
#include <QString>
#include <QVector>

enum class BeaconLabelFrameState
{
    Unreviewed,
    Annotated,
    NoBeacon,
    Ignored
};

struct BeaconFrameLabel
{
    BeaconLabelFrameState state = BeaconLabelFrameState::Unreviewed;
    QVector<QPointF> points;
};

struct BeaconLabelSession
{
    QString sessionPath;
    QString videoPath;
    QSize imageSize;
    int frameCount = 0;
    double videoFps = 0.0;
    int sampleStride = 5;
    QMap<int, BeaconFrameLabel> frames;
};

namespace BeaconLabelSessionIO
{
QString defaultSessionPath(const QString& videoPath);
QString stateKey(BeaconLabelFrameState state);
QString stateDisplayName(BeaconLabelFrameState state);

bool load(const QString& path,
          BeaconLabelSession* session,
          QString* errorMessage = nullptr);
bool save(const BeaconLabelSession& session,
          QString* errorMessage = nullptr);
}

#endif
