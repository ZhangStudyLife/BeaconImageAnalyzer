#ifndef IMAGE_LOG_ALIGNER_H
#define IMAGE_LOG_ALIGNER_H

#include "ImageFrameSidecar.h"
#include "JustFloatLog.h"

#include <QString>
#include <QVector>

enum class ImageLogAlignmentConfidence
{
    Unavailable,
    Low,
    Medium,
    High
};

struct ImageLogAlignmentResult
{
    QVector<int> videoToLogRow;
    ImageLogAlignmentConfidence confidence = ImageLogAlignmentConfidence::Unavailable;
    QString message;
    int sourceCameraId = -1;
    int validVideoFrameCount = 0;
    int matchedVideoFrameCount = 0;
    int candidateCount = 0;
    int manualCycleShift = 0;
    qint64 automaticSequenceOffset = 0;
    qint64 sequenceOffset = 0;
    double coverage = 0.0;
    double meanAttitudeError = 0.0;
    double timeJitterMs = 0.0;
    double scoreGap = 0.0;

    bool hasMapping() const
    {
        return matchedVideoFrameCount > 0 && !videoToLogRow.isEmpty();
    }
};

class ImageLogAligner
{
public:
    static ImageLogAlignmentResult align(const QVector<ImageFrameSidecarRecord>& frames,
                                         const JustFloatLog& log,
                                         int manualCycleShift = 0);
    static QString confidenceName(ImageLogAlignmentConfidence confidence);
};

#endif
