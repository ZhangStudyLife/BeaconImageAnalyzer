#ifndef VIDEO_EXPORTER_H
#define VIDEO_EXPORTER_H

#include <QString>

#include <functional>

class VideoExporter
{
public:
    using ProgressCallback = std::function<bool(int currentFrame, int totalFrames)>;

    bool exportMarkedAvi(const QString& inputPath,
                         const QString& outputPath,
                         double fps,
                         const ProgressCallback& progress,
                         QString* errorMessage) const;

    bool exportResultCsv(const QString& inputPath,
                         const QString& outputPath,
                         double fps,
                         const ProgressCallback& progress,
                         QString* errorMessage) const;
};

#endif
