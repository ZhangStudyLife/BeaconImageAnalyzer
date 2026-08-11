#ifndef VIDEO_EXPORTER_H
#define VIDEO_EXPORTER_H

#include "AnnotationModel.h"
#include "AlgorithmRunner.h"

#include <QString>

#include <functional>

class AlgorithmRunner;

class VideoExporter
{
public:
    using ProgressCallback = std::function<bool(int currentFrame, int totalFrames)>;

    bool exportMarkedAvi(const QString& inputPath,
                         const QString& outputPath,
                         double fps,
                         const AlgorithmRunner* runner,
                         const AnnotationModel* annotations,
                         const ProgressCallback& progress,
                         QString* errorMessage,
                         CarLampMode carLampMode = CarLampMode::Single) const;

    bool exportResultCsv(const QString& inputPath,
                         const QString& outputPath,
                         double fps,
                         const AlgorithmRunner* runner,
                         const ProgressCallback& progress,
                         QString* errorMessage,
                         CarLampMode carLampMode = CarLampMode::Single) const;
};

#endif
