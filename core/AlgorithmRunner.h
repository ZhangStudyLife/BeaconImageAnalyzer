#ifndef ALGORITHM_RUNNER_H
#define ALGORITHM_RUNNER_H

#include "ImageResult.h"

#include <QImage>
#include <QLibrary>
#include <QString>

class AlgorithmRunner
{
public:
    AlgorithmRunner();
    ~AlgorithmRunner();

    bool loadSourceFile(const QString& sourcePath, const QString& buildDir, QString* errorMessage = nullptr);
    QString sourcePath() const;
    bool usesDynamicLibrary() const;
    beacon_result_t process(const QImage& grayImage) const;
    QImage binaryImage(const QImage& grayImage) const;

private:
    using InitFn = void (*)();
    using ImageUpdateFn = void (*)();
    using ImageCircleCountPtr = unsigned char*;
    using ImageCirclePtr = beacon_circle_t*;
    using ImageCarLampCountPtr = unsigned char*;
    using ImageCarLampPtr = beacon_rect_t*;

    QString m_sourcePath;
    QLibrary m_library;
    InitFn m_initFn = nullptr;
    ImageUpdateFn m_imageUpdateFn = nullptr;
    ImageCircleCountPtr m_dynamicBeaconCount = nullptr;
    ImageCirclePtr m_dynamicBeacons = nullptr;
    ImageCarLampCountPtr m_dynamicCarLampCount = nullptr;
    ImageCarLampPtr m_dynamicCarLamps = nullptr;
    unsigned char* m_dynamicFrameBuffer = nullptr;
    unsigned char* m_dynamicFinishFlag = nullptr;
};

#endif
