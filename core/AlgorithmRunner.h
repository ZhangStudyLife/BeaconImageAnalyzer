#ifndef ALGORITHM_RUNNER_H
#define ALGORITHM_RUNNER_H

#include "beacon_image.h"

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
    using ProcessFn = void (*)(const unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W], beacon_result_t*);
    using BinaryFn = void (*)(const unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W],
                              unsigned char[BEACON_IMAGE_H][BEACON_IMAGE_W]);

    QString m_sourcePath;
    QLibrary m_library;
    InitFn m_initFn = nullptr;
    ProcessFn m_processFn = nullptr;
    BinaryFn m_binaryFn = nullptr;
};

#endif
