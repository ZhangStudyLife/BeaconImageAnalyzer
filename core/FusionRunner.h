#ifndef FUSION_RUNNER_H
#define FUSION_RUNNER_H

#include "beacon_fusion.h"

#include <QLibrary>
#include <QString>

class FusionRunner
{
public:
    FusionRunner();
    ~FusionRunner();

    bool loadSourceFile(const QString& sourcePath, const QString& buildDir, QString* errorMessage = nullptr);
    QString sourcePath() const;
    bool usesDynamicLibrary() const;
    beacon_fusion_result_t analyze(const beacon_result_t cameraResults[BEACON_CAMERA_COUNT]) const;

private:
    using InitFn = void (*)();
    using AnalyzeFn = void (*)(const beacon_result_t[BEACON_CAMERA_COUNT], beacon_fusion_result_t*);

    QString m_sourcePath;
    QLibrary m_library;
    InitFn m_initFn = nullptr;
    AnalyzeFn m_analyzeFn = nullptr;
};

#endif
