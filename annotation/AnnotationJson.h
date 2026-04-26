#ifndef ANNOTATION_JSON_H
#define ANNOTATION_JSON_H

#include "AnnotationModel.h"

#include <QString>

struct AnnotationVideoInfo
{
    QString file;
    int width = 0;
    int height = 0;
    double fpsUsed = 50.0;
    int frameCount = 0;
};

class AnnotationJson
{
public:
    static bool save(const QString& path,
                     const AnnotationVideoInfo& videoInfo,
                     const AnnotationModel& model,
                     QString* errorMessage);

    static bool load(const QString& path,
                     AnnotationModel* model,
                     QString* errorMessage);
};

#endif
