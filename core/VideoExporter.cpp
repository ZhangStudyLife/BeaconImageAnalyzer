#include "VideoExporter.h"

#include "AlgorithmRunner.h"
#include "CameraFrameTransform.h"
#include "DetectionBoundary.h"
#include "FrameRenderer.h"
#include "VideoReader.h"

#include <QFile>
#include <QTextStream>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace
{
cv::Mat qImageToBgrMat(const QImage& image)
{
    const QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat rgbMat(rgb.height(),
                   rgb.width(),
                   CV_8UC3,
                   const_cast<uchar*>(rgb.bits()),
                   (size_t)rgb.bytesPerLine());
    cv::Mat bgr;
    cv::cvtColor(rgbMat, bgr, cv::COLOR_RGB2BGR);
    return bgr;
}

bool openWriter(cv::VideoWriter* writer,
                const QString& outputPath,
                double fps,
                int width,
                int height,
                QString* codecName)
{
    const cv::Size size(width, height);
    const double safeFps = fps > 0.0 ? fps : 50.0;

    struct Candidate
    {
        int fourcc;
        const char* name;
    };

    const Candidate candidates[] = {
        { cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), "MJPG" },
        { cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), "XVID" },
        { 0, "RAW" }
    };

    for (const Candidate& candidate : candidates)
    {
        writer->release();
        if (writer->open(outputPath.toStdString(), candidate.fourcc, safeFps, size, true))
        {
            if (codecName != nullptr)
            {
                *codecName = QString::fromLatin1(candidate.name);
            }
            return true;
        }
    }

    return false;
}

QVector<ErrorCircle> recordErrorCircles(const AnnotationRecord& record)
{
    if (!record.errorCircles.isEmpty())
    {
        return record.errorCircles;
    }
    if (record.circleIndex >= 0)
    {
        return QVector<ErrorCircle>{ ErrorCircle{ record.circleIndex, -1 } };
    }
    return {};
}

CorrectionShape correctionFromRecord(const AnnotationRecord& record, int frame)
{
    CorrectionShape shape;
    shape.name = annotationTypesDisplayName(record.types.isEmpty()
                                                ? QStringList{ record.type }
                                                : record.types);
    shape.frame = frame;
    shape.errorType = record.type.isEmpty() ? QStringLiteral("other") : record.type;
    shape.errorTypes = record.types.isEmpty() ? QStringList{ shape.errorType } : record.types;
    shape.errorCircles = recordErrorCircles(record);
    shape.description = record.description;
    shape.lineWidth = 1;
    return shape;
}

QVector<CorrectionShape> frameCorrections(const AnnotationModel* annotations, int frame)
{
    QVector<CorrectionShape> corrections;
    if (annotations == nullptr)
    {
        return corrections;
    }

    corrections = annotations->correctionsForFrame(frame);
    const QVector<AnnotationRecord> records = annotations->recordsForFrame(frame);
    for (const AnnotationRecord& record : records)
    {
        corrections.push_back(correctionFromRecord(record, frame));
    }
    return corrections;
}
}

bool VideoExporter::exportMarkedAvi(const QString& inputPath,
                                     const QString& outputPath,
                                     double fps,
                                     const AlgorithmRunner* runner,
                                     const AnnotationModel* annotations,
                                     int cameraIndex,
                                     const DetectionBoundary* boundary,
                                     const ProgressCallback& progress,
                                     QString* errorMessage) const
{
    VideoReader reader;
    if (!reader.open(inputPath, errorMessage))
    {
        return false;
    }

    cv::VideoWriter writer;
    QString codecName;
    if (!openWriter(&writer, outputPath, fps, reader.width(), reader.height(), &codecName))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("OpenCV 无法创建输出 AVI");
        }
        return false;
    }

    AlgorithmRunner fallbackRunner;
    const AlgorithmRunner* activeRunner = runner != nullptr ? runner : &fallbackRunner;
    for (int frame = 0; frame < reader.frameCount(); ++frame)
    {
        QImage gray;
        if (!reader.readFrame(frame, &gray, errorMessage))
        {
            return false;
        }

        gray = CameraFrameTransform::applyForCameraIndex(gray, cameraIndex);
        const beacon_result_t rawResult = activeRunner->process(gray);
        const beacon_result_t result = DetectionBoundaryRules::apply(rawResult, boundary);
        const QImage rendered = FrameRenderer::render(gray,
                                                      result,
                                                      frameCorrections(annotations, frame),
                                                      1,
                                                      true,
                                                      boundary);
        writer.write(qImageToBgrMat(rendered));

        if (progress && !progress(frame + 1, reader.frameCount()))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("用户取消导出");
            }
            return false;
        }
    }

    writer.release();
    return true;
}

bool VideoExporter::exportResultCsv(const QString& inputPath,
                                    const QString& outputPath,
                                    double fps,
                                    const AlgorithmRunner* runner,
                                    int cameraIndex,
                                    const DetectionBoundary* boundary,
                                    const ProgressCallback& progress,
                                    QString* errorMessage) const
{
    VideoReader reader;
    if (!reader.open(inputPath, errorMessage))
    {
        return false;
    }

    QFile file(outputPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = file.errorString();
        }
        return false;
    }

    QTextStream stream(&file);
    stream << "frame,time_sec,index,valid,x,y,radius\n";

    AlgorithmRunner fallbackRunner;
    const AlgorithmRunner* activeRunner = runner != nullptr ? runner : &fallbackRunner;
    for (int frame = 0; frame < reader.frameCount(); ++frame)
    {
        QImage gray;
        if (!reader.readFrame(frame, &gray, errorMessage))
        {
            return false;
        }

        gray = CameraFrameTransform::applyForCameraIndex(gray, cameraIndex);
        const beacon_result_t rawResult = activeRunner->process(gray);
        const beacon_result_t result = DetectionBoundaryRules::apply(rawResult, boundary);
        const double timeSec = (double)frame / (fps > 0.0 ? fps : 50.0);
        for (int i = 0; i < result.count && i < BEACON_MAX_CIRCLE_COUNT; ++i)
        {
            const beacon_circle_t& circle = result.circles[i];
            if (circle.valid == 0)
            {
                continue;
            }
            stream << frame << ','
                   << QString::number(timeSec, 'f', 3) << ','
                   << i << ','
                   << (int)circle.valid << ','
                   << QString::number(circle.x, 'f', 3) << ','
                   << QString::number(circle.y, 'f', 3) << ','
                   << QString::number(circle.radius, 'f', 3) << '\n';
        }

        if (progress && !progress(frame + 1, reader.frameCount()))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("用户取消导出");
            }
            return false;
        }
    }

    return true;
}
