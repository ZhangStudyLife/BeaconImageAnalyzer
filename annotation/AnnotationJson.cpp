#include "AnnotationJson.h"

#include "ImageResult.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <cmath>

namespace
{
constexpr qint64 MaxAnnotationJsonBytes = 16LL * 1024LL * 1024LL;
constexpr int MaxAnnotationItems = 200000;
constexpr int MaxPointItemsPerShape = 1024;
constexpr int MaxErrorCircleItems = 64;
constexpr int MaxTextLength = 8192;
constexpr double MaxPointAbs = 100000.0;

void setError(QString* errorMessage, const QString& message)
{
    if (errorMessage != nullptr)
    {
        *errorMessage = message;
    }
}

bool validFrameRange(int startFrame, int endFrame)
{
    return startFrame >= 0 && endFrame >= startFrame;
}

bool validCircleIndex(int index)
{
    return index >= -1 && index < BEACON_MAX_CIRCLE_COUNT;
}

QString boundedString(const QJsonValue& value, const QString& fallback = QString())
{
    QString text = value.toString(fallback);
    if (text.size() > MaxTextLength)
    {
        text.truncate(MaxTextLength);
    }
    return text;
}

bool finitePoint(double x, double y)
{
    return std::isfinite(x) &&
           std::isfinite(y) &&
           std::abs(x) <= MaxPointAbs &&
           std::abs(y) <= MaxPointAbs;
}

QJsonArray stringListToJson(const QStringList& values)
{
    QJsonArray array;
    for (const QString& value : values)
    {
        if (!value.trimmed().isEmpty())
        {
            array.append(value);
        }
    }
    return array;
}

QStringList stringListFromJson(const QJsonValue& value, const QString& fallback)
{
    QStringList result;
    if (!value.isUndefined() && !value.isArray())
    {
        return fallback.trimmed().isEmpty() ? result : QStringList{ fallback.trimmed() };
    }

    const QJsonArray array = value.toArray();
    for (const QJsonValue& item : array)
    {
        const QString text = item.toString().trimmed();
        if (!text.isEmpty())
        {
            result.push_back(text.left(MaxTextLength));
        }
        if (result.size() >= MaxErrorCircleItems)
        {
            break;
        }
    }
    if (result.isEmpty() && !fallback.trimmed().isEmpty())
    {
        result.push_back(fallback);
    }
    return result;
}

QJsonArray errorCirclesToJson(const QVector<ErrorCircle>& circles)
{
    QJsonArray array;
    for (const ErrorCircle& circle : circles)
    {
        QJsonObject object;
        object.insert(QStringLiteral("circle_index"), circle.circleIndex);
        object.insert(QStringLiteral("expected_index"), circle.expectedIndex);
        array.append(object);
    }
    return array;
}

QVector<ErrorCircle> errorCirclesFromJson(const QJsonValue& value, int legacyCircle, int legacyExpected)
{
    QVector<ErrorCircle> result;
    if (!value.isUndefined() && !value.isArray())
    {
        return result;
    }

    const QJsonArray array = value.toArray();
    for (const QJsonValue& item : array)
    {
        if (!item.isObject() || result.size() >= MaxErrorCircleItems)
        {
            continue;
        }

        const QJsonObject object = item.toObject();
        ErrorCircle circle;
        circle.circleIndex = object.value(QStringLiteral("circle_index")).toInt(-1);
        circle.expectedIndex = object.value(QStringLiteral("expected_index")).toInt(-1);
        if (validCircleIndex(circle.circleIndex) &&
            validCircleIndex(circle.expectedIndex) &&
            (circle.circleIndex >= 0 || circle.expectedIndex >= 0))
        {
            result.push_back(circle);
        }
    }

    if (result.isEmpty() && validCircleIndex(legacyCircle) && legacyCircle >= 0)
    {
        ErrorCircle circle;
        circle.circleIndex = legacyCircle;
        circle.expectedIndex = validCircleIndex(legacyExpected) ? legacyExpected : -1;
        result.push_back(circle);
    }

    return result;
}

QJsonArray pointsToJson(const QVector<QPointF>& points)
{
    QJsonArray array;
    for (const QPointF& point : points)
    {
        QJsonObject pointObject;
        pointObject.insert(QStringLiteral("x"), point.x());
        pointObject.insert(QStringLiteral("y"), point.y());
        array.append(pointObject);
    }
    return array;
}
}

bool AnnotationJson::save(const QString& path,
                          const AnnotationVideoInfo& videoInfo,
                          const AnnotationModel& model,
                          QString* errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = file.errorString();
        }
        return false;
    }

    QJsonObject root;
    root.insert(QStringLiteral("project"), QStringLiteral("BeaconImageAnalyzer"));

    QJsonObject video;
    video.insert(QStringLiteral("file"), QFileInfo(videoInfo.file).fileName());
    video.insert(QStringLiteral("width"), videoInfo.width);
    video.insert(QStringLiteral("height"), videoInfo.height);
    video.insert(QStringLiteral("fps_used"), videoInfo.fpsUsed);
    video.insert(QStringLiteral("frame_count"), videoInfo.frameCount);
    root.insert(QStringLiteral("video"), video);

    QJsonObject algorithm;
    algorithm.insert(QStringLiteral("name"), QStringLiteral("image_update"));
    algorithm.insert(QStringLiteral("version"), QStringLiteral("v1"));
    algorithm.insert(QStringLiteral("note"), QStringLiteral("simple threshold + connected components"));
    root.insert(QStringLiteral("algorithm"), algorithm);

    QJsonArray annotations;
    for (const AnnotationRecord& record : model.records())
    {
        QJsonObject item;
        item.insert(QStringLiteral("type"), record.type);
        item.insert(QStringLiteral("types"), stringListToJson(record.types));
        item.insert(QStringLiteral("start_frame"), record.startFrame);
        item.insert(QStringLiteral("end_frame"), record.endFrame);
        item.insert(QStringLiteral("start_time_sec"), record.startTimeSec);
        item.insert(QStringLiteral("end_time_sec"), record.endTimeSec);
        item.insert(QStringLiteral("circle_index"), record.circleIndex);
        item.insert(QStringLiteral("error_circles"), errorCirclesToJson(record.errorCircles));
        item.insert(QStringLiteral("description"), record.description);
        annotations.append(item);
    }
    root.insert(QStringLiteral("annotations"), annotations);

    QJsonArray corrections;
    for (const CorrectionShape& shape : model.corrections())
    {
        QJsonObject item;
        item.insert(QStringLiteral("name"), shape.name);
        item.insert(QStringLiteral("shape_type"), shape.shapeType);
        item.insert(QStringLiteral("frame"), shape.frame);
        item.insert(QStringLiteral("error_type"), shape.errorType);
        item.insert(QStringLiteral("error_types"), stringListToJson(shape.errorTypes));
        item.insert(QStringLiteral("expected_index"), shape.expectedIndex);
        item.insert(QStringLiteral("error_circles"), errorCirclesToJson(shape.errorCircles));
        item.insert(QStringLiteral("description"), shape.description);
        item.insert(QStringLiteral("line_color"), shape.lineColor.name(QColor::HexRgb));
        item.insert(QStringLiteral("line_width"), shape.lineWidth);
        item.insert(QStringLiteral("points"), pointsToJson(shape.points));
        corrections.append(item);
    }
    root.insert(QStringLiteral("corrections"), corrections);

    const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (file.write(payload) != payload.size())
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    return true;
}

bool AnnotationJson::load(const QString& path,
                          AnnotationModel* model,
                          QString* errorMessage)
{
    if (model == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("AnnotationModel is null");
        }
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    if (file.size() > MaxAnnotationJsonBytes)
    {
        setError(errorMessage, QStringLiteral("Annotation JSON is too large"));
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        setError(errorMessage, parseError.errorString());
        return false;
    }

    const QJsonObject root = document.object();
    const QJsonValue annotationsValue = root.value(QStringLiteral("annotations"));
    const QJsonValue correctionsValue = root.value(QStringLiteral("corrections"));
    if ((!annotationsValue.isUndefined() && !annotationsValue.isArray()) ||
        (!correctionsValue.isUndefined() && !correctionsValue.isArray()))
    {
        setError(errorMessage, QStringLiteral("annotations/corrections must be arrays"));
        return false;
    }

    const QJsonArray annotations = annotationsValue.toArray();
    const QJsonArray corrections = correctionsValue.toArray();
    if (annotations.size() > MaxAnnotationItems || corrections.size() > MaxAnnotationItems)
    {
        setError(errorMessage, QStringLiteral("Annotation JSON contains too many items"));
        return false;
    }

    AnnotationModel loaded;
    for (const QJsonValue& value : annotations)
    {
        if (!value.isObject())
        {
            setError(errorMessage, QStringLiteral("Annotation item must be an object"));
            return false;
        }

        const QJsonObject object = value.toObject();
        AnnotationRecord record;
        record.type = boundedString(object.value(QStringLiteral("type")), QStringLiteral("other")).trimmed();
        record.types = stringListFromJson(object.value(QStringLiteral("types")), record.type);
        record.startFrame = object.value(QStringLiteral("start_frame")).toInt();
        record.endFrame = object.value(QStringLiteral("end_frame")).toInt(record.startFrame);
        record.startTimeSec = object.value(QStringLiteral("start_time_sec")).toDouble();
        record.endTimeSec = object.value(QStringLiteral("end_time_sec")).toDouble(record.startTimeSec);
        record.circleIndex = object.value(QStringLiteral("circle_index")).toInt(-1);
        if (!validFrameRange(record.startFrame, record.endFrame) ||
            !std::isfinite(record.startTimeSec) ||
            !std::isfinite(record.endTimeSec) ||
            record.endTimeSec < record.startTimeSec ||
            !validCircleIndex(record.circleIndex))
        {
            setError(errorMessage, QStringLiteral("Annotation item contains invalid frame/time/circle fields"));
            return false;
        }

        record.errorCircles = errorCirclesFromJson(object.value(QStringLiteral("error_circles")),
                                                   record.circleIndex,
                                                   -1);
        record.description = boundedString(object.value(QStringLiteral("description")));
        loaded.add(record);
    }

    for (const QJsonValue& value : corrections)
    {
        if (!value.isObject())
        {
            setError(errorMessage, QStringLiteral("Correction item must be an object"));
            return false;
        }

        const QJsonObject object = value.toObject();
        CorrectionShape shape;
        shape.name = boundedString(object.value(QStringLiteral("name")));
        shape.shapeType = boundedString(object.value(QStringLiteral("shape_type"))).trimmed();
        shape.frame = object.value(QStringLiteral("frame")).toInt();
        shape.errorType = boundedString(object.value(QStringLiteral("error_type")), QStringLiteral("other")).trimmed();
        shape.errorTypes = stringListFromJson(object.value(QStringLiteral("error_types")), shape.errorType);
        shape.expectedIndex = object.value(QStringLiteral("expected_index")).toInt(-1);
        if (shape.frame < 0 || !validCircleIndex(shape.expectedIndex))
        {
            setError(errorMessage, QStringLiteral("Correction item contains invalid frame or expected index"));
            return false;
        }

        shape.errorCircles = errorCirclesFromJson(object.value(QStringLiteral("error_circles")),
                                                  -1,
                                                  shape.expectedIndex);
        shape.description = boundedString(object.value(QStringLiteral("description")));
        shape.lineColor = QColor(boundedString(object.value(QStringLiteral("line_color")), QStringLiteral("#ff5050")));
        if (!shape.lineColor.isValid())
        {
            shape.lineColor = QColor(255, 80, 80);
        }
        shape.lineWidth = qBound(1, object.value(QStringLiteral("line_width")).toInt(1), 15);

        const QJsonValue pointsValue = object.value(QStringLiteral("points"));
        if (!pointsValue.isUndefined() && !pointsValue.isArray())
        {
            setError(errorMessage, QStringLiteral("Correction points must be an array"));
            return false;
        }
        const QJsonArray points = pointsValue.toArray();
        if (points.size() > MaxPointItemsPerShape)
        {
            setError(errorMessage, QStringLiteral("Correction item contains too many points"));
            return false;
        }
        for (const QJsonValue& pointValue : points)
        {
            if (!pointValue.isObject())
            {
                setError(errorMessage, QStringLiteral("Correction point must be an object"));
                return false;
            }

            const QJsonObject pointObject = pointValue.toObject();
            const double x = pointObject.value(QStringLiteral("x")).toDouble();
            const double y = pointObject.value(QStringLiteral("y")).toDouble();
            if (!finitePoint(x, y))
            {
                setError(errorMessage, QStringLiteral("Correction point contains invalid coordinates"));
                return false;
            }
            shape.points.push_back(QPointF(x, y));
        }

        if (!shape.errorType.isEmpty() ||
            !shape.name.trimmed().isEmpty() ||
            !shape.description.trimmed().isEmpty() ||
            (!shape.shapeType.isEmpty() && !shape.points.isEmpty()))
        {
            loaded.addCorrection(shape);
        }
    }

    model->clear();
    for (const AnnotationRecord& record : loaded.records())
    {
        model->add(record);
    }
    for (const CorrectionShape& shape : loaded.corrections())
    {
        model->addCorrection(shape);
    }
    return true;
}
