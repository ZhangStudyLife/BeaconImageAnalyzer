#include "AnnotationJson.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace
{
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
    const QJsonArray array = value.toArray();
    for (const QJsonValue& item : array)
    {
        const QString text = item.toString().trimmed();
        if (!text.isEmpty())
        {
            result.push_back(text);
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
    const QJsonArray array = value.toArray();
    for (const QJsonValue& item : array)
    {
        const QJsonObject object = item.toObject();
        ErrorCircle circle;
        circle.circleIndex = object.value(QStringLiteral("circle_index")).toInt(-1);
        circle.expectedIndex = object.value(QStringLiteral("expected_index")).toInt(-1);
        if (circle.circleIndex >= 0 || circle.expectedIndex >= 0)
        {
            result.push_back(circle);
        }
    }

    if (result.isEmpty() && legacyCircle >= 0)
    {
        ErrorCircle circle;
        circle.circleIndex = legacyCircle;
        circle.expectedIndex = legacyExpected;
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
    algorithm.insert(QStringLiteral("name"), QStringLiteral("beacon_image_process"));
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

    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
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
        if (errorMessage != nullptr)
        {
            *errorMessage = file.errorString();
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = parseError.errorString();
        }
        return false;
    }

    const QJsonArray annotations = document.object().value(QStringLiteral("annotations")).toArray();
    model->clear();
    for (const QJsonValue& value : annotations)
    {
        const QJsonObject object = value.toObject();
        AnnotationRecord record;
        record.type = object.value(QStringLiteral("type")).toString(QStringLiteral("other"));
        record.types = stringListFromJson(object.value(QStringLiteral("types")), record.type);
        record.startFrame = object.value(QStringLiteral("start_frame")).toInt();
        record.endFrame = object.value(QStringLiteral("end_frame")).toInt(record.startFrame);
        record.startTimeSec = object.value(QStringLiteral("start_time_sec")).toDouble();
        record.endTimeSec = object.value(QStringLiteral("end_time_sec")).toDouble(record.startTimeSec);
        record.circleIndex = object.value(QStringLiteral("circle_index")).toInt(-1);
        record.errorCircles = errorCirclesFromJson(object.value(QStringLiteral("error_circles")),
                                                   record.circleIndex,
                                                   -1);
        record.description = object.value(QStringLiteral("description")).toString();
        model->add(record);
    }

    const QJsonArray corrections = document.object().value(QStringLiteral("corrections")).toArray();
    for (const QJsonValue& value : corrections)
    {
        const QJsonObject object = value.toObject();
        CorrectionShape shape;
        shape.name = object.value(QStringLiteral("name")).toString();
        shape.shapeType = object.value(QStringLiteral("shape_type")).toString();
        shape.frame = object.value(QStringLiteral("frame")).toInt();
        shape.errorType = object.value(QStringLiteral("error_type")).toString(QStringLiteral("other"));
        shape.errorTypes = stringListFromJson(object.value(QStringLiteral("error_types")), shape.errorType);
        shape.expectedIndex = object.value(QStringLiteral("expected_index")).toInt(-1);
        shape.errorCircles = errorCirclesFromJson(object.value(QStringLiteral("error_circles")),
                                                  -1,
                                                  shape.expectedIndex);
        shape.description = object.value(QStringLiteral("description")).toString();
        shape.lineColor = QColor(object.value(QStringLiteral("line_color")).toString(QStringLiteral("#ff5050")));
        if (!shape.lineColor.isValid())
        {
            shape.lineColor = QColor(255, 80, 80);
        }
        shape.lineWidth = qBound(1, object.value(QStringLiteral("line_width")).toInt(1), 15);

        const QJsonArray points = object.value(QStringLiteral("points")).toArray();
        for (const QJsonValue& pointValue : points)
        {
            const QJsonObject pointObject = pointValue.toObject();
            shape.points.push_back(QPointF(pointObject.value(QStringLiteral("x")).toDouble(),
                                           pointObject.value(QStringLiteral("y")).toDouble()));
        }

        if (!shape.errorType.isEmpty() ||
            !shape.name.trimmed().isEmpty() ||
            !shape.description.trimmed().isEmpty() ||
            (!shape.shapeType.isEmpty() && !shape.points.isEmpty()))
        {
            model->addCorrection(shape);
        }
    }

    return true;
}
