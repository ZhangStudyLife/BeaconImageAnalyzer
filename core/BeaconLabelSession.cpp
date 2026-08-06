#include "BeaconLabelSession.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QSaveFile>

#include <cmath>

namespace
{
constexpr int SessionVersion = 3;
constexpr int MaximumFrameCount = 10000000;
constexpr int MaximumLabelsPerFrame = 64;

void setError(QString* errorMessage, const QString& message)
{
    if (errorMessage != nullptr)
    {
        *errorMessage = message;
    }
}

bool stateFromKey(const QString& key, BeaconLabelFrameState* state)
{
    if (state == nullptr)
    {
        return false;
    }
    if (key == QStringLiteral("annotated"))
    {
        *state = BeaconLabelFrameState::Annotated;
        return true;
    }
    if (key == QStringLiteral("no_beacon"))
    {
        *state = BeaconLabelFrameState::NoBeacon;
        return true;
    }
    if (key == QStringLiteral("ignored"))
    {
        *state = BeaconLabelFrameState::Ignored;
        return true;
    }
    if (key == QStringLiteral("unreviewed"))
    {
        *state = BeaconLabelFrameState::Unreviewed;
        return true;
    }
    return false;
}

bool pointValid(const QPointF& point, const QSize& imageSize)
{
    return std::isfinite(point.x()) && std::isfinite(point.y())
           && point.x() >= 0.0 && point.y() >= 0.0
           && point.x() < imageSize.width() && point.y() < imageSize.height();
}

bool rectValid(const QRectF& rect, const QSize& imageSize)
{
    return std::isfinite(rect.x()) && std::isfinite(rect.y())
           && std::isfinite(rect.width()) && std::isfinite(rect.height())
           && rect.width() > 0.0 && rect.height() > 0.0
           && rect.left() >= 0.0 && rect.top() >= 0.0
           && rect.right() < imageSize.width() && rect.bottom() < imageSize.height();
}

bool labelValid(BeaconLabelFrameState state, int count)
{
    return (state == BeaconLabelFrameState::Annotated) == (count > 0);
}
}

QString BeaconLabelSessionIO::defaultSessionPath(const QString& videoPath, quint8 cameraId)
{
    const QFileInfo video(videoPath);
    return QDir(video.absolutePath()).filePath(video.completeBaseName()
        + (cameraId == 2U ? QStringLiteral(".down-label.json")
                          : QStringLiteral(".beacon-label.json")));
}

QString BeaconLabelSessionIO::stateKey(BeaconLabelFrameState state)
{
    switch (state)
    {
    case BeaconLabelFrameState::Annotated:
        return QStringLiteral("annotated");
    case BeaconLabelFrameState::NoBeacon:
        return QStringLiteral("no_beacon");
    case BeaconLabelFrameState::Ignored:
        return QStringLiteral("ignored");
    case BeaconLabelFrameState::Unreviewed:
        break;
    }
    return QStringLiteral("unreviewed");
}

QString BeaconLabelSessionIO::stateDisplayName(BeaconLabelFrameState state)
{
    switch (state)
    {
    case BeaconLabelFrameState::Annotated:
        return QStringLiteral("已标注");
    case BeaconLabelFrameState::NoBeacon:
        return QStringLiteral("无信标");
    case BeaconLabelFrameState::Ignored:
        return QStringLiteral("忽略");
    case BeaconLabelFrameState::Unreviewed:
        return QStringLiteral("未处理");
    }
    return QStringLiteral("未处理");
}

bool BeaconLabelSessionIO::load(const QString& path,
                                BeaconLabelSession* session,
                                QString* errorMessage)
{
    if (session == nullptr)
    {
        setError(errorMessage, QStringLiteral("标注会话输出为空。"));
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        setError(errorMessage, file.errorString());
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        setError(errorMessage,
                 QStringLiteral("信标标注 JSON 解析失败：%1").arg(parseError.errorString()));
        return false;
    }

    const QJsonObject root = document.object();
    if (root.value(QStringLiteral("format")).toString() != QStringLiteral("beacon_label_session")
        || root.value(QStringLiteral("version")).toInt() < 1
        || root.value(QStringLiteral("version")).toInt() > SessionVersion)
    {
        setError(errorMessage, QStringLiteral("不是受支持的信标标注会话。"));
        return false;
    }

    BeaconLabelSession loaded;
    loaded.sessionPath = QFileInfo(path).absoluteFilePath();
    const QDir directory(QFileInfo(path).absolutePath());
    loaded.videoPath = directory.absoluteFilePath(root.value(QStringLiteral("video_file")).toString());
    loaded.imageSize = QSize(root.value(QStringLiteral("image_width")).toInt(),
                             root.value(QStringLiteral("image_height")).toInt());
    loaded.frameCount = root.value(QStringLiteral("frame_count")).toInt();
    loaded.videoFps = root.value(QStringLiteral("video_fps")).toDouble();
    loaded.sampleStride = root.value(QStringLiteral("sample_stride")).toInt(5);
    loaded.cameraId = root.value(QStringLiteral("version")).toInt() >= 3
        ? (quint8)root.value(QStringLiteral("camera_id")).toInt(255) : 0U;
    if (loaded.videoPath.isEmpty() || loaded.imageSize.width() <= 0
        || loaded.imageSize.height() <= 0 || loaded.frameCount <= 0
        || loaded.frameCount > MaximumFrameCount || loaded.videoFps <= 0.0
        || !std::isfinite(loaded.videoFps) || loaded.sampleStride <= 0
        || loaded.sampleStride > loaded.frameCount || loaded.cameraId > 2U)
    {
        setError(errorMessage, QStringLiteral("信标标注会话的基础字段无效。"));
        return false;
    }

    const QJsonArray frames = root.value(QStringLiteral("frames")).toArray();
    const int version = root.value(QStringLiteral("version")).toInt();
    for (const QJsonValue& value : frames)
    {
        if (!value.isObject())
        {
            setError(errorMessage, QStringLiteral("信标标注帧记录格式无效。"));
            return false;
        }
        const QJsonObject object = value.toObject();
        const int frameIndex = object.value(QStringLiteral("frame_index")).toInt(-1);
        BeaconFrameLabel label;
        if (frameIndex < 0 || frameIndex >= loaded.frameCount
            || !stateFromKey(object.value(QStringLiteral("state")).toString(), &label.state)
            || loaded.frames.contains(frameIndex))
        {
            setError(errorMessage, QStringLiteral("信标标注帧索引或状态无效。"));
            return false;
        }

        const QJsonArray points = object.value(QStringLiteral("points")).toArray();
        if (points.size() > MaximumLabelsPerFrame)
        {
            setError(errorMessage, QStringLiteral("单帧信标标注数量超出限制。"));
            return false;
        }
        for (const QJsonValue& pointValue : points)
        {
            const QJsonObject pointObject = pointValue.toObject();
            const QPointF point(pointObject.value(QStringLiteral("x")).toDouble(qQNaN()),
                                pointObject.value(QStringLiteral("y")).toDouble(qQNaN()));
            if (!pointValid(point, loaded.imageSize))
            {
                setError(errorMessage, QStringLiteral("信标中心点坐标超出图像范围。"));
                return false;
            }
            label.points.push_back(point);
        }
        if (!labelValid(label.state, label.points.size()))
        {
            setError(errorMessage, QStringLiteral("已标注状态和信标中心点数量不一致。"));
            return false;
        }
        if (version >= 2 && object.contains(QStringLiteral("lamp_state")))
        {
            if (!stateFromKey(object.value(QStringLiteral("lamp_state")).toString(), &label.lampState))
            {
                setError(errorMessage, QStringLiteral("车灯标注状态无效。"));
                return false;
            }
            const QJsonArray boxes = object.value(QStringLiteral("lamp_boxes")).toArray();
            if (boxes.size() > MaximumLabelsPerFrame)
            {
                setError(errorMessage, QStringLiteral("单帧车灯标注数量超出限制。"));
                return false;
            }
            for (const QJsonValue& boxValue : boxes)
            {
                const QJsonObject box = boxValue.toObject();
                const QRectF rect(box.value(QStringLiteral("x")).toDouble(qQNaN()),
                                  box.value(QStringLiteral("y")).toDouble(qQNaN()),
                                  box.value(QStringLiteral("width")).toDouble(qQNaN()),
                                  box.value(QStringLiteral("height")).toDouble(qQNaN()));
                if (!rectValid(rect, loaded.imageSize))
                {
                    setError(errorMessage, QStringLiteral("车灯框坐标超出图像范围。"));
                    return false;
                }
                label.lampBoxes.push_back(rect);
            }
            if (!labelValid(label.lampState, label.lampBoxes.size()))
            {
                setError(errorMessage, QStringLiteral("车灯状态和车灯框数量不一致。"));
                return false;
            }
        }
        loaded.frames.insert(frameIndex, label);
    }

    *session = loaded;
    return true;
}

bool BeaconLabelSessionIO::save(const BeaconLabelSession& session,
                                QString* errorMessage)
{
    if (session.sessionPath.isEmpty() || session.videoPath.isEmpty()
        || session.imageSize.width() <= 0 || session.imageSize.height() <= 0
        || session.frameCount <= 0 || session.frameCount > MaximumFrameCount
        || session.videoFps <= 0.0 || !std::isfinite(session.videoFps)
        || session.sampleStride <= 0 || session.sampleStride > session.frameCount
        || session.cameraId > 2U)
    {
        setError(errorMessage, QStringLiteral("信标标注会话字段不完整。"));
        return false;
    }

    QJsonObject root;
    root.insert(QStringLiteral("format"), QStringLiteral("beacon_label_session"));
    root.insert(QStringLiteral("version"), SessionVersion);
    const QDir directory(QFileInfo(session.sessionPath).absolutePath());
    root.insert(QStringLiteral("video_file"), directory.relativeFilePath(session.videoPath));
    root.insert(QStringLiteral("image_width"), session.imageSize.width());
    root.insert(QStringLiteral("image_height"), session.imageSize.height());
    root.insert(QStringLiteral("frame_count"), session.frameCount);
    root.insert(QStringLiteral("video_fps"), session.videoFps);
    root.insert(QStringLiteral("sample_stride"), session.sampleStride);
    root.insert(QStringLiteral("camera_id"), session.cameraId);
    root.insert(QStringLiteral("updated_utc"),
                QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));

    QJsonArray frames;
    for (auto iterator = session.frames.cbegin(); iterator != session.frames.cend(); ++iterator)
    {
        const int frameIndex = iterator.key();
        const BeaconFrameLabel& label = iterator.value();
        if (label.state == BeaconLabelFrameState::Unreviewed
            && label.lampState == BeaconLabelFrameState::Unreviewed)
        {
            continue;
        }
        if (frameIndex < 0 || frameIndex >= session.frameCount
            || label.points.size() > MaximumLabelsPerFrame
            || !labelValid(label.state, label.points.size())
            || label.lampBoxes.size() > MaximumLabelsPerFrame
            || !labelValid(label.lampState, label.lampBoxes.size()))
        {
            setError(errorMessage, QStringLiteral("待保存的信标标注帧无效。"));
            return false;
        }

        QJsonArray points;
        for (const QPointF& point : label.points)
        {
            if (!pointValid(point, session.imageSize))
            {
                setError(errorMessage, QStringLiteral("待保存的信标中心点超出图像范围。"));
                return false;
            }
            QJsonObject pointObject;
            pointObject.insert(QStringLiteral("x"), point.x());
            pointObject.insert(QStringLiteral("y"), point.y());
            points.push_back(pointObject);
        }

        QJsonObject frameObject;
        frameObject.insert(QStringLiteral("frame_index"), frameIndex);
        frameObject.insert(QStringLiteral("state"), stateKey(label.state));
        frameObject.insert(QStringLiteral("points"), points);
        frameObject.insert(QStringLiteral("lamp_state"), stateKey(label.lampState));
        QJsonArray lampBoxes;
        for (const QRectF& box : label.lampBoxes)
        {
            if (!rectValid(box, session.imageSize))
            {
                setError(errorMessage, QStringLiteral("待保存的车灯框超出图像范围。"));
                return false;
            }
            lampBoxes.push_back(QJsonObject({{QStringLiteral("x"), box.x()},
                                             {QStringLiteral("y"), box.y()},
                                             {QStringLiteral("width"), box.width()},
                                             {QStringLiteral("height"), box.height()}}));
        }
        frameObject.insert(QStringLiteral("lamp_boxes"), lampBoxes);
        frames.push_back(frameObject);
    }
    root.insert(QStringLiteral("frames"), frames);

    QSaveFile file(session.sessionPath);
    if (!file.open(QIODevice::WriteOnly))
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    if (file.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) < 0
        || !file.commit())
    {
        setError(errorMessage, file.errorString());
        return false;
    }
    return true;
}
