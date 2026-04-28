#include "AnnotationModel.h"

void AnnotationModel::clear()
{
    m_records.clear();
    m_corrections.clear();
}

void AnnotationModel::add(const AnnotationRecord& record)
{
    m_records.push_back(record);
}

bool AnnotationModel::removeAt(int index)
{
    if (index < 0 || index >= m_records.size())
    {
        return false;
    }
    m_records.removeAt(index);
    return true;
}

const QVector<AnnotationRecord>& AnnotationModel::records() const
{
    return m_records;
}

QVector<AnnotationRecord> AnnotationModel::recordsForFrame(int frame) const
{
    QVector<AnnotationRecord> result;
    for (const AnnotationRecord& record : m_records)
    {
        if (frame >= record.startFrame && frame <= record.endFrame)
        {
            result.push_back(record);
        }
    }
    return result;
}

void AnnotationModel::addCorrection(const CorrectionShape& shape)
{
    m_corrections.push_back(shape);
}

bool AnnotationModel::removeCorrectionAt(int index)
{
    if (index < 0 || index >= m_corrections.size())
    {
        return false;
    }
    m_corrections.removeAt(index);
    return true;
}

bool AnnotationModel::removeCorrectionsForFrame(int frame)
{
    bool changed = false;
    for (int i = m_corrections.size() - 1; i >= 0; --i)
    {
        if (m_corrections[i].frame == frame)
        {
            m_corrections.removeAt(i);
            changed = true;
        }
    }
    return changed;
}

const QVector<CorrectionShape>& AnnotationModel::corrections() const
{
    return m_corrections;
}

QVector<CorrectionShape> AnnotationModel::correctionsForFrame(int frame) const
{
    QVector<CorrectionShape> result;
    for (const CorrectionShape& shape : m_corrections)
    {
        if (shape.frame == frame)
        {
            result.push_back(shape);
        }
    }
    return result;
}

QString annotationTypeDisplayName(const QString& type)
{
    if (type.startsWith(QStringLiteral("custom:")))
    {
        const QString name = type.mid(QStringLiteral("custom:").size()).trimmed();
        return name.isEmpty() ? QStringLiteral("自定义") : name;
    }
    if (type == QStringLiteral("missed_detection"))
    {
        return QStringLiteral("漏检");
    }
    if (type == QStringLiteral("false_positive"))
    {
        return QStringLiteral("误检");
    }
    if (type == QStringLiteral("wrong_order"))
    {
        return QStringLiteral("排序错误");
    }
    if (type == QStringLiteral("sunlight_interference"))
    {
        return QStringLiteral("阳光干扰");
    }
    if (type == QStringLiteral("target_jump"))
    {
        return QStringLiteral("目标跳变");
    }
    return QStringLiteral("其他");
}

QString annotationTypesDisplayName(const QStringList& types)
{
    QStringList names;
    for (const QString& type : types)
    {
        if (!type.trimmed().isEmpty())
        {
            names.push_back(annotationTypeDisplayName(type));
        }
    }
    return names.isEmpty() ? QStringLiteral("未分类") : names.join(QStringLiteral("+"));
}

QString errorCirclesDisplayName(const QVector<ErrorCircle>& circles)
{
    if (circles.isEmpty())
    {
        return QStringLiteral("无错误圆");
    }

    QStringList names;
    for (const ErrorCircle& circle : circles)
    {
        QString text;
        if (circle.circleIndex >= 0)
        {
            text = QStringLiteral("#%1").arg(circle.circleIndex);
        }
        else if (circle.expectedIndex >= 0)
        {
            text = QStringLiteral("漏检");
        }
        else
        {
            text = QStringLiteral("未指定");
        }
        if (circle.expectedIndex >= 0)
        {
            text += QStringLiteral("->GT #%1").arg(circle.expectedIndex);
        }
        names.push_back(text);
    }
    return names.join(QStringLiteral(", "));
}

QString annotationTypeCodeFromIndex(int index)
{
    switch (index)
    {
    case 0:
        return QStringLiteral("missed_detection");
    case 1:
        return QStringLiteral("false_positive");
    case 2:
        return QStringLiteral("wrong_order");
    case 3:
        return QStringLiteral("sunlight_interference");
    case 4:
        return QStringLiteral("target_jump");
    default:
        return QStringLiteral("other");
    }
}
