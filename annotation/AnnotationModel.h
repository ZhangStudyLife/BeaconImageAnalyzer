#ifndef ANNOTATION_MODEL_H
#define ANNOTATION_MODEL_H

#include <QPointF>
#include <QString>
#include <QVector>

struct AnnotationRecord
{
    QString type;
    int startFrame = 0;
    int endFrame = 0;
    double startTimeSec = 0.0;
    double endTimeSec = 0.0;
    int circleIndex = -1;
    QString description;
};

struct CorrectionShape
{
    QString shapeType;
    int frame = 0;
    QString errorType;
    int expectedIndex = -1;
    QString description;
    QVector<QPointF> points;
};

class AnnotationModel
{
public:
    void clear();
    void add(const AnnotationRecord& record);
    bool removeAt(int index);
    const QVector<AnnotationRecord>& records() const;
    QVector<AnnotationRecord> recordsForFrame(int frame) const;
    void addCorrection(const CorrectionShape& shape);
    bool removeCorrectionAt(int index);
    const QVector<CorrectionShape>& corrections() const;
    QVector<CorrectionShape> correctionsForFrame(int frame) const;

private:
    QVector<AnnotationRecord> m_records;
    QVector<CorrectionShape> m_corrections;
};

QString annotationTypeDisplayName(const QString& type);
QString annotationTypeCodeFromIndex(int index);

#endif
