#ifndef ANNOTATION_MODEL_H
#define ANNOTATION_MODEL_H

#include <QPointF>
#include <QColor>
#include <QString>
#include <QStringList>
#include <QVector>

struct ErrorCircle
{
    int circleIndex = -1;
    int expectedIndex = -1;
};

struct AnnotationRecord
{
    QString type;
    QStringList types;
    int startFrame = 0;
    int endFrame = 0;
    double startTimeSec = 0.0;
    double endTimeSec = 0.0;
    int circleIndex = -1;
    QVector<ErrorCircle> errorCircles;
    QString description;
};

struct CorrectionShape
{
    QString name;
    QString shapeType;
    int frame = 0;
    QString errorType;
    QStringList errorTypes;
    int expectedIndex = -1;
    QVector<ErrorCircle> errorCircles;
    QString description;
    QVector<QPointF> points;
    QColor lineColor = QColor(255, 80, 80);
    int lineWidth = 1;
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
    bool removeCorrectionsForFrame(int frame);
    const QVector<CorrectionShape>& corrections() const;
    QVector<CorrectionShape> correctionsForFrame(int frame) const;

private:
    QVector<AnnotationRecord> m_records;
    QVector<CorrectionShape> m_corrections;
};

QString annotationTypeDisplayName(const QString& type);
QString annotationTypesDisplayName(const QStringList& types);
QString errorCirclesDisplayName(const QVector<ErrorCircle>& circles);
QString annotationTypeCodeFromIndex(int index);

#endif
