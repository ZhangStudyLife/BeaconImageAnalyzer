#ifndef ANNOTATION_PANEL_H
#define ANNOTATION_PANEL_H

#include "AnnotationModel.h"

#include <QWidget>

class QComboBox;
class QLabel;
class QListWidget;
class QPushButton;
class QSpinBox;
class QTextEdit;

class AnnotationPanel : public QWidget
{
    Q_OBJECT

public:
    explicit AnnotationPanel(QWidget* parent = nullptr);

    void setCurrentContext(int frame, double timeSec, int circleCount);
    void setSegmentStart(int frame);
    void setSegmentEnd(int frame);
    void setAnnotations(const QVector<AnnotationRecord>& records, const QVector<CorrectionShape>& corrections);
    QString selectedType() const;
    int selectedCircleIndex() const;
    int selectedExpectedIndex() const;
    QString noteText() const;

signals:
    void currentFrameAnnotationRequested(const QString& type, int circleIndex, const QString& description);
    void segmentStartRequested();
    void segmentEndRequested();
    void segmentAnnotationRequested(const QString& type, int circleIndex, const QString& description);
    void deleteAnnotationRequested(int row);
    void deleteCorrectionRequested(int row);
    void correctionToolChanged(const QString& tool);
    void recordActivated(int frame);

private:
    void deleteSelectedRecord();

    QLabel* m_contextLabel = nullptr;
    QComboBox* m_typeCombo = nullptr;
    QComboBox* m_toolCombo = nullptr;
    QSpinBox* m_circleSpin = nullptr;
    QSpinBox* m_expectedSpin = nullptr;
    QTextEdit* m_noteEdit = nullptr;
    QLabel* m_segmentLabel = nullptr;
    QListWidget* m_listWidget = nullptr;
};

#endif
