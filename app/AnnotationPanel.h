#ifndef ANNOTATION_PANEL_H
#define ANNOTATION_PANEL_H

#include "AnnotationModel.h"

#include <QColor>
#include <QMap>
#include <QStringList>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QTextEdit;
class QVBoxLayout;

class AnnotationPanel : public QWidget
{
    Q_OBJECT

public:
    explicit AnnotationPanel(QWidget* parent = nullptr);

    void setCurrentContext(int frame, double timeSec, int circleCount);
    void setSegmentStart(int frame);
    void setSegmentEnd(int frame);
    void setAnnotations(const QVector<AnnotationRecord>& records, const QVector<CorrectionShape>& corrections);
    void setSelectedErrorCircleIndices(const QVector<int>& circleIndices);
    QStringList selectedTypes() const;
    QVector<ErrorCircle> selectedErrorCircles() const;
    QColor selectedCorrectionColor() const;
    int selectedCorrectionLineWidth() const;
    QString noteText() const;

signals:
    void currentFrameAnnotationRequested(const QStringList& types,
                                         const QVector<ErrorCircle>& errorCircles,
                                         const QString& description);
    void segmentStartRequested();
    void segmentEndRequested();
    void segmentAnnotationRequested(const QStringList& types,
                                    const QVector<ErrorCircle>& errorCircles,
                                    const QString& description);
    void deleteAnnotationRequested(int row);
    void deleteCorrectionRequested(int row);
    void deleteAnnotationsRequested(const QVector<int>& rows);
    void deleteCorrectionsRequested(const QVector<int>& rows);
    void correctionToolChanged(const QString& tool);
    void correctionStyleChanged(const QColor& color, int lineWidth);
    void autoIdentifyRequested();
    void recordActivated(int frame);

private:
    void addCustomType();
    void deleteSelectedRecords();
    void updateColorButton();
    void updateExpectedEditors();

    QLabel* m_contextLabel = nullptr;
    QCheckBox* m_falsePositiveCheck = nullptr;
    QCheckBox* m_missedDetectionCheck = nullptr;
    QCheckBox* m_wrongOrderCheck = nullptr;
    QCheckBox* m_targetJumpCheck = nullptr;
    QCheckBox* m_otherCheck = nullptr;
    QLineEdit* m_customTypeEdit = nullptr;
    QVBoxLayout* m_customTypeLayout = nullptr;
    QVector<QCheckBox*> m_customTypeChecks;
    QListWidget* m_errorCircleList = nullptr;
    QWidget* m_expectedEditorWidget = nullptr;
    QFormLayout* m_expectedEditorLayout = nullptr;
    QMap<int, QSpinBox*> m_expectedSpins;
    QComboBox* m_toolCombo = nullptr;
    QPushButton* m_colorButton = nullptr;
    QSpinBox* m_lineWidthSpin = nullptr;
    QColor m_lineColor = QColor(255, 80, 80);
    QTextEdit* m_noteEdit = nullptr;
    QLabel* m_segmentLabel = nullptr;
    QListWidget* m_listWidget = nullptr;
};

#endif
