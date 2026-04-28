#ifndef ANNOTATION_PANEL_H
#define ANNOTATION_PANEL_H

#include "AnnotationModel.h"

#include <QColor>
#include <QMap>
#include <QWidget>

class QComboBox;
class QFormLayout;
class QGroupBox;
class QLabel;
class QLineEdit;
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
    void setCurrentFrameCorrections(const QVector<CorrectionShape>& corrections, bool force = false);
    void setAnnotations(const QVector<AnnotationRecord>& records, const QVector<CorrectionShape>& corrections);
    QVector<CorrectionShape> draftCorrections() const;
    bool activeDraftShape(CorrectionShape* shape) const;
    void applyDrawnCorrectionShape(const QString& shapeType, const QVector<QPointF>& points);
    void applyAutoIdentifiedErrorCircles(const QVector<int>& circleIndices);

signals:
    void saveCurrentFrameCorrectionsRequested(const QVector<CorrectionShape>& corrections);
    void deleteAnnotationsRequested(const QVector<int>& rows);
    void deleteCorrectionsRequested(const QVector<int>& rows);
    void correctionToolChanged(const QString& tool);
    void correctionStyleChanged(const QColor& color, int lineWidth);
    void autoIdentifyRequested();
    void recordActivated(int frame);

private:
    void addDraftEntry();
    void deleteDraftEntry();
    void saveDraftEntries();
    void addCustomType();
    void deleteSelectedRecords();
    void syncActiveDraftFromWidgets();
    void syncWidgetsFromActiveDraft();
    void refreshDraftList();
    void refreshSourceList();
    void refreshExpectedEditors();
    void refreshTypeFields();
    void refreshFilterTypes();
    void refreshSavedList();
    void clearSavedFilters();
    void updateColorButton();
    void ensureCustomTypeAvailable(const QString& type, const QString& description = QString());
    int activeDraftIndex() const;
    QString defaultDraftName(int index) const;
    QString draftDisplayName(const CorrectionShape& draft, int index) const;
    QString savedAnnotationSummary(const AnnotationRecord& record) const;
    QString savedCorrectionSummary(const CorrectionShape& shape) const;
    QStringList checkedFilterTypes() const;
    bool frameRangeAccepted(int* startFrame, int* endFrame);
    bool savedFrameMatches(int startFrame, int endFrame, int itemStartFrame, int itemEndFrame) const;
    bool savedTypesMatch(const QStringList& types) const;
    bool validateDraft(const CorrectionShape& draft, int index, QString* errorMessage) const;
    bool typeRequiresSource(const QString& type) const;
    bool typeRequiresExpectedForSources(const QString& type) const;
    bool isCustomType(const QString& type) const;

    QLabel* m_contextLabel = nullptr;
    QListWidget* m_draftList = nullptr;
    QLineEdit* m_nameEdit = nullptr;
    QComboBox* m_typeCombo = nullptr;
    QLineEdit* m_customTypeNameEdit = nullptr;
    QTextEdit* m_customTypeDescriptionEdit = nullptr;
    QGroupBox* m_sourceGroup = nullptr;
    QListWidget* m_errorCircleList = nullptr;
    QWidget* m_expectedEditorWidget = nullptr;
    QFormLayout* m_expectedEditorLayout = nullptr;
    QMap<int, QSpinBox*> m_expectedSpins;
    QGroupBox* m_missingGroup = nullptr;
    QLabel* m_missingExpectedLabel = nullptr;
    QSpinBox* m_missingExpectedSpin = nullptr;
    QTextEdit* m_noteEdit = nullptr;
    QGroupBox* m_toolGroup = nullptr;
    QComboBox* m_toolCombo = nullptr;
    QPushButton* m_colorButton = nullptr;
    QSpinBox* m_lineWidthSpin = nullptr;
    QSpinBox* m_filterStartSpin = nullptr;
    QSpinBox* m_filterEndSpin = nullptr;
    QListWidget* m_filterTypeList = nullptr;
    QListWidget* m_savedList = nullptr;

    QVector<AnnotationRecord> m_allRecords;
    QVector<CorrectionShape> m_allCorrections;
    QVector<CorrectionShape> m_drafts;
    QMap<QString, QString> m_customTypeDescriptions;
    int m_currentFrame = -1;
    int m_circleCount = 0;
    int m_nextDraftNumber = 1;
    bool m_draftsDirty = false;
    bool m_updating = false;
    QColor m_lineColor = QColor(255, 80, 80);
};

#endif
