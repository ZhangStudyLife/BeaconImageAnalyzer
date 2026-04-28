#include "AnnotationPanel.h"

#include "beacon_image.h"

#include <QAbstractItemView>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QSplitter>
#include <QTextEdit>
#include <QVBoxLayout>

#include <algorithm>
#include <functional>

namespace
{
constexpr int KindRole = Qt::UserRole;
constexpr int IndexRole = Qt::UserRole + 1;
constexpr int FrameRole = Qt::UserRole + 2;
constexpr int CircleIndexRole = Qt::UserRole + 3;
constexpr int TypeCodeRole = Qt::UserRole + 4;

QString customTypeCode(const QString& name)
{
    return QStringLiteral("custom:%1").arg(name.trimmed());
}

bool isMissedType(const QString& type)
{
    return type == QStringLiteral("missed_detection");
}

bool isOtherType(const QString& type)
{
    return type == QStringLiteral("other");
}

bool isFalsePositiveType(const QString& type)
{
    return type == QStringLiteral("false_positive");
}
}

AnnotationPanel::AnnotationPanel(QWidget* parent)
    : QWidget(parent)
{
    m_contextLabel = new QLabel(QStringLiteral("Frame: -"), this);
    m_contextLabel->setObjectName(QStringLiteral("SoftLabel"));

    m_draftList = new QListWidget(this);
    m_draftList->setSelectionMode(QAbstractItemView::SingleSelection);
    m_draftList->setMinimumHeight(96);

    auto* addDraftButton = new QPushButton(QStringLiteral("新增纠错条目"), this);
    addDraftButton->setProperty("role", QStringLiteral("primary"));
    auto* deleteDraftButton = new QPushButton(QStringLiteral("删除当前条目"), this);
    auto* saveDraftButton = new QPushButton(QStringLiteral("保存当前帧纠错"), this);

    auto* entryGroup = new QGroupBox(QStringLiteral("纠错条目"), this);
    auto* entryLayout = new QVBoxLayout(entryGroup);
    entryLayout->setContentsMargins(10, 8, 10, 10);
    entryLayout->setSpacing(8);
    auto* entryButtons = new QHBoxLayout;
    entryButtons->setContentsMargins(0, 0, 0, 0);
    entryButtons->setSpacing(6);
    entryButtons->addWidget(addDraftButton);
    entryButtons->addWidget(deleteDraftButton);
    entryLayout->addWidget(m_draftList);
    entryLayout->addLayout(entryButtons);
    entryLayout->addWidget(saveDraftButton);

    m_nameEdit = new QLineEdit(this);
    m_nameEdit->setPlaceholderText(QStringLiteral("条目名称"));
    m_typeCombo = new QComboBox(this);
    m_typeCombo->addItem(QStringLiteral("请选择"), QString());
    m_typeCombo->addItem(QStringLiteral("误检"), QStringLiteral("false_positive"));
    m_typeCombo->addItem(QStringLiteral("漏检"), QStringLiteral("missed_detection"));
    m_typeCombo->addItem(QStringLiteral("排序错误"), QStringLiteral("wrong_order"));
    m_typeCombo->addItem(QStringLiteral("目标跳变"), QStringLiteral("target_jump"));
    m_typeCombo->addItem(QStringLiteral("其他"), QStringLiteral("other"));

    m_noteEdit = new QTextEdit(this);
    m_noteEdit->setPlaceholderText(QStringLiteral("描述"));
    m_noteEdit->setFixedHeight(64);

    auto* typeGroup = new QGroupBox(QStringLiteral("单条配置"), this);
    auto* typeForm = new QFormLayout(typeGroup);
    typeForm->setContentsMargins(10, 8, 10, 10);
    typeForm->setVerticalSpacing(7);
    typeForm->setHorizontalSpacing(8);
    typeForm->addRow(QStringLiteral("名称"), m_nameEdit);
    typeForm->addRow(QStringLiteral("错误类型"), m_typeCombo);
    typeForm->addRow(QStringLiteral("描述"), m_noteEdit);

    m_customTypeNameEdit = new QLineEdit(this);
    m_customTypeNameEdit->setPlaceholderText(QStringLiteral("自定义类型名称"));
    m_customTypeDescriptionEdit = new QTextEdit(this);
    m_customTypeDescriptionEdit->setPlaceholderText(QStringLiteral("配套描述"));
    m_customTypeDescriptionEdit->setFixedHeight(56);
    auto* addTypeButton = new QPushButton(QStringLiteral("新增类型"), this);

    auto* customTypeGroup = new QGroupBox(QStringLiteral("自定义错误类型"), this);
    auto* customTypeForm = new QFormLayout(customTypeGroup);
    customTypeForm->setContentsMargins(10, 8, 10, 10);
    customTypeForm->setVerticalSpacing(7);
    customTypeForm->setHorizontalSpacing(8);
    customTypeForm->addRow(QStringLiteral("名称"), m_customTypeNameEdit);
    customTypeForm->addRow(QStringLiteral("描述"), m_customTypeDescriptionEdit);
    customTypeForm->addRow(QString(), addTypeButton);

    m_sourceGroup = new QGroupBox(QStringLiteral("错误源"), this);
    auto* sourceLayout = new QVBoxLayout(m_sourceGroup);
    sourceLayout->setContentsMargins(10, 8, 10, 10);
    sourceLayout->setSpacing(7);
    m_errorCircleList = new QListWidget(m_sourceGroup);
    m_errorCircleList->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_errorCircleList->setMinimumHeight(72);
    m_errorCircleList->setMaximumHeight(112);
    m_expectedEditorWidget = new QWidget(m_sourceGroup);
    m_expectedEditorLayout = new QFormLayout(m_expectedEditorWidget);
    m_expectedEditorLayout->setContentsMargins(0, 0, 0, 0);
    m_expectedEditorLayout->setVerticalSpacing(6);
    m_expectedEditorLayout->setHorizontalSpacing(8);
    sourceLayout->addWidget(m_errorCircleList);
    sourceLayout->addWidget(m_expectedEditorWidget);

    m_missingExpectedLabel = new QLabel(QStringLiteral("漏检正确序号"), this);
    m_missingExpectedLabel->setObjectName(QStringLiteral("SoftLabel"));
    m_missingExpectedSpin = new QSpinBox(this);
    m_missingExpectedSpin->setRange(-1, BEACON_MAX_CIRCLE_COUNT - 1);
    m_missingExpectedSpin->setSpecialValueText(QStringLiteral("未填"));

    m_missingGroup = new QGroupBox(QStringLiteral("漏检目标"), this);
    auto* missingForm = new QFormLayout(m_missingGroup);
    missingForm->setContentsMargins(10, 8, 10, 10);
    missingForm->setVerticalSpacing(7);
    missingForm->setHorizontalSpacing(8);
    missingForm->addRow(m_missingExpectedLabel, m_missingExpectedSpin);
    typeForm->addRow(m_sourceGroup);
    typeForm->addRow(m_missingGroup);

    m_toolCombo = new QComboBox(this);
    m_toolCombo->addItem(QStringLiteral("选择"), QStringLiteral("select"));
    m_toolCombo->addItem(QStringLiteral("画圆"), QStringLiteral("circle"));
    m_toolCombo->addItem(QStringLiteral("画矩形"), QStringLiteral("rect"));
    m_toolCombo->addItem(QStringLiteral("画点"), QStringLiteral("point"));
    m_toolCombo->addItem(QStringLiteral("自由闭合"), QStringLiteral("polygon"));
    m_colorButton = new QPushButton(this);
    m_lineWidthSpin = new QSpinBox(this);
    m_lineWidthSpin->setRange(1, 15);
    m_lineWidthSpin->setValue(2);
    updateColorButton();
    auto* autoIdentifyButton = new QPushButton(QStringLiteral("自动识别"), this);

    m_toolGroup = new QGroupBox(QStringLiteral("纠错工具"), this);
    auto* toolForm = new QFormLayout(m_toolGroup);
    toolForm->setContentsMargins(10, 8, 10, 10);
    toolForm->setVerticalSpacing(7);
    toolForm->setHorizontalSpacing(8);
    toolForm->addRow(QStringLiteral("绘制"), m_toolCombo);
    toolForm->addRow(QStringLiteral("线条颜色"), m_colorButton);
    toolForm->addRow(QStringLiteral("线条粗细"), m_lineWidthSpin);
    toolForm->addRow(QString(), autoIdentifyButton);

    m_savedList = new QListWidget(this);
    m_savedList->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_savedList->setMinimumHeight(160);
    m_savedList->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* deleteSavedButton = new QPushButton(QStringLiteral("删除选中标注"), this);

    m_filterStartSpin = new QSpinBox(this);
    m_filterStartSpin->setRange(-1, 100000000);
    m_filterStartSpin->setSpecialValueText(QStringLiteral("不限"));
    m_filterStartSpin->setValue(-1);
    m_filterStartSpin->setMinimumWidth(96);
    m_filterEndSpin = new QSpinBox(this);
    m_filterEndSpin->setRange(-1, 100000000);
    m_filterEndSpin->setSpecialValueText(QStringLiteral("不限"));
    m_filterEndSpin->setValue(-1);
    m_filterEndSpin->setMinimumWidth(96);
    m_filterTypeList = new QListWidget(this);
    m_filterTypeList->setSelectionMode(QAbstractItemView::NoSelection);
    m_filterTypeList->setMaximumHeight(96);
    auto* applyFilterButton = new QPushButton(QStringLiteral("应用筛选"), this);
    auto* clearFilterButton = new QPushButton(QStringLiteral("清除筛选"), this);

    auto* filterGroup = new QGroupBox(QStringLiteral("已保存标注筛选"), this);
    auto* filterLayout = new QVBoxLayout(filterGroup);
    filterLayout->setContentsMargins(10, 8, 10, 10);
    filterLayout->setSpacing(7);
    auto makeSpinColumn = [](const QString& labelText, QWidget* editor, QWidget* parent) {
        auto* container = new QWidget(parent);
        auto* layout = new QVBoxLayout(container);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(4);
        layout->addWidget(new QLabel(labelText, container));
        layout->addWidget(editor);
        return container;
    };

    auto* frameFilterRow = new QHBoxLayout;
    frameFilterRow->setContentsMargins(0, 0, 0, 0);
    frameFilterRow->setSpacing(6);
    frameFilterRow->addWidget(makeSpinColumn(QStringLiteral("起始帧"), m_filterStartSpin, filterGroup), 1);
    frameFilterRow->addWidget(makeSpinColumn(QStringLiteral("结束帧"), m_filterEndSpin, filterGroup), 1);
    filterLayout->addLayout(frameFilterRow);
    filterLayout->addWidget(new QLabel(QStringLiteral("错误类型"), this));
    filterLayout->addWidget(m_filterTypeList);
    auto* filterButtonRow = new QHBoxLayout;
    filterButtonRow->setContentsMargins(0, 0, 0, 0);
    filterButtonRow->setSpacing(6);
    filterButtonRow->addWidget(applyFilterButton);
    filterButtonRow->addWidget(clearFilterButton);
    filterLayout->addLayout(filterButtonRow);

    auto* savedGroup = new QGroupBox(QStringLiteral("已保存标注"), this);
    auto* savedLayout = new QVBoxLayout(savedGroup);
    savedLayout->setContentsMargins(10, 8, 10, 10);
    savedLayout->setSpacing(7);

    m_batchGroup = new QGroupBox(QStringLiteral("跨帧批量操作"), this);
    auto* batchLayout = new QVBoxLayout(m_batchGroup);
    batchLayout->setContentsMargins(10, 8, 10, 10);
    batchLayout->setSpacing(8);
    m_batchSelectionLabel = new QLabel(m_batchGroup);
    m_batchSelectionLabel->setObjectName(QStringLiteral("SoftLabel"));
    m_batchSelectionLabel->setWordWrap(true);
    m_batchSelectionLabel->setText(QStringLiteral("请选择当前帧已保存纠错条目后执行批量操作。"));
    batchLayout->addWidget(m_batchSelectionLabel);

    m_batchManualGroup = new QGroupBox(QStringLiteral("手动批量添加"), m_batchGroup);
    auto* manualLayout = new QFormLayout(m_batchManualGroup);
    manualLayout->setContentsMargins(10, 8, 10, 10);
    manualLayout->setVerticalSpacing(7);
    manualLayout->setHorizontalSpacing(8);
    m_batchStartSpin = new QSpinBox(m_batchManualGroup);
    m_batchStartSpin->setRange(0, 0);
    m_batchStartSpin->setMinimumWidth(96);
    m_batchEndSpin = new QSpinBox(m_batchManualGroup);
    m_batchEndSpin->setRange(0, 0);
    m_batchEndSpin->setMinimumWidth(96);
    auto* manualFrameRow = new QHBoxLayout;
    manualFrameRow->setContentsMargins(0, 0, 0, 0);
    manualFrameRow->setSpacing(6);
    manualFrameRow->addWidget(makeSpinColumn(QStringLiteral("起始帧"), m_batchStartSpin, m_batchManualGroup), 1);
    manualFrameRow->addWidget(makeSpinColumn(QStringLiteral("结束帧"), m_batchEndSpin, m_batchManualGroup), 1);
    auto* manualAddButton = new QPushButton(QStringLiteral("确认批量添加"), m_batchManualGroup);
    manualLayout->addRow(QStringLiteral("目标帧"), manualFrameRow);
    m_batchManualOverlapSpin = new QDoubleSpinBox(m_batchManualGroup);
    m_batchManualOverlapSpin->setRange(0.0, BEACON_IMAGE_W * BEACON_IMAGE_H);
    m_batchManualOverlapSpin->setDecimals(0);
    m_batchManualOverlapSpin->setValue(1.0);
    m_batchManualOverlapSpin->setSuffix(QStringLiteral(" px"));
    m_batchManualOverlapSpin->setMinimumWidth(110);
    m_batchManualOverlapWidget = makeSpinColumn(QStringLiteral("两帧目标圆最小重叠像素阈值"),
                                                m_batchManualOverlapSpin,
                                                m_batchManualGroup);
    manualLayout->addRow(m_batchManualOverlapWidget);
    manualLayout->addRow(QString(), manualAddButton);
    batchLayout->addWidget(m_batchManualGroup);

    m_batchAutoGroup = new QGroupBox(QStringLiteral("自动批量添加"), m_batchGroup);
    auto* autoLayout = new QFormLayout(m_batchAutoGroup);
    autoLayout->setContentsMargins(10, 8, 10, 10);
    autoLayout->setVerticalSpacing(7);
    autoLayout->setHorizontalSpacing(8);
    m_batchBackwardSpin = new QSpinBox(m_batchAutoGroup);
    m_batchBackwardSpin->setRange(0, 100000);
    m_batchBackwardSpin->setValue(30);
    m_batchBackwardSpin->setMinimumWidth(110);
    m_batchForwardSpin = new QSpinBox(m_batchAutoGroup);
    m_batchForwardSpin->setRange(0, 100000);
    m_batchForwardSpin->setValue(30);
    m_batchForwardSpin->setMinimumWidth(110);
    m_batchThresholdSpin = new QDoubleSpinBox(m_batchAutoGroup);
    m_batchThresholdSpin->setRange(0.0, 1000.0);
    m_batchThresholdSpin->setDecimals(2);
    m_batchThresholdSpin->setValue(3.0);
    m_batchThresholdSpin->setSuffix(QStringLiteral(" px"));
    m_batchThresholdSpin->setMinimumWidth(110);
    m_batchAutoFrameWidget = new QWidget(m_batchAutoGroup);
    auto* autoFrameRow = new QHBoxLayout(m_batchAutoFrameWidget);
    autoFrameRow->setContentsMargins(0, 0, 0, 0);
    autoFrameRow->setSpacing(6);
    autoFrameRow->addWidget(makeSpinColumn(QStringLiteral("向前最大帧数"), m_batchBackwardSpin, m_batchAutoGroup), 1);
    autoFrameRow->addWidget(makeSpinColumn(QStringLiteral("向后最大帧数"), m_batchForwardSpin, m_batchAutoGroup), 1);
    autoLayout->addRow(m_batchAutoFrameWidget);
    m_batchPositionThresholdWidget = makeSpinColumn(QStringLiteral("坐标偏差阈值"),
                                                    m_batchThresholdSpin,
                                                    m_batchAutoGroup);
    autoLayout->addRow(m_batchPositionThresholdWidget);
    m_batchAutoOverlapSpin = new QDoubleSpinBox(m_batchAutoGroup);
    m_batchAutoOverlapSpin->setRange(0.0, BEACON_IMAGE_W * BEACON_IMAGE_H);
    m_batchAutoOverlapSpin->setDecimals(0);
    m_batchAutoOverlapSpin->setValue(1.0);
    m_batchAutoOverlapSpin->setSuffix(QStringLiteral(" px"));
    m_batchAutoOverlapSpin->setMinimumWidth(110);
    m_batchAutoOverlapWidget = makeSpinColumn(QStringLiteral("两帧目标圆最小重叠像素阈值"),
                                              m_batchAutoOverlapSpin,
                                              m_batchAutoGroup);
    autoLayout->addRow(m_batchAutoOverlapWidget);
    auto* autoMatchButton = new QPushButton(QStringLiteral("自动识别匹配帧"), m_batchAutoGroup);
    m_batchApplyAutoButton = new QPushButton(QStringLiteral("全部批量标记"), m_batchAutoGroup);
    m_batchApplyAutoButton->setEnabled(false);
    auto* autoButtonRow = new QHBoxLayout;
    autoButtonRow->setContentsMargins(0, 0, 0, 0);
    autoButtonRow->setSpacing(6);
    autoButtonRow->addWidget(autoMatchButton);
    autoButtonRow->addWidget(m_batchApplyAutoButton);
    autoLayout->addRow(QString(), autoButtonRow);
    m_batchMatchedFrameList = new QListWidget(m_batchAutoGroup);
    m_batchMatchedFrameList->setSelectionMode(QAbstractItemView::NoSelection);
    m_batchMatchedFrameList->setMaximumHeight(82);
    autoLayout->addRow(QStringLiteral("匹配帧"), m_batchMatchedFrameList);
    batchLayout->addWidget(m_batchAutoGroup);
    m_batchGroup->setVisible(true);
    setBatchPanelMode(false);

    savedLayout->addWidget(m_savedList, 1);
    savedLayout->addWidget(deleteSavedButton);

    auto* coreGroup = new QGroupBox(QStringLiteral("核心操作区"), this);
    auto* coreLayout = new QGridLayout(coreGroup);
    coreLayout->setContentsMargins(10, 10, 10, 10);
    coreLayout->setHorizontalSpacing(10);
    coreLayout->setVerticalSpacing(10);
    coreLayout->addWidget(entryGroup, 0, 0);
    coreLayout->addWidget(typeGroup, 0, 1);
    coreLayout->addWidget(customTypeGroup, 1, 0);
    coreLayout->addWidget(m_toolGroup, 1, 1);
    coreLayout->setColumnStretch(0, 1);
    coreLayout->setColumnStretch(1, 1);
    coreLayout->setRowStretch(0, 1);
    coreLayout->setRowStretch(1, 1);

    auto* managementGroup = new QGroupBox(QStringLiteral("标注管理区"), this);
    auto* managementLayout = new QGridLayout(managementGroup);
    managementLayout->setContentsMargins(10, 10, 10, 10);
    managementLayout->setHorizontalSpacing(10);
    managementLayout->setVerticalSpacing(10);
    filterGroup->setMaximumWidth(260);
    filterGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    savedGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_batchGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    managementLayout->addWidget(savedGroup, 0, 0, 1, 2);
    managementLayout->addWidget(filterGroup, 1, 0);
    managementLayout->addWidget(m_batchGroup, 1, 1);
    managementLayout->setColumnStretch(0, 0);
    managementLayout->setColumnStretch(1, 1);
    managementLayout->setRowStretch(0, 2);
    managementLayout->setRowStretch(1, 1);

    auto* splitter = new QSplitter(Qt::Vertical, this);
    splitter->setChildrenCollapsible(false);
    splitter->addWidget(coreGroup);
    splitter->addWidget(managementGroup);
    splitter->setStretchFactor(0, 2);
    splitter->setStretchFactor(1, 1);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(9);
    layout->addWidget(m_contextLabel);
    layout->addWidget(splitter, 1);

    connect(addDraftButton, &QPushButton::clicked, this, &AnnotationPanel::addDraftEntry);
    connect(deleteDraftButton, &QPushButton::clicked, this, &AnnotationPanel::deleteDraftEntry);
    connect(saveDraftButton, &QPushButton::clicked, this, &AnnotationPanel::saveDraftEntries);
    connect(addTypeButton, &QPushButton::clicked, this, &AnnotationPanel::addCustomType);
    connect(m_customTypeNameEdit, &QLineEdit::returnPressed, this, &AnnotationPanel::addCustomType);
    connect(deleteSavedButton, &QPushButton::clicked, this, &AnnotationPanel::deleteSelectedRecords);
    connect(applyFilterButton, &QPushButton::clicked, this, &AnnotationPanel::refreshSavedList);
    connect(clearFilterButton, &QPushButton::clicked, this, &AnnotationPanel::clearSavedFilters);
    connect(manualAddButton, &QPushButton::clicked, this, &AnnotationPanel::applyManualBatchAdd);
    connect(autoMatchButton, &QPushButton::clicked, this, &AnnotationPanel::requestAutoBatchMatch);
    connect(m_batchApplyAutoButton, &QPushButton::clicked, this, &AnnotationPanel::applyAutoBatchAdd);
    connect(m_filterTypeList, &QListWidget::itemChanged, this, [this]() {
        refreshSavedList();
    });

    auto* deleteShortcut = new QShortcut(QKeySequence::Delete, m_savedList);
    deleteShortcut->setContext(Qt::WidgetShortcut);
    connect(deleteShortcut, &QShortcut::activated, this, &AnnotationPanel::deleteSelectedRecords);

    connect(m_draftList, &QListWidget::currentRowChanged, this, [this]() {
        syncWidgetsFromActiveDraft();
    });
    connect(m_nameEdit, &QLineEdit::textEdited, this, &AnnotationPanel::syncActiveDraftFromWidgets);
    connect(m_typeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        if (m_typeCombo->currentData().toString() == QStringLiteral("missed_detection"))
        {
            const int circleTool = m_toolCombo->findData(QStringLiteral("circle"));
            if (circleTool >= 0)
            {
                m_toolCombo->setCurrentIndex(circleTool);
            }
        }
        refreshTypeFields();
        syncActiveDraftFromWidgets();
    });
    connect(m_noteEdit, &QTextEdit::textChanged, this, &AnnotationPanel::syncActiveDraftFromWidgets);
    connect(m_missingExpectedSpin, qOverload<int>(&QSpinBox::valueChanged), this, &AnnotationPanel::syncActiveDraftFromWidgets);
    connect(m_errorCircleList, &QListWidget::itemSelectionChanged, this, [this]() {
        refreshExpectedEditors();
        syncActiveDraftFromWidgets();
    });
    connect(m_toolCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        emit correctionToolChanged(m_toolCombo->currentData().toString());
    });
    connect(m_colorButton, &QPushButton::clicked, this, [this]() {
        const QColor color = QColorDialog::getColor(m_lineColor, this, QStringLiteral("选择线条颜色"));
        if (!color.isValid())
        {
            return;
        }
        m_lineColor = color;
        updateColorButton();
        emit correctionStyleChanged(m_lineColor, m_lineWidthSpin->value());
        syncActiveDraftFromWidgets();
    });
    connect(m_lineWidthSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
        emit correctionStyleChanged(m_lineColor, value);
        syncActiveDraftFromWidgets();
    });
    connect(autoIdentifyButton, &QPushButton::clicked, this, &AnnotationPanel::autoIdentifyRequested);
    connect(m_savedList, &QListWidget::itemSelectionChanged, this, &AnnotationPanel::refreshBatchPanel);
    connect(m_savedList, &QListWidget::currentItemChanged, this, [this](QListWidgetItem*, QListWidgetItem*) {
        refreshBatchPanel();
    });
    connect(m_savedList, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
        if (item != nullptr)
        {
            emit recordActivated(item->data(FrameRole).toInt());
        }
    });

    refreshSourceList();
    refreshFilterTypes();
    refreshTypeFields();
}

void AnnotationPanel::setCurrentContext(int frame, double timeSec, int circleCount)
{
    m_contextLabel->setText(QStringLiteral("Frame %1 / %2 s").arg(frame).arg(timeSec, 0, 'f', 3));
    if (frame != m_currentFrame)
    {
        m_currentFrame = frame;
        m_drafts.clear();
        m_nextDraftNumber = 1;
        m_draftsDirty = false;
        refreshDraftList();
    }
    m_circleCount = qBound(0, circleCount, BEACON_MAX_CIRCLE_COUNT);
    refreshSourceList();
    refreshBatchPanel();
}

void AnnotationPanel::setSegmentStart(int frame)
{
    Q_UNUSED(frame);
}

void AnnotationPanel::setSegmentEnd(int frame)
{
    Q_UNUSED(frame);
}

void AnnotationPanel::setVideoFrameRange(int firstFrame, int lastFrame)
{
    m_firstFrame = qMax(0, firstFrame);
    m_lastFrame = qMax(m_firstFrame, lastFrame);

    m_batchStartSpin->setRange(m_firstFrame, m_lastFrame);
    m_batchEndSpin->setRange(m_firstFrame, m_lastFrame);
    m_batchBackwardSpin->setMaximum(qMax(0, m_lastFrame - m_firstFrame));
    m_batchForwardSpin->setMaximum(qMax(0, m_lastFrame - m_firstFrame));
    if (m_currentFrame >= m_firstFrame && m_currentFrame <= m_lastFrame)
    {
        m_batchStartSpin->setValue(m_currentFrame);
        m_batchEndSpin->setValue(m_currentFrame);
    }
}

void AnnotationPanel::setCurrentFrameCorrections(const QVector<CorrectionShape>& corrections, bool force)
{
    if (m_draftsDirty && !force)
    {
        return;
    }

    m_drafts = corrections;
    m_nextDraftNumber = m_drafts.size() + 1;
    m_draftsDirty = false;
    refreshDraftList();
    if (!m_drafts.isEmpty())
    {
        m_draftList->setCurrentRow(0);
        syncWidgetsFromActiveDraft();
    }
    else
    {
        syncWidgetsFromActiveDraft();
    }
}

void AnnotationPanel::setAnnotations(const QVector<AnnotationRecord>& records, const QVector<CorrectionShape>& corrections)
{
    m_allRecords = records;
    m_allCorrections = corrections;
    for (const AnnotationRecord& record : m_allRecords)
    {
        const QStringList types = record.types.isEmpty() ? QStringList{ record.type } : record.types;
        for (const QString& type : types)
        {
            ensureCustomTypeAvailable(type);
        }
    }
    for (const CorrectionShape& shape : m_allCorrections)
    {
        const QStringList types = shape.errorTypes.isEmpty() ? QStringList{ shape.errorType } : shape.errorTypes;
        for (const QString& type : types)
        {
            ensureCustomTypeAvailable(type);
        }
    }
    refreshFilterTypes();
    refreshSavedList();
}

void AnnotationPanel::ensureCustomTypeAvailable(const QString& type, const QString& description)
{
    const QString code = type.trimmed();
    if (!isCustomType(code))
    {
        return;
    }

    const QString name = code.mid(QStringLiteral("custom:").size()).trimmed();
    if (name.isEmpty())
    {
        return;
    }

    const QString trimmedDescription = description.trimmed();
    if (!trimmedDescription.isEmpty())
    {
        m_customTypeDescriptions.insert(code, trimmedDescription);
    }

    const int existingIndex = m_typeCombo->findData(code);
    if (existingIndex >= 0)
    {
        if (!trimmedDescription.isEmpty())
        {
            m_typeCombo->setItemData(existingIndex, trimmedDescription, Qt::ToolTipRole);
        }
        return;
    }

    const int otherIndex = m_typeCombo->findData(QStringLiteral("other"));
    const int insertIndex = otherIndex >= 0 ? otherIndex : m_typeCombo->count();
    m_typeCombo->insertItem(insertIndex, name, code);
    if (!trimmedDescription.isEmpty())
    {
        m_typeCombo->setItemData(insertIndex, trimmedDescription, Qt::ToolTipRole);
    }
}

QStringList AnnotationPanel::checkedFilterTypes() const
{
    QStringList types;
    for (int row = 0; row < m_filterTypeList->count(); ++row)
    {
        const QListWidgetItem* item = m_filterTypeList->item(row);
        if (item != nullptr && item->checkState() == Qt::Checked)
        {
            types.push_back(item->data(TypeCodeRole).toString());
        }
    }
    return types;
}

bool AnnotationPanel::frameRangeAccepted(int* startFrame, int* endFrame)
{
    const int start = m_filterStartSpin->value();
    const int end = m_filterEndSpin->value();
    if (start >= 0 && end >= 0 && start > end)
    {
        QMessageBox::warning(this,
                             QStringLiteral("筛选条件错误"),
                             QStringLiteral("起始帧不能大于结束帧，请修正后再筛选。"));
        return false;
    }

    if (startFrame != nullptr)
    {
        *startFrame = start;
    }
    if (endFrame != nullptr)
    {
        *endFrame = end;
    }
    return true;
}

bool AnnotationPanel::savedFrameMatches(int startFrame, int endFrame, int itemStartFrame, int itemEndFrame) const
{
    if (startFrame >= 0 && itemEndFrame < startFrame)
    {
        return false;
    }
    if (endFrame >= 0 && itemStartFrame > endFrame)
    {
        return false;
    }
    return true;
}

bool AnnotationPanel::savedTypesMatch(const QStringList& types) const
{
    const QStringList checkedTypes = checkedFilterTypes();
    if (checkedTypes.isEmpty())
    {
        return true;
    }
    for (const QString& type : types)
    {
        if (checkedTypes.contains(type))
        {
            return true;
        }
    }
    return false;
}

QVector<int> AnnotationPanel::selectedCurrentFrameCorrectionRows() const
{
    QVector<int> rows;
    for (const QListWidgetItem* item : m_savedList->selectedItems())
    {
        if (item == nullptr ||
            item->data(KindRole).toString() != QStringLiteral("correction") ||
            item->data(FrameRole).toInt() != m_currentFrame)
        {
            continue;
        }
        rows.push_back(item->data(IndexRole).toInt());
    }

    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    return rows;
}

bool AnnotationPanel::selectedBatchTypes(bool* hasMissed, bool* hasNonMissed, bool* latestIsMissed) const
{
    bool missed = false;
    bool nonMissed = false;
    bool latestMissed = false;
    bool latestFound = false;

    const QListWidgetItem* current = m_savedList->currentItem();
    for (const QListWidgetItem* item : m_savedList->selectedItems())
    {
        if (item == nullptr ||
            item->data(KindRole).toString() != QStringLiteral("correction") ||
            item->data(FrameRole).toInt() != m_currentFrame)
        {
            continue;
        }

        QString type = item->data(TypeCodeRole).toString();
        const int row = item->data(IndexRole).toInt();
        if (type.isEmpty() && row >= 0 && row < m_allCorrections.size())
        {
            type = m_allCorrections[row].errorType;
        }

        const bool itemIsMissed = isMissedType(type);
        missed = missed || itemIsMissed;
        nonMissed = nonMissed || !itemIsMissed;
        if (item == current)
        {
            latestMissed = itemIsMissed;
            latestFound = true;
        }
        else if (!latestFound)
        {
            latestMissed = itemIsMissed;
        }
    }

    if (hasMissed != nullptr)
    {
        *hasMissed = missed;
    }
    if (hasNonMissed != nullptr)
    {
        *hasNonMissed = nonMissed;
    }
    if (latestIsMissed != nullptr)
    {
        *latestIsMissed = latestMissed;
    }
    return missed || nonMissed;
}

bool AnnotationPanel::selectedBatchHasMixedTypes() const
{
    bool hasMissed = false;
    bool hasNonMissed = false;
    selectedBatchTypes(&hasMissed, &hasNonMissed, nullptr);
    return hasMissed && hasNonMissed;
}

bool AnnotationPanel::batchPanelPreviewUsesMissed() const
{
    bool latestIsMissed = false;
    selectedBatchTypes(nullptr, nullptr, &latestIsMissed);
    return latestIsMissed;
}

void AnnotationPanel::setBatchPanelMode(bool missedMode)
{
    if (m_batchManualOverlapWidget != nullptr)
    {
        m_batchManualOverlapWidget->setVisible(missedMode);
    }
    if (m_batchAutoOverlapWidget != nullptr)
    {
        m_batchAutoOverlapWidget->setVisible(missedMode);
    }
    if (m_batchAutoFrameWidget != nullptr)
    {
        m_batchAutoFrameWidget->setVisible(!missedMode);
    }
    if (m_batchPositionThresholdWidget != nullptr)
    {
        m_batchPositionThresholdWidget->setVisible(!missedMode);
    }
}

void AnnotationPanel::warnMixedBatchTypes()
{
    QMessageBox::warning(this,
                         QStringLiteral("批量操作"),
                         QStringLiteral("选中条目同时包含漏检和非漏检类型，不可混用批量操作。"));
}

bool AnnotationPanel::batchFrameRangeAccepted(int startFrame, int endFrame) const
{
    if (startFrame > endFrame)
    {
        QMessageBox::warning(const_cast<AnnotationPanel*>(this),
                             QStringLiteral("批量添加"),
                             QStringLiteral("起始帧不能大于结束帧，请修正后再操作。"));
        return false;
    }
    if (m_lastFrame < m_firstFrame ||
        startFrame < m_firstFrame ||
        endFrame > m_lastFrame)
    {
        QMessageBox::warning(const_cast<AnnotationPanel*>(this),
                             QStringLiteral("批量添加"),
                             QStringLiteral("目标帧号超出视频有效范围。"));
        return false;
    }
    return true;
}

QString AnnotationPanel::savedAnnotationSummary(const AnnotationRecord& record) const
{
    const QStringList types = record.types.isEmpty() ? QStringList{ record.type } : record.types;
    const QVector<ErrorCircle> circles = record.errorCircles.isEmpty() && record.circleIndex >= 0
        ? QVector<ErrorCircle>{ ErrorCircle{ record.circleIndex, -1 } }
        : record.errorCircles;
    const QString frameText = record.startFrame == record.endFrame
        ? QStringLiteral("帧 %1").arg(record.startFrame)
        : QStringLiteral("帧 %1-%2").arg(record.startFrame).arg(record.endFrame);

    QStringList parts;
    parts << frameText << QStringLiteral("历史标注") << annotationTypesDisplayName(types);
    if (!circles.isEmpty())
    {
        parts << QStringLiteral("目标 %1").arg(errorCirclesDisplayName(circles));
    }
    if (!record.description.trimmed().isEmpty())
    {
        parts << QStringLiteral("说明：%1").arg(record.description.trimmed());
    }
    return parts.join(QStringLiteral(" | "));
}

QString AnnotationPanel::savedCorrectionSummary(const CorrectionShape& shape) const
{
    const QString type = shape.errorType.trimmed();
    QStringList parts;
    parts << QStringLiteral("帧 %1").arg(shape.frame)
          << annotationTypeDisplayName(type);

    if (isFalsePositiveType(type))
    {
        parts << QStringLiteral("错误源 %1").arg(errorCirclesDisplayName(shape.errorCircles));
    }
    else if (isMissedType(type))
    {
        int expectedIndex = shape.expectedIndex;
        for (const ErrorCircle& circle : shape.errorCircles)
        {
            if (circle.circleIndex < 0)
            {
                expectedIndex = circle.expectedIndex;
                break;
            }
        }
        parts << (expectedIndex >= 0
                      ? QStringLiteral("漏检目标 正确序号 #%1").arg(expectedIndex)
                      : QStringLiteral("漏检目标 正确序号未填"));
    }
    else if (isOtherType(type))
    {
        parts << (shape.description.trimmed().isEmpty()
                      ? QStringLiteral("说明未填")
                      : QStringLiteral("说明：%1").arg(shape.description.trimmed()));
    }
    else
    {
        parts << QStringLiteral("错误源 %1").arg(errorCirclesDisplayName(shape.errorCircles));
    }

    if (!isOtherType(type) && !shape.description.trimmed().isEmpty())
    {
        parts << QStringLiteral("说明：%1").arg(shape.description.trimmed());
    }
    return parts.join(QStringLiteral(" | "));
}

void AnnotationPanel::refreshFilterTypes()
{
    const QStringList checkedTypes = checkedFilterTypes();
    QStringList knownTypes;
    QSignalBlocker blocker(m_filterTypeList);
    m_filterTypeList->clear();

    auto addFilterType = [this, &checkedTypes, &knownTypes](const QString& type, const QString& label) {
        const QString code = type.trimmed();
        if (code.isEmpty() || knownTypes.contains(code))
        {
            return;
        }
        knownTypes.push_back(code);
        auto* item = new QListWidgetItem(label.trimmed().isEmpty() ? annotationTypeDisplayName(code) : label.trimmed());
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(checkedTypes.contains(code) ? Qt::Checked : Qt::Unchecked);
        item->setData(TypeCodeRole, code);
        if (m_customTypeDescriptions.contains(code))
        {
            item->setToolTip(m_customTypeDescriptions.value(code));
        }
        m_filterTypeList->addItem(item);
    };

    for (int i = 0; i < m_typeCombo->count(); ++i)
    {
        const QString type = m_typeCombo->itemData(i).toString();
        if (!type.isEmpty())
        {
            addFilterType(type, m_typeCombo->itemText(i));
        }
    }
}

void AnnotationPanel::refreshSavedList()
{
    int startFrame = -1;
    int endFrame = -1;
    if (!frameRangeAccepted(&startFrame, &endFrame))
    {
        return;
    }

    m_savedList->clear();
    for (int i = 0; i < m_allRecords.size(); ++i)
    {
        const AnnotationRecord& record = m_allRecords[i];
        const QStringList types = record.types.isEmpty() ? QStringList{ record.type } : record.types;
        if (!savedFrameMatches(startFrame, endFrame, record.startFrame, record.endFrame) ||
            !savedTypesMatch(types))
        {
            continue;
        }

        auto* item = new QListWidgetItem(savedAnnotationSummary(record));
        item->setData(KindRole, QStringLiteral("annotation"));
        item->setData(IndexRole, i);
        item->setData(FrameRole, record.startFrame);
        m_savedList->addItem(item);
    }

    for (int i = 0; i < m_allCorrections.size(); ++i)
    {
        const CorrectionShape& shape = m_allCorrections[i];
        const QStringList types = shape.errorTypes.isEmpty() ? QStringList{ shape.errorType } : shape.errorTypes;
        if (!savedFrameMatches(startFrame, endFrame, shape.frame, shape.frame) ||
            !savedTypesMatch(types))
        {
            continue;
        }

        auto* item = new QListWidgetItem(savedCorrectionSummary(shape));
        item->setData(KindRole, QStringLiteral("correction"));
        item->setData(IndexRole, i);
        item->setData(FrameRole, shape.frame);
        item->setData(TypeCodeRole, shape.errorType);
        m_savedList->addItem(item);
    }
    refreshBatchPanel();
}

void AnnotationPanel::clearSavedFilters()
{
    m_filterStartSpin->setValue(-1);
    m_filterEndSpin->setValue(-1);
    {
        QSignalBlocker blocker(m_filterTypeList);
        for (int row = 0; row < m_filterTypeList->count(); ++row)
        {
            QListWidgetItem* item = m_filterTypeList->item(row);
            if (item != nullptr)
            {
                item->setCheckState(Qt::Unchecked);
            }
        }
    }
    refreshSavedList();
}

void AnnotationPanel::refreshBatchPanel()
{
    const QVector<int> rows = selectedCurrentFrameCorrectionRows();
    const bool hasSelection = !rows.isEmpty();
    bool hasMissed = false;
    bool hasNonMissed = false;
    bool latestIsMissed = false;
    selectedBatchTypes(&hasMissed, &hasNonMissed, &latestIsMissed);
    m_batchGroup->setVisible(true);
    if (!hasSelection)
    {
        setBatchPanelMode(false);
        m_batchSelectionLabel->setText(QStringLiteral("请选择当前帧已保存纠错条目后执行批量操作。"));
        m_autoMatchedBatchFrames.clear();
        m_batchMatchedFrameList->clear();
        m_batchApplyAutoButton->setEnabled(false);
        return;
    }

    setBatchPanelMode(latestIsMissed);
    if (hasMissed && hasNonMissed)
    {
        m_batchSelectionLabel->setText(QStringLiteral("已选择当前帧纠错条目：%1 条；包含漏检和非漏检，执行批量操作时会拦截。").arg(rows.size()));
    }
    else if (hasMissed)
    {
        m_batchSelectionLabel->setText(QStringLiteral("已选择当前帧漏检纠错条目：%1 条").arg(rows.size()));
    }
    else
    {
        m_batchSelectionLabel->setText(QStringLiteral("已选择当前帧非漏检纠错条目：%1 条").arg(rows.size()));
    }
    if (m_currentFrame >= m_firstFrame && m_currentFrame <= m_lastFrame)
    {
        if (m_batchStartSpin->value() < m_firstFrame || m_batchStartSpin->value() > m_lastFrame)
        {
            m_batchStartSpin->setValue(m_currentFrame);
        }
        if (m_batchEndSpin->value() < m_firstFrame || m_batchEndSpin->value() > m_lastFrame)
        {
            m_batchEndSpin->setValue(m_currentFrame);
        }
    }

    m_autoMatchedBatchFrames.clear();
    m_batchMatchedFrameList->clear();
    m_batchApplyAutoButton->setEnabled(false);
}

void AnnotationPanel::applyManualBatchAdd()
{
    const QVector<int> rows = selectedCurrentFrameCorrectionRows();
    if (selectedBatchHasMixedTypes())
    {
        warnMixedBatchTypes();
        return;
    }
    if (rows.isEmpty())
    {
        QMessageBox::warning(this,
                             QStringLiteral("批量添加"),
                             QStringLiteral("请先选择当前帧已保存的纠错条目。"));
        return;
    }

    const int startFrame = m_batchStartSpin->value();
    const int endFrame = m_batchEndSpin->value();
    if (!batchFrameRangeAccepted(startFrame, endFrame))
    {
        return;
    }

    emit batchAddCorrectionsRequested(rows,
                                      startFrame,
                                      endFrame,
                                      m_batchManualOverlapSpin != nullptr ? m_batchManualOverlapSpin->value() : 0.0);
}

void AnnotationPanel::requestAutoBatchMatch()
{
    const QVector<int> rows = selectedCurrentFrameCorrectionRows();
    if (selectedBatchHasMixedTypes())
    {
        warnMixedBatchTypes();
        return;
    }
    if (rows.isEmpty())
    {
        QMessageBox::warning(this,
                             QStringLiteral("自动批量添加"),
                             QStringLiteral("请先选择当前帧已保存的纠错条目。"));
        return;
    }

    m_autoMatchedBatchFrames.clear();
    m_batchMatchedFrameList->clear();
    m_batchApplyAutoButton->setEnabled(false);
    emit autoMatchCorrectionFramesRequested(rows,
                                            m_batchBackwardSpin->value(),
                                            m_batchForwardSpin->value(),
                                            m_batchThresholdSpin->value(),
                                            m_batchAutoOverlapSpin != nullptr ? m_batchAutoOverlapSpin->value() : 0.0);
}

void AnnotationPanel::applyAutoBatchAdd()
{
    const QVector<int> rows = selectedCurrentFrameCorrectionRows();
    if (selectedBatchHasMixedTypes())
    {
        warnMixedBatchTypes();
        return;
    }
    if (rows.isEmpty())
    {
        QMessageBox::warning(this,
                             QStringLiteral("自动批量添加"),
                             QStringLiteral("请先选择当前帧已保存的纠错条目。"));
        return;
    }
    if (m_autoMatchedBatchFrames.isEmpty())
    {
        QMessageBox::warning(this,
                             QStringLiteral("自动批量添加"),
                             QStringLiteral("请先点击“自动识别匹配帧”生成匹配帧列表。"));
        return;
    }

    emit batchAddCorrectionsToFramesRequested(rows, m_autoMatchedBatchFrames);
}

void AnnotationPanel::setAutoMatchedBatchFrames(const QVector<int>& frames)
{
    m_autoMatchedBatchFrames = frames;
    std::sort(m_autoMatchedBatchFrames.begin(), m_autoMatchedBatchFrames.end());
    m_autoMatchedBatchFrames.erase(std::unique(m_autoMatchedBatchFrames.begin(), m_autoMatchedBatchFrames.end()),
                                   m_autoMatchedBatchFrames.end());

    m_batchMatchedFrameList->clear();
    if (m_autoMatchedBatchFrames.isEmpty())
    {
        m_batchMatchedFrameList->addItem(QStringLiteral("无匹配帧"));
        m_batchApplyAutoButton->setEnabled(false);
        return;
    }

    for (int frame : m_autoMatchedBatchFrames)
    {
        m_batchMatchedFrameList->addItem(QStringLiteral("帧 %1").arg(frame));
    }
    m_batchApplyAutoButton->setEnabled(true);
}

QVector<CorrectionShape> AnnotationPanel::draftCorrections() const
{
    return m_drafts;
}

bool AnnotationPanel::activeDraftShape(CorrectionShape* shape) const
{
    const int index = activeDraftIndex();
    if (shape == nullptr || index < 0)
    {
        return false;
    }

    const CorrectionShape& draft = m_drafts[index];
    if ((draft.shapeType == QStringLiteral("circle") && draft.points.size() >= 2) ||
        (draft.shapeType == QStringLiteral("rect") && draft.points.size() >= 2) ||
        (draft.shapeType == QStringLiteral("polygon") && draft.points.size() >= 3))
    {
        *shape = draft;
        return true;
    }
    return false;
}

void AnnotationPanel::applyDrawnCorrectionShape(const QString& shapeType, const QVector<QPointF>& points)
{
    if (m_drafts.isEmpty())
    {
        addDraftEntry();
    }

    const int index = activeDraftIndex();
    if (index < 0)
    {
        return;
    }

    CorrectionShape& draft = m_drafts[index];
    draft.shapeType = shapeType;
    draft.points = points;
    draft.lineColor = m_lineColor;
    draft.lineWidth = m_lineWidthSpin->value();
    draft.frame = m_currentFrame;
    m_draftsDirty = true;
    refreshDraftList();
    m_draftList->setCurrentRow(index);
}

void AnnotationPanel::applyAutoIdentifiedErrorCircles(const QVector<int>& circleIndices)
{
    if (circleIndices.isEmpty())
    {
        return;
    }
    if (m_drafts.isEmpty())
    {
        addDraftEntry();
    }

    const int index = activeDraftIndex();
    if (index < 0)
    {
        return;
    }

    CorrectionShape& draft = m_drafts[index];
    draft.errorType = QStringLiteral("false_positive");
    draft.errorTypes = QStringList{ draft.errorType };
    draft.errorCircles.clear();
    for (int circleIndex : circleIndices)
    {
        ErrorCircle circle;
        circle.circleIndex = circleIndex;
        circle.expectedIndex = -1;
        draft.errorCircles.push_back(circle);
    }
    m_draftsDirty = true;
    syncWidgetsFromActiveDraft();
    refreshDraftList();
    m_draftList->setCurrentRow(index);
}

void AnnotationPanel::addDraftEntry()
{
    syncActiveDraftFromWidgets();

    CorrectionShape draft;
    draft.name = defaultDraftName(m_nextDraftNumber);
    draft.frame = m_currentFrame;
    draft.lineColor = m_lineColor;
    draft.lineWidth = m_lineWidthSpin->value();
    m_drafts.push_back(draft);
    ++m_nextDraftNumber;
    m_draftsDirty = true;
    refreshDraftList();
    m_draftList->setCurrentRow(m_drafts.size() - 1);
    syncWidgetsFromActiveDraft();
}

void AnnotationPanel::deleteDraftEntry()
{
    const int index = activeDraftIndex();
    if (index < 0)
    {
        return;
    }
    m_drafts.removeAt(index);
    m_draftsDirty = true;
    refreshDraftList();
    if (!m_drafts.isEmpty())
    {
        m_draftList->setCurrentRow(qMin(index, m_drafts.size() - 1));
        syncWidgetsFromActiveDraft();
    }
    else
    {
        syncWidgetsFromActiveDraft();
    }
}

void AnnotationPanel::saveDraftEntries()
{
    syncActiveDraftFromWidgets();
    for (int i = 0; i < m_drafts.size(); ++i)
    {
        QString error;
        if (!validateDraft(m_drafts[i], i, &error))
        {
            QMessageBox::warning(this, QStringLiteral("保存当前帧纠错"), error);
            m_draftList->setCurrentRow(i);
            return;
        }
    }
    emit saveCurrentFrameCorrectionsRequested(m_drafts);
    m_draftsDirty = false;
}

void AnnotationPanel::addCustomType()
{
    const QString name = m_customTypeNameEdit->text().trimmed();
    if (name.isEmpty())
    {
        return;
    }

    const QString code = customTypeCode(name);
    ensureCustomTypeAvailable(code, m_customTypeDescriptionEdit->toPlainText().trimmed());
    m_typeCombo->setCurrentIndex(m_typeCombo->findData(code));
    refreshFilterTypes();
    m_customTypeNameEdit->clear();
    m_customTypeDescriptionEdit->clear();
}

void AnnotationPanel::deleteSelectedRecords()
{
    QList<QListWidgetItem*> selectedItems = m_savedList->selectedItems();
    if (selectedItems.isEmpty() && m_savedList->currentItem() != nullptr)
    {
        selectedItems.push_back(m_savedList->currentItem());
    }
    if (selectedItems.isEmpty())
    {
        return;
    }

    QVector<int> annotationRows;
    QVector<int> correctionRows;
    for (const QListWidgetItem* item : selectedItems)
    {
        const QString kind = item->data(KindRole).toString();
        const int index = item->data(IndexRole).toInt();
        if (kind == QStringLiteral("annotation"))
        {
            annotationRows.push_back(index);
        }
        else if (kind == QStringLiteral("correction"))
        {
            correctionRows.push_back(index);
        }
    }

    std::sort(annotationRows.begin(), annotationRows.end(), std::greater<int>());
    std::sort(correctionRows.begin(), correctionRows.end(), std::greater<int>());
    annotationRows.erase(std::unique(annotationRows.begin(), annotationRows.end()), annotationRows.end());
    correctionRows.erase(std::unique(correctionRows.begin(), correctionRows.end()), correctionRows.end());

    if (!annotationRows.isEmpty())
    {
        emit deleteAnnotationsRequested(annotationRows);
    }
    if (!correctionRows.isEmpty())
    {
        emit deleteCorrectionsRequested(correctionRows);
    }
}

void AnnotationPanel::syncActiveDraftFromWidgets()
{
    if (m_updating)
    {
        return;
    }

    const int index = activeDraftIndex();
    if (index < 0)
    {
        return;
    }

    CorrectionShape& draft = m_drafts[index];
    draft.name = m_nameEdit->text().trimmed();
    draft.frame = m_currentFrame;
    draft.errorType = m_typeCombo->currentData().toString();
    draft.errorTypes = draft.errorType.isEmpty() ? QStringList() : QStringList{ draft.errorType };
    draft.description = m_noteEdit->toPlainText().trimmed();
    draft.lineColor = m_lineColor;
    draft.lineWidth = m_lineWidthSpin->value();

    if (isMissedType(draft.errorType))
    {
        draft.expectedIndex = m_missingExpectedSpin->value();
        draft.errorCircles = QVector<ErrorCircle>{ ErrorCircle{ -1, m_missingExpectedSpin->value() } };
    }
    else if (typeRequiresSource(draft.errorType))
    {
        draft.errorCircles.clear();
        QList<QListWidgetItem*> selectedItems = m_errorCircleList->selectedItems();
        std::sort(selectedItems.begin(), selectedItems.end(), [](const QListWidgetItem* left, const QListWidgetItem* right) {
            return left->data(CircleIndexRole).toInt() < right->data(CircleIndexRole).toInt();
        });
        for (const QListWidgetItem* item : selectedItems)
        {
            const int circleIndex = item->data(CircleIndexRole).toInt();
            const QSpinBox* spin = m_expectedSpins.value(circleIndex, nullptr);
            const int expectedIndex = typeRequiresExpectedForSources(draft.errorType) && spin != nullptr
                ? spin->value()
                : -1;
            draft.errorCircles.push_back(ErrorCircle{ circleIndex, expectedIndex });
        }
    }
    else
    {
        draft.errorCircles.clear();
    }

    m_draftsDirty = true;
    refreshDraftList();
    m_draftList->setCurrentRow(index);
}

void AnnotationPanel::syncWidgetsFromActiveDraft()
{
    QSignalBlocker listBlocker(m_errorCircleList);
    m_updating = true;

    const int index = activeDraftIndex();
    const bool hasDraft = index >= 0;
    m_nameEdit->setEnabled(hasDraft);
    m_typeCombo->setEnabled(hasDraft);
    m_noteEdit->setEnabled(hasDraft);
    m_errorCircleList->setEnabled(hasDraft);
    m_missingExpectedSpin->setEnabled(hasDraft);

    if (!hasDraft)
    {
        m_nameEdit->clear();
        m_typeCombo->setCurrentIndex(0);
        m_noteEdit->clear();
        for (int row = 0; row < m_errorCircleList->count(); ++row)
        {
            m_errorCircleList->item(row)->setSelected(false);
        }
        m_missingExpectedSpin->setValue(-1);
        m_updating = false;
        refreshExpectedEditors();
        refreshTypeFields();
        return;
    }

    const CorrectionShape& draft = m_drafts[index];
    m_nameEdit->setText(draft.name);
    const int typeIndex = m_typeCombo->findData(draft.errorType);
    m_typeCombo->setCurrentIndex(typeIndex >= 0 ? typeIndex : 0);
    m_noteEdit->setPlainText(draft.description);
    if (draft.lineColor.isValid())
    {
        m_lineColor = draft.lineColor;
    }
    m_lineWidthSpin->setValue(qBound(1, draft.lineWidth, 15));
    updateColorButton();

    for (int row = 0; row < m_errorCircleList->count(); ++row)
    {
        QListWidgetItem* item = m_errorCircleList->item(row);
        const int circleIndex = item->data(CircleIndexRole).toInt();
        bool selected = false;
        for (const ErrorCircle& circle : draft.errorCircles)
        {
            if (circle.circleIndex == circleIndex)
            {
                selected = true;
                break;
            }
        }
        item->setSelected(selected);
    }

    int missingExpected = draft.expectedIndex;
    for (const ErrorCircle& circle : draft.errorCircles)
    {
        if (circle.circleIndex < 0)
        {
            missingExpected = circle.expectedIndex;
            break;
        }
    }
    m_missingExpectedSpin->setValue(missingExpected);

    m_updating = false;
    refreshExpectedEditors();
    for (const ErrorCircle& circle : draft.errorCircles)
    {
        QSpinBox* spin = m_expectedSpins.value(circle.circleIndex, nullptr);
        if (spin != nullptr)
        {
            spin->setValue(circle.expectedIndex);
        }
    }
    refreshTypeFields();
}

void AnnotationPanel::refreshDraftList()
{
    QSignalBlocker blocker(m_draftList);
    const int previousRow = m_draftList->currentRow();
    m_draftList->clear();
    for (int i = 0; i < m_drafts.size(); ++i)
    {
        m_draftList->addItem(draftDisplayName(m_drafts[i], i));
    }
    if (!m_drafts.isEmpty())
    {
        m_draftList->setCurrentRow(qBound(0, previousRow, m_drafts.size() - 1));
    }
}

void AnnotationPanel::refreshSourceList()
{
    const QVector<ErrorCircle> previous = activeDraftIndex() >= 0 ? m_drafts[activeDraftIndex()].errorCircles : QVector<ErrorCircle>();
    QSignalBlocker blocker(m_errorCircleList);
    m_errorCircleList->clear();
    for (int i = 0; i < m_circleCount; ++i)
    {
        auto* item = new QListWidgetItem(QStringLiteral("#%1").arg(i));
        item->setData(CircleIndexRole, i);
        m_errorCircleList->addItem(item);
        for (const ErrorCircle& circle : previous)
        {
            if (circle.circleIndex == i)
            {
                item->setSelected(true);
                break;
            }
        }
    }
    refreshExpectedEditors();
}

void AnnotationPanel::refreshExpectedEditors()
{
    QMap<int, int> previousValues;
    const int index = activeDraftIndex();
    if (index >= 0)
    {
        for (const ErrorCircle& circle : m_drafts[index].errorCircles)
        {
            if (circle.circleIndex >= 0)
            {
                previousValues.insert(circle.circleIndex, circle.expectedIndex);
            }
        }
    }
    for (auto it = m_expectedSpins.constBegin(); it != m_expectedSpins.constEnd(); ++it)
    {
        previousValues.insert(it.key(), it.value()->value());
    }

    while (QLayoutItem* item = m_expectedEditorLayout->takeAt(0))
    {
        if (item->widget() != nullptr)
        {
            delete item->widget();
        }
        delete item;
    }
    m_expectedSpins.clear();

    const QString type = m_typeCombo->currentData().toString();
    if (!typeRequiresExpectedForSources(type))
    {
        return;
    }

    QList<QListWidgetItem*> selectedItems = m_errorCircleList->selectedItems();
    std::sort(selectedItems.begin(), selectedItems.end(), [](const QListWidgetItem* left, const QListWidgetItem* right) {
        return left->data(CircleIndexRole).toInt() < right->data(CircleIndexRole).toInt();
    });

    for (const QListWidgetItem* item : selectedItems)
    {
        const int circleIndex = item->data(CircleIndexRole).toInt();
        auto* spin = new QSpinBox(m_expectedEditorWidget);
        spin->setRange(-1, BEACON_MAX_CIRCLE_COUNT - 1);
        spin->setSpecialValueText(QStringLiteral("未填"));
        spin->setValue(previousValues.value(circleIndex, -1));
        connect(spin, qOverload<int>(&QSpinBox::valueChanged), this, &AnnotationPanel::syncActiveDraftFromWidgets);
        m_expectedEditorLayout->addRow(QStringLiteral("#%1 正确序号").arg(circleIndex), spin);
        m_expectedSpins.insert(circleIndex, spin);
    }
}

void AnnotationPanel::refreshTypeFields()
{
    const QString type = m_typeCombo->currentData().toString();
    m_sourceGroup->setVisible(typeRequiresSource(type));
    m_expectedEditorWidget->setVisible(typeRequiresExpectedForSources(type));
    m_missingGroup->setVisible(isMissedType(type));
    m_noteEdit->setPlaceholderText(isOtherType(type) ? QStringLiteral("其他类型描述，必填") : QStringLiteral("描述"));
}

void AnnotationPanel::updateColorButton()
{
    m_colorButton->setText(m_lineColor.name(QColor::HexRgb).toUpper());
    m_colorButton->setStyleSheet(QStringLiteral("background:%1; color:%2;")
                                     .arg(m_lineColor.name(QColor::HexRgb),
                                          m_lineColor.lightness() > 150 ? QStringLiteral("#111315") : QStringLiteral("#eef1f4")));
}

int AnnotationPanel::activeDraftIndex() const
{
    const int row = m_draftList->currentRow();
    return row >= 0 && row < m_drafts.size() ? row : -1;
}

QString AnnotationPanel::defaultDraftName(int index) const
{
    return QStringLiteral("纠错%1").arg(index);
}

QString AnnotationPanel::draftDisplayName(const CorrectionShape& draft, int index) const
{
    if (!draft.name.trimmed().isEmpty() && !draft.name.startsWith(QStringLiteral("纠错")))
    {
        return draft.name.trimmed();
    }
    if (!draft.errorType.isEmpty())
    {
        return QStringLiteral("%1%2").arg(annotationTypeDisplayName(draft.errorType)).arg(index + 1);
    }
    return draft.name.trimmed().isEmpty() ? defaultDraftName(index + 1) : draft.name.trimmed();
}

bool AnnotationPanel::validateDraft(const CorrectionShape& draft, int index, QString* errorMessage) const
{
    const QString title = draftDisplayName(draft, index);
    if (draft.errorType.isEmpty())
    {
        *errorMessage = QStringLiteral("%1 尚未选择错误类型。").arg(title);
        return false;
    }

    if (isOtherType(draft.errorType))
    {
        if (draft.description.trimmed().isEmpty())
        {
            *errorMessage = QStringLiteral("%1 是“其他”类型，必须填写描述。").arg(title);
            return false;
        }
        return true;
    }

    if (isMissedType(draft.errorType))
    {
        if (draft.shapeType != QStringLiteral("circle") || draft.points.size() < 2)
        {
            *errorMessage = QStringLiteral("%1 是“漏检”类型，必须用纠错工具绘制漏检目标圆。").arg(title);
            return false;
        }
        const int expected = draft.errorCircles.isEmpty() ? draft.expectedIndex : draft.errorCircles.first().expectedIndex;
        if (expected < 0)
        {
            *errorMessage = QStringLiteral("%1 是“漏检”类型，必须填写正确序号。").arg(title);
            return false;
        }
        return true;
    }

    if (typeRequiresSource(draft.errorType) && draft.errorCircles.isEmpty())
    {
        *errorMessage = QStringLiteral("%1 必须选择至少一个错误源。").arg(title);
        return false;
    }

    if (typeRequiresExpectedForSources(draft.errorType))
    {
        for (const ErrorCircle& circle : draft.errorCircles)
        {
            if (circle.expectedIndex < 0)
            {
                *errorMessage = QStringLiteral("%1 的错误源 #%2 必须填写正确序号。").arg(title).arg(circle.circleIndex);
                return false;
            }
        }
    }

    return true;
}

bool AnnotationPanel::typeRequiresSource(const QString& type) const
{
    return isFalsePositiveType(type) ||
           type == QStringLiteral("wrong_order") ||
           type == QStringLiteral("target_jump") ||
           isCustomType(type);
}

bool AnnotationPanel::typeRequiresExpectedForSources(const QString& type) const
{
    return type == QStringLiteral("wrong_order") ||
           type == QStringLiteral("target_jump") ||
           isCustomType(type);
}

bool AnnotationPanel::isCustomType(const QString& type) const
{
    return type.startsWith(QStringLiteral("custom:"));
}
