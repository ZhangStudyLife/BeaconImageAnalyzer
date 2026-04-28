#include "AnnotationPanel.h"

#include "beacon_image.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
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

QString customTypeCode(const QString& name)
{
    return QStringLiteral("custom:%1").arg(name.trimmed());
}
}

AnnotationPanel::AnnotationPanel(QWidget* parent)
    : QWidget(parent)
{
    m_contextLabel = new QLabel(QStringLiteral("Frame: -"), this);
    m_contextLabel->setObjectName(QStringLiteral("SoftLabel"));

    m_falsePositiveCheck = new QCheckBox(QStringLiteral("误检"), this);
    m_missedDetectionCheck = new QCheckBox(QStringLiteral("漏检"), this);
    m_wrongOrderCheck = new QCheckBox(QStringLiteral("排序错误"), this);
    m_targetJumpCheck = new QCheckBox(QStringLiteral("目标跳变"), this);
    m_otherCheck = new QCheckBox(QStringLiteral("其他"), this);

    m_customTypeEdit = new QLineEdit(this);
    m_customTypeEdit->setPlaceholderText(QStringLiteral("自定义错误类型"));
    auto* addTypeButton = new QPushButton(QStringLiteral("新增类型"), this);

    m_customTypeLayout = new QVBoxLayout;
    m_customTypeLayout->setContentsMargins(0, 0, 0, 0);
    m_customTypeLayout->setSpacing(4);

    auto* typeGroup = new QGroupBox(QStringLiteral("错误类型"), this);
    auto* typeLayout = new QVBoxLayout(typeGroup);
    typeLayout->setContentsMargins(10, 8, 10, 10);
    typeLayout->setSpacing(6);
    typeLayout->addWidget(m_falsePositiveCheck);
    typeLayout->addWidget(m_missedDetectionCheck);
    typeLayout->addWidget(m_wrongOrderCheck);
    typeLayout->addWidget(m_targetJumpCheck);
    typeLayout->addWidget(m_otherCheck);
    auto* customTypeRow = new QHBoxLayout;
    customTypeRow->setContentsMargins(0, 0, 0, 0);
    customTypeRow->setSpacing(6);
    customTypeRow->addWidget(m_customTypeEdit, 1);
    customTypeRow->addWidget(addTypeButton);
    typeLayout->addLayout(customTypeRow);
    typeLayout->addLayout(m_customTypeLayout);

    m_errorCircleList = new QListWidget(this);
    m_errorCircleList->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_errorCircleList->setMinimumHeight(72);
    m_errorCircleList->setMaximumHeight(110);

    m_expectedEditorWidget = new QWidget(this);
    m_expectedEditorLayout = new QFormLayout(m_expectedEditorWidget);
    m_expectedEditorLayout->setContentsMargins(0, 0, 0, 0);
    m_expectedEditorLayout->setVerticalSpacing(6);
    m_expectedEditorLayout->setHorizontalSpacing(8);

    auto* sourceGroup = new QGroupBox(QStringLiteral("错误源信息"), this);
    auto* sourceLayout = new QVBoxLayout(sourceGroup);
    sourceLayout->setContentsMargins(10, 8, 10, 10);
    sourceLayout->setSpacing(7);
    auto* sourceHint = new QLabel(QStringLiteral("错误圆"), sourceGroup);
    sourceHint->setObjectName(QStringLiteral("SoftLabel"));
    sourceLayout->addWidget(sourceHint);
    sourceLayout->addWidget(m_errorCircleList);
    sourceLayout->addWidget(m_expectedEditorWidget);

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

    auto* toolGroup = new QGroupBox(QStringLiteral("纠错工具"), this);
    auto* toolForm = new QFormLayout(toolGroup);
    toolForm->setContentsMargins(10, 8, 10, 10);
    toolForm->setVerticalSpacing(7);
    toolForm->setHorizontalSpacing(8);
    toolForm->addRow(QStringLiteral("绘制"), m_toolCombo);
    toolForm->addRow(QStringLiteral("线条颜色"), m_colorButton);
    toolForm->addRow(QStringLiteral("线条粗细"), m_lineWidthSpin);
    toolForm->addRow(QString(), autoIdentifyButton);

    m_noteEdit = new QTextEdit(this);
    m_noteEdit->setPlaceholderText(QStringLiteral("备注"));
    m_noteEdit->setFixedHeight(64);

    auto* markCurrentButton = new QPushButton(QStringLiteral("标记当前帧"), this);
    markCurrentButton->setProperty("role", QStringLiteral("primary"));
    auto* setStartButton = new QPushButton(QStringLiteral("片段开始=当前"), this);
    auto* setEndButton = new QPushButton(QStringLiteral("片段结束=当前"), this);
    auto* saveSegmentButton = new QPushButton(QStringLiteral("保存片段标注"), this);
    auto* deleteButton = new QPushButton(QStringLiteral("删除选中标注"), this);

    m_segmentLabel = new QLabel(QStringLiteral("片段: - 到 -"), this);
    m_segmentLabel->setObjectName(QStringLiteral("SoftLabel"));
    m_listWidget = new QListWidget(this);
    m_listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_listWidget->setMinimumHeight(180);
    m_listWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* contextForm = new QFormLayout;
    contextForm->setContentsMargins(0, 0, 0, 0);
    contextForm->setVerticalSpacing(8);
    contextForm->setHorizontalSpacing(10);
    contextForm->addRow(QStringLiteral("当前"), m_contextLabel);
    contextForm->addRow(QStringLiteral("备注"), m_noteEdit);

    auto* row1 = new QHBoxLayout;
    row1->setContentsMargins(0, 0, 0, 0);
    row1->addWidget(markCurrentButton);

    auto* row2 = new QHBoxLayout;
    row2->setContentsMargins(0, 0, 0, 0);
    row2->setSpacing(8);
    row2->addWidget(setStartButton);
    row2->addWidget(setEndButton);

    auto* layout = new QVBoxLayout(this);
    layout->setSpacing(9);
    layout->addLayout(contextForm);
    layout->addWidget(typeGroup);
    layout->addWidget(sourceGroup);
    layout->addWidget(toolGroup);
    layout->addLayout(row1);
    layout->addWidget(m_segmentLabel);
    layout->addLayout(row2);
    layout->addWidget(saveSegmentButton);
    layout->addWidget(new QLabel(QStringLiteral("标注记录"), this));
    layout->addWidget(m_listWidget);
    layout->addWidget(deleteButton);
    layout->setContentsMargins(0, 0, 0, 0);

    connect(markCurrentButton, &QPushButton::clicked, this, [this]() {
        emit currentFrameAnnotationRequested(selectedTypes(), selectedErrorCircles(), noteText());
    });
    connect(setStartButton, &QPushButton::clicked, this, &AnnotationPanel::segmentStartRequested);
    connect(setEndButton, &QPushButton::clicked, this, &AnnotationPanel::segmentEndRequested);
    connect(saveSegmentButton, &QPushButton::clicked, this, [this]() {
        emit segmentAnnotationRequested(selectedTypes(), selectedErrorCircles(), noteText());
    });
    connect(deleteButton, &QPushButton::clicked, this, [this]() {
        deleteSelectedRecords();
    });

    auto* deleteShortcut = new QShortcut(QKeySequence::Delete, m_listWidget);
    deleteShortcut->setContext(Qt::WidgetShortcut);
    connect(deleteShortcut, &QShortcut::activated, this, [this]() {
        deleteSelectedRecords();
    });

    connect(addTypeButton, &QPushButton::clicked, this, &AnnotationPanel::addCustomType);
    connect(m_customTypeEdit, &QLineEdit::returnPressed, this, &AnnotationPanel::addCustomType);
    connect(m_errorCircleList, &QListWidget::itemSelectionChanged,
            this, &AnnotationPanel::updateExpectedEditors);
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
    });
    connect(m_lineWidthSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
        emit correctionStyleChanged(m_lineColor, value);
    });
    connect(autoIdentifyButton, &QPushButton::clicked, this, &AnnotationPanel::autoIdentifyRequested);
    connect(m_listWidget, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
        if (item != nullptr)
        {
            emit recordActivated(item->data(FrameRole).toInt());
        }
    });
}

void AnnotationPanel::setCurrentContext(int frame, double timeSec, int circleCount)
{
    m_contextLabel->setText(QStringLiteral("Frame %1 / %2 s").arg(frame).arg(timeSec, 0, 'f', 3));

    const QVector<ErrorCircle> previous = selectedErrorCircles();
    m_errorCircleList->clear();
    for (int i = 0; i < circleCount && i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        auto* item = new QListWidgetItem(QStringLiteral("#%1").arg(i));
        item->setData(CircleIndexRole, i);
        m_errorCircleList->addItem(item);
    }

    QVector<int> previousIndices;
    for (const ErrorCircle& circle : previous)
    {
        previousIndices.push_back(circle.circleIndex);
    }
    setSelectedErrorCircleIndices(previousIndices);
    for (const ErrorCircle& circle : previous)
    {
        QSpinBox* spin = m_expectedSpins.value(circle.circleIndex, nullptr);
        if (spin != nullptr && circle.expectedIndex >= 0)
        {
            spin->setValue(circle.expectedIndex);
        }
    }
}

void AnnotationPanel::setSegmentStart(int frame)
{
    const QString text = m_segmentLabel->text();
    const QString endText = text.contains(QStringLiteral("到")) ? text.section(QStringLiteral("到"), 1).trimmed() : QStringLiteral("-");
    m_segmentLabel->setText(QStringLiteral("片段: %1 到 %2").arg(frame).arg(endText));
}

void AnnotationPanel::setSegmentEnd(int frame)
{
    const QString text = m_segmentLabel->text();
    const QString startText = text.contains(QStringLiteral(":")) ? text.section(QStringLiteral(":"), 1).section(QStringLiteral("到"), 0, 0).trimmed() : QStringLiteral("-");
    m_segmentLabel->setText(QStringLiteral("片段: %1 到 %2").arg(startText).arg(frame));
}

void AnnotationPanel::setAnnotations(const QVector<AnnotationRecord>& records, const QVector<CorrectionShape>& corrections)
{
    m_listWidget->clear();
    for (int i = 0; i < records.size(); ++i)
    {
        const AnnotationRecord& record = records[i];
        const QStringList types = record.types.isEmpty() ? QStringList{ record.type } : record.types;
        const QVector<ErrorCircle> circles = record.errorCircles.isEmpty() && record.circleIndex >= 0
            ? QVector<ErrorCircle>{ ErrorCircle{ record.circleIndex, -1 } }
            : record.errorCircles;
        auto* item = new QListWidgetItem(QStringLiteral("[文字/%1] %2-%3 %4 %5")
                                             .arg(annotationTypesDisplayName(types))
                                             .arg(record.startFrame)
                                             .arg(record.endFrame)
                                             .arg(errorCirclesDisplayName(circles))
                                             .arg(record.description));
        item->setData(KindRole, QStringLiteral("annotation"));
        item->setData(IndexRole, i);
        item->setData(FrameRole, record.startFrame);
        m_listWidget->addItem(item);
    }

    for (int i = 0; i < corrections.size(); ++i)
    {
        const CorrectionShape& shape = corrections[i];
        const QStringList types = shape.errorTypes.isEmpty() ? QStringList{ shape.errorType } : shape.errorTypes;
        auto* item = new QListWidgetItem(QStringLiteral("[图形/%1] frame=%2 %3 %4 %5")
                                             .arg(annotationTypesDisplayName(types))
                                             .arg(shape.frame)
                                             .arg(shape.shapeType)
                                             .arg(errorCirclesDisplayName(shape.errorCircles))
                                             .arg(shape.description));
        item->setData(KindRole, QStringLiteral("correction"));
        item->setData(IndexRole, i);
        item->setData(FrameRole, shape.frame);
        m_listWidget->addItem(item);
    }
}

void AnnotationPanel::setSelectedErrorCircleIndices(const QVector<int>& circleIndices)
{
    QSignalBlocker blocker(m_errorCircleList);
    for (int row = 0; row < m_errorCircleList->count(); ++row)
    {
        QListWidgetItem* item = m_errorCircleList->item(row);
        const int index = item->data(CircleIndexRole).toInt();
        item->setSelected(circleIndices.contains(index));
    }
    updateExpectedEditors();
}

QStringList AnnotationPanel::selectedTypes() const
{
    QStringList types;
    if (m_falsePositiveCheck->isChecked())
    {
        types.push_back(QStringLiteral("false_positive"));
    }
    if (m_missedDetectionCheck->isChecked())
    {
        types.push_back(QStringLiteral("missed_detection"));
    }
    if (m_wrongOrderCheck->isChecked())
    {
        types.push_back(QStringLiteral("wrong_order"));
    }
    if (m_targetJumpCheck->isChecked())
    {
        types.push_back(QStringLiteral("target_jump"));
    }
    if (m_otherCheck->isChecked())
    {
        const QString custom = m_customTypeEdit->text().trimmed();
        types.push_back(custom.isEmpty() ? QStringLiteral("other") : customTypeCode(custom));
    }
    for (const QCheckBox* check : m_customTypeChecks)
    {
        if (check->isChecked())
        {
            const QString type = check->property("type_code").toString();
            if (!type.isEmpty())
            {
                types.push_back(type);
            }
        }
    }
    types.removeDuplicates();
    return types;
}

QVector<ErrorCircle> AnnotationPanel::selectedErrorCircles() const
{
    QList<QListWidgetItem*> selectedItems = m_errorCircleList->selectedItems();
    std::sort(selectedItems.begin(), selectedItems.end(), [](const QListWidgetItem* left, const QListWidgetItem* right) {
        return left->data(CircleIndexRole).toInt() < right->data(CircleIndexRole).toInt();
    });

    QVector<ErrorCircle> circles;
    for (const QListWidgetItem* item : selectedItems)
    {
        ErrorCircle circle;
        circle.circleIndex = item->data(CircleIndexRole).toInt();
        const QSpinBox* expectedSpin = m_expectedSpins.value(circle.circleIndex, nullptr);
        circle.expectedIndex = expectedSpin != nullptr ? expectedSpin->value() : circle.circleIndex;
        if (circle.circleIndex >= 0)
        {
            circles.push_back(circle);
        }
    }
    return circles;
}

QColor AnnotationPanel::selectedCorrectionColor() const
{
    return m_lineColor;
}

int AnnotationPanel::selectedCorrectionLineWidth() const
{
    return m_lineWidthSpin->value();
}

QString AnnotationPanel::noteText() const
{
    return m_noteEdit->toPlainText().trimmed();
}

void AnnotationPanel::addCustomType()
{
    const QString name = m_customTypeEdit->text().trimmed();
    if (name.isEmpty())
    {
        return;
    }

    const QString code = customTypeCode(name);
    for (QCheckBox* check : m_customTypeChecks)
    {
        if (check->property("type_code").toString() == code)
        {
            check->setChecked(true);
            m_customTypeEdit->clear();
            return;
        }
    }

    auto* check = new QCheckBox(name, this);
    check->setProperty("type_code", code);
    check->setChecked(true);
    m_customTypeChecks.push_back(check);
    m_customTypeLayout->addWidget(check);
    m_customTypeEdit->clear();
}

void AnnotationPanel::deleteSelectedRecords()
{
    QList<QListWidgetItem*> selectedItems = m_listWidget->selectedItems();
    if (selectedItems.isEmpty() && m_listWidget->currentItem() != nullptr)
    {
        selectedItems.push_back(m_listWidget->currentItem());
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

void AnnotationPanel::updateColorButton()
{
    m_colorButton->setText(m_lineColor.name(QColor::HexRgb).toUpper());
    m_colorButton->setStyleSheet(QStringLiteral("background:%1; color:%2;")
                                     .arg(m_lineColor.name(QColor::HexRgb),
                                          m_lineColor.lightness() > 150 ? QStringLiteral("#111315") : QStringLiteral("#eef1f4")));
}

void AnnotationPanel::updateExpectedEditors()
{
    QMap<int, int> previousValues;
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

    QList<QListWidgetItem*> selectedItems = m_errorCircleList->selectedItems();
    std::sort(selectedItems.begin(), selectedItems.end(), [](const QListWidgetItem* left, const QListWidgetItem* right) {
        return left->data(CircleIndexRole).toInt() < right->data(CircleIndexRole).toInt();
    });

    for (const QListWidgetItem* item : selectedItems)
    {
        const int circleIndex = item->data(CircleIndexRole).toInt();
        if (circleIndex < 0)
        {
            continue;
        }

        auto* spin = new QSpinBox(m_expectedEditorWidget);
        spin->setRange(0, BEACON_MAX_CIRCLE_COUNT - 1);
        spin->setValue(previousValues.value(circleIndex, circleIndex));
        m_expectedEditorLayout->addRow(QStringLiteral("#%1 期望编号").arg(circleIndex), spin);
        m_expectedSpins.insert(circleIndex, spin);
    }
}
