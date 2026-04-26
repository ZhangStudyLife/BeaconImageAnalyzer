#include "AnnotationPanel.h"

#include "beacon_image.h"

#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QKeySequence>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QShortcut>
#include <QSizePolicy>
#include <QSpinBox>
#include <QTextEdit>
#include <QVBoxLayout>

AnnotationPanel::AnnotationPanel(QWidget* parent)
    : QWidget(parent)
{
    m_contextLabel = new QLabel(QStringLiteral("Frame: -"), this);
    m_typeCombo = new QComboBox(this);
    m_typeCombo->addItem(QStringLiteral("漏检"), QStringLiteral("missed_detection"));
    m_typeCombo->addItem(QStringLiteral("误检"), QStringLiteral("false_positive"));
    m_typeCombo->addItem(QStringLiteral("排序错误"), QStringLiteral("wrong_order"));
    m_typeCombo->addItem(QStringLiteral("阳光干扰"), QStringLiteral("sunlight_interference"));
    m_typeCombo->addItem(QStringLiteral("目标跳变"), QStringLiteral("target_jump"));
    m_typeCombo->addItem(QStringLiteral("其他"), QStringLiteral("other"));

    m_circleSpin = new QSpinBox(this);
    m_circleSpin->setRange(-1, BEACON_MAX_CIRCLE_COUNT - 1);
    m_circleSpin->setSpecialValueText(QStringLiteral("全部"));

    m_expectedSpin = new QSpinBox(this);
    m_expectedSpin->setRange(-1, BEACON_MAX_CIRCLE_COUNT - 1);
    m_expectedSpin->setSpecialValueText(QStringLiteral("无"));

    m_toolCombo = new QComboBox(this);
    m_toolCombo->addItem(QStringLiteral("选择"), QStringLiteral("select"));
    m_toolCombo->addItem(QStringLiteral("画圆"), QStringLiteral("circle"));
    m_toolCombo->addItem(QStringLiteral("画矩形"), QStringLiteral("rect"));
    m_toolCombo->addItem(QStringLiteral("画点"), QStringLiteral("point"));
    m_toolCombo->addItem(QStringLiteral("自由闭合"), QStringLiteral("polygon"));

    m_noteEdit = new QTextEdit(this);
    m_noteEdit->setPlaceholderText(QStringLiteral("备注"));
    m_noteEdit->setFixedHeight(48);

    auto* markCurrentButton = new QPushButton(QStringLiteral("标记当前帧"), this);
    auto* setStartButton = new QPushButton(QStringLiteral("片段开始=当前"), this);
    auto* setEndButton = new QPushButton(QStringLiteral("片段结束=当前"), this);
    auto* saveSegmentButton = new QPushButton(QStringLiteral("保存片段标注"), this);
    auto* deleteButton = new QPushButton(QStringLiteral("删除选中标注"), this);

    m_segmentLabel = new QLabel(QStringLiteral("片段: - 到 -"), this);
    m_listWidget = new QListWidget(this);
    m_listWidget->setMinimumHeight(140);
    m_listWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* form = new QFormLayout;
    form->setContentsMargins(0, 0, 0, 0);
    form->setVerticalSpacing(4);
    form->addRow(QStringLiteral("当前"), m_contextLabel);
    form->addRow(QStringLiteral("错误类型"), m_typeCombo);
    form->addRow(QStringLiteral("关联圆"), m_circleSpin);
    form->addRow(QStringLiteral("期望编号"), m_expectedSpin);
    form->addRow(QStringLiteral("纠错工具"), m_toolCombo);
    form->addRow(QStringLiteral("备注"), m_noteEdit);

    auto* row1 = new QHBoxLayout;
    row1->addWidget(markCurrentButton);

    auto* row2 = new QHBoxLayout;
    row2->addWidget(setStartButton);
    row2->addWidget(setEndButton);

    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addLayout(row1);
    layout->addWidget(m_segmentLabel);
    layout->addLayout(row2);
    layout->addWidget(saveSegmentButton);
    layout->addWidget(new QLabel(QStringLiteral("标注记录"), this));
    layout->addWidget(m_listWidget);
    layout->addWidget(deleteButton);
    layout->setContentsMargins(0, 0, 0, 0);

    connect(markCurrentButton, &QPushButton::clicked, this, [this]() {
        emit currentFrameAnnotationRequested(selectedType(), selectedCircleIndex(), noteText());
    });
    connect(setStartButton, &QPushButton::clicked, this, &AnnotationPanel::segmentStartRequested);
    connect(setEndButton, &QPushButton::clicked, this, &AnnotationPanel::segmentEndRequested);
    connect(saveSegmentButton, &QPushButton::clicked, this, [this]() {
        emit segmentAnnotationRequested(selectedType(), selectedCircleIndex(), noteText());
    });
    connect(deleteButton, &QPushButton::clicked, this, [this]() {
        deleteSelectedRecord();
    });

    auto* deleteShortcut = new QShortcut(QKeySequence::Delete, m_listWidget);
    deleteShortcut->setContext(Qt::WidgetShortcut);
    connect(deleteShortcut, &QShortcut::activated, this, [this]() {
        deleteSelectedRecord();
    });

    connect(m_toolCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        emit correctionToolChanged(m_toolCombo->currentData().toString());
    });
    connect(m_listWidget, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
        if (item != nullptr)
        {
            emit recordActivated(item->data(Qt::UserRole + 2).toInt());
        }
    });
}

void AnnotationPanel::setCurrentContext(int frame, double timeSec, int circleCount)
{
    m_contextLabel->setText(QStringLiteral("Frame %1 / %2 s").arg(frame).arg(timeSec, 0, 'f', 3));
    m_circleSpin->setMaximum(qMax(-1, qMin(circleCount - 1, BEACON_MAX_CIRCLE_COUNT - 1)));
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
        const QString circle = record.circleIndex >= 0
            ? QStringLiteral("#%1").arg(record.circleIndex)
            : QStringLiteral("全部");
        auto* item = new QListWidgetItem(QStringLiteral("[文字/%1] %2-%3 %4 %5")
                                             .arg(annotationTypeDisplayName(record.type))
                                             .arg(record.startFrame)
                                             .arg(record.endFrame)
                                             .arg(circle)
                                             .arg(record.description));
        item->setData(Qt::UserRole, QStringLiteral("annotation"));
        item->setData(Qt::UserRole + 1, i);
        item->setData(Qt::UserRole + 2, record.startFrame);
        m_listWidget->addItem(item);
    }

    for (int i = 0; i < corrections.size(); ++i)
    {
        const CorrectionShape& shape = corrections[i];
        const QString expected = shape.expectedIndex >= 0
            ? QStringLiteral("GT #%1").arg(shape.expectedIndex)
            : QStringLiteral("GT 未指定");
        auto* item = new QListWidgetItem(QStringLiteral("[图形/%1] frame=%2 %3 %4 %5")
                                             .arg(annotationTypeDisplayName(shape.errorType))
                                             .arg(shape.frame)
                                             .arg(shape.shapeType)
                                             .arg(expected)
                                             .arg(shape.description));
        item->setData(Qt::UserRole, QStringLiteral("correction"));
        item->setData(Qt::UserRole + 1, i);
        item->setData(Qt::UserRole + 2, shape.frame);
        m_listWidget->addItem(item);
    }
}

QString AnnotationPanel::selectedType() const
{
    return m_typeCombo->currentData().toString();
}

int AnnotationPanel::selectedCircleIndex() const
{
    return m_circleSpin->value();
}

int AnnotationPanel::selectedExpectedIndex() const
{
    return m_expectedSpin->value();
}

QString AnnotationPanel::noteText() const
{
    return m_noteEdit->toPlainText().trimmed();
}

void AnnotationPanel::deleteSelectedRecord()
{
    QListWidgetItem* item = m_listWidget->currentItem();
    if (item == nullptr)
    {
        return;
    }

    const QString kind = item->data(Qt::UserRole).toString();
    const int index = item->data(Qt::UserRole + 1).toInt();
    if (kind == QStringLiteral("annotation"))
    {
        emit deleteAnnotationRequested(index);
    }
    else if (kind == QStringLiteral("correction"))
    {
        emit deleteCorrectionRequested(index);
    }
}
