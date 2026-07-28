#include "HorizonCalibrationWindow.h"

#include "HorizonLineGeometry.h"
#include "VideoWidget.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace
{
QString validityText(const HorizonFrameMetadata& frame, bool heightRecorded)
{
    if (!frame.attitudeValid)
    {
        return QStringLiteral("姿态无效");
    }
    if (!heightRecorded)
    {
        return QStringLiteral("有效");
    }
    return frame.heightValid ? QStringLiteral("姿态/高度有效") : QStringLiteral("高度无效");
}

QString warningText(const HorizonFitResult& fit)
{
    QStringList warnings;
    if (fit.rollMax - fit.rollMin < 30.0)
    {
        warnings.push_back(QStringLiteral("Roll 跨度不足 30°"));
    }
    if (fit.pitchMax - fit.pitchMin < 20.0)
    {
        warnings.push_back(QStringLiteral("Pitch 跨度不足 20°"));
    }
    if (fit.rmse > 1.5)
    {
        warnings.push_back(QStringLiteral("RMSE 超过 1.5 px"));
    }
    if (!fit.exportable)
    {
        warnings.push_back(QStringLiteral("内点不足 12 个，暂不可导出"));
    }
    return warnings.join(QStringLiteral("；"));
}

QPainterPath curvePath(const QVector<QPointF>& points)
{
    QPainterPath path;
    if (points.isEmpty())
    {
        return path;
    }
    path.moveTo(points.first());
    if (points.size() == 2)
    {
        path.lineTo(points.last());
        return path;
    }
    for (int index = 0; index + 1 < points.size(); ++index)
    {
        const QPointF previous = index > 0 ? points[index - 1] : points[index];
        const QPointF current = points[index];
        const QPointF next = points[index + 1];
        const QPointF following = index + 2 < points.size() ? points[index + 2] : next;
        path.cubicTo(current + (next - previous) / 6.0,
                     next - (following - current) / 6.0,
                     next);
    }
    return path;
}
}

HorizonCalibrationWindow::HorizonCalibrationWindow(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    setWindowTitle(QStringLiteral("地平线标定"));
    resize(1080, 760);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    auto* fileRow = new QHBoxLayout;
    auto* openButton = new QPushButton(QStringLiteral("导入 HCAL"), this);
    m_importModelButton = new QPushButton(QStringLiteral("导入模型"), this);
    m_importModelButton->setEnabled(false);
    m_fitButton = new QPushButton(QStringLiteral("拟合模型"), this);
    m_exportButton = new QPushButton(QStringLiteral("导出模型"), this);
    m_exportButton->setEnabled(false);
    m_sessionLabel = new QLabel(QStringLiteral("尚未导入 .hcal.json"), this);
    m_sessionLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_sessionLabel->setWordWrap(true);
    fileRow->addWidget(openButton);
    fileRow->addWidget(m_importModelButton);
    fileRow->addWidget(m_fitButton);
    fileRow->addWidget(m_exportButton);
    fileRow->addWidget(m_sessionLabel, 1);
    root->addLayout(fileRow);

    m_videoWidget = new VideoWidget(this);
    m_videoWidget->setText(QStringLiteral("导入 HCAL 会话后逐帧标注地平线"));
    m_videoWidget->setCorrectionTool(QStringLiteral("polyline"));
    m_videoWidget->setCorrectionStyle(QColor(255, 214, 64), 2);
    m_videoWidget->setToolTip(QStringLiteral("沿栏杆从左到右点击 7～11 个点，右键或双击结束；Backspace 撤销，Esc 取消"));
    root->addWidget(m_videoWidget, 1);

    m_frameLabel = new QLabel(QStringLiteral("帧 -- | Roll -- | Pitch --"), this);
    m_frameLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    root->addWidget(m_frameLabel);

    auto* navigation = new QHBoxLayout;
    auto* previousButton = new QPushButton(QStringLiteral("上一帧"), this);
    auto* nextButton = new QPushButton(QStringLiteral("下一帧"), this);
    auto* previousPendingButton = new QPushButton(QStringLiteral("上一未标注"), this);
    auto* nextPendingButton = new QPushButton(QStringLiteral("下一未标注"), this);
    auto* previousOutlierButton = new QPushButton(QStringLiteral("上一异常"), this);
    auto* nextOutlierButton = new QPushButton(QStringLiteral("下一异常"), this);
    auto* clearButton = new QPushButton(QStringLiteral("清除本帧"), this);
    auto* skipButton = new QPushButton(QStringLiteral("地平线不可见"), this);
    auto* undoPointButton = new QPushButton(QStringLiteral("撤销点"), this);
    auto* finishCurveButton = new QPushButton(QStringLiteral("完成曲线"), this);
    navigation->addWidget(previousButton);
    navigation->addWidget(nextButton);
    navigation->addWidget(previousPendingButton);
    navigation->addWidget(nextPendingButton);
    navigation->addWidget(previousOutlierButton);
    navigation->addWidget(nextOutlierButton);
    navigation->addStretch(1);
    navigation->addWidget(clearButton);
    navigation->addWidget(skipButton);
    navigation->addWidget(undoPointButton);
    navigation->addWidget(finishCurveButton);
    root->addLayout(navigation);

    auto* seekRow = new QHBoxLayout;
    m_frameSlider = new QSlider(Qt::Horizontal, this);
    m_frameSlider->setRange(0, 0);
    m_frameSpin = new QSpinBox(this);
    m_frameSpin->setRange(0, 0);
    seekRow->addWidget(m_frameSlider, 1);
    seekRow->addWidget(m_frameSpin);
    root->addLayout(seekRow);

    m_fitLabel = new QLabel(QStringLiteral("尚未拟合。"), this);
    m_fitLabel->setWordWrap(true);
    m_fitLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    root->addWidget(m_fitLabel);

    connect(openButton, &QPushButton::clicked, this, &HorizonCalibrationWindow::chooseSession);
    connect(m_importModelButton,
            &QPushButton::clicked,
            this,
            &HorizonCalibrationWindow::chooseModel);
    connect(m_fitButton, &QPushButton::clicked, this, &HorizonCalibrationWindow::fitModel);
    connect(m_exportButton, &QPushButton::clicked, this, &HorizonCalibrationWindow::exportModel);
    connect(previousButton, &QPushButton::clicked, this, [this]() { moveFrame(-1); });
    connect(nextButton, &QPushButton::clicked, this, [this]() { moveFrame(1); });
    connect(previousPendingButton, &QPushButton::clicked, this, [this]() { moveToMatch(-1, true); });
    connect(nextPendingButton, &QPushButton::clicked, this, [this]() { moveToMatch(1, true); });
    connect(previousOutlierButton, &QPushButton::clicked, this, [this]() { moveToMatch(-1, false); });
    connect(nextOutlierButton, &QPushButton::clicked, this, [this]() { moveToMatch(1, false); });
    connect(clearButton, &QPushButton::clicked, this, &HorizonCalibrationWindow::clearCurrentFrame);
    connect(skipButton, &QPushButton::clicked, this, &HorizonCalibrationWindow::skipCurrentFrame);
    connect(undoPointButton, &QPushButton::clicked, m_videoWidget, &VideoWidget::undoCorrectionPoint);
    connect(finishCurveButton, &QPushButton::clicked, m_videoWidget, &VideoWidget::finishCorrection);
    connect(m_frameSlider, &QSlider::valueChanged, this, &HorizonCalibrationWindow::showFrame);
    connect(m_frameSpin, qOverload<int>(&QSpinBox::valueChanged), this, &HorizonCalibrationWindow::showFrame);
    connect(m_videoWidget,
            &VideoWidget::correctionShapeFinished,
            this,
            &HorizonCalibrationWindow::saveCurve);

    auto* previousShortcut = new QShortcut(QKeySequence(Qt::Key_Left), this);
    auto* nextShortcut = new QShortcut(QKeySequence(Qt::Key_Right), this);
    connect(previousShortcut, &QShortcut::activated, this, [this]() { moveFrame(-1); });
    connect(nextShortcut, &QShortcut::activated, this, [this]() { moveFrame(1); });
    auto* undoShortcut = new QShortcut(QKeySequence(Qt::Key_Backspace), this);
    auto* cancelShortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);
    connect(undoShortcut, &QShortcut::activated, m_videoWidget, &VideoWidget::undoCorrectionPoint);
    connect(cancelShortcut, &QShortcut::activated, m_videoWidget, &VideoWidget::cancelCorrection);
}

bool HorizonCalibrationWindow::openSession(const QString& path)
{
    HorizonCalibrationSession session;
    QString error;
    if (!HorizonCalibration::loadSession(path, &session, &error))
    {
        QMessageBox::critical(this, QStringLiteral("导入失败"), error);
        return false;
    }
    if (!m_reader.open(session.videoPath, &error))
    {
        QMessageBox::critical(this, QStringLiteral("打开录像失败"), error);
        return false;
    }
    if (m_reader.width() != session.imageSize.width()
        || m_reader.height() != session.imageSize.height())
    {
        QMessageBox::critical(this,
                              QStringLiteral("会话不一致"),
                              QStringLiteral("AVI 尺寸 %1x%2 与 HCAL 尺寸 %3x%4 不一致。")
                                  .arg(m_reader.width())
                                  .arg(m_reader.height())
                                  .arg(session.imageSize.width())
                                  .arg(session.imageSize.height()));
        return false;
    }

    m_session = session;
    m_importedModel = HorizonFisheyeModel();
    m_availableFrames = qMin(m_reader.frameCount(), m_session.frames.size());
    if (m_availableFrames <= 0)
    {
        QMessageBox::critical(this, QStringLiteral("会话为空"), QStringLiteral("AVI 或 HCAL CSV 中没有可用帧。"));
        return false;
    }
    if (m_reader.frameCount() != m_session.frames.size()
        || (m_session.frameCount > 0 && m_session.frameCount != m_session.frames.size()))
    {
        QMessageBox::warning(this,
                             QStringLiteral("帧数不一致"),
                             QStringLiteral("AVI=%1，CSV=%2，JSON=%3；仅使用前 %4 帧。")
                                 .arg(m_reader.frameCount())
                                 .arg(m_session.frames.size())
                                 .arg(m_session.frameCount)
                                 .arg(m_availableFrames));
    }

    m_sessionLabel->setText(QStringLiteral("%1 | %2 | %3x%4 | CSV %5 帧 | 丢帧 %6")
                                .arg(QFileInfo(path).fileName())
                                .arg(HorizonCalibration::cameraName(m_session.cameraId))
                                .arg(m_session.imageSize.width())
                                .arg(m_session.imageSize.height())
                                .arg(m_session.frames.size())
                                .arg(m_session.sourceDroppedFrames + m_session.queueDroppedFrames));
    m_frameSlider->setRange(0, m_availableFrames - 1);
    m_frameSpin->setRange(0, m_availableFrames - 1);
    m_importModelButton->setEnabled(true);
    m_fitButton->setEnabled(!m_session.heightRecorded);
    updateFitText();
    showFrame(0);
    return true;
}

void HorizonCalibrationWindow::chooseSession()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("导入地平线标定会话"),
                                                      QString(),
                                                      QStringLiteral("Horizon Calibration (*.hcal.json)"));
    if (!path.isEmpty())
    {
        openSession(path);
    }
}

void HorizonCalibrationWindow::chooseModel()
{
    if (m_availableFrames <= 0)
    {
        QMessageBox::information(this, QStringLiteral("导入模型"), QStringLiteral("请先导入 HCAL 会话。"));
        return;
    }
    const QString path = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("导入地平线模型"),
        QFileInfo(m_session.sessionPath).absolutePath(),
        QStringLiteral("Horizon Model (*.json)"));
    if (!path.isEmpty())
    {
        openModel(path);
    }
}

bool HorizonCalibrationWindow::openModel(const QString& path)
{
    HorizonFisheyeModel model;
    QString error;
    if (!HorizonCalibration::loadModel(path, &model, &error))
    {
        QMessageBox::critical(this, QStringLiteral("导入模型失败"), error);
        return false;
    }
    if (model.imageSize != m_session.imageSize)
    {
        QMessageBox::critical(this,
                              QStringLiteral("模型不匹配"),
                              QStringLiteral("模型尺寸 %1x%2 与 HCAL 尺寸 %3x%4 不一致。")
                                  .arg(model.imageSize.width())
                                  .arg(model.imageSize.height())
                                  .arg(m_session.imageSize.width())
                                  .arg(m_session.imageSize.height()));
        return false;
    }
    if (model.cameraId != m_session.cameraId)
    {
        QMessageBox::critical(this,
                              QStringLiteral("模型不匹配"),
                              QStringLiteral("模型相机为 %1，HCAL 相机为 %2。")
                                  .arg(HorizonCalibration::cameraName(model.cameraId),
                                       HorizonCalibration::cameraName(m_session.cameraId)));
        return false;
    }
    if (model.heightCompensated && !m_session.heightRecorded)
    {
        QMessageBox::critical(this,
                              QStringLiteral("模型不匹配"),
                              QStringLiteral("该模型需要逐帧高度，但当前 HCAL 会话没有高度数据。"));
        return false;
    }

    m_importedModel = model;
    updateFitText();
    if (m_currentFrame >= 0)
    {
        showFrame(m_currentFrame);
    }
    return true;
}

void HorizonCalibrationWindow::showFrame(int frameIndex)
{
    if (frameIndex < 0 || frameIndex >= m_availableFrames)
    {
        return;
    }
    m_videoWidget->cancelCorrection();
    QImage image;
    QString error;
    if (!m_reader.readFrame(frameIndex, &image, &error))
    {
        QMessageBox::critical(this, QStringLiteral("读取帧失败"), error);
        return;
    }

    m_currentFrame = frameIndex;
    const QSignalBlocker sliderBlocker(m_frameSlider);
    const QSignalBlocker spinBlocker(m_frameSpin);
    m_frameSlider->setValue(frameIndex);
    m_frameSpin->setValue(frameIndex);

    const HorizonFrameMetadata& frame = m_session.frames[frameIndex];
    QImage display = image.convertToFormat(QImage::Format_RGB32);
    QPainter painter(&display);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const bool importedModelReady = m_importedModel.valid
                                    && (!m_importedModel.heightCompensated
                                        || (m_session.heightRecorded && frame.heightValid));
    if (frame.attitudeValid && (importedModelReady || m_session.fit.fitted))
    {
        const QVector<QPointF> predicted = m_importedModel.valid
            ? HorizonCalibration::predictCurve(m_importedModel,
                                                frame.rollDeg,
                                                frame.pitchDeg,
                                                frame.heightMm)
            : HorizonCalibration::predictCurve(m_session.fit.coefficients,
                                                m_session.imageSize,
                                                frame.rollDeg,
                                                frame.pitchDeg);
        if (predicted.size() >= 2)
        {
            painter.setPen(QPen(QColor(0, 220, 255), 1));
            painter.drawPolyline(QPolygonF(predicted));
        }
    }
    const HorizonFrameAnnotation annotation = m_session.annotations.value(frameIndex);
    if (annotation.points.size() >= 2 && !annotation.skipped)
    {
        painter.setPen(QPen(QColor(255, 214, 64), 2,
                            annotation.legacyLine ? Qt::DashLine : Qt::SolidLine));
        if (annotation.legacyLine)
        {
            QLineF manual;
            if (HorizonLineGeometry::clipThroughPoints(annotation.points[0],
                                                       annotation.points[1],
                                                       m_session.imageSize,
                                                       &manual))
            {
                painter.drawLine(manual);
            }
        }
        else
        {
            painter.drawPath(curvePath(annotation.points));
        }
        painter.setBrush(QColor(255, 214, 64));
        for (const QPointF& point : annotation.points)
        {
            painter.drawEllipse(point, 2.0, 2.0);
        }
    }
    painter.end();

    const QString annotationState = annotation.skipped
        ? QStringLiteral("已跳过")
        : annotation.legacyLine ? QStringLiteral("旧直线，需重标")
        : annotation.points.size() >= 5
            ? QStringLiteral("已标注 %1 点").arg(annotation.points.size())
            : QStringLiteral("未标注");
    const QString errorText = !m_importedModel.valid && m_session.fit.frameErrors.contains(frameIndex)
        ? QStringLiteral(" | 预测误差 %1 px").arg(m_session.fit.frameErrors.value(frameIndex), 0, 'f', 2)
        : QString();
    const double rollMin = m_importedModel.valid ? m_importedModel.rollMin : m_session.fit.rollMin;
    const double rollMax = m_importedModel.valid ? m_importedModel.rollMax : m_session.fit.rollMax;
    const double pitchMin = m_importedModel.valid ? m_importedModel.pitchMin : m_session.fit.pitchMin;
    const double pitchMax = m_importedModel.valid ? m_importedModel.pitchMax : m_session.fit.pitchMax;
    const QString extrapolationText = (m_importedModel.valid || m_session.fit.fitted)
        && frame.attitudeValid
        && (frame.rollDeg < rollMin || frame.rollDeg > rollMax
            || frame.pitchDeg < pitchMin || frame.pitchDeg > pitchMax
            || (m_importedModel.heightCompensated
                && (!frame.heightValid || frame.heightMm < m_importedModel.heightMinMm
                    || frame.heightMm > m_importedModel.heightMaxMm)))
        ? QStringLiteral(" | 外推姿态/高度")
        : QString();
    const QString modelText = m_importedModel.valid
        ? (m_importedModel.heightCompensated ? QStringLiteral(" | 外部高度鱼眼模型")
                                             : QStringLiteral(" | 外部鱼眼模型"))
        : QString();
    const QString heightText = m_session.heightRecorded && std::isfinite(frame.heightMm)
        ? QString::number(frame.heightMm, 'f', 1)
        : QStringLiteral("--");
    m_frameLabel->setText(QStringLiteral("帧 %1/%2 | BIMG #%3 | %4 | Roll %5° | Pitch %6° | Height %7 mm | %8 | %9%10")
                              .arg(frameIndex)
                              .arg(m_availableFrames - 1)
                              .arg(frame.bimgSequence)
                              .arg(HorizonCalibration::cameraName(frame.cameraId))
                              .arg(frame.rollDeg, 0, 'f', 3)
                              .arg(frame.pitchDeg, 0, 'f', 3)
                              .arg(heightText)
                              .arg(validityText(frame, m_session.heightRecorded))
                              .arg(annotationState)
                              .arg(errorText + extrapolationText + modelText));
    m_videoWidget->setFrameGeometry(m_session.imageSize, 1);
    m_videoWidget->setPixelSourceImage(image);
    m_videoWidget->setImage(display);
}

void HorizonCalibrationWindow::saveCurve(const QString& shapeType, const QVector<QPointF>& points)
{
    if (shapeType != QStringLiteral("polyline")
        || m_currentFrame < 0 || m_currentFrame >= m_session.frames.size())
    {
        return;
    }
    if (!m_session.frames[m_currentFrame].attitudeValid)
    {
        QMessageBox::information(this,
                                 QStringLiteral("姿态无效"),
                                 QStringLiteral("该帧没有有效的同帧 Roll/Pitch，不能加入拟合样本。"));
        return;
    }
    if (m_session.heightRecorded && !m_session.frames[m_currentFrame].heightValid)
    {
        QMessageBox::information(this,
                                 QStringLiteral("高度无效"),
                                 QStringLiteral("该帧没有有效的同帧融合高度，不能加入高度标定样本。"));
        return;
    }
    if (points.size() < 5)
    {
        QMessageBox::information(this,
                                 QStringLiteral("标注点不足"),
                                 QStringLiteral("每帧请沿栏杆至少标注 5 个点，建议 7～11 个点。"));
        return;
    }

    HorizonFrameAnnotation annotation;
    annotation.points = points;
    std::sort(annotation.points.begin(), annotation.points.end(), [](const QPointF& a, const QPointF& b) {
        return a.x() < b.x();
    });
    m_session.annotations.insert(m_currentFrame, annotation);
    m_session.fit = HorizonFitResult();
    saveSession();
    updateFitText();
    showFrame(m_currentFrame);
}

void HorizonCalibrationWindow::clearCurrentFrame()
{
    if (m_currentFrame < 0)
    {
        return;
    }
    m_session.annotations.remove(m_currentFrame);
    m_session.fit = HorizonFitResult();
    saveSession();
    updateFitText();
    showFrame(m_currentFrame);
}

void HorizonCalibrationWindow::skipCurrentFrame()
{
    if (m_currentFrame < 0)
    {
        return;
    }
    HorizonFrameAnnotation annotation;
    annotation.skipped = true;
    m_session.annotations.insert(m_currentFrame, annotation);
    m_session.fit = HorizonFitResult();
    saveSession();
    updateFitText();
    showFrame(m_currentFrame);
}

void HorizonCalibrationWindow::fitModel()
{
    if (m_availableFrames <= 0)
    {
        QMessageBox::information(this, QStringLiteral("拟合模型"), QStringLiteral("请先导入 HCAL 会话。"));
        return;
    }
    if (m_session.heightRecorded)
    {
        QMessageBox::information(this,
                                 QStringLiteral("拟合模型"),
                                 QStringLiteral("该会话包含高度数据，请先完成高度模型分析；当前不会拟合或导出旧Roll/Pitch模型。"));
        return;
    }
    m_session.fit = HorizonCalibration::fit(m_session);
    if (!m_session.fit.fitted)
    {
        updateFitText();
        QMessageBox::warning(this, QStringLiteral("拟合失败"), m_session.fit.error);
        return;
    }
    saveSession();
    updateFitText();
    showFrame(m_currentFrame);
}

void HorizonCalibrationWindow::exportModel()
{
    if (!m_session.fit.exportable)
    {
        return;
    }
    QString path = QFileDialog::getSaveFileName(this,
                                                QStringLiteral("导出地平线模型"),
                                                defaultModelPath(),
                                                QStringLiteral("Horizon Model (*.json)"));
    if (path.isEmpty())
    {
        return;
    }
    if (!path.endsWith(QStringLiteral(".json"), Qt::CaseInsensitive))
    {
        path += QStringLiteral(".json");
    }
    const QString headerPath = path.left(path.size() - 5) + QStringLiteral(".h");
    QString error;
    if (!HorizonCalibration::exportModel(m_session, path, headerPath, &error))
    {
        QMessageBox::critical(this, QStringLiteral("导出失败"), error);
        return;
    }
    QMessageBox::information(this,
                             QStringLiteral("导出完成"),
                             QStringLiteral("已生成：\n%1\n%2").arg(path, headerPath));
}

void HorizonCalibrationWindow::moveFrame(int direction)
{
    if (m_availableFrames > 0)
    {
        showFrame(qBound(0, m_currentFrame + direction, m_availableFrames - 1));
    }
}

void HorizonCalibrationWindow::moveToMatch(int direction, bool unannotated)
{
    if (m_availableFrames <= 0)
    {
        return;
    }
    for (int frame = m_currentFrame + direction;
         frame >= 0 && frame < m_availableFrames;
         frame += direction)
    {
        const HorizonFrameAnnotation annotation = m_session.annotations.value(frame);
        const bool matches = unannotated
            ? (annotation.legacyLine || annotation.points.size() < 5) && !annotation.skipped
            : m_session.fit.outlierFrames.contains(frame);
        if (matches)
        {
            showFrame(frame);
            return;
        }
    }
    QMessageBox::information(this,
                             unannotated ? QStringLiteral("未标注帧") : QStringLiteral("异常样本"),
                             unannotated ? QStringLiteral("该方向没有更多未标注帧。")
                                         : QStringLiteral("该方向没有更多异常样本。"));
}

bool HorizonCalibrationWindow::saveSession()
{
    QString error;
    if (!HorizonCalibration::saveSession(m_session, &error))
    {
        QMessageBox::critical(this, QStringLiteral("保存 HCAL 失败"), error);
        return false;
    }
    return true;
}

QString HorizonCalibrationWindow::defaultModelPath() const
{
    QString base = m_session.sessionPath;
    if (base.endsWith(QStringLiteral(".hcal.json"), Qt::CaseInsensitive))
    {
        base.chop(10);
    }
    else
    {
        base = QFileInfo(base).absolutePath() + QLatin1Char('/') + QFileInfo(base).completeBaseName();
    }
    return base + QStringLiteral("_horizon_model.json");
}

void HorizonCalibrationWindow::updateFitText()
{
    if (m_importedModel.valid)
    {
        const QString heightDetails = m_importedModel.heightCompensated
            ? QStringLiteral(" | 有效距离 %1 mm | 等高高度 %2 mm | Height [%3, %4] mm")
                  .arg(m_importedModel.effectiveDistanceMm, 0, 'f', 1)
                  .arg(m_importedModel.heightZeroMm, 0, 'f', 1)
                  .arg(m_importedModel.heightMinMm, 0, 'f', 1)
                  .arg(m_importedModel.heightMaxMm, 0, 'f', 1)
            : QString();
        m_fitLabel->setText(QStringLiteral("已导入 %1 | 样本 %2 | 内点 %3 | RMSE %4 px | 中位 %5 px | 最大 %6 px | "
                                          "Roll [%7, %8]° | Pitch [%9, %10]°%11")
                                .arg(QFileInfo(m_importedModel.sourcePath).fileName())
                                .arg(m_importedModel.sampleCount)
                                .arg(m_importedModel.inlierCount)
                                .arg(m_importedModel.rmse, 0, 'f', 3)
                                .arg(m_importedModel.medianError, 0, 'f', 3)
                                .arg(m_importedModel.maxError, 0, 'f', 3)
                                .arg(m_importedModel.rollMin, 0, 'f', 1)
                                .arg(m_importedModel.rollMax, 0, 'f', 1)
                                .arg(m_importedModel.pitchMin, 0, 'f', 1)
                                .arg(m_importedModel.pitchMax, 0, 'f', 1)
                                .arg(heightDetails));
        m_exportButton->setEnabled(m_session.fit.exportable);
        return;
    }
    if (m_session.heightRecorded)
    {
        m_fitLabel->setText(QStringLiteral("已记录同帧Roll、Pitch和融合高度；等待高度模型分析，当前不导出旧Roll/Pitch模型。"));
        m_exportButton->setEnabled(false);
        return;
    }
    if (!m_session.fit.fitted)
    {
        m_fitLabel->setText(m_session.fit.error.isEmpty() ? QStringLiteral("尚未拟合。") : m_session.fit.error);
        m_exportButton->setEnabled(false);
        return;
    }
    const QString warning = warningText(m_session.fit);
    m_fitLabel->setText(QStringLiteral("样本 %1 | 内点 %2 | 异常 %3 | RMSE %4 px | 中位 %5 px | 最大 %6 px | "
                                       "Roll [%7, %8]° | Pitch [%9, %10]°%11")
                            .arg(m_session.fit.sampleCount)
                            .arg(m_session.fit.inlierFrames.size())
                            .arg(m_session.fit.outlierFrames.size())
                            .arg(m_session.fit.rmse, 0, 'f', 3)
                            .arg(m_session.fit.medianError, 0, 'f', 3)
                            .arg(m_session.fit.maxError, 0, 'f', 3)
                            .arg(m_session.fit.rollMin, 0, 'f', 1)
                            .arg(m_session.fit.rollMax, 0, 'f', 1)
                            .arg(m_session.fit.pitchMin, 0, 'f', 1)
                            .arg(m_session.fit.pitchMax, 0, 'f', 1)
                            .arg(warning.isEmpty() ? QString() : QStringLiteral(" | 警告：%1").arg(warning)));
    m_exportButton->setEnabled(m_session.fit.exportable);
}
