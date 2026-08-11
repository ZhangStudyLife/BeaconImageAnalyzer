#include "ImageLogReviewWindow.h"

#include "FrameRenderer.h"
#include "SingleLampLogDiagnostics.h"
#include "VideoWidget.h"

#include <QCheckBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPolygonF>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QtMath>

#include <cmath>
#include <utility>

namespace
{
bool finite(float value)
{
    return std::isfinite(static_cast<double>(value));
}

QString cameraName(int camera)
{
    if (camera == 0) { return QStringLiteral("前摄"); }
    if (camera == 1) { return QStringLiteral("下摄"); }
    if (camera == 2) { return QStringLiteral("后摄"); }
    return QStringLiteral("未知");
}

QPointF imagePoint(float x, float y)
{
    return FrameRenderer::imageDataToImagePoint(x, y);
}

void drawCross(QPainter* painter, const QPointF& point, const QColor& color, const QString& text)
{
    if (painter == nullptr || !finite(static_cast<float>(point.x())) || !finite(static_cast<float>(point.y())))
    {
        return;
    }
    painter->setPen(QPen(color, 2));
    painter->setBrush(Qt::NoBrush);
    painter->drawEllipse(point, 4.0, 4.0);
    painter->drawLine(point + QPointF(-7, 0), point + QPointF(7, 0));
    painter->drawLine(point + QPointF(0, -7), point + QPointF(0, 7));
    painter->drawText(point + QPointF(6, -6), text);
}

void drawLamp(QPainter* painter, const JustFloatCarLamp& lamp)
{
    if (painter == nullptr || !lamp.valid || !finite(lamp.cx) || !finite(lamp.cy) || !finite(lamp.angle))
    {
        return;
    }
    const QPointF center = imagePoint(lamp.cx, lamp.cy);
    const qreal radians = qDegreesToRadians(static_cast<qreal>(lamp.angle));
    const qreal halfLength = qMax(5.0f, lamp.length * 0.5f);
    const qreal halfWidth = qMax(3.0f, lamp.width * 0.5f);
    const QPointF major(qCos(radians) * halfLength, qSin(radians) * halfLength);
    const QPointF minor(-qSin(radians) * halfWidth, qCos(radians) * halfWidth);
    QPolygonF polygon;
    polygon << center - major - minor << center + major - minor
            << center + major + minor << center - major + minor;
    painter->setPen(QPen(QColor(255, 95, 45), 2));
    painter->setBrush(Qt::NoBrush);
    painter->drawPolygon(polygon);
    painter->drawText(center + QPointF(6, -6), QStringLiteral("CAR"));
}
}

ImageLogReviewWindow::ImageLogReviewWindow(QWidget* parent)
    : QWidget(parent),
      m_playTimer(new QTimer(this))
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    setWindowTitle(QStringLiteral("图像-日志对齐审查"));
    resize(1120, 790);

    auto* root = new QVBoxLayout(this);
    auto* files = new QHBoxLayout;
    auto* videoButton = new QPushButton(QStringLiteral("导入 AVI"), this);
    auto* logButton = new QPushButton(QStringLiteral("导入 I0..I35 CSV"), this);
    m_sourceLabel = new QLabel(QStringLiteral("尚未导入 AVI 与日志"), this);
    m_sourceLabel->setWordWrap(true);
    m_sourceLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    files->addWidget(videoButton);
    files->addWidget(logButton);
    files->addWidget(m_sourceLabel, 1);
    root->addLayout(files);

    auto* alignment = new QHBoxLayout;
    m_alignmentLabel = new QLabel(QStringLiteral("配准：等待输入"), this);
    m_alignmentLabel->setWordWrap(true);
    m_alignmentLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    auto* minusCycle = new QPushButton(QStringLiteral("-128 周期"), this);
    auto* plusCycle = new QPushButton(QStringLiteral("+128 周期"), this);
    auto* resetCycle = new QPushButton(QStringLiteral("重置修正"), this);
    m_confirmLowConfidenceCheck = new QCheckBox(QStringLiteral("低置信度：我确认仍叠加"), this);
    alignment->addWidget(minusCycle);
    alignment->addWidget(plusCycle);
    alignment->addWidget(resetCycle);
    alignment->addWidget(m_confirmLowConfidenceCheck);
    alignment->addWidget(m_alignmentLabel, 1);
    root->addLayout(alignment);

    m_videoWidget = new VideoWidget(this);
    m_videoWidget->setText(QStringLiteral("导入 AVI 后显示灰度帧；仅在配准可信或明确确认时叠加日志"));
    root->addWidget(m_videoWidget, 1);

    m_frameLabel = new QLabel(QStringLiteral("帧：--"), this);
    m_frameLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    root->addWidget(m_frameLabel);

    auto* navigation = new QHBoxLayout;
    auto* previous = new QPushButton(QStringLiteral("上一帧"), this);
    m_playButton = new QPushButton(QStringLiteral("播放"), this);
    auto* next = new QPushButton(QStringLiteral("下一帧"), this);
    m_frameSlider = new QSlider(Qt::Horizontal, this);
    m_frameSlider->setRange(0, 0);
    navigation->addWidget(previous);
    navigation->addWidget(m_playButton);
    navigation->addWidget(next);
    navigation->addWidget(m_frameSlider, 1);
    root->addLayout(navigation);

    connect(videoButton, &QPushButton::clicked, this, &ImageLogReviewWindow::chooseVideo);
    connect(logButton, &QPushButton::clicked, this, &ImageLogReviewWindow::chooseLog);
    const auto applyCycleShift = [this](int shift) {
        m_manualCycleShift = shift;
        m_confirmLowConfidenceCheck->setChecked(false);
        rebuildAlignment();
    };
    connect(minusCycle, &QPushButton::clicked, this, [this, applyCycleShift]() {
        applyCycleShift(m_manualCycleShift - 1);
    });
    connect(plusCycle, &QPushButton::clicked, this, [this, applyCycleShift]() {
        applyCycleShift(m_manualCycleShift + 1);
    });
    connect(resetCycle, &QPushButton::clicked, this, [applyCycleShift]() {
        applyCycleShift(0);
    });
    connect(m_confirmLowConfidenceCheck, &QCheckBox::toggled, this, [this]() { showFrame(m_currentFrame); });
    connect(previous, &QPushButton::clicked, this, [this]() { moveFrame(-1); });
    connect(next, &QPushButton::clicked, this, [this]() { moveFrame(1); });
    connect(m_playButton, &QPushButton::clicked, this, &ImageLogReviewWindow::togglePlayback);
    connect(m_frameSlider, &QSlider::valueChanged, this, &ImageLogReviewWindow::showFrame);
    connect(m_playTimer, &QTimer::timeout, this, &ImageLogReviewWindow::updatePlayback);
}

bool ImageLogReviewWindow::openVideo(const QString& aviPath)
{
    if (m_playing)
    {
        togglePlayback();
    }
    QString error;
    VideoReader reader;
    if (!reader.open(aviPath, &error))
    {
        QMessageBox::critical(this, QStringLiteral("打开 AVI 失败"), error);
        return false;
    }
    QVector<ImageFrameSidecarRecord> sidecar;
    const QString sidecarPath = imageFrameSidecarPathForVideo(aviPath);
    if (QFileInfo::exists(sidecarPath) && !loadImageFrameSidecar(sidecarPath, &sidecar, &error))
    {
        QMessageBox::warning(this, QStringLiteral("读取 frames.csv 失败"), error);
        sidecar.clear();
    }
    m_reader = std::move(reader);
    m_sidecar = std::move(sidecar);
    m_manualCycleShift = 0;
    m_confirmLowConfidenceCheck->setChecked(false);
    m_frameSlider->setRange(0, qMax(0, m_reader.frameCount() - 1));
    const QString sidecarSummary = m_sidecar.isEmpty()
        ? QStringLiteral("未找到/为空")
        : m_sidecar.size() == m_reader.frameCount()
            ? QStringLiteral("%1 行").arg(m_sidecar.size())
            : QStringLiteral("%1 行，与视频帧数不一致").arg(m_sidecar.size());
    m_sourceLabel->setText(QStringLiteral("AVI：%1（%2 帧，%3x%4） | frames.csv：%5")
                               .arg(QFileInfo(aviPath).fileName()).arg(m_reader.frameCount())
                               .arg(m_reader.width()).arg(m_reader.height())
                               .arg(sidecarSummary));
    rebuildAlignment();
    showFrame(0);
    return true;
}

bool ImageLogReviewWindow::openLog(const QString& csvPath)
{
    JustFloatLog log;
    QString error;
    if (!JustFloatLog::loadCsv(csvPath, &log, &error))
    {
        QMessageBox::critical(this, QStringLiteral("导入 CSV 失败"), error);
        return false;
    }
    m_log = std::move(log);
    m_manualCycleShift = 0;
    m_confirmLowConfidenceCheck->setChecked(false);
    m_sourceLabel->setText(m_sourceLabel->text() + QStringLiteral(" | 日志：%1（%2 行，%3）")
                                                   .arg(QFileInfo(csvPath).fileName())
                                                   .arg(m_log.rowCount())
                                                   .arg(JustFloatLog::layoutName(m_log.layout())));
    rebuildAlignment();
    showFrame(m_currentFrame);
    return true;
}

void ImageLogReviewWindow::chooseVideo()
{
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入 AVI"), QString(), QStringLiteral("AVI 视频 (*.avi)"));
    if (!path.isEmpty()) { openVideo(path); }
}

void ImageLogReviewWindow::chooseLog()
{
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入 I0..I35 CSV"), QString(), QStringLiteral("CSV 文件 (*.csv)"));
    if (!path.isEmpty()) { openLog(path); }
}

void ImageLogReviewWindow::rebuildAlignment()
{
    m_alignment = ImageLogAligner::align(m_sidecar, m_log, m_manualCycleShift);
    updateStatus();
    if (m_currentFrame >= 0) { showFrame(m_currentFrame); }
}

bool ImageLogReviewWindow::overlayAllowed() const
{
    return m_alignment.confidence != ImageLogAlignmentConfidence::Unavailable
           && (m_alignment.confidence != ImageLogAlignmentConfidence::Low || m_confirmLowConfidenceCheck->isChecked());
}

void ImageLogReviewWindow::updateStatus()
{
    const QString confidence = ImageLogAligner::confidenceName(m_alignment.confidence);
    m_alignmentLabel->setText(QStringLiteral("配准置信度：%1 | 覆盖率：%2% | 摄像头：%3 | 偏移：%4（自动 %5，手工 %6×128） | %7")
                                   .arg(confidence).arg(m_alignment.coverage * 100.0, 0, 'f', 1)
                                   .arg(cameraName(m_alignment.sourceCameraId)).arg(m_alignment.sequenceOffset)
                                   .arg(m_alignment.automaticSequenceOffset).arg(m_manualCycleShift)
                                   .arg(m_alignment.message));
}

void ImageLogReviewWindow::showFrame(int frameIndex)
{
    if (!m_reader.isOpen() || frameIndex < 0 || frameIndex >= m_reader.frameCount()) { return; }
    QImage gray;
    QString error;
    if (!m_reader.readFrame(frameIndex, &gray, &error))
    {
        QMessageBox::warning(this, QStringLiteral("读取视频帧失败"), error);
        return;
    }
    m_currentFrame = frameIndex;
    const QSignalBlocker blocker(m_frameSlider);
    m_frameSlider->setValue(frameIndex);
    QString detail;
    m_videoWidget->setPixelSourceImage(gray);
    m_videoWidget->setImage(overlayFrame(gray, frameIndex, &detail));
    if (frameIndex < m_sidecar.size())
    {
        const ImageFrameSidecarRecord& record = m_sidecar[frameIndex];
        m_frameLabel->setText(
            QStringLiteral("视频帧 %1/%2 | sidecar：BIMG=%3 source=%4(%5) frame_valid=%6 "
                           "capture=%7 ms time_valid=%8 host=%9 | %10")
                .arg(frameIndex).arg(m_reader.frameCount() - 1).arg(record.bimgSequence)
                .arg(record.sourceFrameSequence).arg(cameraName(record.sourceCameraId))
                .arg(record.sourceFrameValid ? QStringLiteral("是") : QStringLiteral("否"))
                .arg(record.captureTimeMs)
                .arg(record.captureTimeValid ? QStringLiteral("是") : QStringLiteral("否"))
                .arg(record.hostTimeMs).arg(detail));
    }
    else
    {
        m_frameLabel->setText(QStringLiteral("视频帧 %1/%2 | 缺少对应 sidecar 行 | %3")
                                  .arg(frameIndex).arg(m_reader.frameCount() - 1).arg(detail));
    }
}

QImage ImageLogReviewWindow::overlayFrame(const QImage& gray, int frameIndex, QString* detail) const
{
    QImage image = gray.convertToFormat(QImage::Format_RGB32);
    if (!overlayAllowed() || frameIndex >= m_alignment.videoToLogRow.size())
    {
        if (detail != nullptr) *detail = m_alignment.confidence == ImageLogAlignmentConfidence::Low
            ? QStringLiteral("低置信度，叠加已默认禁用") : QStringLiteral("无可用日志映射");
        return image;
    }
    const int rowIndex = m_alignment.videoToLogRow[frameIndex];
    if (rowIndex < 0 || rowIndex >= m_log.rowCount())
    {
        if (detail != nullptr) *detail = QStringLiteral("该帧未映射到日志行");
        return image;
    }
    const int camera = m_alignment.sourceCameraId;
    const JustFloatLogRow& row = m_log.rowAt(rowIndex);
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const JustFloatCameraFrame& observation = row.cameras[camera];
    drawLamp(&painter, observation.carLamp);
    for (int index = 0; index < 2; ++index)
    {
        if (observation.beacons[index].valid)
        {
            drawCross(&painter, imagePoint(observation.beacons[index].x, observation.beacons[index].y),
                      QColor(0, 230, 90), QStringLiteral("B%1").arg(index));
        }
    }
    SingleLampLogDiagnostics diagnostic;
    QString decodeError;
    if (SingleLampLogDecoder::decode(row, &diagnostic, &decodeError))
    {
        QPointF projected;
        if (diagnostic.track.valid && SingleLampLogDecoder::projectCenterToCamera(camera, diagnostic.track.center, &projected))
        {
            const QPointF center = imagePoint(static_cast<float>(projected.x()), static_cast<float>(projected.y()));
            painter.setPen(QPen(QColor(55, 205, 255), 2, Qt::DashLine));
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(center, 5, 5);
            painter.drawText(center + QPointF(6, 12), QStringLiteral("轨迹预计中心"));
            const qreal gate = camera == 2 ? 12.0 : 8.0;
            painter.setPen(QPen(QColor(255, 215, 50), 1, Qt::DashLine));
            painter.drawEllipse(center, gate, gate);
            painter.drawText(center + QPointF(6, 23),
                             QStringLiteral("闭环门 %1 px").arg(gate, 0, 'f', 0));
        }
        if (detail != nullptr)
        {
            *detail = QStringLiteral("log 行 %1 | I32=%2 support=%3 ROI(valid=%4 hit=%5) conflict=%6 fallback=%7 measured=%8 mode=%9 | Yaw=%10° skew=%11 ms")
                          .arg(rowIndex).arg(SingleLampLogDecoder::trackStateName(diagnostic.crossCheck.state))
                          .arg(SingleLampLogDecoder::cameraMaskText(diagnostic.crossCheck.supportCameraMask))
                          .arg(SingleLampLogDecoder::cameraMaskText(diagnostic.crossCheck.roiValidMask))
                          .arg(SingleLampLogDecoder::cameraMaskText(diagnostic.crossCheck.roiHitMask))
                          .arg(SingleLampLogDecoder::cameraMaskText(diagnostic.crossCheck.conflictCameraMask))
                          .arg(SingleLampLogDecoder::cameraMaskText(diagnostic.crossCheck.fullFrameFallbackMask))
                          .arg(SingleLampLogDecoder::cameraMaskText(diagnostic.crossCheck.measuredCameraMask))
                          .arg(diagnostic.crossCheck.actualRoiMode ? QStringLiteral("ROI") : QStringLiteral("full"))
                          .arg(diagnostic.relativeYawDeg, 0, 'f', 1).arg(diagnostic.maxFrameSkewMs, 0, 'f', 1);
        }
    }
    else if (detail != nullptr)
    {
        *detail = QStringLiteral("log 行 %1 | %2").arg(rowIndex).arg(decodeError);
    }
    return image;
}

void ImageLogReviewWindow::moveFrame(int delta)
{
    if (m_reader.isOpen()) showFrame(qBound(0, m_currentFrame + delta, m_reader.frameCount() - 1));
}

void ImageLogReviewWindow::togglePlayback()
{
    if (!m_reader.isOpen() || m_reader.frameCount() <= 0)
    {
        return;
    }
    m_playing = !m_playing;
    m_playButton->setText(m_playing ? QStringLiteral("暂停") : QStringLiteral("播放"));
    if (m_playing) m_playTimer->start(qMax(1, qRound(1000.0 / qMax(1.0, m_reader.videoFps()))));
    else m_playTimer->stop();
}

void ImageLogReviewWindow::updatePlayback()
{
    if (m_currentFrame + 1 >= m_reader.frameCount()) { togglePlayback(); return; }
    showFrame(m_currentFrame + 1);
}
