#include "LogReplayWindow.h"

#include "FrameRenderer.h"
#include "VideoWidget.h"

#include <QComboBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStringList>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QtMath>

#include <cmath>
#include <cstring>

namespace
{
constexpr int CameraCount = 3;
constexpr float Pi = 3.1415926f;

const QStringList CameraNames = {
    QStringLiteral("Front"),
    QStringLiteral("Center"),
    QStringLiteral("Back")
};

bool isFinitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

void setGrayMax(QImage* image, int x, int y, int value)
{
    if (image == nullptr ||
        x < 0 || x >= BEACON_IMAGE_W ||
        y < 0 || y >= BEACON_IMAGE_H)
    {
        return;
    }

    uchar* line = image->scanLine(y);
    line[x] = (uchar)qBound(0, qMax((int)line[x], value), 255);
}

double rowPlaybackTimeMs(const JustFloatLogRow& row)
{
    if (isFinitePositive(row.syncTimeMs))
    {
        return row.syncTimeMs;
    }
    if (isFinitePositive(row.rowTime))
    {
        return row.rowTime;
    }
    return 0.0;
}
}

LogReplayWindow::LogReplayWindow(QWidget* parent)
    : QWidget(parent),
      m_timer(new QTimer(this))
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    setWindowTitle(QStringLiteral("JustFloat 日志回放"));
    resize(1120, 760);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    auto* topRow = new QHBoxLayout;
    auto* importButton = new QPushButton(QStringLiteral("导入 CSV"), this);
    m_statusLabel = new QLabel(QStringLiteral("未导入日志"), this);
    m_statusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_statusLabel->setWordWrap(true);
    m_returnGridButton = new QPushButton(QStringLiteral("返回三摄"), this);
    m_returnGridButton->setEnabled(false);
    topRow->addWidget(importButton);
    topRow->addWidget(m_statusLabel, 1);
    topRow->addWidget(m_returnGridButton);
    root->addLayout(topRow);

    m_cameraLayout = new QHBoxLayout;
    m_cameraLayout->setSpacing(8);
    for (int i = 0; i < CameraCount; ++i)
    {
        auto* group = new QGroupBox(CameraNames[i], this);
        auto* groupLayout = new QVBoxLayout(group);
        groupLayout->setContentsMargins(8, 8, 8, 8);
        groupLayout->setSpacing(6);

        auto* widget = new VideoWidget(group);
        widget->setText(QStringLiteral("导入日志后显示"));
        widget->setFrameGeometry(QSize(BEACON_IMAGE_W, BEACON_IMAGE_H), 1);
        auto* info = new QLabel(QStringLiteral("无数据"), group);
        info->setTextInteractionFlags(Qt::TextSelectableByMouse);
        info->setWordWrap(true);

        groupLayout->addWidget(widget, 1);
        groupLayout->addWidget(info);
        m_cameraGroups[i] = group;
        m_videoWidgets[i] = widget;
        m_cameraInfoLabels[i] = info;
        m_cameraLayout->addWidget(group, 1);

        connect(widget, &VideoWidget::activated, this, [this, i]() {
            setFocusCamera(i);
        });
    }
    root->addLayout(m_cameraLayout, 1);

    auto* controls = new QHBoxLayout;
    m_playButton = new QPushButton(QStringLiteral("播放"), this);
    auto* previousButton = new QPushButton(QStringLiteral("上一帧"), this);
    auto* nextButton = new QPushButton(QStringLiteral("下一帧"), this);
    m_slider = new QSlider(Qt::Horizontal, this);
    m_slider->setRange(0, 0);
    m_frameSpin = new QSpinBox(this);
    m_frameSpin->setRange(0, 0);
    m_frameSpin->setFixedWidth(92);
    m_speedCombo = new QComboBox(this);
    m_speedCombo->addItem(QStringLiteral("0.5x"), 0.5);
    m_speedCombo->addItem(QStringLiteral("1.0x"), 1.0);
    m_speedCombo->addItem(QStringLiteral("2.0x"), 2.0);
    m_speedCombo->addItem(QStringLiteral("4.0x"), 4.0);
    m_speedCombo->setCurrentIndex(1);
    controls->addWidget(m_playButton);
    controls->addWidget(previousButton);
    controls->addWidget(nextButton);
    controls->addWidget(new QLabel(QStringLiteral("Frame"), this));
    controls->addWidget(m_frameSpin);
    controls->addWidget(m_slider, 1);
    controls->addWidget(new QLabel(QStringLiteral("速度"), this));
    controls->addWidget(m_speedCombo);
    root->addLayout(controls);

    m_infoText = new QTextEdit(this);
    m_infoText->setReadOnly(true);
    m_infoText->setMinimumHeight(120);
    root->addWidget(m_infoText);

    connect(importButton, &QPushButton::clicked, this, &LogReplayWindow::importCsv);
    connect(m_returnGridButton, &QPushButton::clicked, this, &LogReplayWindow::showAllCameras);
    connect(m_playButton, &QPushButton::clicked, this, &LogReplayWindow::togglePlayback);
    connect(previousButton, &QPushButton::clicked, this, [this]() {
        setCurrentRow(m_currentRow - 1);
    });
    connect(nextButton, &QPushButton::clicked, this, [this]() {
        setCurrentRow(m_currentRow + 1);
    });
    connect(m_slider, &QSlider::valueChanged, this, &LogReplayWindow::setCurrentRow);
    connect(m_frameSpin, qOverload<int>(&QSpinBox::valueChanged), this, &LogReplayWindow::setCurrentRow);
    connect(m_timer, &QTimer::timeout, this, &LogReplayWindow::advancePlayback);
}

void LogReplayWindow::importCsv()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("导入 JustFloat CSV"),
                                                      QString(),
                                                      QStringLiteral("CSV (*.csv);;所有文件 (*.*)"));
    if (path.isEmpty())
    {
        return;
    }
    (void)loadCsv(path);
}

bool LogReplayWindow::loadCsv(const QString& path)
{
    JustFloatLog log;
    QString error;
    if (!JustFloatLog::loadCsv(path, &log, &error))
    {
        QMessageBox::critical(this, QStringLiteral("导入失败"), error);
        return false;
    }

    m_timer->stop();
    m_playing = false;
    m_playButton->setText(QStringLiteral("播放"));
    m_log = log;
    m_currentRow = -1;
    m_focusCamera = -1;

    const int lastRow = qMax(0, m_log.rowCount() - 1);
    m_slider->setRange(0, lastRow);
    m_frameSpin->setRange(0, lastRow);
    m_statusLabel->setText(QStringLiteral("%1 | %2 行")
                               .arg(QFileInfo(path).fileName())
                               .arg(m_log.rowCount()));
    updateCameraVisibility();
    setCurrentRow(0);
    return true;
}

void LogReplayWindow::setCurrentRow(int row)
{
    if (m_log.rowCount() <= 0)
    {
        return;
    }

    const int clamped = qBound(0, row, m_log.rowCount() - 1);
    if (m_currentRow == clamped)
    {
        return;
    }

    m_currentRow = clamped;
    {
        const QSignalBlocker blockSlider(m_slider);
        const QSignalBlocker blockSpin(m_frameSpin);
        m_slider->setValue(m_currentRow);
        m_frameSpin->setValue(m_currentRow);
    }
    renderCurrentRow();
    updateInfoText();
}

void LogReplayWindow::renderCurrentRow()
{
    for (int i = 0; i < CameraCount; ++i)
    {
        renderCamera(i);
    }
}

void LogReplayWindow::renderCamera(int cameraIndex)
{
    if (m_currentRow < 0 || cameraIndex < 0 || cameraIndex >= CameraCount)
    {
        return;
    }

    const QImage gray = syntheticImageForCamera(cameraIndex);
    const beacon_result_t result = resultForCamera(cameraIndex);
    const QImage rendered = FrameRenderer::render(gray, result, {}, 1, true);
    m_videoWidgets[cameraIndex]->setPixelSourceImage(gray);
    m_videoWidgets[cameraIndex]->setFrameGeometry(QSize(BEACON_IMAGE_W, BEACON_IMAGE_H), 1);
    m_videoWidgets[cameraIndex]->setImage(rendered);

    const JustFloatCameraFrame& camera = m_log.rowAt(m_currentRow).cameras[cameraIndex];
    m_cameraInfoLabels[cameraIndex]->setText(QStringLiteral("B0:%1  B1:%2  CAR:%3")
                                                 .arg(camera.beacons[0].valid ? QStringLiteral("有效") : QStringLiteral("无"))
                                                 .arg(camera.beacons[1].valid ? QStringLiteral("有效") : QStringLiteral("无"))
                                                 .arg(camera.carLamp.valid ? QStringLiteral("有效") : QStringLiteral("无")));
}

void LogReplayWindow::togglePlayback()
{
    if (m_log.rowCount() <= 0)
    {
        return;
    }

    m_playing = !m_playing;
    m_playButton->setText(m_playing ? QStringLiteral("暂停") : QStringLiteral("播放"));
    if (m_playing)
    {
        if (m_currentRow >= m_log.rowCount() - 1)
        {
            setCurrentRow(0);
        }
        scheduleNextFrame();
    }
    else
    {
        m_timer->stop();
    }
}

void LogReplayWindow::advancePlayback()
{
    if (!m_playing)
    {
        return;
    }

    if (m_currentRow >= m_log.rowCount() - 1)
    {
        m_playing = false;
        m_playButton->setText(QStringLiteral("播放"));
        return;
    }

    setCurrentRow(m_currentRow + 1);
    scheduleNextFrame();
}

void LogReplayWindow::scheduleNextFrame()
{
    if (m_playing)
    {
        m_timer->start(playbackIntervalMs());
    }
}

int LogReplayWindow::playbackIntervalMs() const
{
    if (m_currentRow < 0 || m_currentRow + 1 >= m_log.rowCount())
    {
        return 20;
    }

    const double now = rowPlaybackTimeMs(m_log.rowAt(m_currentRow));
    const double next = rowPlaybackTimeMs(m_log.rowAt(m_currentRow + 1));
    double delta = next - now;
    if (!std::isfinite(delta) || delta < 1.0 || delta > 1000.0)
    {
        delta = 20.0;
    }

    const double speed = qMax(0.1, m_speedCombo->currentData().toDouble());
    return qBound(1, (int)std::lround(delta / speed), 1000);
}

void LogReplayWindow::setFocusCamera(int cameraIndex)
{
    if (cameraIndex < 0 || cameraIndex >= CameraCount)
    {
        return;
    }
    m_focusCamera = cameraIndex;
    updateCameraVisibility();
}

void LogReplayWindow::showAllCameras()
{
    m_focusCamera = -1;
    updateCameraVisibility();
}

void LogReplayWindow::updateCameraVisibility()
{
    for (int i = 0; i < CameraCount; ++i)
    {
        m_cameraGroups[i]->setVisible(m_focusCamera < 0 || m_focusCamera == i);
    }
    m_returnGridButton->setEnabled(m_focusCamera >= 0);
}

void LogReplayWindow::updateInfoText()
{
    if (m_currentRow < 0 || m_log.rowCount() <= 0)
    {
        m_infoText->setText(QStringLiteral("未导入日志"));
        return;
    }

    const JustFloatLogRow& row = m_log.rowAt(m_currentRow);
    QString text;
    text += QStringLiteral("Frame %1 / %2\n").arg(m_currentRow).arg(m_log.rowCount() - 1);
    text += QStringLiteral("I0: %1  Sync: %2 ms\n")
                .arg(row.rowTime, 0, 'f', 3)
                .arg(row.syncTimeMs, 0, 'f', 3);
    text += QStringLiteral("Pitch: %1  Roll: %2  Yaw: %3\n")
                .arg(row.pitch, 0, 'f', 3)
                .arg(row.roll, 0, 'f', 3)
                .arg(row.yaw, 0, 'f', 3);

    for (int camera = 0; camera < CameraCount; ++camera)
    {
        const JustFloatCameraFrame& cameraFrame = row.cameras[camera];
        text += QStringLiteral("%1: ").arg(CameraNames[camera]);
        for (int i = 0; i < 2; ++i)
        {
            const JustFloatBeacon& beacon = cameraFrame.beacons[i];
            text += QStringLiteral("B%1[%2 x=%3 y=%4 area=%5] ")
                        .arg(i)
                        .arg(beacon.valid ? QStringLiteral("有效") : QStringLiteral("无"))
                        .arg(beacon.x, 0, 'f', 2)
                        .arg(beacon.y, 0, 'f', 2)
                        .arg(beacon.area, 0, 'f', 2);
        }
        const JustFloatCarLamp& lamp = cameraFrame.carLamp;
        text += QStringLiteral("CAR[%1 cx=%2 cy=%3 angle=%4 w=%5 len=%6]\n")
                    .arg(lamp.valid ? QStringLiteral("有效") : QStringLiteral("无"))
                    .arg(lamp.cx, 0, 'f', 2)
                    .arg(lamp.cy, 0, 'f', 2)
                    .arg(lamp.angle, 0, 'f', 2)
                    .arg(lamp.width, 0, 'f', 2)
                    .arg(lamp.length, 0, 'f', 2);
    }
    m_infoText->setText(text);
}

beacon_result_t LogReplayWindow::resultForCamera(int cameraIndex) const
{
    beacon_result_t result;
    memset(&result, 0, sizeof(result));
    if (m_currentRow < 0 || cameraIndex < 0 || cameraIndex >= CameraCount)
    {
        return result;
    }

    const JustFloatCameraFrame& camera = m_log.rowAt(m_currentRow).cameras[cameraIndex];
    int beaconCount = 0;
    for (int i = 0; i < 2; ++i)
    {
        const JustFloatBeacon& source = camera.beacons[i];
        if (!source.valid)
        {
            continue;
        }
        result.beacons[i].x = source.x;
        result.beacons[i].y = source.y;
        result.beacons[i].radius = std::sqrt(qMax(0.0f, source.area) / Pi);
        result.beacons[i].valid = 1;
        result.circles[i] = result.beacons[i];
        beaconCount = i + 1;
    }
    result.beacon_count = (unsigned char)beaconCount;
    result.count = (unsigned char)beaconCount;

    if (camera.carLamp.valid)
    {
        result.car_lamps[0].cx = camera.carLamp.cx;
        result.car_lamps[0].cy = camera.carLamp.cy;
        result.car_lamps[0].angle = camera.carLamp.angle;
        result.car_lamps[0].width = camera.carLamp.width;
        result.car_lamps[0].length = camera.carLamp.length;
        result.car_lamps[0].valid = 1;
        result.car_lamp_count = 1;
    }
    return result;
}

QImage LogReplayWindow::syntheticImageForCamera(int cameraIndex) const
{
    QImage image(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    image.fill(0);
    if (m_currentRow < 0 || cameraIndex < 0 || cameraIndex >= CameraCount)
    {
        return image;
    }

    const JustFloatCameraFrame& camera = m_log.rowAt(m_currentRow).cameras[cameraIndex];
    for (const JustFloatBeacon& beacon : camera.beacons)
    {
        if (!beacon.valid)
        {
            continue;
        }

        const QPointF center = FrameRenderer::algorithmToImagePoint(beacon.x, beacon.y);
        const double radius = qMax(1.0, std::sqrt(qMax(0.0f, beacon.area) / Pi));
        const int extent = (int)std::ceil(radius + 4.0);
        const int cx = (int)std::lround(center.x());
        const int cy = (int)std::lround(center.y());
        for (int y = cy - extent; y <= cy + extent; ++y)
        {
            for (int x = cx - extent; x <= cx + extent; ++x)
            {
                const double distance = std::hypot((double)x - center.x(), (double)y - center.y());
                if (distance <= radius)
                {
                    setGrayMax(&image, x, y, 245);
                }
                else if (distance <= radius + 4.0)
                {
                    const double fade = 1.0 - (distance - radius) / 4.0;
                    setGrayMax(&image, x, y, 30 + (int)std::lround(90.0 * fade));
                }
            }
        }
    }

    if (camera.carLamp.valid)
    {
        const QPointF center = FrameRenderer::algorithmToImagePoint(camera.carLamp.cx, camera.carLamp.cy);
        const double radians = qDegreesToRadians((double)camera.carLamp.angle);
        const double cosA = std::cos(radians);
        const double sinA = std::sin(radians);
        const double halfLength = qMax(1.0, (double)camera.carLamp.length * 0.5);
        const double halfWidth = qMax(1.0, (double)camera.carLamp.width * 0.5);
        const int extent = (int)std::ceil(std::hypot(halfLength, halfWidth)) + 2;
        const int cx = (int)std::lround(center.x());
        const int cy = (int)std::lround(center.y());
        for (int y = cy - extent; y <= cy + extent; ++y)
        {
            for (int x = cx - extent; x <= cx + extent; ++x)
            {
                const double dx = (double)x - center.x();
                const double dy = (double)y - center.y();
                const double major = dx * cosA + dy * sinA;
                const double minor = -dx * sinA + dy * cosA;
                if (std::abs(major) <= halfLength &&
                    std::abs(minor) <= halfWidth)
                {
                    setGrayMax(&image, x, y, 190);
                }
            }
        }
    }

    return image;
}
