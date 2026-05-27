#include "MainWindow.h"

#include "AnnotationJson.h"
#include "AnnotationPanel.h"
#include "FrameRenderer.h"
#include "VideoExporter.h"
#include "VideoWidget.h"

#include <QAbstractSpinBox>
#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QList>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QPainter>
#include <QPaintEvent>
#include <QPlainTextEdit>
#include <QPolygonF>
#include <QProgressDialog>
#include <QPushButton>
#include <QRectF>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSlider>
#include <QSpinBox>
#include <QStatusBar>
#include <QStyle>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QLineF>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
constexpr double Pi = 3.14159265358979323846;

bool isEditingWidget(QWidget* widget)
{
    while (widget != nullptr)
    {
        if (qobject_cast<QLineEdit*>(widget) != nullptr ||
            qobject_cast<QTextEdit*>(widget) != nullptr ||
            qobject_cast<QPlainTextEdit*>(widget) != nullptr ||
            qobject_cast<QAbstractSpinBox*>(widget) != nullptr ||
            qobject_cast<QComboBox*>(widget) != nullptr)
        {
            return true;
        }
        widget = widget->parentWidget();
    }
    return false;
}

int validCircleCount(const beacon_result_t& result)
{
    int count = 0;
    for (int i = 0; i < result.count && i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        if (result.circles[i].valid != 0)
        {
            ++count;
        }
    }
    return count;
}

double circleArea(const beacon_circle_t& circle)
{
    return Pi * (double)circle.radius * (double)circle.radius;
}

QString cameraBuildDir(const QString& name)
{
    return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("imported_algorithms/%1").arg(name));
}
}

class RadarWidget : public QWidget
{
public:
    explicit RadarWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumSize(300, 260);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    void setResult(const beacon_fusion_result_t& result)
    {
        m_result = result;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.fillRect(rect(), QColor(8, 10, 13));

        const QRectF area = QRectF(rect()).adjusted(18.0, 18.0, -18.0, -18.0);
        const QPointF center = area.center();
        const double radius = qMin(area.width(), area.height()) * 0.46;
        if (radius <= 0.0)
        {
            return;
        }

        double maxDistance = 10.0;
        for (int i = 0; i < m_result.beacon_count && i < BEACON_FUSION_MAX_BEACONS; ++i)
        {
            const beacon_fusion_beacon_t& beacon = m_result.beacon[i];
            if (beacon.valid != 0 && std::isfinite(beacon.range_proxy))
            {
                maxDistance = qMax(maxDistance, (double)beacon.range_proxy);
            }
        }

        QPen gridPen(QColor(125, 150, 170, 90));
        gridPen.setWidth(1);
        painter.setPen(gridPen);
        painter.setBrush(Qt::NoBrush);
        for (int ring = 1; ring <= 4; ++ring)
        {
            const double ringRadius = radius * (double)ring / 4.0;
            painter.drawEllipse(center, ringRadius, ringRadius);
        }

        for (int degree = 0; degree < 360; degree += 30)
        {
            const double radians = (double)degree * Pi / 180.0;
            const QPointF end(center.x() + std::sin(radians) * radius,
                              center.y() - std::cos(radians) * radius);
            painter.drawLine(center, end);
        }

        QFont font = painter.font();
        font.setPixelSize(11);
        painter.setFont(font);
        painter.setPen(QColor(225, 232, 238));
        painter.drawText(QRectF(center.x() - 18.0, center.y() - radius - 17.0, 36.0, 16.0),
                         Qt::AlignCenter,
                         QStringLiteral("0"));
        painter.drawText(QRectF(center.x() + radius + 2.0, center.y() - 8.0, 34.0, 16.0),
                         Qt::AlignLeft | Qt::AlignVCenter,
                         QStringLiteral("90"));
        painter.drawText(QRectF(center.x() - 22.0, center.y() + radius + 1.0, 44.0, 16.0),
                         Qt::AlignCenter,
                         QStringLiteral("180"));
        painter.drawText(QRectF(center.x() - radius - 38.0, center.y() - 8.0, 34.0, 16.0),
                         Qt::AlignRight | Qt::AlignVCenter,
                         QStringLiteral("270"));

        const QColor colors[BEACON_FUSION_MAX_BEACONS] = {
            QColor(64, 211, 255),
            QColor(255, 196, 70),
            QColor(120, 235, 126),
            QColor(255, 112, 178),
            QColor(190, 140, 255)
        };
        for (int i = 0; i < m_result.beacon_count && i < BEACON_FUSION_MAX_BEACONS; ++i)
        {
            const beacon_fusion_beacon_t& beacon = m_result.beacon[i];
            if (beacon.valid == 0 || !std::isfinite(beacon.bearing_deg) || !std::isfinite(beacon.range_proxy))
            {
                continue;
            }

            const double clampedDistance = qBound(0.0, (double)beacon.range_proxy, maxDistance);
            const double normalizedDistance = clampedDistance / maxDistance;
            const double radians = (double)beacon.bearing_deg * Pi / 180.0;
            const QPointF point(center.x() + std::sin(radians) * radius * normalizedDistance,
                                center.y() - std::cos(radians) * radius * normalizedDistance);
            const QColor color = colors[i % BEACON_FUSION_MAX_BEACONS];

            painter.setPen(QPen(color, 2));
            painter.setBrush(color);
            painter.drawEllipse(point, 5.0, 5.0);
            painter.drawText(point + QPointF(7.0, -7.0),
                             QStringLiteral("#%1").arg(i));
        }

        painter.setPen(QColor(225, 232, 238));
        painter.drawText(area.adjusted(4.0, 4.0, -4.0, -4.0),
                         Qt::AlignLeft | Qt::AlignTop,
                         QStringLiteral("360度雷达图  最大距离 %1").arg(maxDistance, 0, 'f', 2));
    }

private:
    beacon_fusion_result_t m_result = {};
};

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    for (int i = 0; i < BEACON_CAMERA_COUNT; ++i)
    {
        m_cameras[i].index = i;
        m_cameras[i].name = QStringLiteral("摄像头 %1").arg(i + 1);
        std::memset(&m_cameras[i].currentResult, 0, sizeof(m_cameras[i].currentResult));
    }

    setWindowTitle(QStringLiteral("BeaconImageAnalyzer - 三摄像头融合版"));
    resize(1480, 900);
    buildUi();
    buildMenus();
    qApp->installEventFilter(this);

    connect(&m_playTimer, &QTimer::timeout, this, [this]() {
        if (!m_playing || !hasAnyVideo())
        {
            return;
        }

        const int maxFrame = timelineMaxFrame();
        if (m_timelineFrame >= maxFrame)
        {
            pause();
            return;
        }

        const double fps = currentCamera() != nullptr && currentCamera()->usedFps > 0.0
            ? currentCamera()->usedFps
            : 50.0;
        int targetFrame = m_playbackStartFrame +
                          (int)std::floor((double)m_playbackClock.elapsed() * fps * m_playbackSpeed / 1000.0);
        targetFrame = qMax(m_timelineFrame + 1, targetFrame);
        targetFrame = qMin(targetFrame, maxFrame);
        showFrame(targetFrame);
        if (m_timelineFrame >= maxFrame)
        {
            pause();
        }
    });

    selectCamera(0);
    showFrame(0);
}

MainWindow::~MainWindow() = default;

void MainWindow::buildUi()
{
    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* root = new QHBoxLayout(central);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(10);

    auto* workspace = new QWidget(central);
    auto* workspaceLayout = new QVBoxLayout(workspace);
    workspaceLayout->setContentsMargins(0, 0, 0, 0);
    workspaceLayout->setSpacing(10);

    auto* cameraGroup = new QGroupBox(QStringLiteral("三路视频"), workspace);
    auto* cameraGrid = new QGridLayout(cameraGroup);
    cameraGrid->setContentsMargins(10, 10, 10, 10);
    cameraGrid->setHorizontalSpacing(10);
    cameraGrid->setVerticalSpacing(10);
    for (int i = 0; i < BEACON_CAMERA_COUNT; ++i)
    {
        buildCameraPanel(cameraGrid, i, i);
    }
    workspaceLayout->addWidget(cameraGroup, 5);

    auto* fusionGroup = new QGroupBox(QStringLiteral("融合结果"), workspace);
    auto* fusionLayout = new QHBoxLayout(fusionGroup);
    fusionLayout->setContentsMargins(10, 10, 10, 10);
    fusionLayout->setSpacing(10);

    m_fusionText = new QTextEdit(fusionGroup);
    m_fusionText->setReadOnly(true);
    m_fusionText->setMinimumHeight(180);
    m_fusionText->setText(QStringLiteral("等待导入视频。"));
    m_radarWidget = new RadarWidget(fusionGroup);
    fusionLayout->addWidget(m_fusionText, 2);
    fusionLayout->addWidget(m_radarWidget, 3);
    workspaceLayout->addWidget(fusionGroup, 2);

    auto* controls = new QFrame(workspace);
    controls->setFrameShape(QFrame::StyledPanel);
    auto* controlsLayout = new QHBoxLayout(controls);
    controlsLayout->setContentsMargins(10, 8, 10, 8);
    controlsLayout->setSpacing(8);

    auto* previousButton = new QPushButton(style()->standardIcon(QStyle::SP_MediaSkipBackward), QString(), controls);
    previousButton->setToolTip(QStringLiteral("上一帧"));
    m_playPauseButton = new QPushButton(style()->standardIcon(QStyle::SP_MediaPlay), QString(), controls);
    m_playPauseButton->setToolTip(QStringLiteral("播放/暂停"));
    auto* nextButton = new QPushButton(style()->standardIcon(QStyle::SP_MediaSkipForward), QString(), controls);
    nextButton->setToolTip(QStringLiteral("下一帧"));

    m_slider = new QSlider(Qt::Horizontal, controls);
    m_slider->setRange(0, 0);

    m_frameSpin = new QSpinBox(controls);
    m_frameSpin->setRange(0, 0);
    m_frameSpin->setPrefix(QStringLiteral("同步帧 "));

    m_timeSpin = new QDoubleSpinBox(controls);
    m_timeSpin->setRange(0.0, 0.0);
    m_timeSpin->setDecimals(3);
    m_timeSpin->setSuffix(QStringLiteral(" s"));

    m_viewModeCombo = new QComboBox(controls);
    m_viewModeCombo->addItem(QStringLiteral("原图"), QStringLiteral("original"));
    m_viewModeCombo->addItem(QStringLiteral("二值图"), QStringLiteral("binary"));

    m_speedCombo = new QComboBox(controls);
    const QList<QPair<QString, double>> speeds = {
        { QStringLiteral("1/8x"), 0.125 },
        { QStringLiteral("1/4x"), 0.25 },
        { QStringLiteral("1/2x"), 0.5 },
        { QStringLiteral("1x"), 1.0 },
        { QStringLiteral("2x"), 2.0 },
        { QStringLiteral("4x"), 4.0 },
        { QStringLiteral("8x"), 8.0 }
    };
    for (const auto& item : speeds)
    {
        m_speedCombo->addItem(item.first, item.second);
    }
    m_speedCombo->setCurrentIndex(3);

    m_showOverlayCheck = new QCheckBox(QStringLiteral("标注层"), controls);
    m_showOverlayCheck->setChecked(true);

    controlsLayout->addWidget(previousButton);
    controlsLayout->addWidget(m_playPauseButton);
    controlsLayout->addWidget(nextButton);
    controlsLayout->addWidget(m_slider, 1);
    controlsLayout->addWidget(m_frameSpin);
    controlsLayout->addWidget(m_timeSpin);
    controlsLayout->addWidget(m_viewModeCombo);
    controlsLayout->addWidget(m_speedCombo);
    controlsLayout->addWidget(m_showOverlayCheck);
    workspaceLayout->addWidget(controls);

    m_videoInfoLabel = new QLabel(QStringLiteral("未导入视频"), workspace);
    m_videoInfoLabel->setWordWrap(true);
    m_frameInfoLabel = new QLabel(workspace);
    m_frameInfoLabel->setWordWrap(true);
    m_pixelInfoLabel = new QLabel(QStringLiteral("像素：-"), workspace);
    m_pixelInfoLabel->setWordWrap(true);
    workspaceLayout->addWidget(m_videoInfoLabel);
    workspaceLayout->addWidget(m_frameInfoLabel);
    workspaceLayout->addWidget(m_pixelInfoLabel);

    m_annotationPanel = new AnnotationPanel(central);
    auto* annotationScroll = new QScrollArea(central);
    annotationScroll->setWidgetResizable(true);
    annotationScroll->setMinimumWidth(360);
    annotationScroll->setWidget(m_annotationPanel);

    root->addWidget(workspace, 1);
    root->addWidget(annotationScroll);

    connect(previousButton, &QPushButton::clicked, this, &MainWindow::previousFrame);
    connect(m_playPauseButton, &QPushButton::clicked, this, &MainWindow::togglePlayPause);
    connect(nextButton, &QPushButton::clicked, this, &MainWindow::nextFrame);
    connect(m_slider, &QSlider::sliderMoved, this, &MainWindow::showFrameFromSlider);
    connect(m_frameSpin, &QSpinBox::editingFinished, this, &MainWindow::jumpToFrame);
    connect(m_timeSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        if (!m_updatingControls && m_timeSpin->hasFocus())
        {
            jumpToTime();
        }
    });
    connect(m_viewModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        showFrame(m_timelineFrame);
    });
    connect(m_speedCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        setPlaybackSpeed(m_speedCombo->currentData().toDouble());
    });
    connect(m_showOverlayCheck, &QCheckBox::toggled, this, [this](bool checked) {
        m_showOverlay = checked;
        showFrame(m_timelineFrame);
    });

    connect(m_annotationPanel,
            &AnnotationPanel::saveCurrentFrameCorrectionsRequested,
            this,
            &MainWindow::saveCurrentFrameCorrections);
    connect(m_annotationPanel, &AnnotationPanel::deleteAnnotationsRequested, this, &MainWindow::deleteAnnotations);
    connect(m_annotationPanel, &AnnotationPanel::deleteCorrectionsRequested, this, &MainWindow::deleteCorrections);
    connect(m_annotationPanel,
            &AnnotationPanel::batchAddCorrectionsRequested,
            this,
            &MainWindow::batchAddCorrections);
    connect(m_annotationPanel,
            &AnnotationPanel::autoMatchCorrectionFramesRequested,
            this,
            &MainWindow::autoMatchCorrectionFrames);
    connect(m_annotationPanel,
            &AnnotationPanel::batchAddCorrectionsToFramesRequested,
            this,
            &MainWindow::batchAddCorrectionsToFrames);
    connect(m_annotationPanel, &AnnotationPanel::correctionToolChanged, this, [this](const QString& tool) {
        for (VideoWidget* widget : m_videoWidgets)
        {
            if (widget != nullptr)
            {
                widget->setCorrectionTool(tool);
            }
        }
    });
    connect(m_annotationPanel, &AnnotationPanel::correctionStyleChanged, this, [this](const QColor& color, int width) {
        for (VideoWidget* widget : m_videoWidgets)
        {
            if (widget != nullptr)
            {
                widget->setCorrectionStyle(color, width);
            }
        }
    });
    connect(m_annotationPanel, &AnnotationPanel::autoIdentifyRequested, this, &MainWindow::autoIdentifyCorrectionTargets);
    connect(m_annotationPanel, &AnnotationPanel::recordActivated, this, &MainWindow::jumpToRecordFrame);

    setStyleSheet(QStringLiteral(
        "QMainWindow { background:#101318; color:#e7edf3; }"
        "QWidget { color:#e7edf3; font-size:13px; }"
        "QGroupBox { border:1px solid #3a4350; border-radius:6px; margin-top:12px; padding-top:8px; }"
        "QGroupBox::title { subcontrol-origin: margin; left:10px; padding:0 4px; }"
        "QPushButton { background:#263241; border:1px solid #506070; border-radius:4px; padding:6px 10px; }"
        "QPushButton:checked { background:#1e6f9f; border-color:#4bd1ff; }"
        "QPushButton:hover { background:#334255; }"
        "QTextEdit, QListWidget, QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {"
        " background:#151b23; border:1px solid #3a4350; border-radius:4px; padding:3px; }"
        "QLabel { color:#d6dee8; }"));
}

void MainWindow::buildCameraPanel(QGridLayout* layout, int cameraIndex, int column)
{
    if (layout == nullptr || !isValidCameraIndex(cameraIndex))
    {
        return;
    }

    auto* panel = new QWidget(layout->parentWidget());
    auto* panelLayout = new QVBoxLayout(panel);
    panelLayout->setContentsMargins(0, 0, 0, 0);
    panelLayout->setSpacing(6);

    auto* header = new QHBoxLayout();
    header->setContentsMargins(0, 0, 0, 0);
    header->setSpacing(6);

    auto* selectButton = new QPushButton(QStringLiteral("摄像头 %1").arg(cameraIndex + 1), panel);
    selectButton->setCheckable(true);
    auto* videoButton = new QPushButton(QStringLiteral("导入视频"), panel);
    auto* algorithmButton = new QPushButton(QStringLiteral("导入处理代码"), panel);
    auto* syncSpin = new QSpinBox(panel);
    syncSpin->setRange(0, 0);
    syncSpin->setEnabled(false);
    syncSpin->setPrefix(QStringLiteral("同步帧 "));

    header->addWidget(selectButton);
    header->addWidget(videoButton);
    header->addWidget(algorithmButton);
    header->addWidget(syncSpin);
    panelLayout->addLayout(header);

    auto* videoWidget = new VideoWidget(panel);
    videoWidget->setText(QStringLiteral("导入摄像头 %1 视频").arg(cameraIndex + 1));
    auto* infoLabel = new QLabel(QStringLiteral("未导入视频"), panel);
    infoLabel->setWordWrap(true);
    infoLabel->setMinimumHeight(86);

    panelLayout->addWidget(videoWidget, 1);
    panelLayout->addWidget(infoLabel);
    layout->addWidget(panel, 0, column);

    m_videoWidgets[cameraIndex] = videoWidget;
    m_cameraInfoLabels[cameraIndex] = infoLabel;
    m_syncFrameSpins[cameraIndex] = syncSpin;
    m_cameraSelectButtons[cameraIndex] = selectButton;

    connect(selectButton, &QPushButton::clicked, this, [this, cameraIndex]() {
        selectCamera(cameraIndex);
    });
    connect(videoButton, &QPushButton::clicked, this, [this, cameraIndex]() {
        importCameraVideo(cameraIndex);
    });
    connect(algorithmButton, &QPushButton::clicked, this, [this, cameraIndex]() {
        importCameraAlgorithm(cameraIndex);
    });
    connect(syncSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this, cameraIndex](int value) {
        if (!isValidCameraIndex(cameraIndex))
        {
            return;
        }
        m_cameras[cameraIndex].syncFrame = value;
        const int minFrame = timelineMinFrame();
        const int maxFrame = timelineMaxFrame();
        m_timelineFrame = qBound(minFrame, m_timelineFrame, maxFrame);
        showFrame(m_timelineFrame);
    });
    connect(videoWidget, &VideoWidget::activated, this, [this, cameraIndex]() {
        selectCamera(cameraIndex);
    });
    connect(videoWidget,
            &VideoWidget::correctionShapeFinished,
            this,
            [this, cameraIndex](const QString& shapeType, const QVector<QPointF>& points) {
                selectCamera(cameraIndex);
                addCorrectionShape(shapeType, points);
            });
    connect(videoWidget, &VideoWidget::hoverPixelChanged, this, &MainWindow::updateHoverPixelInfo);
}

void MainWindow::buildMenus()
{
    QMenu* fileMenu = menuBar()->addMenu(QStringLiteral("文件"));
    fileMenu->addAction(QStringLiteral("同时导入三路视频"), this, &MainWindow::importAllCameraVideos);
    for (int i = 0; i < BEACON_CAMERA_COUNT; ++i)
    {
        fileMenu->addAction(QStringLiteral("导入摄像头 %1 视频").arg(i + 1), this, [this, i]() {
            importCameraVideo(i);
        });
    }
    fileMenu->addSeparator();
    for (int i = 0; i < BEACON_CAMERA_COUNT; ++i)
    {
        fileMenu->addAction(QStringLiteral("导入摄像头 %1 处理代码").arg(i + 1), this, [this, i]() {
            importCameraAlgorithm(i);
        });
    }
    fileMenu->addAction(QStringLiteral("导入三路融合分析代码"), this, &MainWindow::importFusionAlgorithm);
    fileMenu->addSeparator();
    fileMenu->addAction(QStringLiteral("保存当前摄像头标注"), this, &MainWindow::saveAnnotation);
    fileMenu->addAction(QStringLiteral("加载当前摄像头标注"), this, &MainWindow::loadAnnotation);
    fileMenu->addSeparator();
    fileMenu->addAction(QStringLiteral("导出当前摄像头标注视频"), this, &MainWindow::exportMarkedAvi);
    fileMenu->addAction(QStringLiteral("导出当前摄像头检测 CSV"), this, &MainWindow::exportCsv);

    QMenu* playbackMenu = menuBar()->addMenu(QStringLiteral("播放"));
    playbackMenu->addAction(QStringLiteral("播放"), this, &MainWindow::play);
    playbackMenu->addAction(QStringLiteral("暂停"), this, &MainWindow::pause);
    playbackMenu->addAction(QStringLiteral("上一帧"), this, &MainWindow::previousFrame);
    playbackMenu->addAction(QStringLiteral("下一帧"), this, &MainWindow::nextFrame);

    QMenu* viewMenu = menuBar()->addMenu(QStringLiteral("视图"));
    QAction* overlayAction = viewMenu->addAction(QStringLiteral("显示检测标注层"));
    overlayAction->setCheckable(true);
    overlayAction->setChecked(true);
    connect(overlayAction, &QAction::toggled, this, [this](bool checked) {
        m_showOverlayCheck->setChecked(checked);
    });
    connect(m_showOverlayCheck, &QCheckBox::toggled, overlayAction, &QAction::setChecked);
}

void MainWindow::importAllCameraVideos()
{
    const QStringList paths = QFileDialog::getOpenFileNames(this,
                                                            QStringLiteral("按摄像头 1/2/3 顺序选择三路视频"),
                                                            QString(),
                                                            QStringLiteral("视频文件 (*.avi *.mp4 *.mov *.mkv);;所有文件 (*)"));
    if (paths.isEmpty())
    {
        return;
    }
    if (paths.size() != BEACON_CAMERA_COUNT)
    {
        QMessageBox::warning(this,
                             QStringLiteral("导入三路视频"),
                             QStringLiteral("请一次选择 3 个视频文件，选择顺序对应摄像头 1、2、3。"));
        return;
    }

    for (int i = 0; i < BEACON_CAMERA_COUNT; ++i)
    {
        if (!loadCameraVideo(i, paths[i]))
        {
            return;
        }
    }
    showFrame(0);
}

bool MainWindow::isValidCameraIndex(int cameraIndex) const
{
    return cameraIndex >= 0 && cameraIndex < BEACON_CAMERA_COUNT;
}

CameraChannel* MainWindow::currentCamera()
{
    return isValidCameraIndex(m_currentCameraIndex) ? &m_cameras[m_currentCameraIndex] : nullptr;
}

const CameraChannel* MainWindow::currentCamera() const
{
    return isValidCameraIndex(m_currentCameraIndex) ? &m_cameras[m_currentCameraIndex] : nullptr;
}

void MainWindow::importCameraVideo(int cameraIndex)
{
    if (!isValidCameraIndex(cameraIndex))
    {
        return;
    }

    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("导入摄像头 %1 视频").arg(cameraIndex + 1),
                                                      QString(),
                                                      QStringLiteral("视频文件 (*.avi *.mp4 *.mov *.mkv);;所有文件 (*)"));
    if (path.isEmpty())
    {
        return;
    }

    loadCameraVideo(cameraIndex, path);
}

bool MainWindow::loadCameraVideo(int cameraIndex, const QString& path)
{
    if (!isValidCameraIndex(cameraIndex))
    {
        return false;
    }

    CameraChannel& camera = m_cameras[cameraIndex];
    QString error;
    if (!camera.reader.open(path, &error))
    {
        QMessageBox::warning(this, QStringLiteral("导入视频失败"), error);
        return false;
    }

    camera.videoPath = QFileInfo(path).absoluteFilePath();
    camera.usedFps = camera.reader.videoFps() > 0.0 ? camera.reader.videoFps() : 50.0;
    camera.syncFrame = 0;
    camera.loaded = true;
    camera.annotations.clear();
    std::memset(&camera.currentResult, 0, sizeof(camera.currentResult));
    camera.currentGray = QImage();

    QSpinBox* syncSpin = m_syncFrameSpins[cameraIndex];
    if (syncSpin != nullptr)
    {
        const QSignalBlocker blocker(syncSpin);
        syncSpin->setEnabled(true);
        syncSpin->setRange(0, qMax(0, camera.reader.frameCount() - 1));
        syncSpin->setValue(0);
    }

    if (camera.reader.width() != BEACON_IMAGE_W || camera.reader.height() != BEACON_IMAGE_H)
    {
        QMessageBox::warning(this,
                             QStringLiteral("视频尺寸不匹配"),
                             QStringLiteral("摄像头 %1 视频尺寸为 %2x%3，算法输入要求 %4x%5。仍会显示视频，但检测结果可能为空。")
                                 .arg(cameraIndex + 1)
                                 .arg(camera.reader.width())
                                 .arg(camera.reader.height())
                                 .arg(BEACON_IMAGE_W)
                                 .arg(BEACON_IMAGE_H));
    }

    selectCamera(cameraIndex);
    m_timelineFrame = qBound(timelineMinFrame(), m_timelineFrame, timelineMaxFrame());
    showFrame(m_timelineFrame);
    statusBar()->showMessage(QStringLiteral("已导入摄像头 %1 视频：%2").arg(cameraIndex + 1).arg(camera.videoPath), 3000);
    return true;
}

void MainWindow::importCameraAlgorithm(int cameraIndex)
{
    if (!isValidCameraIndex(cameraIndex))
    {
        return;
    }

    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("导入摄像头 %1 图像处理 C 文件").arg(cameraIndex + 1),
                                                      QString(),
                                                      QStringLiteral("C 文件 (*.c);;所有文件 (*)"));
    if (path.isEmpty())
    {
        return;
    }

    loadCameraAlgorithm(cameraIndex, path);
}

bool MainWindow::loadCameraAlgorithm(int cameraIndex, const QString& path)
{
    if (!isValidCameraIndex(cameraIndex))
    {
        return false;
    }

    CameraChannel& camera = m_cameras[cameraIndex];
    QString error;
    if (!camera.runner.loadSourceFile(path, cameraBuildDir(QStringLiteral("camera_%1").arg(cameraIndex + 1)), &error))
    {
        QMessageBox::warning(this, QStringLiteral("导入处理代码失败"), error);
        return false;
    }

    camera.algorithmPath = camera.runner.sourcePath();
    showFrame(m_timelineFrame);
    statusBar()->showMessage(QStringLiteral("已导入摄像头 %1 处理代码").arg(cameraIndex + 1), 3000);
    return true;
}

void MainWindow::importFusionAlgorithm()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("导入三摄像头融合分析 C 文件"),
                                                      QString(),
                                                      QStringLiteral("C 文件 (*.c);;所有文件 (*)"));
    if (path.isEmpty())
    {
        return;
    }

    QString error;
    if (!m_fusionRunner.loadSourceFile(path, cameraBuildDir(QStringLiteral("fusion")), &error))
    {
        QMessageBox::warning(this, QStringLiteral("导入融合分析代码失败"), error);
        return;
    }

    updateFusion();
    statusBar()->showMessage(QStringLiteral("已导入三路融合分析代码"), 3000);
}

bool MainWindow::hasAnyVideo() const
{
    return std::any_of(m_cameras.begin(), m_cameras.end(), [](const CameraChannel& camera) {
        return camera.loaded && camera.reader.isOpen();
    });
}

bool MainWindow::hasAllVideos() const
{
    return std::all_of(m_cameras.begin(), m_cameras.end(), [](const CameraChannel& camera) {
        return camera.loaded && camera.reader.isOpen();
    });
}

int MainWindow::timelineMinFrame() const
{
    bool found = false;
    int minFrame = 0;
    for (const CameraChannel& camera : m_cameras)
    {
        if (!camera.loaded || !camera.reader.isOpen())
        {
            continue;
        }

        const int cameraMin = -camera.syncFrame;
        minFrame = found ? qMax(minFrame, cameraMin) : cameraMin;
        found = true;
    }
    return found ? minFrame : 0;
}

int MainWindow::timelineMaxFrame() const
{
    bool found = false;
    int maxFrame = 0;
    for (const CameraChannel& camera : m_cameras)
    {
        if (!camera.loaded || !camera.reader.isOpen())
        {
            continue;
        }

        const int cameraMax = camera.reader.frameCount() - 1 - camera.syncFrame;
        maxFrame = found ? qMin(maxFrame, cameraMax) : cameraMax;
        found = true;
    }
    return found ? qMax(timelineMinFrame(), maxFrame) : 0;
}

int MainWindow::timelineFrameCount() const
{
    return timelineMaxFrame() - timelineMinFrame() + 1;
}

int MainWindow::cameraFrameForTimeline(const CameraChannel& camera, int timelineFrame) const
{
    return camera.syncFrame + timelineFrame;
}

void MainWindow::showFrame(int frameIndex)
{
    if (!hasAnyVideo())
    {
        updateFrameInfo();
        updateAllCameraInfo();
        updateFusion();
        return;
    }

    const int minFrame = timelineMinFrame();
    const int maxFrame = timelineMaxFrame();
    m_timelineFrame = qBound(minFrame, frameIndex, maxFrame);

    for (CameraChannel& camera : m_cameras)
    {
        if (!camera.loaded || !camera.reader.isOpen())
        {
            continue;
        }

        const int cameraFrame = cameraFrameForTimeline(camera, m_timelineFrame);
        if (cameraFrame < 0 || cameraFrame >= camera.reader.frameCount())
        {
            camera.currentGray = QImage();
            std::memset(&camera.currentResult, 0, sizeof(camera.currentResult));
            continue;
        }

        QImage gray;
        QString error;
        if (!camera.reader.readFrame(cameraFrame, &gray, &error))
        {
            statusBar()->showMessage(QStringLiteral("%1 读取帧失败：%2").arg(camera.name, error), 3000);
            camera.currentGray = QImage();
            std::memset(&camera.currentResult, 0, sizeof(camera.currentResult));
            continue;
        }

        camera.currentGray = gray;
        camera.currentResult = camera.runner.process(gray);
    }

    refreshCurrentCameraUi();
    for (CameraChannel& camera : m_cameras)
    {
        renderCamera(&camera);
    }
    updateFusion();
    updateAllCameraInfo();
    updateFrameInfo();

    m_updatingControls = true;
    m_slider->setRange(minFrame, maxFrame);
    m_slider->setValue(m_timelineFrame);
    m_frameSpin->setRange(minFrame, maxFrame);
    m_frameSpin->setValue(m_timelineFrame);
    m_timeSpin->setRange(frameTime(minFrame), frameTime(maxFrame));
    m_timeSpin->setValue(frameTime(m_timelineFrame));
    m_updatingControls = false;
}

void MainWindow::renderCamera(CameraChannel* camera)
{
    if (camera == nullptr || !isValidCameraIndex(camera->index))
    {
        return;
    }

    VideoWidget* widget = m_videoWidgets[camera->index];
    if (widget == nullptr)
    {
        return;
    }

    if (!camera->loaded || camera->currentGray.isNull())
    {
        widget->setImage(QImage());
        widget->setPixelSourceImage(QImage());
        return;
    }

    QImage displayImage = camera->currentGray;
    if (viewMode() == QStringLiteral("binary"))
    {
        const QImage binary = camera->runner.binaryImage(camera->currentGray);
        if (!binary.isNull())
        {
            displayImage = binary;
        }
    }

    QVector<CorrectionShape> corrections =
        camera->annotations.correctionsForFrame(cameraFrameForTimeline(*camera, m_timelineFrame));
    if (camera->index == m_currentCameraIndex && m_annotationPanel != nullptr)
    {
        corrections = m_annotationPanel->draftCorrections();
    }

    const QImage rendered = FrameRenderer::render(displayImage,
                                                  camera->currentResult,
                                                  corrections,
                                                  1,
                                                  m_showOverlay);
    widget->setFrameGeometry(QSize(camera->reader.width(), camera->reader.height()), 1);
    widget->setPixelSourceImage(camera->currentGray);
    widget->setImage(rendered);
    widget->setSelected(camera->index == m_currentCameraIndex);
}

void MainWindow::updateFusion()
{
    beacon_result_t cameraResults[BEACON_CAMERA_COUNT];
    std::memset(cameraResults, 0, sizeof(cameraResults));
    for (int i = 0; i < BEACON_CAMERA_COUNT; ++i)
    {
        if (m_cameras[i].loaded)
        {
            cameraResults[i] = m_cameras[i].currentResult;
        }
    }

    m_fusionResult = m_fusionRunner.analyze(cameraResults);
    if (m_radarWidget != nullptr)
    {
        m_radarWidget->setResult(m_fusionResult);
    }

    if (m_fusionText == nullptr)
    {
        return;
    }

    QStringList lines;
    lines << QStringLiteral("融合后信标灯总数：%1").arg((int)m_fusionResult.beacon_count);
    lines << QStringLiteral("观测总数：%1  最优目标：%2  更新次数：%3")
                 .arg((int)m_fusionResult.observation_count)
                 .arg(m_fusionResult.best_index < BEACON_FUSION_MAX_BEACONS
                          ? QStringLiteral("#%1").arg((int)m_fusionResult.best_index)
                          : QStringLiteral("-"))
                 .arg((unsigned int)m_fusionResult.update_count);
    if (!hasAllVideos())
    {
        lines << QStringLiteral("提示：尚未导入全部三路视频，融合结果只使用已导入摄像头的当前帧。");
    }
    if (!m_fusionRunner.usesDynamicLibrary())
    {
        lines << QStringLiteral("融合代码：使用内置默认分析逻辑");
    }
    else
    {
        lines << QStringLiteral("融合代码：%1").arg(m_fusionRunner.sourcePath());
    }
    lines << QString();
    for (int i = 0; i < m_fusionResult.beacon_count && i < BEACON_FUSION_MAX_BEACONS; ++i)
    {
        const beacon_fusion_beacon_t& beacon = m_fusionResult.beacon[i];
        if (beacon.valid == 0)
        {
            continue;
        }
        lines << QStringLiteral("信标 #%1").arg(i);
        lines << QStringLiteral("  角度 bearing_deg：%1").arg(beacon.bearing_deg, 0, 'f', 2);
        lines << QStringLiteral("  距离 range_proxy：%1").arg(beacon.range_proxy, 0, 'f', 2);
        lines << QStringLiteral("  车体 X x_body：%1").arg(beacon.x_body, 0, 'f', 2);
        lines << QStringLiteral("  车体 Y y_body：%1").arg(beacon.y_body, 0, 'f', 2);
        lines << QStringLiteral("  控制 X control_x：%1").arg(beacon.control_x, 0, 'f', 2);
        lines << QStringLiteral("  控制 Y control_y：%1").arg(beacon.control_y, 0, 'f', 2);
        lines << QStringLiteral("  观测数 observations：%1").arg((int)beacon.observation_count);
        lines << QStringLiteral("  摄像头掩码 camera_mask：0x%1").arg((int)beacon.source_camera_mask, 0, 16);
        lines << QStringLiteral("  置信度 confidence：%1").arg(beacon.confidence, 0, 'f', 2);
        lines << QStringLiteral("  稳定帧 stable_ticks：%1").arg((int)beacon.stable_ticks);
        lines << QString();
    }

    m_fusionText->setPlainText(lines.join(QLatin1Char('\n')));
}

void MainWindow::updateCameraInfo(const CameraChannel& camera)
{
    if (!isValidCameraIndex(camera.index) || m_cameraInfoLabels[camera.index] == nullptr)
    {
        return;
    }

    QLabel* label = m_cameraInfoLabels[camera.index];
    if (!camera.loaded || !camera.reader.isOpen())
    {
        label->setText(QStringLiteral("未导入视频"));
        return;
    }

    const int frame = cameraFrameForTimeline(camera, m_timelineFrame);
    if (frame < 0 || frame >= camera.reader.frameCount())
    {
        label->setText(QStringLiteral("帧：%1 超出视频范围 0-%2\n当前同步设置下无可显示帧")
                           .arg(frame)
                           .arg(qMax(0, camera.reader.frameCount() - 1)));
        return;
    }

    QStringList lines;
    lines << QStringLiteral("帧：%1 / %2").arg(frame).arg(qMax(0, camera.reader.frameCount() - 1));
    lines << QStringLiteral("识别数量：%1").arg(validCircleCount(camera.currentResult));
    for (int i = 0; i < camera.currentResult.count && i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        const beacon_circle_t& circle = camera.currentResult.circles[i];
        if (circle.valid == 0)
        {
            continue;
        }
        lines << QStringLiteral("#%1  X=%2  Y=%3  面积=%4")
                     .arg(i)
                     .arg(circle.x, 0, 'f', 2)
                     .arg(circle.y, 0, 'f', 2)
                     .arg(circleArea(circle), 0, 'f', 2);
    }
    if (camera.currentResult.count == 0)
    {
        lines << QStringLiteral("当前帧无有效信标灯");
    }
    label->setText(lines.join(QLatin1Char('\n')));
}

void MainWindow::updateAllCameraInfo()
{
    for (const CameraChannel& camera : m_cameras)
    {
        updateCameraInfo(camera);
    }
}

void MainWindow::updateFrameInfo()
{
    QStringList videoLines;
    for (const CameraChannel& camera : m_cameras)
    {
        if (!camera.loaded || !camera.reader.isOpen())
        {
            videoLines << QStringLiteral("%1：未导入").arg(camera.name);
            continue;
        }
        videoLines << QStringLiteral("%1：%2x%3，%4 帧，FPS %5，后端 %6，同步帧 %7")
                          .arg(camera.name)
                          .arg(camera.reader.width())
                          .arg(camera.reader.height())
                          .arg(camera.reader.frameCount())
                          .arg(camera.usedFps, 0, 'f', 3)
                          .arg(camera.reader.backendName())
                          .arg(camera.syncFrame);
    }
    m_videoInfoLabel->setText(videoLines.join(QStringLiteral("  |  ")));

    QStringList frameLines;
    frameLines << QStringLiteral("同步时间轴帧：%1，范围：%2 至 %3，共 %4 帧")
                      .arg(m_timelineFrame)
                      .arg(timelineMinFrame())
                      .arg(timelineMaxFrame())
                      .arg(timelineFrameCount());
    for (const CameraChannel& camera : m_cameras)
    {
        if (camera.loaded)
        {
            frameLines << QStringLiteral("%1 当前帧：%2").arg(camera.name).arg(cameraFrameForTimeline(camera, m_timelineFrame));
        }
    }
    m_frameInfoLabel->setText(frameLines.join(QStringLiteral("  |  ")));
}

void MainWindow::refreshCurrentCameraUi()
{
    for (int i = 0; i < BEACON_CAMERA_COUNT; ++i)
    {
        if (m_cameraSelectButtons[i] != nullptr)
        {
            const QSignalBlocker blocker(m_cameraSelectButtons[i]);
            m_cameraSelectButtons[i]->setChecked(i == m_currentCameraIndex);
        }
        if (m_videoWidgets[i] != nullptr)
        {
            m_videoWidgets[i]->setSelected(i == m_currentCameraIndex);
        }
    }

    CameraChannel* camera = currentCamera();
    if (camera == nullptr || m_annotationPanel == nullptr)
    {
        return;
    }

    if (!camera->loaded || !camera->reader.isOpen())
    {
        m_annotationPanel->setVideoFrameRange(0, 0);
        m_annotationPanel->setCurrentContext(0, 0.0, 0);
        m_annotationPanel->setCurrentFrameCorrections({}, true);
        m_annotationPanel->setAnnotations({}, {});
        return;
    }

    const int frame = cameraFrameForTimeline(*camera, m_timelineFrame);
    m_annotationPanel->setVideoFrameRange(0, qMax(0, camera->reader.frameCount() - 1));
    m_annotationPanel->setCurrentContext(frame, (double)frame / camera->usedFps, validCircleCount(camera->currentResult));
    m_annotationPanel->setCurrentFrameCorrections(camera->annotations.correctionsForFrame(frame), true);
    m_annotationPanel->setAnnotations(camera->annotations.records(), camera->annotations.corrections());
}

void MainWindow::selectCamera(int cameraIndex)
{
    if (!isValidCameraIndex(cameraIndex))
    {
        return;
    }

    m_currentCameraIndex = cameraIndex;
    refreshCurrentCameraUi();
    for (CameraChannel& camera : m_cameras)
    {
        renderCamera(&camera);
    }
}

void MainWindow::saveAnnotation()
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        QMessageBox::information(this, QStringLiteral("保存标注"), QStringLiteral("请先导入当前摄像头的视频。"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("保存当前摄像头标注"),
                                                      annotationPathForCamera(camera->index),
                                                      QStringLiteral("JSON 文件 (*.json);;所有文件 (*)"));
    if (path.isEmpty())
    {
        return;
    }

    AnnotationVideoInfo info;
    info.file = camera->videoPath;
    info.width = camera->reader.width();
    info.height = camera->reader.height();
    info.fpsUsed = camera->usedFps;
    info.frameCount = camera->reader.frameCount();

    QString error;
    if (!AnnotationJson::save(path, info, camera->annotations, &error))
    {
        QMessageBox::warning(this, QStringLiteral("保存标注失败"), error);
        return;
    }
    statusBar()->showMessage(QStringLiteral("标注已保存：%1").arg(path), 3000);
}

void MainWindow::loadAnnotation()
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        QMessageBox::information(this, QStringLiteral("加载标注"), QStringLiteral("请先导入当前摄像头的视频。"));
        return;
    }

    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("加载当前摄像头标注"),
                                                      QFileInfo(annotationPathForCamera(camera->index)).absolutePath(),
                                                      QStringLiteral("JSON 文件 (*.json);;所有文件 (*)"));
    if (path.isEmpty())
    {
        return;
    }

    QString error;
    if (!AnnotationJson::load(path, &camera->annotations, &error))
    {
        QMessageBox::warning(this, QStringLiteral("加载标注失败"), error);
        return;
    }
    showFrame(m_timelineFrame);
    statusBar()->showMessage(QStringLiteral("标注已加载：%1").arg(path), 3000);
}

void MainWindow::exportMarkedAvi()
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        QMessageBox::information(this, QStringLiteral("导出视频"), QStringLiteral("请先导入当前摄像头的视频。"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("导出当前摄像头标注视频"),
                                                      defaultOutputPath(camera->index, QStringLiteral("_marked.avi")),
                                                      QStringLiteral("AVI 文件 (*.avi);;所有文件 (*)"));
    if (path.isEmpty())
    {
        return;
    }

    QProgressDialog progress(QStringLiteral("正在导出 AVI..."), QStringLiteral("取消"), 0, camera->reader.frameCount(), this);
    progress.setWindowModality(Qt::WindowModal);

    VideoExporter exporter;
    QString error;
    const bool ok = exporter.exportMarkedAvi(camera->videoPath,
                                             path,
                                             camera->usedFps,
                                             &camera->runner,
                                             &camera->annotations,
                                             [&progress](int current, int total) {
                                                 progress.setRange(0, total);
                                                 progress.setValue(current);
                                                 qApp->processEvents();
                                                 return !progress.wasCanceled();
                                             },
                                             &error);
    if (!ok)
    {
        QMessageBox::warning(this, QStringLiteral("导出 AVI 失败"), error);
        return;
    }
    statusBar()->showMessage(QStringLiteral("AVI 已导出：%1").arg(path), 3000);
}

void MainWindow::exportCsv()
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        QMessageBox::information(this, QStringLiteral("导出 CSV"), QStringLiteral("请先导入当前摄像头的视频。"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("导出当前摄像头检测 CSV"),
                                                      defaultOutputPath(camera->index, QStringLiteral("_result.csv")),
                                                      QStringLiteral("CSV 文件 (*.csv);;所有文件 (*)"));
    if (path.isEmpty())
    {
        return;
    }

    QProgressDialog progress(QStringLiteral("正在导出 CSV..."), QStringLiteral("取消"), 0, camera->reader.frameCount(), this);
    progress.setWindowModality(Qt::WindowModal);

    VideoExporter exporter;
    QString error;
    const bool ok = exporter.exportResultCsv(camera->videoPath,
                                             path,
                                             camera->usedFps,
                                             &camera->runner,
                                             [&progress](int current, int total) {
                                                 progress.setRange(0, total);
                                                 progress.setValue(current);
                                                 qApp->processEvents();
                                                 return !progress.wasCanceled();
                                             },
                                             &error);
    if (!ok)
    {
        QMessageBox::warning(this, QStringLiteral("导出 CSV 失败"), error);
        return;
    }
    statusBar()->showMessage(QStringLiteral("CSV 已导出：%1").arg(path), 3000);
}

void MainWindow::play()
{
    if (!hasAnyVideo())
    {
        return;
    }
    m_playing = true;
    resetPlaybackClock();
    m_playTimer.start(playbackIntervalMs());
    updatePlayPauseButton();
}

void MainWindow::pause()
{
    m_playing = false;
    m_playTimer.stop();
    updatePlayPauseButton();
}

void MainWindow::nextFrame()
{
    pause();
    showFrame(qMin(timelineMaxFrame(), m_timelineFrame + 1));
}

void MainWindow::previousFrame()
{
    pause();
    showFrame(qMax(timelineMinFrame(), m_timelineFrame - 1));
}

void MainWindow::jumpToFrame()
{
    if (m_updatingControls)
    {
        return;
    }
    pause();
    showFrame(m_frameSpin->value());
}

void MainWindow::jumpToTime()
{
    if (m_updatingControls)
    {
        return;
    }
    const double fps = currentCamera() != nullptr && currentCamera()->usedFps > 0.0
        ? currentCamera()->usedFps
        : 50.0;
    pause();
    showFrame((int)std::llround(m_timeSpin->value() * fps));
}

void MainWindow::showFrameFromSlider(int value)
{
    pause();
    showFrame(value);
}

void MainWindow::saveCurrentFrameCorrections(const QVector<CorrectionShape>& corrections)
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        return;
    }

    const int frame = cameraFrameForTimeline(*camera, m_timelineFrame);
    camera->annotations.removeCorrectionsForFrame(frame);
    for (CorrectionShape correction : corrections)
    {
        correction.frame = frame;
        camera->annotations.addCorrection(correction);
    }

    refreshCurrentCameraUi();
    renderCamera(camera);
    updateAnnotationList();
    statusBar()->showMessage(QStringLiteral("已保存当前帧纠错标注"), 2000);
}

void MainWindow::batchAddCorrections(const QVector<int>& correctionRows,
                                     int startFrame,
                                     int endFrame,
                                     double overlapPixelThreshold)
{
    Q_UNUSED(overlapPixelThreshold);

    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        return;
    }

    const QVector<CorrectionShape>& allCorrections = camera->annotations.corrections();
    if (startFrame > endFrame)
    {
        std::swap(startFrame, endFrame);
    }

    int added = 0;
    for (int row : correctionRows)
    {
        if (row < 0 || row >= allCorrections.size())
        {
            continue;
        }

        const CorrectionShape source = allCorrections[row];
        for (int frame = qMax(0, startFrame); frame <= endFrame && frame < camera->reader.frameCount(); ++frame)
        {
            CorrectionShape copy = source;
            copy.frame = frame;
            camera->annotations.addCorrection(copy);
            ++added;
        }
    }

    showFrame(m_timelineFrame);
    statusBar()->showMessage(QStringLiteral("批量添加纠错 %1 条").arg(added), 3000);
}

void MainWindow::autoMatchCorrectionFrames(const QVector<int>& correctionRows,
                                           int backwardMaxFrames,
                                           int forwardMaxFrames,
                                           double positionThreshold,
                                           double overlapPixelThreshold)
{
    Q_UNUSED(correctionRows);
    Q_UNUSED(positionThreshold);
    Q_UNUSED(overlapPixelThreshold);

    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        return;
    }

    const int currentFrame = cameraFrameForTimeline(*camera, m_timelineFrame);
    QVector<int> frames;
    const int startFrame = qMax(0, currentFrame - qMax(0, backwardMaxFrames));
    const int endFrame = qMin(camera->reader.frameCount() - 1, currentFrame + qMax(0, forwardMaxFrames));
    for (int frame = startFrame; frame <= endFrame; ++frame)
    {
        if (frame != currentFrame)
        {
            frames.push_back(frame);
        }
    }

    m_annotationPanel->setAutoMatchedBatchFrames(frames);
    statusBar()->showMessage(QStringLiteral("已生成批量候选帧：%1 帧").arg(frames.size()), 3000);
}

void MainWindow::batchAddCorrectionsToFrames(const QVector<int>& correctionRows, const QVector<int>& frames)
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        return;
    }

    const QVector<CorrectionShape>& allCorrections = camera->annotations.corrections();
    int added = 0;
    for (int row : correctionRows)
    {
        if (row < 0 || row >= allCorrections.size())
        {
            continue;
        }

        const CorrectionShape source = allCorrections[row];
        for (int frame : frames)
        {
            if (frame < 0 || frame >= camera->reader.frameCount())
            {
                continue;
            }

            CorrectionShape copy = source;
            copy.frame = frame;
            camera->annotations.addCorrection(copy);
            ++added;
        }
    }

    showFrame(m_timelineFrame);
    statusBar()->showMessage(QStringLiteral("已向匹配帧添加纠错 %1 条").arg(added), 3000);
}

void MainWindow::deleteAnnotation(int row)
{
    CameraChannel* camera = currentCamera();
    if (camera != nullptr && camera->annotations.removeAt(row))
    {
        updateAnnotationList();
    }
}

void MainWindow::deleteCorrection(int row)
{
    CameraChannel* camera = currentCamera();
    if (camera != nullptr && camera->annotations.removeCorrectionAt(row))
    {
        showFrame(m_timelineFrame);
    }
}

void MainWindow::deleteAnnotations(const QVector<int>& rows)
{
    QVector<int> sorted = rows;
    std::sort(sorted.begin(), sorted.end(), std::greater<int>());
    for (int row : sorted)
    {
        deleteAnnotation(row);
    }
}

void MainWindow::deleteCorrections(const QVector<int>& rows)
{
    QVector<int> sorted = rows;
    std::sort(sorted.begin(), sorted.end(), std::greater<int>());
    for (int row : sorted)
    {
        deleteCorrection(row);
    }
}

void MainWindow::addCorrectionShape(const QString& shapeType, const QVector<QPointF>& points)
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded || points.isEmpty())
    {
        return;
    }

    m_annotationPanel->applyDrawnCorrectionShape(shapeType, points);
    renderCamera(camera);
}

void MainWindow::autoIdentifyCorrectionTargets()
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        return;
    }

    CorrectionShape shape;
    if (!m_annotationPanel->activeDraftShape(&shape))
    {
        QMessageBox::information(this, QStringLiteral("自动识别"), QStringLiteral("请先绘制一个封闭纠错图形。"));
        return;
    }

    QVector<int> matchedIndices;
    for (int i = 0; i < camera->currentResult.count && i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        const beacon_circle_t& circle = camera->currentResult.circles[i];
        if (circle.valid == 0)
        {
            continue;
        }

        const QPointF imagePoint = FrameRenderer::algorithmToImagePoint(circle.x, circle.y);
        bool contains = false;
        if (shape.shapeType == QStringLiteral("circle") && shape.points.size() >= 2)
        {
            contains = QLineF(shape.points[0], imagePoint).length() <= QLineF(shape.points[0], shape.points[1]).length();
        }
        else if (shape.shapeType == QStringLiteral("rect") && shape.points.size() >= 2)
        {
            contains = QRectF(shape.points[0], shape.points[1]).normalized().contains(imagePoint);
        }
        else if (shape.shapeType == QStringLiteral("polygon") && shape.points.size() >= 3)
        {
            contains = QPolygonF(shape.points).containsPoint(imagePoint, Qt::OddEvenFill);
        }

        if (contains)
        {
            matchedIndices.push_back(i);
        }
    }

    if (matchedIndices.isEmpty())
    {
        QMessageBox::information(this, QStringLiteral("自动识别"), QStringLiteral("封闭区域内没有已识别信标灯。"));
        return;
    }

    m_annotationPanel->applyAutoIdentifiedErrorCircles(matchedIndices);
    renderCamera(camera);
}

void MainWindow::jumpToRecordFrame(int frame)
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || !camera->loaded)
    {
        return;
    }

    pause();
    showFrame(frame - camera->syncFrame);
}

void MainWindow::updateHoverPixelInfo(int x, int y, int gray, bool valid)
{
    if (!valid)
    {
        m_pixelInfoLabel->setText(QStringLiteral("像素：-"));
        return;
    }
    m_pixelInfoLabel->setText(QStringLiteral("像素：X=%1  Y=%2  Gray=%3").arg(x).arg(y).arg(gray));
}

void MainWindow::updateAnnotationList()
{
    CameraChannel* camera = currentCamera();
    if (camera == nullptr || m_annotationPanel == nullptr)
    {
        return;
    }
    m_annotationPanel->setAnnotations(camera->annotations.records(), camera->annotations.corrections());
}

void MainWindow::togglePlayPause()
{
    if (m_playing)
    {
        pause();
    }
    else
    {
        play();
    }
}

void MainWindow::updatePlayPauseButton()
{
    if (m_playPauseButton == nullptr)
    {
        return;
    }
    m_playPauseButton->setIcon(style()->standardIcon(m_playing ? QStyle::SP_MediaPause : QStyle::SP_MediaPlay));
}

void MainWindow::setPlaybackSpeed(double speed)
{
    m_playbackSpeed = qBound(0.125, speed, 8.0);
    if (m_playing)
    {
        resetPlaybackClock();
        m_playTimer.start(playbackIntervalMs());
    }
}

int MainWindow::playbackIntervalMs() const
{
    const double fps = currentCamera() != nullptr && currentCamera()->usedFps > 0.0
        ? currentCamera()->usedFps
        : 50.0;
    return qBound(1, (int)std::floor(1000.0 / qMax(1.0, fps * m_playbackSpeed)), 100);
}

void MainWindow::resetPlaybackClock()
{
    m_playbackStartFrame = m_timelineFrame;
    m_playbackClock.restart();
}

double MainWindow::frameTime(int frame) const
{
    const double fps = currentCamera() != nullptr && currentCamera()->usedFps > 0.0
        ? currentCamera()->usedFps
        : 50.0;
    return (double)frame / fps;
}

QString MainWindow::viewMode() const
{
    return m_viewModeCombo != nullptr
        ? m_viewModeCombo->currentData().toString()
        : QStringLiteral("original");
}

QString MainWindow::defaultOutputPath(int cameraIndex, const QString& suffix) const
{
    if (!isValidCameraIndex(cameraIndex) || m_cameras[cameraIndex].videoPath.isEmpty())
    {
        return QString();
    }

    const QFileInfo info(m_cameras[cameraIndex].videoPath);
    return info.absolutePath() + QLatin1Char('/') + info.completeBaseName() + suffix;
}

QString MainWindow::annotationPathForCamera(int cameraIndex) const
{
    if (!isValidCameraIndex(cameraIndex) || m_cameras[cameraIndex].videoPath.isEmpty())
    {
        return QString();
    }

    const QFileInfo info(m_cameras[cameraIndex].videoPath);
    return info.absolutePath() + QLatin1Char('/') +
           info.completeBaseName() +
           QStringLiteral("_camera%1_annotations.json").arg(cameraIndex + 1);
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event)
{
    Q_UNUSED(watched);
    if (event->type() != QEvent::KeyPress || qApp->activeWindow() != this)
    {
        return false;
    }

    auto* keyEvent = static_cast<QKeyEvent*>(event);
    if (isEditingWidget(qApp->focusWidget()))
    {
        return false;
    }

    if (keyEvent->key() == Qt::Key_Space && !keyEvent->isAutoRepeat())
    {
        togglePlayPause();
        return true;
    }
    if (keyEvent->key() == Qt::Key_Left)
    {
        previousFrame();
        return true;
    }
    if (keyEvent->key() == Qt::Key_Right)
    {
        nextFrame();
        return true;
    }
    if (keyEvent->key() >= Qt::Key_1 && keyEvent->key() <= Qt::Key_3)
    {
        selectCamera(keyEvent->key() - Qt::Key_1);
        return true;
    }
    return false;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    pause();
    QMainWindow::closeEvent(event);
}
