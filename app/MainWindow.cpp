#include "MainWindow.h"

#include "AnnotationJson.h"
#include "AnnotationPanel.h"
#include "FrameRenderer.h"
#include "VideoExporter.h"
#include "VideoWidget.h"

#include <QAction>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QDesktopServices>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressDialog>
#include <QPushButton>
#include <QSettings>
#include <QScrollArea>
#include <QSlider>
#include <QSizePolicy>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QTextEdit>
#include <QToolButton>
#include <QUrl>
#include <QVBoxLayout>

namespace
{
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

QJsonArray pointsToJson(const QVector<QPointF>& points)
{
    QJsonArray array;
    for (const QPointF& point : points)
    {
        QJsonObject object;
        object.insert(QStringLiteral("x"), point.x());
        object.insert(QStringLiteral("y"), point.y());
        array.append(object);
    }
    return array;
}

QString djiStyleSheet()
{
    return QStringLiteral(R"(
QMainWindow,
QWidget#AppRoot {
    background: #090a0b;
    color: #eef1f4;
}

QWidget {
    color: #eef1f4;
    font-family: "Microsoft YaHei UI", "Segoe UI", Arial;
    font-size: 12px;
}

QMenuBar {
    background: #101214;
    color: #d8dde2;
    border-bottom: 1px solid #252a2f;
    padding: 3px 8px;
}

QMenuBar::item {
    background: transparent;
    padding: 6px 10px;
    border-radius: 4px;
}

QMenuBar::item:selected {
    background: #24282d;
}

QMenu {
    background: #171a1d;
    color: #eef1f4;
    border: 1px solid #343a40;
}

QMenu::item {
    padding: 7px 24px;
}

QMenu::item:selected {
    background: #2b3036;
}

QFrame#Rail {
    background: #151719;
    border: 1px solid #262b30;
    border-radius: 8px;
}

QLabel#Brand {
    background: #f2f4f5;
    color: #08090a;
    border-radius: 4px;
    font-weight: 800;
    font-size: 12px;
}

QToolButton#RailButton {
    background: transparent;
    border: 1px solid transparent;
    border-radius: 7px;
    padding: 7px;
}

QToolButton#RailButton:hover,
QToolButton#RailButton:checked {
    background: #292e33;
    border-color: #3b4249;
}

QSplitter::handle {
    background: #090a0b;
}

QFrame#VideoCard,
QFrame#ControlConsole,
QGroupBox {
    background: #171a1d;
    border: 1px solid #2a3035;
    border-radius: 8px;
}

QFrame#VideoCard {
    background: #0e1012;
}

QLabel#FeedTitle,
QLabel#ConsoleTitle {
    color: #f4f6f7;
    font-weight: 800;
    letter-spacing: 1px;
}

QLabel#FeedMeta,
QLabel#SoftLabel {
    color: #a7afb7;
}

QLabel#Pill {
    background: #24292e;
    color: #f6d44a;
    border: 1px solid #3a4148;
    border-radius: 6px;
    padding: 6px 10px;
    font-weight: 700;
}

QGroupBox {
    margin-top: 18px;
    padding-top: 18px;
    font-weight: 700;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 6px;
    color: #cfd5db;
}

QPushButton {
    background: #24292e;
    border: 1px solid #3a4148;
    border-radius: 6px;
    color: #eef1f4;
    min-height: 28px;
    padding: 5px 12px;
}

QPushButton:hover {
    background: #30363d;
    border-color: #515b64;
}

QPushButton:pressed {
    background: #1d2226;
}

QPushButton[role="primary"] {
    background: #f6d44a;
    border-color: #f6d44a;
    color: #111315;
    font-weight: 800;
}

QComboBox,
QSpinBox,
QDoubleSpinBox,
QTextEdit,
QListWidget {
    background: #101214;
    border: 1px solid #343b42;
    border-radius: 6px;
    color: #eef1f4;
    selection-background-color: #f6d44a;
    selection-color: #111315;
}

QComboBox,
QSpinBox,
QDoubleSpinBox {
    min-height: 28px;
    padding: 2px 8px;
}

QTextEdit,
QListWidget {
    padding: 6px;
}

QComboBox::drop-down {
    border-left: 1px solid #343b42;
    width: 22px;
}

QSlider::groove:horizontal {
    height: 4px;
    background: #30363d;
    border-radius: 2px;
}

QSlider::sub-page:horizontal {
    background: #d5d9dd;
    border-radius: 2px;
}

QSlider::handle:horizontal {
    background: #f6d44a;
    border: 2px solid #111315;
    width: 16px;
    height: 16px;
    margin: -7px 0;
    border-radius: 8px;
}

QScrollArea,
QWidget#RightPanel,
QWidget#Workspace {
    background: transparent;
}

QScrollBar:vertical {
    background: #111315;
    width: 10px;
    margin: 0;
}

QScrollBar::handle:vertical {
    background: #343b42;
    border-radius: 5px;
    min-height: 28px;
}

QStatusBar {
    background: #101214;
    color: #aeb6be;
    border-top: 1px solid #252a2f;
}
)");
}

}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    buildUi();
    buildMenus();
    setWindowTitle(QStringLiteral("BeaconImageAnalyzer"));
    setMinimumSize(1120, 680);
    resize(1320, 780);

    m_playTimer.setInterval(20);
    connect(&m_playTimer, &QTimer::timeout, this, &MainWindow::nextFrame);
    qApp->installEventFilter(this);
    QTimer::singleShot(0, this, &MainWindow::restoreLastSession);
}

void MainWindow::buildUi()
{
    auto* central = new QWidget(this);
    central->setObjectName(QStringLiteral("AppRoot"));
    auto* root = new QHBoxLayout(central);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(12);

    auto* rail = new QFrame(central);
    rail->setObjectName(QStringLiteral("Rail"));
    rail->setFixedWidth(64);
    auto* railLayout = new QVBoxLayout(rail);
    railLayout->setContentsMargins(10, 12, 10, 12);
    railLayout->setSpacing(12);

    auto* brand = new QLabel(QStringLiteral("DJI"), rail);
    brand->setObjectName(QStringLiteral("Brand"));
    brand->setAlignment(Qt::AlignCenter);
    brand->setFixedSize(40, 28);
    railLayout->addWidget(brand, 0, Qt::AlignHCenter);
    railLayout->addSpacing(10);

    auto makeRailButton = [this](QWidget* parent, QStyle::StandardPixmap icon, const QString& tooltip) {
        auto* button = new QToolButton(parent);
        button->setObjectName(QStringLiteral("RailButton"));
        button->setIcon(style()->standardIcon(icon));
        button->setIconSize(QSize(20, 20));
        button->setFixedSize(40, 40);
        button->setToolTip(tooltip);
        button->setAutoRaise(false);
        return button;
    };

    auto* openRailButton = makeRailButton(rail, QStyle::SP_DialogOpenButton, QStringLiteral("打开 AVI"));
    auto* saveRailButton = makeRailButton(rail, QStyle::SP_DialogSaveButton, QStringLiteral("保存标注"));
    auto* loadRailButton = makeRailButton(rail, QStyle::SP_FileDialogContentsView, QStringLiteral("读取标注"));
    auto* exportAviRailButton = makeRailButton(rail, QStyle::SP_DialogApplyButton, QStringLiteral("导出标注 AVI"));
    auto* exportCsvRailButton = makeRailButton(rail, QStyle::SP_FileIcon, QStringLiteral("导出 CSV"));
    auto* exitRailButton = makeRailButton(rail, QStyle::SP_DialogCloseButton, QStringLiteral("退出"));
    railLayout->addWidget(openRailButton, 0, Qt::AlignHCenter);
    railLayout->addWidget(saveRailButton, 0, Qt::AlignHCenter);
    railLayout->addWidget(loadRailButton, 0, Qt::AlignHCenter);
    railLayout->addSpacing(10);
    railLayout->addWidget(exportAviRailButton, 0, Qt::AlignHCenter);
    railLayout->addWidget(exportCsvRailButton, 0, Qt::AlignHCenter);
    railLayout->addStretch(1);
    railLayout->addWidget(exitRailButton, 0, Qt::AlignHCenter);
    root->addWidget(rail);

    auto* splitter = new QSplitter(Qt::Horizontal, central);
    splitter->setHandleWidth(2);
    root->addWidget(splitter, 1);

    auto* workspace = new QWidget;
    workspace->setObjectName(QStringLiteral("Workspace"));
    auto* workspaceLayout = new QVBoxLayout(workspace);
    workspaceLayout->setContentsMargins(0, 0, 0, 0);
    workspaceLayout->setSpacing(12);

    auto* videoCard = new QFrame(workspace);
    videoCard->setObjectName(QStringLiteral("VideoCard"));
    auto* videoLayout = new QVBoxLayout(videoCard);
    videoLayout->setContentsMargins(14, 12, 14, 14);
    videoLayout->setSpacing(10);

    auto* feedHeader = new QHBoxLayout;
    feedHeader->setSpacing(12);
    auto* feedTitle = new QLabel(QStringLiteral("IR BEACON FEED"), videoCard);
    feedTitle->setObjectName(QStringLiteral("FeedTitle"));
    m_videoInfoLabel = new QLabel(QStringLiteral("未打开视频"), videoCard);
    m_videoInfoLabel->setObjectName(QStringLiteral("FeedMeta"));
    m_videoInfoLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_videoInfoLabel->setWordWrap(true);
    auto* feedPill = new QLabel(QStringLiteral("RAW · BINARY · OVERLAY"), videoCard);
    feedPill->setObjectName(QStringLiteral("Pill"));
    feedPill->setAlignment(Qt::AlignCenter);
    feedHeader->addWidget(feedTitle);
    feedHeader->addWidget(m_videoInfoLabel, 1);
    feedHeader->addWidget(feedPill);

    m_videoWidget = new VideoWidget(videoCard);
    videoLayout->addLayout(feedHeader);
    videoLayout->addWidget(m_videoWidget, 1);
    workspaceLayout->addWidget(videoCard, 1);

    auto* rightScroll = new QScrollArea;
    rightScroll->setObjectName(QStringLiteral("RightScroll"));
    rightScroll->setWidgetResizable(true);
    rightScroll->setFrameShape(QFrame::NoFrame);
    rightScroll->setMinimumWidth(340);
    rightScroll->setMaximumWidth(430);

    auto* rightPanel = new QWidget(rightScroll);
    rightPanel->setObjectName(QStringLiteral("RightPanel"));
    rightPanel->setMinimumWidth(320);
    auto* rightLayout = new QVBoxLayout(rightPanel);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(12);
    m_frameInfoLabel = new QLabel(QStringLiteral("Frame: -"), rightPanel);
    m_frameInfoLabel->setObjectName(QStringLiteral("SoftLabel"));
    m_frameInfoLabel->setWordWrap(true);
    m_currentAnnotationsLabel = new QLabel(QStringLiteral("当前帧标注: -"), rightPanel);
    m_currentAnnotationsLabel->setObjectName(QStringLiteral("SoftLabel"));
    m_currentAnnotationsLabel->setWordWrap(true);
    m_currentAnnotationsLabel->setMaximumHeight(84);
    m_resultText = new QTextEdit(rightPanel);
    m_resultText->setReadOnly(true);
    m_resultText->setFixedHeight(112);
    m_annotationPanel = new AnnotationPanel(rightPanel);

    auto* infoGroup = new QGroupBox(QStringLiteral("飞控状态 / 当前帧"), rightPanel);
    auto* infoLayout = new QVBoxLayout(infoGroup);
    infoLayout->setContentsMargins(12, 8, 12, 12);
    infoLayout->setSpacing(8);
    infoLayout->addWidget(m_frameInfoLabel);
    infoLayout->addWidget(m_currentAnnotationsLabel);

    auto* resultGroup = new QGroupBox(QStringLiteral("检测结果"), rightPanel);
    auto* resultLayout = new QVBoxLayout(resultGroup);
    resultLayout->setContentsMargins(12, 8, 12, 12);
    resultLayout->addWidget(m_resultText);

    auto* algorithmGroup = new QGroupBox(QStringLiteral("算法信息"), rightPanel);
    auto* algorithmLayout = new QVBoxLayout(algorithmGroup);
    algorithmLayout->setContentsMargins(12, 8, 12, 12);
    algorithmLayout->setSpacing(8);
    m_algorithmInfoLabel = new QLabel(QStringLiteral("入口: beacon_image_process()\n文件: algorithm/beacon_image.c"), algorithmGroup);
    m_algorithmInfoLabel->setObjectName(QStringLiteral("SoftLabel"));
    m_algorithmInfoLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_algorithmInfoLabel->setWordWrap(true);
    auto* openAlgorithmButton = new QPushButton(QStringLiteral("打开算法文件位置"), algorithmGroup);
    algorithmLayout->addWidget(m_algorithmInfoLabel);
    algorithmLayout->addWidget(openAlgorithmButton);

    auto* annotationGroup = new QGroupBox(QStringLiteral("错误标注"), rightPanel);
    auto* annotationLayout = new QVBoxLayout(annotationGroup);
    annotationLayout->setContentsMargins(12, 8, 12, 12);
    annotationLayout->addWidget(m_annotationPanel);

    rightLayout->addWidget(infoGroup);
    rightLayout->addWidget(resultGroup);
    rightLayout->addWidget(algorithmGroup);
    rightLayout->addWidget(annotationGroup, 1);

    rightScroll->setWidget(rightPanel);
    splitter->addWidget(workspace);
    splitter->addWidget(rightScroll);
    splitter->setStretchFactor(0, 4);
    splitter->setStretchFactor(1, 1);

    auto* controlsFrame = new QFrame(workspace);
    controlsFrame->setObjectName(QStringLiteral("ControlConsole"));
    controlsFrame->setMinimumHeight(112);
    controlsFrame->setMaximumHeight(138);
    controlsFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto* controls = new QVBoxLayout(controlsFrame);
    controls->setContentsMargins(14, 12, 14, 12);
    controls->setSpacing(10);
    auto* controlsTitle = new QLabel(QStringLiteral("PLAYBACK CONSOLE"), controlsFrame);
    controlsTitle->setObjectName(QStringLiteral("ConsoleTitle"));
    auto* playbackRow = new QHBoxLayout;
    playbackRow->setSpacing(8);
    auto* playButton = new QPushButton(QStringLiteral("播放"), controlsFrame);
    playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    playButton->setProperty("role", QStringLiteral("primary"));
    auto* pauseButton = new QPushButton(QStringLiteral("暂停"), controlsFrame);
    pauseButton->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    auto* previousButton = new QPushButton(QStringLiteral("上一帧"), controlsFrame);
    previousButton->setIcon(style()->standardIcon(QStyle::SP_MediaSeekBackward));
    auto* nextButton = new QPushButton(QStringLiteral("下一帧"), controlsFrame);
    nextButton->setIcon(style()->standardIcon(QStyle::SP_MediaSeekForward));
    auto* jumpFrameButton = new QPushButton(QStringLiteral("跳转帧"), controlsFrame);
    auto* jumpTimeButton = new QPushButton(QStringLiteral("跳转时间"), controlsFrame);

    m_slider = new QSlider(Qt::Horizontal, controlsFrame);
    m_slider->setRange(0, 0);
    m_slider->setMinimumWidth(220);
    m_frameSpin = new QSpinBox(controlsFrame);
    m_frameSpin->setRange(0, 0);
    m_frameSpin->setFixedWidth(84);
    m_timeSpin = new QDoubleSpinBox(controlsFrame);
    m_timeSpin->setRange(0.0, 0.0);
    m_timeSpin->setDecimals(3);
    m_timeSpin->setSuffix(QStringLiteral(" s"));
    m_timeSpin->setFixedWidth(108);
    m_viewModeCombo = new QComboBox(controlsFrame);
    m_viewModeCombo->addItem(QStringLiteral("原始图像"), QStringLiteral("original"));
    m_viewModeCombo->addItem(QStringLiteral("二值化图像"), QStringLiteral("binary"));
    m_viewModeCombo->setFixedWidth(128);

    auto* viewLabel = new QLabel(QStringLiteral("视图"), controlsFrame);
    viewLabel->setObjectName(QStringLiteral("SoftLabel"));
    auto* frameLabel = new QLabel(QStringLiteral("Frame"), controlsFrame);
    frameLabel->setObjectName(QStringLiteral("SoftLabel"));
    auto* timeLabel = new QLabel(QStringLiteral("Time"), controlsFrame);
    timeLabel->setObjectName(QStringLiteral("SoftLabel"));

    playbackRow->addWidget(playButton);
    playbackRow->addWidget(pauseButton);
    playbackRow->addWidget(previousButton);
    playbackRow->addWidget(nextButton);
    playbackRow->addSpacing(10);
    playbackRow->addWidget(viewLabel);
    playbackRow->addWidget(m_viewModeCombo);
    playbackRow->addStretch(1);
    playbackRow->addWidget(frameLabel);
    playbackRow->addWidget(m_frameSpin);
    playbackRow->addWidget(jumpFrameButton);
    playbackRow->addWidget(timeLabel);
    playbackRow->addWidget(m_timeSpin);
    playbackRow->addWidget(jumpTimeButton);
    controls->addWidget(controlsTitle);
    controls->addLayout(playbackRow);
    controls->addWidget(m_slider);
    workspaceLayout->addWidget(controlsFrame, 0);

    setCentralWidget(central);

    connect(openRailButton, &QToolButton::clicked, this, &MainWindow::openVideo);
    connect(saveRailButton, &QToolButton::clicked, this, &MainWindow::saveAnnotation);
    connect(loadRailButton, &QToolButton::clicked, this, &MainWindow::loadAnnotation);
    connect(exportAviRailButton, &QToolButton::clicked, this, &MainWindow::exportMarkedAvi);
    connect(exportCsvRailButton, &QToolButton::clicked, this, &MainWindow::exportCsv);
    connect(exitRailButton, &QToolButton::clicked, this, &QWidget::close);
    connect(playButton, &QPushButton::clicked, this, &MainWindow::play);
    connect(pauseButton, &QPushButton::clicked, this, &MainWindow::pause);
    connect(previousButton, &QPushButton::clicked, this, &MainWindow::previousFrame);
    connect(nextButton, &QPushButton::clicked, this, &MainWindow::nextFrame);
    connect(jumpFrameButton, &QPushButton::clicked, this, &MainWindow::jumpToFrame);
    connect(jumpTimeButton, &QPushButton::clicked, this, &MainWindow::jumpToTime);
    connect(openAlgorithmButton, &QPushButton::clicked, this, &MainWindow::openAlgorithmLocation);
    connect(m_slider, &QSlider::sliderPressed, this, &MainWindow::pause);
    connect(m_slider, &QSlider::sliderMoved, this, [this](int value) {
        pause();
        showFrame(value);
    });
    connect(m_slider, &QSlider::valueChanged, this, &MainWindow::showFrameFromSlider);
    connect(m_viewModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        showFrame(m_currentFrame);
    });
    connect(m_annotationPanel, &AnnotationPanel::currentFrameAnnotationRequested,
            this, &MainWindow::markCurrentFrameAnnotation);
    connect(m_annotationPanel, &AnnotationPanel::segmentStartRequested,
            this, &MainWindow::setSegmentStart);
    connect(m_annotationPanel, &AnnotationPanel::segmentEndRequested,
            this, &MainWindow::setSegmentEnd);
    connect(m_annotationPanel, &AnnotationPanel::segmentAnnotationRequested,
            this, &MainWindow::saveSegmentAnnotation);
    connect(m_annotationPanel, &AnnotationPanel::deleteAnnotationRequested,
            this, &MainWindow::deleteAnnotation);
    connect(m_annotationPanel, &AnnotationPanel::deleteCorrectionRequested,
            this, &MainWindow::deleteCorrection);
    connect(m_annotationPanel, &AnnotationPanel::correctionToolChanged,
            m_videoWidget, &VideoWidget::setCorrectionTool);
    connect(m_annotationPanel, &AnnotationPanel::recordActivated,
            this, &MainWindow::jumpToRecordFrame);
    connect(m_videoWidget, &VideoWidget::correctionShapeFinished,
            this, &MainWindow::addCorrectionShape);
}

void MainWindow::buildMenus()
{
    auto* fileMenu = menuBar()->addMenu(QStringLiteral("文件"));
    fileMenu->addAction(QStringLiteral("打开视频"), this, &MainWindow::openVideo);
    fileMenu->addAction(QStringLiteral("保存标注"), this, &MainWindow::saveAnnotation);
    fileMenu->addAction(QStringLiteral("读取标注"), this, &MainWindow::loadAnnotation);
    fileMenu->addSeparator();
    fileMenu->addAction(QStringLiteral("导出标注 AVI"), this, &MainWindow::exportMarkedAvi);
    fileMenu->addAction(QStringLiteral("导出 CSV"), this, &MainWindow::exportCsv);
    fileMenu->addSeparator();
    fileMenu->addAction(QStringLiteral("退出"), this, &QWidget::close);

    auto* viewMenu = menuBar()->addMenu(QStringLiteral("视图"));
    auto* fitAction = viewMenu->addAction(QStringLiteral("自适应窗口 / 保持比例"));
    fitAction->setCheckable(true);
    fitAction->setChecked(true);
    fitAction->setEnabled(false);
    auto* overlayAction = viewMenu->addAction(QStringLiteral("显示检测覆盖"));
    overlayAction->setCheckable(true);
    overlayAction->setChecked(true);
    connect(overlayAction, &QAction::toggled, this, [this](bool checked) {
        m_showOverlay = checked;
        showFrame(m_currentFrame);
    });

    auto* helpMenu = menuBar()->addMenu(QStringLiteral("帮助"));
    helpMenu->addAction(QStringLiteral("关于"), this, [this]() {
        QMessageBox::about(this,
                           QStringLiteral("关于 BeaconImageAnalyzer"),
                           QStringLiteral("离线红外信标图像分析、标注和导出工具。"));
    });

    setStyleSheet(djiStyleSheet());
}

void MainWindow::openVideo()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("打开 AVI"),
                                                      QString(),
                                                      QStringLiteral("AVI Video (*.avi)"));
    if (path.isEmpty())
    {
        return;
    }

    saveProject();
    loadVideoFile(path, true, 0);
}

bool MainWindow::loadVideoFile(const QString& path, bool restoreProject, int fallbackFrame)
{
    QString error;
    if (!m_reader.open(path, &error))
    {
        QMessageBox::critical(this, QStringLiteral("打开失败"), error);
        return false;
    }

    m_currentVideoPath = path;
    m_currentFrame = fallbackFrame;
    m_segmentStartFrame = -1;
    m_segmentEndFrame = -1;
    m_annotations.clear();

    m_slider->setRange(0, qMax(0, m_reader.frameCount() - 1));
    m_frameSpin->setRange(0, qMax(0, m_reader.frameCount() - 1));
    m_timeSpin->setRange(0.0, frameTime(qMax(0, m_reader.frameCount() - 1)));

    m_videoInfoLabel->setText(QStringLiteral("%1 | %2x%3 | %4帧\nFPS: %5 / 使用 %6 | %7 | %8 %9-bit")
                                  .arg(QFileInfo(path).fileName())
                                  .arg(m_reader.width())
                                  .arg(m_reader.height())
                                  .arg(m_reader.frameCount())
                                  .arg(m_reader.videoFps(), 0, 'f', 3)
                                  .arg(m_usedFps, 0, 'f', 3)
                                  .arg(m_reader.backendName())
                                  .arg(m_reader.codecName().trimmed())
                                  .arg(m_reader.bitCount()));

    if (m_reader.width() != BEACON_IMAGE_W || m_reader.height() != BEACON_IMAGE_H)
    {
        QMessageBox::warning(this,
                             QStringLiteral("尺寸提醒"),
                             QStringLiteral("当前算法只处理 188x120，视频尺寸不匹配时检测结果会为空。"));
    }

    if (restoreProject)
    {
        loadProject();
    }

    updateAnnotationList();
    showFrame(qBound(0, m_currentFrame, qMax(0, m_reader.frameCount() - 1)));
    statusBar()->showMessage(QStringLiteral("视频已打开"), 3000);
    return true;
}

void MainWindow::saveAnnotation()
{
    if (!m_reader.isOpen())
    {
        QMessageBox::warning(this, QStringLiteral("无法保存"), QStringLiteral("请先打开视频"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("保存标注"),
                                                      defaultOutputPath(QStringLiteral("_annotation.json")),
                                                      QStringLiteral("JSON (*.json)"));
    if (path.isEmpty())
    {
        return;
    }

    AnnotationVideoInfo info;
    info.file = m_currentVideoPath;
    info.width = m_reader.width();
    info.height = m_reader.height();
    info.fpsUsed = m_usedFps;
    info.frameCount = m_reader.frameCount();

    QString error;
    if (!AnnotationJson::save(path, info, m_annotations, &error))
    {
        QMessageBox::critical(this, QStringLiteral("保存失败"), error);
        return;
    }
    statusBar()->showMessage(QStringLiteral("标注已保存"), 3000);
}

void MainWindow::loadAnnotation()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("读取标注"),
                                                      QString(),
                                                      QStringLiteral("JSON (*.json)"));
    if (path.isEmpty())
    {
        return;
    }

    QString error;
    if (!AnnotationJson::load(path, &m_annotations, &error))
    {
        QMessageBox::critical(this, QStringLiteral("读取失败"), error);
        return;
    }
    updateAnnotationList();
    showFrame(m_currentFrame);
    statusBar()->showMessage(QStringLiteral("标注已读取"), 3000);
}

void MainWindow::exportMarkedAvi()
{
    if (!m_reader.isOpen())
    {
        QMessageBox::warning(this, QStringLiteral("无法导出"), QStringLiteral("请先打开视频"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("导出标注 AVI"),
                                                      defaultOutputPath(QStringLiteral("_marked.avi")),
                                                      QStringLiteral("AVI Video (*.avi)"));
    if (path.isEmpty())
    {
        return;
    }

    QProgressDialog progress(QStringLiteral("正在导出 AVI..."), QStringLiteral("取消"), 0, m_reader.frameCount(), this);
    progress.setWindowModality(Qt::WindowModal);

    VideoExporter exporter;
    QString error;
    const bool ok = exporter.exportMarkedAvi(m_currentVideoPath, path, m_usedFps,
                                             [&](int current, int total) {
                                                 progress.setMaximum(total);
                                                 progress.setValue(current);
                                                 QApplication::processEvents();
                                                 return !progress.wasCanceled();
                                             },
                                             &error);
    progress.setValue(progress.maximum());
    if (!ok)
    {
        QMessageBox::critical(this, QStringLiteral("导出失败"), error);
        return;
    }
    statusBar()->showMessage(QStringLiteral("AVI 已导出"), 3000);
}

void MainWindow::exportCsv()
{
    if (!m_reader.isOpen())
    {
        QMessageBox::warning(this, QStringLiteral("无法导出"), QStringLiteral("请先打开视频"));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("导出 CSV"),
                                                      defaultOutputPath(QStringLiteral("_result.csv")),
                                                      QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty())
    {
        return;
    }

    QProgressDialog progress(QStringLiteral("正在导出 CSV..."), QStringLiteral("取消"), 0, m_reader.frameCount(), this);
    progress.setWindowModality(Qt::WindowModal);

    VideoExporter exporter;
    QString error;
    const bool ok = exporter.exportResultCsv(m_currentVideoPath, path, m_usedFps,
                                             [&](int current, int total) {
                                                 progress.setMaximum(total);
                                                 progress.setValue(current);
                                                 QApplication::processEvents();
                                                 return !progress.wasCanceled();
                                             },
                                             &error);
    progress.setValue(progress.maximum());
    if (!ok)
    {
        QMessageBox::critical(this, QStringLiteral("导出失败"), error);
        return;
    }
    statusBar()->showMessage(QStringLiteral("CSV 已导出"), 3000);
}

void MainWindow::play()
{
    if (m_reader.isOpen())
    {
        m_playTimer.start(qMax(1, (int)(1000.0 / m_usedFps)));
    }
}

void MainWindow::pause()
{
    m_playTimer.stop();
}

void MainWindow::togglePlayPause()
{
    if (m_playTimer.isActive())
    {
        pause();
    }
    else
    {
        play();
    }
}

void MainWindow::nextFrame()
{
    if (!m_reader.isOpen())
    {
        return;
    }
    if (m_currentFrame + 1 >= m_reader.frameCount())
    {
        pause();
        return;
    }
    showFrame(m_currentFrame + 1);
}

void MainWindow::previousFrame()
{
    if (m_reader.isOpen())
    {
        showFrame(qMax(0, m_currentFrame - 1));
    }
}

void MainWindow::jumpToFrame()
{
    showFrame(m_frameSpin->value());
}

void MainWindow::jumpToTime()
{
    showFrame(qBound(0, (int)(m_timeSpin->value() * m_usedFps + 0.5), qMax(0, m_reader.frameCount() - 1)));
}

void MainWindow::showFrameFromSlider(int value)
{
    if (!m_updatingControls)
    {
        pause();
        showFrame(value);
    }
}

void MainWindow::showFrame(int frameIndex)
{
    if (!m_reader.isOpen())
    {
        return;
    }

    frameIndex = qBound(0, frameIndex, m_reader.frameCount() - 1);
    QImage gray;
    QString error;
    if (!m_reader.readFrame(frameIndex, &gray, &error))
    {
        QMessageBox::critical(this, QStringLiteral("读取帧失败"), error);
        pause();
        return;
    }

    m_currentFrame = frameIndex;
    const beacon_result_t result = m_runner.process(gray);
    QImage displayImage = gray;
    if (viewMode() == QStringLiteral("binary"))
    {
        const QImage binary = m_runner.binaryImage(gray);
        if (!binary.isNull())
        {
            displayImage = binary;
        }
    }
    const QVector<CorrectionShape> corrections = m_annotations.correctionsForFrame(m_currentFrame);
    m_videoWidget->setFrameGeometry(QSize(BEACON_IMAGE_W, BEACON_IMAGE_H), 1);
    m_videoWidget->setImage(FrameRenderer::render(displayImage, result, corrections, 1, m_showOverlay));

    m_updatingControls = true;
    m_slider->setValue(frameIndex);
    m_frameSpin->setValue(frameIndex);
    m_timeSpin->setValue(frameTime(frameIndex));
    m_updatingControls = false;

    updateFrameInfo(result);
}

void MainWindow::updateFrameInfo(const beacon_result_t& result)
{
    m_frameInfoLabel->setText(QStringLiteral("Frame %1/%2 | %3s/%4s | Circles %5")
                                  .arg(m_currentFrame)
                                  .arg(qMax(0, m_reader.frameCount() - 1))
                                  .arg(frameTime(m_currentFrame), 0, 'f', 3)
                                  .arg(frameTime(qMax(0, m_reader.frameCount() - 1)), 0, 'f', 3)
                                  .arg(result.count));

    QString text;
    for (int i = 0; i < result.count && i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        const beacon_circle_t& circle = result.circles[i];
        if (circle.valid == 0)
        {
            continue;
        }
        text += QStringLiteral("#%1 valid=%2 x=%3 y=%4 r=%5\n")
                    .arg(i)
                    .arg((int)circle.valid)
                    .arg(circle.x, 0, 'f', 2)
                    .arg(circle.y, 0, 'f', 2)
                    .arg(circle.radius, 0, 'f', 2);
    }
    if (text.isEmpty())
    {
        text = QStringLiteral("无有效圆");
    }
    m_resultText->setPlainText(text);
    m_annotationPanel->setCurrentContext(m_currentFrame, frameTime(m_currentFrame), result.count);
    updateCurrentAnnotationInfo();
}

void MainWindow::updateCurrentAnnotationInfo()
{
    if (m_currentAnnotationsLabel == nullptr)
    {
        return;
    }

    const QVector<AnnotationRecord> records = m_annotations.recordsForFrame(m_currentFrame);
    const QVector<CorrectionShape> corrections = m_annotations.correctionsForFrame(m_currentFrame);
    if (records.isEmpty() && corrections.isEmpty())
    {
        m_currentAnnotationsLabel->setText(QStringLiteral("当前帧标注: 无"));
        return;
    }

    QString text = QStringLiteral("当前帧标注: 文字 %1 条 / 图形 %2 条\n").arg(records.size()).arg(corrections.size());
    for (const AnnotationRecord& record : records)
    {
        const QString circle = record.circleIndex >= 0
            ? QStringLiteral("#%1").arg(record.circleIndex)
            : QStringLiteral("全部");
        text += QStringLiteral("- %1 %2 %3\n")
                    .arg(annotationTypeDisplayName(record.type))
                    .arg(circle)
                    .arg(record.description);
    }
    for (const CorrectionShape& shape : corrections)
    {
        const QString expected = shape.expectedIndex >= 0
            ? QStringLiteral("GT #%1").arg(shape.expectedIndex)
            : QStringLiteral("GT 未指定");
        text += QStringLiteral("- 图形 %1 %2 %3 %4\n")
                    .arg(annotationTypeDisplayName(shape.errorType))
                    .arg(shape.shapeType)
                    .arg(expected)
                    .arg(shape.description);
    }
    m_currentAnnotationsLabel->setText(text.trimmed());
}

void MainWindow::markCurrentFrameAnnotation(const QString& type, int circleIndex, const QString& description)
{
    if (!m_reader.isOpen())
    {
        return;
    }

    AnnotationRecord record;
    record.type = type;
    record.startFrame = m_currentFrame;
    record.endFrame = m_currentFrame;
    record.startTimeSec = frameTime(m_currentFrame);
    record.endTimeSec = record.startTimeSec;
    record.circleIndex = circleIndex;
    record.description = description;
    m_annotations.add(record);
    updateAnnotationList();
}

void MainWindow::setSegmentStart()
{
    if (!m_reader.isOpen())
    {
        return;
    }
    m_segmentStartFrame = m_currentFrame;
    m_annotationPanel->setSegmentStart(m_segmentStartFrame);
}

void MainWindow::setSegmentEnd()
{
    if (!m_reader.isOpen())
    {
        return;
    }
    m_segmentEndFrame = m_currentFrame;
    m_annotationPanel->setSegmentEnd(m_segmentEndFrame);
}

void MainWindow::saveSegmentAnnotation(const QString& type, int circleIndex, const QString& description)
{
    if (!m_reader.isOpen() || m_segmentStartFrame < 0 || m_segmentEndFrame < 0)
    {
        QMessageBox::warning(this, QStringLiteral("片段不完整"), QStringLiteral("请先设置片段开始和结束"));
        return;
    }

    const int startFrame = qMin(m_segmentStartFrame, m_segmentEndFrame);
    const int endFrame = qMax(m_segmentStartFrame, m_segmentEndFrame);
    AnnotationRecord record;
    record.type = type;
    record.startFrame = startFrame;
    record.endFrame = endFrame;
    record.startTimeSec = frameTime(startFrame);
    record.endTimeSec = frameTime(endFrame);
    record.circleIndex = circleIndex;
    record.description = description;
    m_annotations.add(record);
    updateAnnotationList();
}

void MainWindow::deleteAnnotation(int row)
{
    if (m_annotations.removeAt(row))
    {
        updateAnnotationList();
        showFrame(m_currentFrame);
    }
}

void MainWindow::deleteCorrection(int row)
{
    if (m_annotations.removeCorrectionAt(row))
    {
        updateAnnotationList();
        showFrame(m_currentFrame);
    }
}

void MainWindow::addCorrectionShape(const QString& shapeType, const QVector<QPointF>& points)
{
    if (!m_reader.isOpen() || points.isEmpty())
    {
        return;
    }

    CorrectionShape shape;
    shape.shapeType = shapeType;
    shape.frame = m_currentFrame;
    shape.errorType = m_annotationPanel->selectedType();
    shape.expectedIndex = m_annotationPanel->selectedExpectedIndex();
    shape.description = m_annotationPanel->noteText();
    shape.points = points;
    m_annotations.addCorrection(shape);
    updateAnnotationList();
    showFrame(m_currentFrame);
}

void MainWindow::openAlgorithmLocation()
{
#ifdef BEACON_SOURCE_DIR
    const QString algorithmPath = QDir(QStringLiteral(BEACON_SOURCE_DIR)).absoluteFilePath(QStringLiteral("algorithm/beacon_image.c"));
#else
    const QString algorithmPath = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../algorithm/beacon_image.c"));
#endif
    QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(algorithmPath).absolutePath()));
}

void MainWindow::jumpToRecordFrame(int frame)
{
    if (!m_reader.isOpen())
    {
        return;
    }
    pause();
    showFrame(qBound(0, frame, qMax(0, m_reader.frameCount() - 1)));
}

void MainWindow::updateAnnotationList()
{
    m_annotationPanel->setAnnotations(m_annotations.records(), m_annotations.corrections());
    updateCurrentAnnotationInfo();
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
        pause();
        previousFrame();
        return true;
    }
    if (keyEvent->key() == Qt::Key_Right)
    {
        pause();
        nextFrame();
        return true;
    }

    return false;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    saveProject();
    QMainWindow::closeEvent(event);
}

void MainWindow::restoreLastSession()
{
    QSettings settings(QStringLiteral("BeaconImageAnalyzer"), QStringLiteral("BeaconImageAnalyzer"));
    const QString projectPath = settings.value(QStringLiteral("last_project_path")).toString();
    if (projectPath.isEmpty())
    {
        return;
    }

    QFile file(projectPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        return;
    }

    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    const QString videoPath = document.object()
                                  .value(QStringLiteral("session")).toObject()
                                  .value(QStringLiteral("video_path")).toString();
    if (!videoPath.isEmpty() && QFileInfo::exists(videoPath))
    {
        loadVideoFile(videoPath, true, 0);
    }
}

void MainWindow::saveProject()
{
    if (!m_reader.isOpen() || m_currentVideoPath.isEmpty())
    {
        return;
    }

    const QString path = projectPathForVideo(m_currentVideoPath);
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        return;
    }

    QJsonObject root;
    root.insert(QStringLiteral("project"), QStringLiteral("BeaconImageAnalyzer"));

    QJsonObject video;
    video.insert(QStringLiteral("file"), QFileInfo(m_currentVideoPath).fileName());
    video.insert(QStringLiteral("width"), m_reader.width());
    video.insert(QStringLiteral("height"), m_reader.height());
    video.insert(QStringLiteral("fps_used"), m_usedFps);
    video.insert(QStringLiteral("frame_count"), m_reader.frameCount());
    root.insert(QStringLiteral("video"), video);

    QJsonObject session;
    session.insert(QStringLiteral("video_path"), QFileInfo(m_currentVideoPath).absoluteFilePath());
    session.insert(QStringLiteral("current_frame"), m_currentFrame);
    session.insert(QStringLiteral("zoom"), m_zoom);
    session.insert(QStringLiteral("show_overlay"), m_showOverlay);
    session.insert(QStringLiteral("view_mode"), viewMode());
    session.insert(QStringLiteral("window_geometry"), QString::fromLatin1(saveGeometry().toBase64()));
    root.insert(QStringLiteral("session"), session);

    QJsonObject algorithm;
    algorithm.insert(QStringLiteral("name"), QStringLiteral("beacon_image_process"));
    algorithm.insert(QStringLiteral("version"), QStringLiteral("v1"));
    algorithm.insert(QStringLiteral("note"), QStringLiteral("simple threshold + connected components"));
    root.insert(QStringLiteral("algorithm"), algorithm);

    QJsonArray annotations;
    for (const AnnotationRecord& record : m_annotations.records())
    {
        QJsonObject item;
        item.insert(QStringLiteral("type"), record.type);
        item.insert(QStringLiteral("start_frame"), record.startFrame);
        item.insert(QStringLiteral("end_frame"), record.endFrame);
        item.insert(QStringLiteral("start_time_sec"), record.startTimeSec);
        item.insert(QStringLiteral("end_time_sec"), record.endTimeSec);
        item.insert(QStringLiteral("circle_index"), record.circleIndex);
        item.insert(QStringLiteral("description"), record.description);
        annotations.append(item);
    }
    root.insert(QStringLiteral("annotations"), annotations);

    QJsonArray corrections;
    for (const CorrectionShape& shape : m_annotations.corrections())
    {
        QJsonObject item;
        item.insert(QStringLiteral("shape_type"), shape.shapeType);
        item.insert(QStringLiteral("frame"), shape.frame);
        item.insert(QStringLiteral("error_type"), shape.errorType);
        item.insert(QStringLiteral("expected_index"), shape.expectedIndex);
        item.insert(QStringLiteral("description"), shape.description);
        item.insert(QStringLiteral("points"), pointsToJson(shape.points));
        corrections.append(item);
    }
    root.insert(QStringLiteral("corrections"), corrections);

    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));

    QSettings settings(QStringLiteral("BeaconImageAnalyzer"), QStringLiteral("BeaconImageAnalyzer"));
    settings.setValue(QStringLiteral("last_project_path"), path);
}

bool MainWindow::loadProject()
{
    if (m_currentVideoPath.isEmpty())
    {
        return false;
    }

    const QString path = projectPathForVideo(m_currentVideoPath);
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        return false;
    }

    const QJsonObject root = document.object();
    const QJsonObject session = root.value(QStringLiteral("session")).toObject();
    m_currentFrame = session.value(QStringLiteral("current_frame")).toInt(m_currentFrame);
    m_zoom = 1;
    m_showOverlay = session.value(QStringLiteral("show_overlay")).toBool(m_showOverlay);

    const QString restoredViewMode = session.value(QStringLiteral("view_mode")).toString(QStringLiteral("original"));
    const int viewIndex = m_viewModeCombo->findData(restoredViewMode);
    if (viewIndex >= 0)
    {
        m_viewModeCombo->setCurrentIndex(viewIndex);
    }

    const QByteArray geometry = QByteArray::fromBase64(session.value(QStringLiteral("window_geometry")).toString().toLatin1());
    if (!geometry.isEmpty())
    {
        restoreGeometry(geometry);
    }

    QString error;
    if (!AnnotationJson::load(path, &m_annotations, &error))
    {
        return false;
    }
    return true;
}

QString MainWindow::projectPathForVideo(const QString& videoPath) const
{
    const QFileInfo info(videoPath);
    return info.absolutePath() + QLatin1Char('/') + info.completeBaseName() + QStringLiteral(".bia_project.json");
}

QString MainWindow::viewMode() const
{
    if (m_viewModeCombo == nullptr)
    {
        return QStringLiteral("original");
    }
    return m_viewModeCombo->currentData().toString();
}

QString MainWindow::defaultOutputPath(const QString& suffix) const
{
    if (m_currentVideoPath.isEmpty())
    {
        return QString();
    }
    const QFileInfo info(m_currentVideoPath);
    return info.absolutePath() + QLatin1Char('/') + info.completeBaseName() + suffix;
}

double MainWindow::frameTime(int frame) const
{
    return (double)frame / m_usedFps;
}
