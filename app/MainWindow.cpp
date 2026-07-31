#include "MainWindow.h"

#include "AnnotationJson.h"
#include "AnnotationPanel.h"
#include "AiInstanceDialog.h"
#include "BeaconLabelWindow.h"
#include "BeaconResultUtils.h"
#include "FrameRenderer.h"
#include "HorizonCalibration.h"
#include "LogReplayWindow.h"
#include "TcpImageWindow.h"
#include "HorizonCalibrationWindow.h"
#include "VideoExporter.h"
#include "VideoWidget.h"

#include <QAction>
#include <QAbstractSocket>
#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDesktopServices>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QLabel>
#include <QLineF>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QNetworkInterface>
#include <QPlainTextEdit>
#include <QPolygonF>
#include <QProgressDialog>
#include <QPushButton>
#include <QPixmap>
#include <QRectF>
#include <QSaveFile>
#include <QSettings>
#include <QSignalBlocker>
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

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>

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

QJsonArray stringListToJson(const QStringList& values)
{
    QJsonArray array;
    for (const QString& value : values)
    {
        if (!value.trimmed().isEmpty())
        {
            array.append(value);
        }
    }
    return array;
}

QJsonArray errorCirclesToJson(const QVector<ErrorCircle>& circles)
{
    QJsonArray array;
    for (const ErrorCircle& circle : circles)
    {
        QJsonObject object;
        object.insert(QStringLiteral("circle_index"), circle.circleIndex);
        object.insert(QStringLiteral("expected_index"), circle.expectedIndex);
        array.append(object);
    }
    return array;
}

bool typesRequireErrorSource(const QStringList& types)
{
    if (types.isEmpty())
    {
        return false;
    }
    for (const QString& type : types)
    {
        if (type != QStringLiteral("missed_detection"))
        {
            return true;
        }
    }
    return false;
}

bool isClosedCorrectionShape(const CorrectionShape& shape)
{
    return (shape.shapeType == QStringLiteral("circle") && shape.points.size() >= 2) ||
           (shape.shapeType == QStringLiteral("rect") && shape.points.size() >= 2) ||
           (shape.shapeType == QStringLiteral("polygon") && shape.points.size() >= 3);
}

bool shapeContainsPoint(const CorrectionShape& shape, const QPointF& point)
{
    if (shape.shapeType == QStringLiteral("circle") && shape.points.size() >= 2)
    {
        return QLineF(shape.points[0], point).length() <= QLineF(shape.points[0], shape.points[1]).length();
    }
    if (shape.shapeType == QStringLiteral("rect") && shape.points.size() >= 2)
    {
        return QRectF(shape.points[0], shape.points[1]).normalized().contains(point);
    }
    if (shape.shapeType == QStringLiteral("polygon") && shape.points.size() >= 3)
    {
        return QPolygonF(shape.points).containsPoint(point, Qt::OddEvenFill);
    }
    return false;
}

constexpr double Pi = 3.14159265358979323846;

struct MissedTargetCandidate
{
    QPointF center;
    double area = 0.0;
    double circularityScore = 0.0;
};

struct MissedBatchTarget
{
    CorrectionShape baseCorrection;
    QPointF previousCenter;
    QPointF radiusVector;
    double radius = 0.0;
    double baseArea = 0.0;
    double previousArea = 0.0;
};

bool isMissedCorrectionType(const QString& type)
{
    return type == QStringLiteral("missed_detection");
}

CorrectionShape batchCorrectionConfigOnly(const CorrectionShape& correction)
{
    CorrectionShape sanitized = correction;
    sanitized.lineColor = QColor();
    sanitized.lineWidth = 1;

    if (isMissedCorrectionType(sanitized.errorType) &&
        sanitized.shapeType == QStringLiteral("circle") &&
        sanitized.points.size() >= 2)
    {
        sanitized.points = QVector<QPointF>{ sanitized.points[0], sanitized.points[1] };
    }
    else
    {
        sanitized.shapeType.clear();
        sanitized.points.clear();
    }

    return sanitized;
}

int validCircleCount(const beacon_result_t& result)
{
    return BeaconResultUtils::totalTargetCount(result);
}

double circleImageDistance(const beacon_circle_t& previous, const beacon_circle_t& current)
{
    return QLineF(FrameRenderer::algorithmToImagePoint(previous.x, previous.y),
                  FrameRenderer::algorithmToImagePoint(current.x, current.y)).length();
}

QVector<int> normalizedRows(QVector<int> rows)
{
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    return rows;
}

QString framesText(const QVector<int>& frames)
{
    QStringList parts;
    for (int frame : frames)
    {
        parts.push_back(QString::number(frame));
    }
    return parts.join(QStringLiteral(", "));
}

QString defaultAlgorithmPath()
{
#ifdef BEACON_SOURCE_DIR
    return QDir(QStringLiteral(BEACON_SOURCE_DIR)).absoluteFilePath(QStringLiteral("algorithm/beacon_image.c"));
#else
    return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../algorithm/beacon_image.c"));
#endif
}

QString defaultAlgorithmHeaderPath(const QString& fileName)
{
#ifdef BEACON_SOURCE_DIR
    return QDir(QStringLiteral(BEACON_SOURCE_DIR)).absoluteFilePath(QStringLiteral("algorithm/%1").arg(fileName));
#else
    return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../algorithm/%1").arg(fileName));
#endif
}

QString defaultInstancesRoot()
{
    return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("instances"));
}

QString twoBl3ImageSourceFromInstance(const QString& rootDir, QString* errorMessage)
{
    const QDir root(rootDir);
    const QStringList candidates = {
        root.absoluteFilePath(QStringLiteral("Image")),
        root.absoluteFilePath(QStringLiteral("algorithm/Image")),
        root.absoluteFilePath(QStringLiteral("algorithm")),
        root.absolutePath()
    };
    const auto imageSource = [](const QString& directory) {
        const QDir imageDir(directory);
        return QFileInfo::exists(imageDir.absoluteFilePath(QStringLiteral("image.c")))
                   && QFileInfo::exists(imageDir.absoluteFilePath(QStringLiteral("image.h")))
                   && QFileInfo::exists(imageDir.absoluteFilePath(QStringLiteral("image_params.c")))
               ? imageDir.absoluteFilePath(QStringLiteral("image.c"))
               : QString();
    };

    for (const QString& candidate : candidates)
    {
        const QString source = imageSource(candidate);
        if (!source.isEmpty())
        {
            return source;
        }
    }

    QFile manifest(root.absoluteFilePath(QStringLiteral("two_bl3_instance.json")));
    if (!manifest.exists())
    {
        return {};
    }
    if (!manifest.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法读取2BL3实例描述文件：%1").arg(manifest.fileName());
        }
        return {};
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(manifest.readAll(), &parseError);
    const QString configuredPath = document.object()
                                       .value(QStringLiteral("image_directory"))
                                       .toString()
                                       .trimmed();
    if (parseError.error != QJsonParseError::NoError || !document.isObject()
        || configuredPath.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("2BL3实例描述文件格式错误：%1").arg(manifest.fileName());
        }
        return {};
    }

    const QString imageDirectory = QDir::isAbsolutePath(configuredPath)
                                       ? QDir(configuredPath).absolutePath()
                                       : root.absoluteFilePath(configuredPath);
    const QString source = imageSource(imageDirectory);
    if (source.isEmpty() && errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("2BL3实例缺少image.c、image.h或image_params.c：%1")
                            .arg(QDir(imageDirectory).absolutePath());
    }
    return source;
}

bool copyFileIfNeeded(const QString& source, const QString& destination, QString* errorMessage)
{
    if (QFileInfo::exists(destination))
    {
        return true;
    }
    if (!QFile::copy(source, destination))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("复制文件失败：%1 -> %2").arg(source, destination);
        }
        return false;
    }
    return true;
}

bool writeTextFile(const QString& destination, const QString& content, QString* errorMessage)
{
    QSaveFile file(destination);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法写入文件：%1").arg(destination);
        }
        return false;
    }

    file.write(content.toUtf8());
    if (!file.commit())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("保存文件失败：%1").arg(destination);
        }
        return false;
    }
    return true;
}

QString normalizedSourceForStaticCheck(const QString& source)
{
    QString normalized;
    normalized.reserve(source.size());
    for (const QChar ch : source)
    {
        if (!ch.isSpace())
        {
            normalized.append(ch.toLower());
        }
    }
    normalized.replace(QStringLiteral("(float)"), QString());
    normalized.replace(QStringLiteral("(double)"), QString());
    return normalized;
}

bool seedAlgorithmFolder(const QString& rootDir,
                         const QString& sourceCFile,
                         QString* algorithmPath,
                         QString* errorMessage)
{
    const QString algorithmDirPath = QDir(rootDir).absoluteFilePath(QStringLiteral("algorithm"));
    if (!QDir().mkpath(algorithmDirPath))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法创建实例算法目录：%1").arg(algorithmDirPath);
        }
        return false;
    }

    const QString targetCPath = QDir(algorithmDirPath).absoluteFilePath(QStringLiteral("beacon_image.c"));
    if (QFileInfo::exists(targetCPath))
    {
        if (algorithmPath != nullptr)
        {
            *algorithmPath = targetCPath;
        }
        return true;
    }
    if (!QFile::copy(sourceCFile, targetCPath))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("复制 C 文件失败：%1 -> %2").arg(sourceCFile, targetCPath);
        }
        return false;
    }

    copyFileIfNeeded(defaultAlgorithmHeaderPath(QStringLiteral("beacon_image.h")),
                     QDir(algorithmDirPath).absoluteFilePath(QStringLiteral("beacon_image.h")),
                     nullptr);
    copyFileIfNeeded(defaultAlgorithmHeaderPath(QStringLiteral("beacon_image_config.h")),
                     QDir(algorithmDirPath).absoluteFilePath(QStringLiteral("beacon_image_config.h")),
                     nullptr);

    if (algorithmPath != nullptr)
    {
        *algorithmPath = targetCPath;
    }
    return true;
}

double correctionCircleRadius(const CorrectionShape& correction)
{
    if (correction.shapeType != QStringLiteral("circle") || correction.points.size() < 2)
    {
        return 0.0;
    }
    return QLineF(correction.points[0], correction.points[1]).length();
}

CorrectionShape correctionWithCenter(const CorrectionShape& correction,
                                     const QPointF& center,
                                     const QPointF& radiusVector)
{
    CorrectionShape updated = correction;
    if (updated.points.size() < 2)
    {
        return updated;
    }

    QPointF safeRadiusVector = radiusVector;
    if (QLineF(QPointF(0.0, 0.0), safeRadiusVector).length() <= 0.0)
    {
        safeRadiusVector = QPointF(correctionCircleRadius(correction), 0.0);
    }
    updated.points[0] = center;
    updated.points[1] = center + safeRadiusVector;
    return updated;
}

cv::Mat qImageToGrayMat(const QImage& image)
{
    const QImage gray = image.format() == QImage::Format_Grayscale8
        ? image
        : image.convertToFormat(QImage::Format_Grayscale8);
    cv::Mat mat(gray.height(), gray.width(), CV_8UC1);
    for (int y = 0; y < gray.height(); ++y)
    {
        memcpy(mat.ptr(y), gray.constScanLine(y), gray.width());
    }
    return mat;
}

void addCandidateUnique(QVector<MissedTargetCandidate>* candidates, const MissedTargetCandidate& candidate)
{
    if (candidates == nullptr)
    {
        return;
    }

    for (MissedTargetCandidate& existing : *candidates)
    {
        if (QLineF(existing.center, candidate.center).length() <= 1.25)
        {
            if (candidate.circularityScore > existing.circularityScore)
            {
                existing = candidate;
            }
            return;
        }
    }
    candidates->push_back(candidate);
}

void appendConnectedComponentCandidates(const cv::Mat& binary, QVector<MissedTargetCandidate>* candidates)
{
    if (binary.empty() || candidates == nullptr)
    {
        return;
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int componentCount = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_32S);
    for (int label = 1; label < componentCount; ++label)
    {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        if (area < 2 || width <= 0 || height <= 0)
        {
            continue;
        }

        const double aspect = (double)qMax(width, height) / (double)qMax(1, qMin(width, height));
        const double fillRatio = (double)area / (double)(width * height);
        if (aspect > 2.6 || fillRatio < 0.22)
        {
            continue;
        }

        MissedTargetCandidate candidate;
        candidate.center = QPointF(centroids.at<double>(label, 0), centroids.at<double>(label, 1));
        candidate.area = (double)area;
        candidate.circularityScore = fillRatio / aspect;
        addCandidateUnique(candidates, candidate);
    }
}

QString defaultTcpSaveDir()
{
    return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("tcp_frames"));
}

QVector<TcpListenAddress> localTcpListenAddresses()
{
    QVector<TcpListenAddress> addresses;
    addresses.push_back({ QStringLiteral("所有 IPv4 地址 - 0.0.0.0"), QStringLiteral("0.0.0.0") });

    const QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    for (const QNetworkInterface& networkInterface : interfaces)
    {
        const QNetworkInterface::InterfaceFlags flags = networkInterface.flags();
        if (!flags.testFlag(QNetworkInterface::IsUp) || !flags.testFlag(QNetworkInterface::IsRunning))
        {
            continue;
        }

        const QList<QNetworkAddressEntry> entries = networkInterface.addressEntries();
        for (const QNetworkAddressEntry& entry : entries)
        {
            const QHostAddress address = entry.ip();
            if (address.protocol() != QAbstractSocket::IPv4Protocol || address.isLoopback())
            {
                continue;
            }

            const QString ip = address.toString();
            const QString label = QStringLiteral("%1 - %2")
                                      .arg(networkInterface.humanReadableName().isEmpty()
                                               ? networkInterface.name()
                                               : networkInterface.humanReadableName(),
                                           ip);
            addresses.push_back({ label, ip });
        }
    }
    return addresses;
}

QVector<MissedTargetCandidate> extractMissedTargetCandidates(const QImage& gray)
{
    QVector<MissedTargetCandidate> candidates;
    const cv::Mat mat = qImageToGrayMat(gray);
    if (mat.empty())
    {
        return candidates;
    }

    cv::Mat binary;
    cv::threshold(mat, binary, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);
    appendConnectedComponentCandidates(binary, &candidates);

    const int minDimension = qMin(mat.cols, mat.rows);
    const int blockSizes[] = { 9, 15, 21, 31 };
    const int constants[] = { -8, -4, 0, 4, 8 };
    for (int blockSize : blockSizes)
    {
        if (blockSize >= minDimension || blockSize % 2 == 0)
        {
            continue;
        }
        for (int constant : constants)
        {
            cv::adaptiveThreshold(mat,
                                  binary,
                                  255.0,
                                  cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv::THRESH_BINARY,
                                  blockSize,
                                  (double)constant);
            appendConnectedComponentCandidates(binary, &candidates);
        }
    }

    return candidates;
}

double circleOverlapPixelCount(const QPointF& first, const QPointF& second, double radius)
{
    if (radius <= 0.0)
    {
        return 0.0;
    }

    const int minX = qMax(0, (int)std::floor(qMax(first.x() - radius, second.x() - radius)));
    const int maxX = qMin(BEACON_IMAGE_W - 1, (int)std::ceil(qMin(first.x() + radius, second.x() + radius)));
    const int minY = qMax(0, (int)std::floor(qMax(first.y() - radius, second.y() - radius)));
    const int maxY = qMin(BEACON_IMAGE_H - 1, (int)std::ceil(qMin(first.y() + radius, second.y() + radius)));
    if (minX > maxX || minY > maxY)
    {
        return 0.0;
    }

    const double radiusSquared = radius * radius;
    int overlap = 0;
    for (int y = minY; y <= maxY; ++y)
    {
        for (int x = minX; x <= maxX; ++x)
        {
            const double pixelX = (double)x + 0.5;
            const double pixelY = (double)y + 0.5;
            const double firstDx = pixelX - first.x();
            const double firstDy = pixelY - first.y();
            const double secondDx = pixelX - second.x();
            const double secondDy = pixelY - second.y();
            if (firstDx * firstDx + firstDy * firstDy <= radiusSquared &&
                secondDx * secondDx + secondDy * secondDy <= radiusSquared)
            {
                ++overlap;
            }
        }
    }
    return (double)overlap;
}

bool prepareMissedBatchTargets(const QVector<CorrectionShape>& corrections,
                               QVector<MissedBatchTarget>* targets,
                               QString* errorMessage)
{
    if (targets == nullptr)
    {
        return false;
    }
    targets->clear();

    for (const CorrectionShape& correction : corrections)
    {
        if (!isMissedCorrectionType(correction.errorType) ||
            correction.shapeType != QStringLiteral("circle") ||
            correction.points.size() < 2)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("漏检批量要求选中的每个条目都是已绘制圆形的漏检纠错。");
            }
            return false;
        }

        const double radius = correctionCircleRadius(correction);
        if (radius <= 0.0)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("漏检批量要求基准漏检圆半径大于 0。");
            }
            return false;
        }

        MissedBatchTarget target;
        target.baseCorrection = correction;
        target.previousCenter = correction.points[0];
        target.radiusVector = correction.points[1] - correction.points[0];
        target.radius = radius;
        target.baseArea = Pi * radius * radius;
        target.previousArea = target.baseArea;
        targets->push_back(target);
    }

    return !targets->isEmpty();
}

bool matchMissedTargetsInImage(const QImage& gray,
                               const QVector<MissedBatchTarget>& previousTargets,
                               double overlapPixelThreshold,
                               QVector<MissedBatchTarget>* nextTargets,
                               QVector<CorrectionShape>* corrections)
{
    if (nextTargets == nullptr || corrections == nullptr)
    {
        return false;
    }
    nextTargets->clear();
    corrections->clear();

    const QVector<MissedTargetCandidate> candidates = extractMissedTargetCandidates(gray);
    if (candidates.isEmpty())
    {
        return false;
    }

    QVector<bool> used(candidates.size(), false);
    for (const MissedBatchTarget& target : previousTargets)
    {
        int bestIndex = -1;
        double bestScore = std::numeric_limits<double>::max();
        for (int i = 0; i < candidates.size(); ++i)
        {
            if (used[i])
            {
                continue;
            }

            const MissedTargetCandidate& candidate = candidates[i];
            if (candidate.area < target.baseArea * 0.12 || candidate.area > target.baseArea * 6.0)
            {
                continue;
            }

            const double overlapPixels = circleOverlapPixelCount(target.previousCenter,
                                                                 candidate.center,
                                                                 target.radius);
            if (overlapPixels + 1.0e-6 < overlapPixelThreshold)
            {
                continue;
            }

            const double centerDistance = QLineF(target.previousCenter, candidate.center).length();
            const double areaDelta = std::abs(candidate.area - target.previousArea) / qMax(1.0, target.previousArea);
            const double score = centerDistance +
                                 areaDelta * qMax(1.0, target.radius) -
                                 candidate.circularityScore;
            if (score < bestScore)
            {
                bestScore = score;
                bestIndex = i;
            }
        }

        if (bestIndex < 0)
        {
            return false;
        }

        used[bestIndex] = true;
        const MissedTargetCandidate& candidate = candidates[bestIndex];
        MissedBatchTarget nextTarget = target;
        nextTarget.previousCenter = candidate.center;
        nextTarget.previousArea = candidate.area;
        nextTargets->push_back(nextTarget);
        corrections->push_back(correctionWithCenter(target.baseCorrection,
                                                    candidate.center,
                                                    target.radiusVector));
    }

    return true;
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
    border-radius: 8px;
    padding: 4px;
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

#define m_reader m_globalReader
#define m_runner (currentInstance()->runner)
#define m_annotations (currentInstance()->annotations)
#define m_currentVideoPath m_globalVideoPath
#define m_currentFrame m_globalCurrentFrame
#define m_zoom m_globalZoom
#define m_showOverlay m_globalShowOverlay
#define m_usedFps m_globalUsedFps
#define m_playbackSpeed m_globalPlaybackSpeed
#define m_segmentStartFrame (currentInstance()->segmentStartFrame)
#define m_segmentEndFrame (currentInstance()->segmentEndFrame)
#define m_currentResult (currentInstance()->currentResult)
#define m_currentProfile (currentInstance()->currentProfile)
#define m_pendingAutoBatchRows (currentInstance()->pendingAutoBatchRows)
#define m_pendingAutoMatchedCorrections (currentInstance()->pendingAutoMatchedCorrections)

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    m_decodedFrameCache.setMaxCost(64 * 1024);
    setWindowIcon(QIcon(QStringLiteral(":/img/logo.png")));
    buildUi();
    buildMenus();
    refreshInstanceList();
    updateSplitLayout();
    updateAnnotationList();
    setWindowTitle(QStringLiteral("BeaconImageAnalyzer"));
    setMinimumSize(900, 620);
    resize(1320, 780);

    m_playTimer.setInterval(playbackIntervalMs());
    connect(&m_playTimer, &QTimer::timeout, this, &MainWindow::advancePlayingInstances);
    qApp->installEventFilter(this);
    QTimer::singleShot(0, this, &MainWindow::restoreLastSession);
}

MainWindow::~MainWindow()
{
    qDeleteAll(m_instances);
    m_instances.clear();
}

AnalyzerInstance* MainWindow::currentInstance()
{
    AnalyzerInstance* instance = instanceById(m_currentInstanceId);
    if (instance != nullptr)
    {
        return instance;
    }
    if (!m_instances.isEmpty())
    {
        m_currentInstanceId = m_instances.first()->id;
        return m_instances.first();
    }
    return nullptr;
}

const AnalyzerInstance* MainWindow::currentInstance() const
{
    for (const AnalyzerInstance* instance : m_instances)
    {
        if (instance != nullptr && instance->id == m_currentInstanceId)
        {
            return instance;
        }
    }
    return m_instances.isEmpty() ? nullptr : m_instances.first();
}

AnalyzerInstance* MainWindow::instanceById(int id)
{
    for (AnalyzerInstance* instance : m_instances)
    {
        if (instance != nullptr && instance->id == id)
        {
            return instance;
        }
    }
    return nullptr;
}

const AnalyzerInstance* MainWindow::instanceById(int id) const
{
    for (const AnalyzerInstance* instance : m_instances)
    {
        if (instance != nullptr && instance->id == id)
        {
            return instance;
        }
    }
    return nullptr;
}

AnalyzerInstance* MainWindow::createInstance(const QString& rootDir,
                                             const QString& algorithmPath,
                                             const QString& name,
                                             bool compileAlgorithm,
                                             QString* errorMessage)
{
    auto* instance = new AnalyzerInstance;
    instance->id = m_nextInstanceId++;
    instance->rootDir = QFileInfo(rootDir).absoluteFilePath();
    instance->algorithmPath = QFileInfo(algorithmPath).absoluteFilePath();
    instance->name = name.trimmed().isEmpty()
        ? QStringLiteral("实例 %1").arg(instance->id)
        : name.trimmed();

    if (compileAlgorithm)
    {
        QString compileError;
        const QString buildDir = QDir(instance->rootDir).absoluteFilePath(QStringLiteral("build"));
        const QFileInfo algorithmInfo(instance->algorithmPath);
        const bool isTwoBl3Firmware = algorithmInfo.fileName() == QStringLiteral("image.c")
                                      && QFileInfo::exists(algorithmInfo.dir().absoluteFilePath(
                                          QStringLiteral("image_params.c")));
        const bool loaded = isTwoBl3Firmware
                                ? instance->runner.loadTwoBl3Firmware(
                                      algorithmInfo.absolutePath(), buildDir, &compileError)
                                : instance->runner.loadSourceFile(
                                      instance->algorithmPath, buildDir, &compileError);
        if (!loaded)
        {
            delete instance;
            if (errorMessage != nullptr)
            {
                *errorMessage = compileError;
            }
            return nullptr;
        }
    }

    m_instances.push_back(instance);
    if (m_currentInstanceId < 0)
    {
        m_currentInstanceId = instance->id;
    }
    setInstanceVisible(instance->id, true);
    refreshInstanceList();
    updateSplitLayout();
    return instance;
}

bool MainWindow::ensureDefaultInstance(QString* errorMessage)
{
    if (!m_instances.isEmpty())
    {
        return true;
    }

    const QString rootDir = QDir(defaultInstancesRoot()).absoluteFilePath(QStringLiteral("default"));
    QString algorithmPath;
    if (!seedAlgorithmFolder(rootDir, defaultAlgorithmPath(), &algorithmPath, errorMessage))
    {
        return false;
    }
    return createInstance(rootDir, algorithmPath, QStringLiteral("默认实例"), true, errorMessage) != nullptr;
}

AnalyzerInstance* MainWindow::requireCurrentInstance(const QString& actionName)
{
    AnalyzerInstance* instance = currentInstance();
    if (instance != nullptr)
    {
        return instance;
    }

    QMessageBox::information(this,
                             actionName,
                             QStringLiteral("当前没有实例，请先加载或添加实例。"));
    return nullptr;
}

int MainWindow::slotForInstance(int instanceId) const
{
    for (int i = 0; i < m_splitSlotInstanceIds.size(); ++i)
    {
        if (m_splitSlotInstanceIds[i] == instanceId)
        {
            return i;
        }
    }
    return -1;
}

int MainWindow::slotAtGlobalPos(const QPoint& globalPos) const
{
    for (int i = 0; i < m_videoWidgets.size(); ++i)
    {
        const VideoWidget* widget = m_videoWidgets[i];
        if (widget != nullptr && widget->isVisible() && widget->rect().contains(widget->mapFromGlobal(globalPos)))
        {
            return i;
        }
    }
    return -1;
}

void MainWindow::setInstanceVisible(int instanceId, bool visible)
{
    if (m_splitSlotInstanceIds.size() != 4)
    {
        m_splitSlotInstanceIds = QVector<int>(4, -1);
    }

    const int existingSlot = slotForInstance(instanceId);
    if (!visible)
    {
        if (instanceId == m_currentInstanceId)
        {
            return;
        }
        if (existingSlot >= 0)
        {
            m_splitSlotInstanceIds[existingSlot] = -1;
        }
        return;
    }
    if (existingSlot >= 0)
    {
        return;
    }

    int visibleCount = 0;
    for (int id : m_splitSlotInstanceIds)
    {
        if (id >= 0)
        {
            ++visibleCount;
        }
    }
    if (visibleCount >= 4)
    {
        QMessageBox::warning(this, QStringLiteral("分屏显示"), QStringLiteral("最多同时勾选 4 个实例。"));
        return;
    }

    for (int i = 0; i < m_splitSlotInstanceIds.size(); ++i)
    {
        if (m_splitSlotInstanceIds[i] < 0)
        {
            m_splitSlotInstanceIds[i] = instanceId;
            return;
        }
    }
}

void MainWindow::selectInstance(int id)
{
    if (instanceById(id) == nullptr)
    {
        return;
    }
    m_currentInstanceId = id;
    setInstanceVisible(id, true);
    AnalyzerInstance* instance = instanceById(id);
    if (instance != nullptr)
    {
        m_annotationPanel->setCurrentFrameCorrections(instance->annotations.correctionsForFrame(m_currentFrame), true);
    }
    refreshInstanceList();
    updateSplitLayout();
    refreshCurrentInstanceUi();
}

void MainWindow::refreshInstanceList()
{
    if (m_instanceList == nullptr)
    {
        return;
    }

    QSignalBlocker blocker(m_instanceList);
    m_instanceList->clear();
    for (const AnalyzerInstance* instance : m_instances)
    {
        if (instance == nullptr)
        {
            continue;
        }
        QString displayName = instance->name;
        const quint32 buildId = instance->runner.algorithmBuildId();
        if (buildId != 0U)
        {
            const QString buildText = QString::number(buildId, 16)
                                          .rightJustified(8, QLatin1Char('0'))
                                          .toUpper();
            displayName += QStringLiteral("  [Build 0x%1]").arg(buildText);
        }
        auto* item = new QListWidgetItem(displayName);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(slotForInstance(instance->id) >= 0 ? Qt::Checked : Qt::Unchecked);
        item->setData(Qt::UserRole, instance->id);
        if (instance->id == m_currentInstanceId)
        {
            QFont font = item->font();
            font.setBold(true);
            item->setFont(font);
        }
        m_instanceList->addItem(item);
    }

    if (m_instances.isEmpty())
    {
        auto* item = new QListWidgetItem(QStringLiteral("暂无实例"));
        item->setFlags(Qt::NoItemFlags);
        m_instanceList->addItem(item);
    }
}

void MainWindow::updateCurrentVideoWidget()
{
    const int slot = slotForInstance(m_currentInstanceId);
    m_videoWidget = slot >= 0 && slot < m_videoWidgets.size() ? m_videoWidgets[slot] : nullptr;
    for (int i = 0; i < m_videoWidgets.size(); ++i)
    {
        m_videoWidgets[i]->setSelected(i == slot && m_splitSlotInstanceIds.value(i, -1) >= 0);
    }
}

void MainWindow::updateSplitLayout()
{
    if (m_videoGrid == nullptr || m_videoWidgets.size() != 4)
    {
        return;
    }

    while (QLayoutItem* item = m_videoGrid->takeAt(0))
    {
        delete item;
    }

    QVector<int> visibleSlots;
    for (int i = 0; i < m_splitSlotInstanceIds.size(); ++i)
    {
        if (m_splitSlotInstanceIds[i] >= 0)
        {
            visibleSlots.push_back(i);
        }
    }

    for (int row = 0; row < 2; ++row)
    {
        m_videoGrid->setRowMinimumHeight(row, 0);
        m_videoGrid->setRowStretch(row, 1);
    }
    for (int column = 0; column < 2; ++column)
    {
        m_videoGrid->setColumnMinimumWidth(column, 0);
        m_videoGrid->setColumnStretch(column, 1);
    }

    if (visibleSlots.size() <= 1)
    {
        const int slot = visibleSlots.isEmpty() ? 0 : visibleSlots.first();
        for (int i = 0; i < m_videoWidgets.size(); ++i)
        {
            m_videoWidgets[i]->setVisible(i == slot);
            if (visibleSlots.isEmpty())
            {
                m_videoWidgets[i]->setImage(QImage());
                m_videoWidgets[i]->setPixelSourceImage(QImage());
                m_videoWidgets[i]->setText(QStringLiteral("暂无实例"));
            }
        }
        m_videoGrid->addWidget(m_videoWidgets[slot], 0, 0, 2, 2);
    }
    else if (visibleSlots.size() == 2)
    {
        for (int i = 0; i < m_videoWidgets.size(); ++i)
        {
            m_videoWidgets[i]->setVisible(false);
        }
        for (int column = 0; column < visibleSlots.size(); ++column)
        {
            const int slot = visibleSlots[column];
            m_videoWidgets[slot]->setVisible(true);
            m_videoGrid->addWidget(m_videoWidgets[slot], 0, column);
        }
    }
    else if (visibleSlots.size() == 3)
    {
        for (int i = 0; i < m_videoWidgets.size(); ++i)
        {
            m_videoWidgets[i]->setVisible(false);
        }
        for (int index = 0; index < visibleSlots.size(); ++index)
        {
            const int slot = visibleSlots[index];
            m_videoWidgets[slot]->setVisible(true);
            if (index < 2)
            {
                m_videoGrid->addWidget(m_videoWidgets[slot], 0, index);
            }
            else
            {
                m_videoGrid->addWidget(m_videoWidgets[slot], 1, 0, 1, 2);
            }
        }
    }
    else
    {
        for (int i = 0; i < m_videoWidgets.size(); ++i)
        {
            m_videoWidgets[i]->setVisible(true);
            m_videoGrid->addWidget(m_videoWidgets[i], i / 2, i % 2);
        }
    }

    updateCurrentVideoWidget();
    renderAllDisplayedInstances();
}

void MainWindow::handleSlotActivated(int slot)
{
    if (slot < 0 || slot >= m_splitSlotInstanceIds.size())
    {
        return;
    }
    const int instanceId = m_splitSlotInstanceIds[slot];
    if (instanceId >= 0)
    {
        selectInstance(instanceId);
    }
}

void MainWindow::handleSlotMiddleDragStarted(int slot)
{
    int visibleCount = 0;
    for (int id : m_splitSlotInstanceIds)
    {
        if (id >= 0)
        {
            ++visibleCount;
        }
    }
    m_dragSourceSlot = visibleCount >= 2 ? slot : -1;
}

void MainWindow::handleSlotMiddleDragReleased(int slot, const QPoint& globalPos)
{
    Q_UNUSED(slot);
    if (m_dragSourceSlot < 0 || m_dragSourceSlot >= m_splitSlotInstanceIds.size())
    {
        return;
    }
    const int targetSlot = slotAtGlobalPos(globalPos);
    if (targetSlot >= 0 && targetSlot < m_splitSlotInstanceIds.size() && targetSlot != m_dragSourceSlot)
    {
        std::swap(m_splitSlotInstanceIds[m_dragSourceSlot], m_splitSlotInstanceIds[targetSlot]);
        updateSplitLayout();
    }
    m_dragSourceSlot = -1;
}

void MainWindow::handleSlotContextCorrection(int slot, const QPointF& imagePoint, const QPoint& globalPos)
{
    if (slot < 0 || slot >= m_splitSlotInstanceIds.size() ||
        m_splitSlotInstanceIds[slot] != m_currentInstanceId ||
        !m_reader.isOpen())
    {
        return;
    }

    const QVector<int> targetIndices = targetIndicesNearPoint(imagePoint, 10.0);
    if (targetIndices.isEmpty())
    {
        QMenu menu(this);
        QAction* missedAction = menu.addAction(QStringLiteral("漏检"));
        QAction* otherAction = menu.addAction(QStringLiteral("其他"));
        QAction* selected = menu.exec(globalPos);
        if (selected == missedAction)
        {
            CorrectionShape correction;
            if (promptMissedCircle(imagePoint, &correction))
            {
                addQuickCorrection(correction);
            }
        }
        else if (selected == otherAction)
        {
            QString description;
            if (promptDescription(QStringLiteral("其他"), &description))
            {
                CorrectionShape correction;
                correction.name = annotationTypeDisplayName(QStringLiteral("other"));
                correction.frame = m_currentFrame;
                correction.errorType = QStringLiteral("other");
                correction.errorTypes = QStringList{ correction.errorType };
                correction.description = description;
                addQuickCorrection(correction);
            }
        }
        return;
    }

    if (targetIndices.size() == 1)
    {
        runSingleTargetQuickCorrection(targetIndices.first(), globalPos);
        return;
    }

    QMenu targetMenu(this);
    for (int index : targetIndices)
    {
        QAction* action = targetMenu.addAction(QStringLiteral("目标 #%1").arg(index));
        action->setData(index);
    }
    QAction* selectedTarget = targetMenu.exec(globalPos);
    if (selectedTarget != nullptr)
    {
        runSingleTargetQuickCorrection(selectedTarget->data().toInt(), globalPos);
    }
}

QVector<int> MainWindow::targetIndicesNearPoint(const QPointF& imagePoint, double radiusPixels) const
{
    QVector<int> indices;
    const beacon_result_t& result = m_currentResult;
    for (int i = 0; i < result.count && i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        const beacon_circle_t& circle = result.circles[i];
        if (circle.valid == 0)
        {
            continue;
        }

        const QPointF circlePoint = FrameRenderer::algorithmToImagePoint(circle.x, circle.y);
        if (QLineF(circlePoint, imagePoint).length() <= circle.radius + radiusPixels)
        {
            indices.push_back(i);
        }
    }
    return indices;
}

bool MainWindow::promptExpectedIndex(const QString& title, int* expectedIndex)
{
    if (expectedIndex == nullptr)
    {
        return false;
    }
    bool ok = false;
    const int value = QInputDialog::getInt(this,
                                           title,
                                           QStringLiteral("正确序号"),
                                           0,
                                           0,
                                           BEACON_MAX_CIRCLE_COUNT - 1,
                                           1,
                                           &ok);
    if (!ok)
    {
        return false;
    }
    *expectedIndex = value;
    return true;
}

bool MainWindow::promptDescription(const QString& title, QString* description)
{
    if (description == nullptr)
    {
        return false;
    }
    bool ok = false;
    const QString text = QInputDialog::getMultiLineText(this,
                                                        title,
                                                        QStringLiteral("描述"),
                                                        QString(),
                                                        &ok).trimmed();
    if (!ok || text.isEmpty())
    {
        return false;
    }
    *description = text;
    return true;
}

bool MainWindow::promptMissedCircle(const QPointF& center, CorrectionShape* correction)
{
    if (correction == nullptr)
    {
        return false;
    }

    int expectedIndex = -1;
    if (!promptExpectedIndex(QStringLiteral("漏检"), &expectedIndex))
    {
        return false;
    }

    bool ok = false;
    const double radius = QInputDialog::getDouble(this,
                                                  QStringLiteral("漏检"),
                                                  QStringLiteral("目标圆半径"),
                                                  5.0,
                                                  1.0,
                                                  100.0,
                                                  1,
                                                  &ok);
    if (!ok)
    {
        return false;
    }

    correction->name = annotationTypeDisplayName(QStringLiteral("missed_detection"));
    correction->frame = m_currentFrame;
    correction->errorType = QStringLiteral("missed_detection");
    correction->errorTypes = QStringList{ correction->errorType };
    correction->expectedIndex = expectedIndex;
    correction->errorCircles = QVector<ErrorCircle>{ ErrorCircle{ -1, expectedIndex } };
    correction->shapeType = QStringLiteral("circle");
    correction->points = QVector<QPointF>{ center, center + QPointF(radius, 0.0) };
    return true;
}

void MainWindow::addQuickCorrection(const CorrectionShape& correction)
{
    CorrectionShape saved = correction;
    if (saved.name.trimmed().isEmpty())
    {
        saved.name = annotationTypeDisplayName(saved.errorType);
    }
    m_annotations.addCorrection(saved);
    m_annotationPanel->setCurrentFrameCorrections(m_annotations.correctionsForFrame(m_currentFrame), true);
    updateAnnotationList();
    showFrame(m_currentFrame);
}

void MainWindow::runSingleTargetQuickCorrection(int circleIndex, const QPoint& globalPos)
{
    QMenu menu(this);
    QAction* falsePositiveAction = menu.addAction(QStringLiteral("误检"));
    QAction* wrongOrderAction = menu.addAction(QStringLiteral("排序错误"));
    QAction* targetJumpAction = menu.addAction(QStringLiteral("目标跳变"));
    QAction* otherAction = menu.addAction(QStringLiteral("其他"));
    QAction* selected = menu.exec(globalPos);
    if (selected == nullptr)
    {
        return;
    }

    CorrectionShape correction;
    correction.frame = m_currentFrame;
    correction.lineWidth = 1;

    if (selected == falsePositiveAction)
    {
        correction.errorType = QStringLiteral("false_positive");
        correction.errorTypes = QStringList{ correction.errorType };
        correction.errorCircles = QVector<ErrorCircle>{ ErrorCircle{ circleIndex, -1 } };
    }
    else if (selected == wrongOrderAction || selected == targetJumpAction)
    {
        int expectedIndex = -1;
        if (!promptExpectedIndex(selected == wrongOrderAction
                                     ? QStringLiteral("排序错误")
                                     : QStringLiteral("目标跳变"),
                                 &expectedIndex))
        {
            return;
        }
        correction.errorType = selected == wrongOrderAction
            ? QStringLiteral("wrong_order")
            : QStringLiteral("target_jump");
        correction.errorTypes = QStringList{ correction.errorType };
        correction.expectedIndex = expectedIndex;
        correction.errorCircles = QVector<ErrorCircle>{ ErrorCircle{ circleIndex, expectedIndex } };
    }
    else if (selected == otherAction)
    {
        QString description;
        if (!promptDescription(QStringLiteral("其他"), &description))
        {
            return;
        }
        correction.errorType = QStringLiteral("other");
        correction.errorTypes = QStringList{ correction.errorType };
        correction.description = description;
        correction.errorCircles = QVector<ErrorCircle>{ ErrorCircle{ circleIndex, -1 } };
    }

    correction.name = annotationTypeDisplayName(correction.errorType);
    addQuickCorrection(correction);
}

void MainWindow::renderInstance(AnalyzerInstance* instance)
{
    if (instance == nullptr)
    {
        return;
    }

    if (m_liveMode && !m_liveFrame.isNull())
    {
        const beacon_result_t result = processCausalFrame(instance, m_currentFrame, m_liveFrame);
        renderInstance(instance, m_liveFrame, result, m_currentFrame);
        return;
    }

    if (!m_reader.isOpen())
    {
        return;
    }

    QImage gray;
    QString error;
    const int frameIndex = qBound(0, m_currentFrame, qMax(0, m_reader.frameCount() - 1));
    if (!readFrameCached(frameIndex, &gray, &error))
    {
        return;
    }
    const beacon_result_t result = processCausalFrame(instance, frameIndex, gray);
    renderInstance(instance, gray, result, frameIndex);
}

void MainWindow::resetInstanceTemporal(AnalyzerInstance* instance)
{
    if (instance == nullptr)
    {
        return;
    }
    instance->runner.resetTemporal();
    instance->temporalFrameCache.clear();
    instance->temporalProfileCache.clear();
    instance->temporalDetectionMetricsCache.clear();
    instance->temporalHorizonCache.clear();
    instance->currentDetectionMetrics = {};
    instance->currentHorizon = {};
    instance->temporalLastFrame = -1;
}

bool MainWindow::rebuildTemporalCacheToFrame(AnalyzerInstance* instance,
                                             int targetFrame,
                                             QString* errorMessage)
{
    if (instance == nullptr || !m_reader.isOpen())
    {
        return false;
    }

    const int cachedLastFrame = instance->temporalLastFrame;
    const bool hasCompletePrefix = cachedLastFrame >= 0 &&
        instance->temporalFrameCache.size() == cachedLastFrame + 1 &&
        instance->temporalProfileCache.size() == cachedLastFrame + 1 &&
        instance->temporalDetectionMetricsCache.size() == cachedLastFrame + 1 &&
        instance->temporalHorizonCache.size() == cachedLastFrame + 1 &&
        instance->temporalFrameCache.contains(0) &&
        instance->temporalFrameCache.contains(cachedLastFrame) &&
        instance->temporalProfileCache.contains(0) &&
        instance->temporalProfileCache.contains(cachedLastFrame) &&
        instance->temporalDetectionMetricsCache.contains(0) &&
        instance->temporalDetectionMetricsCache.contains(cachedLastFrame) &&
        instance->temporalHorizonCache.contains(0) &&
        instance->temporalHorizonCache.contains(cachedLastFrame);

    int firstFrame = 0;
    if (hasCompletePrefix && targetFrame > cachedLastFrame)
    {
        firstFrame = cachedLastFrame + 1;
    }
    else
    {
        resetInstanceTemporal(instance);
    }

    for (int frame = firstFrame; frame <= targetFrame; ++frame)
    {
        QImage frameImage;
        QString readError;
        if (!readFrameCached(frame, &frameImage, &readError))
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = readError;
            }
            return false;
        }
        instance->runner.setFrameTelemetry(frameTelemetryForFrame(frame));
        const beacon_result_t result = instance->runner.process(frameImage);
        const AlgorithmProcessProfile profile = instance->runner.lastProcessProfile();
        const AlgorithmDetectionMetrics metrics = instance->runner.lastDetectionMetrics();
        const AlgorithmHorizonCurve horizon = instance->runner.horizonCurve();
        instance->temporalFrameCache.insert(frame, result);
        instance->temporalProfileCache.insert(frame, profile);
        instance->temporalDetectionMetricsCache.insert(frame, metrics);
        instance->temporalHorizonCache.insert(frame, horizon);
        instance->temporalLastFrame = frame;
    }
    return true;
}

beacon_result_t MainWindow::processCausalFrame(AnalyzerInstance* instance,
                                               int frameIndex,
                                               const QImage& gray)
{
    beacon_result_t empty = {};
    if (instance == nullptr || gray.isNull())
    {
        return empty;
    }

    if (instance->temporalFrameCache.contains(frameIndex))
    {
        instance->currentProfile = instance->temporalProfileCache.value(frameIndex);
        instance->currentDetectionMetrics = instance->temporalDetectionMetricsCache.value(frameIndex);
        instance->currentHorizon = instance->temporalHorizonCache.value(frameIndex);
        return instance->temporalFrameCache.value(frameIndex);
    }

    if (m_liveMode || frameIndex == instance->temporalLastFrame + 1)
    {
        if (frameIndex != instance->temporalLastFrame + 1)
        {
            resetInstanceTemporal(instance);
        }
        instance->runner.setFrameTelemetry(frameTelemetryForFrame(frameIndex));
        const beacon_result_t result = instance->runner.process(gray);
        const AlgorithmProcessProfile profile = instance->runner.lastProcessProfile();
        const AlgorithmDetectionMetrics metrics = instance->runner.lastDetectionMetrics();
        const AlgorithmHorizonCurve horizon = instance->runner.horizonCurve();
        instance->temporalFrameCache.insert(frameIndex, result);
        instance->temporalProfileCache.insert(frameIndex, profile);
        instance->temporalDetectionMetricsCache.insert(frameIndex, metrics);
        instance->temporalHorizonCache.insert(frameIndex, horizon);
        instance->temporalLastFrame = frameIndex;
        instance->currentProfile = profile;
        instance->currentDetectionMetrics = metrics;
        instance->currentHorizon = horizon;
        return result;
    }

    QString error;
    if (rebuildTemporalCacheToFrame(instance, frameIndex, &error))
    {
        instance->currentProfile = instance->temporalProfileCache.value(frameIndex);
        instance->currentDetectionMetrics = instance->temporalDetectionMetricsCache.value(frameIndex);
        instance->currentHorizon = instance->temporalHorizonCache.value(frameIndex);
        return instance->temporalFrameCache.value(frameIndex);
    }

    instance->runner.setFrameTelemetry(frameTelemetryForFrame(frameIndex));
    const beacon_result_t result = instance->runner.process(gray);
    instance->currentProfile = instance->runner.lastProcessProfile();
    instance->currentDetectionMetrics = instance->runner.lastDetectionMetrics();
    instance->currentHorizon = instance->runner.horizonCurve();
    return result;
}

void MainWindow::renderInstance(AnalyzerInstance* instance,
                                const QImage& gray,
                                const beacon_result_t& result,
                                int frameIndex)
{
    if (instance == nullptr || gray.isNull())
    {
        return;
    }

    instance->currentResult = result;
    const QVector<CorrectionShape> corrections = instance->annotations.correctionsForFrame(frameIndex);
    QImage displayImage = gray;
    if (viewMode() == QStringLiteral("binary"))
    {
        const QImage binary = instance->runner.binaryImage(gray);
        if (!binary.isNull())
        {
            displayImage = binary;
        }
    }
    QVector<CorrectionShape> visibleCorrections = corrections;
    if (instance->id == m_currentInstanceId && m_annotationPanel != nullptr)
    {
        visibleCorrections = m_annotationPanel->draftCorrections();
    }
    const QImage rendered = FrameRenderer::render(displayImage,
                                                  result,
                                                  visibleCorrections,
                                                  1,
                                                  m_showOverlay,
                                                  &instance->currentHorizon);

    const int slot = slotForInstance(instance->id);
    if (slot >= 0 && slot < m_videoWidgets.size())
    {
        m_videoWidgets[slot]->setFrameGeometry(gray.size(), 1);
        m_videoWidgets[slot]->setPixelSourceImage(gray);
        m_videoWidgets[slot]->setImage(rendered);
    }
}

void MainWindow::renderAllDisplayedInstances()
{
    for (int instanceId : m_splitSlotInstanceIds)
    {
        renderInstance(instanceById(instanceId));
    }
    updateCurrentVideoWidget();
}

void MainWindow::renderAllDisplayedInstances(const QImage& gray,
                                             const QVector<QPair<AnalyzerInstance*, beacon_result_t>>& results)
{
    for (int instanceId : m_splitSlotInstanceIds)
    {
        AnalyzerInstance* instance = instanceById(instanceId);
        if (instance == nullptr)
        {
            continue;
        }

        beacon_result_t result = {};
        bool hasResult = false;
        for (const auto& item : results)
        {
            if (item.first == instance)
            {
                result = item.second;
                hasResult = true;
                break;
            }
        }
        if (!hasResult)
        {
            result = processCausalFrame(instance, m_currentFrame, gray);
        }
        renderInstance(instance, gray, result, m_currentFrame);
    }
    updateCurrentVideoWidget();
}

void MainWindow::showLiveFrame(const QImage& gray, quint16 localPort, const QString& peerName)
{
    if (gray.isNull())
    {
        return;
    }

    const bool wasLiveMode = m_liveMode;
    pause();
    m_liveMode = true;
    m_liveFrame = gray.convertToFormat(QImage::Format_Grayscale8);
    m_liveLocalPort = localPort;
    m_livePeerName = peerName;
    ++m_liveFrameIndex;
    m_currentFrame = qMax(0, m_liveFrameIndex);
    if (!wasLiveMode)
    {
        for (AnalyzerInstance* instance : m_instances)
        {
            resetInstanceTemporal(instance);
        }
    }

    QVector<QPair<AnalyzerInstance*, beacon_result_t>> results;
    for (AnalyzerInstance* instance : m_instances)
    {
        if (instance == nullptr)
        {
            continue;
        }
        const beacon_result_t result = processCausalFrame(instance, m_currentFrame, m_liveFrame);
        instance->currentResult = result;
        results.push_back(qMakePair(instance, result));
    }

    const beacon_result_t result = m_currentResult;
    if (m_annotationPanel != nullptr)
    {
        m_annotationPanel->setCurrentContext(m_currentFrame, frameTime(m_currentFrame), result.count);
        m_annotationPanel->setCurrentFrameCorrections(m_annotations.correctionsForFrame(m_currentFrame));
    }
    renderAllDisplayedInstances(m_liveFrame, results);

    m_updatingControls = true;
    m_slider->setRange(0, qMax(m_currentFrame, m_slider->maximum()));
    m_frameSpin->setRange(0, qMax(m_currentFrame, m_frameSpin->maximum()));
    m_timeSpin->setRange(0.0, qMax(frameTime(m_currentFrame), m_timeSpin->maximum()));
    m_slider->setValue(m_currentFrame);
    m_frameSpin->setValue(m_currentFrame);
    m_timeSpin->setValue(frameTime(m_currentFrame));
    m_updatingControls = false;

    m_videoInfoLabel->setText(QStringLiteral("TCP 实时 | 本地端口 %1 | 来源 %2 | %3x%4 | 帧 %5")
                                  .arg(localPort)
                                  .arg(peerName)
                                  .arg(m_liveFrame.width())
                                  .arg(m_liveFrame.height())
                                  .arg(m_currentFrame));
    updateFrameInfo(result);
    m_frameInfoLabel->setText(QStringLiteral("TCP 实时帧: %1\n本地端口: %2\n来源: %3\n信标: %4  车灯: %5  总目标: %6")
                                  .arg(m_currentFrame)
                                  .arg(m_liveLocalPort)
                                  .arg(m_livePeerName)
                                  .arg(BeaconResultUtils::beaconCount(result))
                                  .arg(BeaconResultUtils::carLampCount(result))
                                  .arg(BeaconResultUtils::totalTargetCount(result)));
    m_frameInfoLabel->setText(m_frameInfoLabel->text() + QStringLiteral("\n") +
                              AlgorithmProcessProfiler::format(m_currentProfile, m_usedFps));
    updateTcpStatusLabel(QStringLiteral("最近收到图像：%1").arg(peerName));
}

void MainWindow::refreshCurrentInstanceUi()
{
    AnalyzerInstance* instance = currentInstance();
    if (instance == nullptr)
    {
        return;
    }
    if (m_reader.isOpen())
    {
        m_slider->setRange(0, qMax(0, m_reader.frameCount() - 1));
        m_frameSpin->setRange(0, qMax(0, m_reader.frameCount() - 1));
        m_timeSpin->setRange(0.0, frameTime(qMax(0, m_reader.frameCount() - 1)));
        m_annotationPanel->setVideoFrameRange(0, qMax(0, m_reader.frameCount() - 1));
        showFrame(m_currentFrame);
    }
    else if (m_liveMode && !m_liveFrame.isNull())
    {
        renderAllDisplayedInstances();
    }
    else
    {
        m_videoInfoLabel->setText(QStringLiteral("未打开视频"));
        m_frameInfoLabel->setText(QStringLiteral("Frame: -"));
        m_resultText->clear();
        m_annotationPanel->setAnnotations(instance->annotations.records(), instance->annotations.corrections());
        m_annotationPanel->setCurrentFrameCorrections(QVector<CorrectionShape>(), true);
    }
    updatePlayPauseButton();
}

void MainWindow::advancePlayingInstances()
{
    if (!m_globalPlaying || !m_reader.isOpen())
    {
        m_playTimer.stop();
        updatePlayPauseButton();
        return;
    }
    if (m_currentFrame + 1 >= m_reader.frameCount())
    {
        m_globalPlaying = false;
        m_playTimer.stop();
        updatePlayPauseButton();
        return;
    }
    showFrame(m_currentFrame + 1);
}

void MainWindow::buildUi()
{
    auto* central = new QWidget(this);
    central->setObjectName(QStringLiteral("AppRoot"));
    auto* root = new QHBoxLayout(central);
    root->setContentsMargins(6, 8, 8, 8);
    root->setSpacing(8);

    auto* rail = new QFrame(central);
    rail->setObjectName(QStringLiteral("Rail"));
    rail->setFixedWidth(56);
    auto* railLayout = new QVBoxLayout(rail);
    railLayout->setContentsMargins(8, 10, 8, 10);
    railLayout->setSpacing(10);

    auto* brand = new QLabel(rail);
    brand->setObjectName(QStringLiteral("Brand"));
    brand->setAlignment(Qt::AlignCenter);
    brand->setFixedSize(40, 40);
    brand->setPixmap(QPixmap(QStringLiteral(":/img/logo.png")).scaled(28, 28, Qt::KeepAspectRatio, Qt::SmoothTransformation));
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
    auto* tcpRailButton = makeRailButton(rail, QStyle::SP_ComputerIcon, QStringLiteral("TCP 接收"));
    auto* logReplayRailButton = makeRailButton(rail, QStyle::SP_FileDialogDetailedView, QStringLiteral("日志回放"));
    auto* horizonRailButton = makeRailButton(rail, QStyle::SP_FileDialogContentsView, QStringLiteral("地平线标定"));
    auto* loadInstanceRailButton = makeRailButton(rail, QStyle::SP_DialogOpenButton, QStringLiteral("加载实例"));
    auto* addInstanceRailButton = makeRailButton(rail, QStyle::SP_FileDialogNewFolder, QStringLiteral("添加实例"));
    auto* saveRailButton = makeRailButton(rail, QStyle::SP_DialogSaveButton, QStringLiteral("保存标注"));
    auto* loadRailButton = makeRailButton(rail, QStyle::SP_FileDialogContentsView, QStringLiteral("读取标注"));
    auto* exportAviRailButton = makeRailButton(rail, QStyle::SP_DialogApplyButton, QStringLiteral("导出标注 AVI"));
    auto* exportCsvRailButton = makeRailButton(rail, QStyle::SP_FileIcon, QStringLiteral("导出 CSV"));
    auto* exitRailButton = makeRailButton(rail, QStyle::SP_DialogCloseButton, QStringLiteral("退出"));
    railLayout->addWidget(openRailButton, 0, Qt::AlignHCenter);
    railLayout->addWidget(tcpRailButton, 0, Qt::AlignHCenter);
    railLayout->addWidget(logReplayRailButton, 0, Qt::AlignHCenter);
    railLayout->addWidget(horizonRailButton, 0, Qt::AlignHCenter);
    railLayout->addWidget(loadInstanceRailButton, 0, Qt::AlignHCenter);
    railLayout->addWidget(addInstanceRailButton, 0, Qt::AlignHCenter);
    railLayout->addSpacing(10);
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
    workspaceLayout->setSpacing(8);

    auto* videoCard = new QFrame(workspace);
    videoCard->setObjectName(QStringLiteral("VideoCard"));
    auto* videoLayout = new QVBoxLayout(videoCard);
    videoLayout->setContentsMargins(8, 8, 8, 10);
    videoLayout->setSpacing(8);

    auto* feedHeader = new QHBoxLayout;
    feedHeader->setSpacing(8);
    auto* feedTitle = new QLabel(QStringLiteral("IR BEACON FEED"), videoCard);
    feedTitle->setObjectName(QStringLiteral("FeedTitle"));
    m_videoInfoLabel = new QLabel(QStringLiteral("未打开视频"), videoCard);
    m_videoInfoLabel->setObjectName(QStringLiteral("FeedMeta"));
    m_videoInfoLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_videoInfoLabel->setWordWrap(true);
    m_tcpStatusLabel = new QLabel(QStringLiteral("TCP 未监听"), videoCard);
    m_tcpStatusLabel->setObjectName(QStringLiteral("Pill"));
    m_tcpStatusLabel->setAlignment(Qt::AlignCenter);
    m_tcpStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_tcpStatusLabel->setWordWrap(true);
    auto* feedPill = new QLabel(QStringLiteral("RAW · BINARY · OVERLAY"), videoCard);
    feedPill->setObjectName(QStringLiteral("Pill"));
    feedPill->setAlignment(Qt::AlignCenter);
    feedHeader->addWidget(feedTitle);
    feedHeader->addWidget(m_videoInfoLabel, 1);
    feedHeader->addWidget(m_tcpStatusLabel);
    feedHeader->addWidget(feedPill);

    videoLayout->addLayout(feedHeader);

    auto* instanceRow = new QHBoxLayout;
    instanceRow->setSpacing(8);
    m_instanceList = new QListWidget(videoCard);
    m_instanceList->setMaximumHeight(78);
    m_instanceList->setSelectionMode(QAbstractItemView::SingleSelection);
    auto* loadInstanceButton = new QPushButton(QStringLiteral("加载实例"), videoCard);
    auto* addInstanceButton = new QPushButton(QStringLiteral("添加实例"), videoCard);
    auto* deleteInstanceButton = new QPushButton(QStringLiteral("删除实例"), videoCard);
    instanceRow->addWidget(m_instanceList, 1);
    instanceRow->addWidget(loadInstanceButton);
    instanceRow->addWidget(addInstanceButton);
    instanceRow->addWidget(deleteInstanceButton);
    videoLayout->addLayout(instanceRow);

    auto* gridHost = new QWidget(videoCard);
    m_videoGrid = new QGridLayout(gridHost);
    m_videoGrid->setContentsMargins(0, 0, 0, 0);
    m_videoGrid->setSpacing(8);
    m_splitSlotInstanceIds = QVector<int>(4, -1);
    for (int slot = 0; slot < 4; ++slot)
    {
        auto* widget = new VideoWidget(gridHost);
        widget->setText(QStringLiteral("空分屏"));
        m_videoWidgets.push_back(widget);
        connect(widget, &VideoWidget::activated, this, [this, slot]() {
            handleSlotActivated(slot);
        });
        connect(widget, &VideoWidget::middleDragStarted, this, [this, slot]() {
            handleSlotMiddleDragStarted(slot);
        });
        connect(widget, &VideoWidget::middleDragReleased, this, [this, slot](const QPoint& globalPos) {
            handleSlotMiddleDragReleased(slot, globalPos);
        });
        connect(widget, &VideoWidget::contextCorrectionRequested, this, [this, slot](const QPointF& imagePoint, const QPoint& globalPos) {
            handleSlotContextCorrection(slot, imagePoint, globalPos);
        });
        connect(widget, &VideoWidget::correctionShapeFinished, this, &MainWindow::addCorrectionShape);
        connect(widget, &VideoWidget::hoverPixelChanged, this, [this, slot](int x, int y, int gray, bool valid) {
            if (slot >= 0 && slot < m_splitSlotInstanceIds.size() &&
                m_splitSlotInstanceIds[slot] == m_currentInstanceId)
            {
                updateHoverPixelInfo(x, y, gray, valid);
            }
        });
    }
    m_videoWidget = m_videoWidgets.first();
    videoLayout->addWidget(gridHost, 1);
    workspaceLayout->addWidget(videoCard, 1);

    auto* rightScroll = new QScrollArea;
    rightScroll->setObjectName(QStringLiteral("RightScroll"));
    rightScroll->setWidgetResizable(true);
    rightScroll->setFrameShape(QFrame::NoFrame);
    rightScroll->setMinimumWidth(360);
    rightScroll->setMaximumWidth(720);

    auto* rightPanel = new QWidget(rightScroll);
    rightPanel->setObjectName(QStringLiteral("RightPanel"));
    rightPanel->setMinimumWidth(340);
    auto* rightLayout = new QVBoxLayout(rightPanel);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(12);
    m_frameInfoLabel = new QLabel(QStringLiteral("Frame: -"), rightPanel);
    m_frameInfoLabel->setObjectName(QStringLiteral("SoftLabel"));
    m_frameInfoLabel->setWordWrap(true);
    m_pixelInfoLabel = new QLabel(QStringLiteral("鼠标: -"), rightPanel);
    m_pixelInfoLabel->setObjectName(QStringLiteral("Pill"));
    m_pixelInfoLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_pixelInfoLabel->setWordWrap(true);
    m_currentAnnotationsLabel = new QLabel(QStringLiteral("当前帧标注: -"), rightPanel);
    m_currentAnnotationsLabel->setObjectName(QStringLiteral("SoftLabel"));
    m_currentAnnotationsLabel->setWordWrap(true);
    m_currentAnnotationsLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_resultText = new QTextEdit(rightPanel);
    m_resultText->setReadOnly(true);
    m_resultText->setMinimumHeight(160);
    m_resultText->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_annotationPanel = new AnnotationPanel(rightPanel);

    auto* infoGroup = new QGroupBox(QStringLiteral("当前帧信息"), rightPanel);
    auto* infoLayout = new QVBoxLayout(infoGroup);
    infoLayout->setContentsMargins(12, 8, 12, 12);
    infoLayout->setSpacing(8);
    infoLayout->addWidget(m_frameInfoLabel);
    infoLayout->addWidget(m_pixelInfoLabel);
    infoLayout->addWidget(m_currentAnnotationsLabel);
    infoLayout->addWidget(m_resultText, 1);

    auto* annotationGroup = new QGroupBox(QStringLiteral("错误标注"), rightPanel);
    auto* annotationLayout = new QVBoxLayout(annotationGroup);
    annotationLayout->setContentsMargins(12, 8, 12, 12);
    annotationLayout->addWidget(m_annotationPanel);

    rightLayout->addWidget(infoGroup);
    rightLayout->addWidget(annotationGroup, 1);

    rightScroll->setWidget(rightPanel);
    splitter->addWidget(workspace);
    splitter->addWidget(rightScroll);
    splitter->setStretchFactor(0, 4);
    splitter->setStretchFactor(1, 2);
    splitter->setCollapsible(0, false);
    splitter->setCollapsible(1, true);

    auto* controlsFrame = new QFrame(workspace);
    controlsFrame->setObjectName(QStringLiteral("ControlConsole"));
    controlsFrame->setMinimumHeight(156);
    controlsFrame->setMaximumHeight(220);
    controlsFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    auto* controls = new QVBoxLayout(controlsFrame);
    controls->setContentsMargins(14, 12, 14, 12);
    controls->setSpacing(10);
    auto* controlsTitle = new QLabel(QStringLiteral("PLAYBACK CONSOLE"), controlsFrame);
    controlsTitle->setObjectName(QStringLiteral("ConsoleTitle"));
    auto* playbackRow = new QHBoxLayout;
    playbackRow->setSpacing(8);
    m_playPauseButton = new QPushButton(QStringLiteral("播放"), controlsFrame);
    m_playPauseButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    m_playPauseButton->setProperty("role", QStringLiteral("primary"));
    auto* previousButton = new QPushButton(QStringLiteral("上一帧"), controlsFrame);
    previousButton->setIcon(style()->standardIcon(QStyle::SP_MediaSeekBackward));
    auto* nextButton = new QPushButton(QStringLiteral("下一帧"), controlsFrame);
    nextButton->setIcon(style()->standardIcon(QStyle::SP_MediaSeekForward));
    auto* jumpFrameButton = new QPushButton(QStringLiteral("跳转帧"), controlsFrame);
    auto* jumpTimeButton = new QPushButton(QStringLiteral("跳转时间"), controlsFrame);

    m_slider = new QSlider(Qt::Horizontal, controlsFrame);
    m_slider->setRange(0, 0);
    m_slider->setMinimumWidth(120);
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
    m_speedCombo = new QComboBox(controlsFrame);
    m_speedCombo->addItem(QStringLiteral("1/8倍速"), 0.125);
    m_speedCombo->addItem(QStringLiteral("1/4倍速"), 0.25);
    m_speedCombo->addItem(QStringLiteral("1/2倍速"), 0.5);
    m_speedCombo->addItem(QStringLiteral("正常1倍速"), 1.0);
    m_speedCombo->addItem(QStringLiteral("2倍速"), 2.0);
    m_speedCombo->addItem(QStringLiteral("4倍速"), 4.0);
    m_speedCombo->addItem(QStringLiteral("8倍速"), 8.0);
    m_speedCombo->setCurrentIndex(3);
    m_speedCombo->setFixedWidth(118);
    m_autoPauseEnableCheck = new QCheckBox(QStringLiteral("自动暂停"), controlsFrame);
    m_autoPauseJumpCheck = new QCheckBox(QStringLiteral("目标跳变"), controlsFrame);
    m_autoPauseCountCheck = new QCheckBox(QStringLiteral("数量变化"), controlsFrame);
    m_autoPauseJumpThresholdSpin = new QDoubleSpinBox(controlsFrame);
    m_autoPauseJumpThresholdSpin->setRange(0.0, 1000.0);
    m_autoPauseJumpThresholdSpin->setDecimals(1);
    m_autoPauseJumpThresholdSpin->setValue(10.0);
    m_autoPauseJumpThresholdSpin->setSuffix(QStringLiteral(" px"));
    m_autoPauseJumpThresholdSpin->setFixedWidth(92);
    auto* viewLabel = new QLabel(QStringLiteral("视图"), controlsFrame);
    viewLabel->setObjectName(QStringLiteral("SoftLabel"));
    auto* speedLabel = new QLabel(QStringLiteral("倍速"), controlsFrame);
    speedLabel->setObjectName(QStringLiteral("SoftLabel"));
    auto* frameLabel = new QLabel(QStringLiteral("Frame"), controlsFrame);
    frameLabel->setObjectName(QStringLiteral("SoftLabel"));
    auto* timeLabel = new QLabel(QStringLiteral("Time"), controlsFrame);
    timeLabel->setObjectName(QStringLiteral("SoftLabel"));

    auto* transportRow = new QHBoxLayout;
    transportRow->setSpacing(8);
    transportRow->addWidget(m_playPauseButton);
    transportRow->addWidget(previousButton);
    transportRow->addWidget(nextButton);
    transportRow->addSpacing(10);
    transportRow->addWidget(viewLabel);
    transportRow->addWidget(m_viewModeCombo);
    transportRow->addWidget(speedLabel);
    transportRow->addWidget(m_speedCombo);
    transportRow->addStretch(1);

    auto* jumpRow = new QHBoxLayout;
    jumpRow->setSpacing(8);
    jumpRow->addWidget(frameLabel);
    jumpRow->addWidget(m_frameSpin);
    jumpRow->addWidget(jumpFrameButton);
    jumpRow->addWidget(timeLabel);
    jumpRow->addWidget(m_timeSpin);
    jumpRow->addWidget(jumpTimeButton);
    jumpRow->addStretch(1);

    playbackRow->addLayout(transportRow, 1);
    playbackRow->addLayout(jumpRow, 1);
    auto* autoPauseRow = new QHBoxLayout;
    autoPauseRow->setSpacing(8);
    autoPauseRow->addWidget(m_autoPauseEnableCheck);
    autoPauseRow->addWidget(m_autoPauseJumpCheck);
    autoPauseRow->addWidget(m_autoPauseJumpThresholdSpin);
    autoPauseRow->addWidget(m_autoPauseCountCheck);
    autoPauseRow->addStretch(1);
    controls->addWidget(controlsTitle);
    controls->addWidget(m_slider);
    controls->addLayout(playbackRow);
    controls->addLayout(autoPauseRow);
    workspaceLayout->addWidget(controlsFrame, 0);

    setCentralWidget(central);

    connect(openRailButton, &QToolButton::clicked, this, &MainWindow::openVideo);
    connect(tcpRailButton, &QToolButton::clicked, this, &MainWindow::configureTcpReceiver);
    connect(logReplayRailButton, &QToolButton::clicked, this, &MainWindow::openJustFloatLogWindow);
    connect(horizonRailButton, &QToolButton::clicked, this, &MainWindow::openHorizonCalibrationWindow);
    connect(loadInstanceRailButton, &QToolButton::clicked, this, &MainWindow::loadInstance);
    connect(addInstanceRailButton, &QToolButton::clicked, this, &MainWindow::addInstance);
    connect(loadInstanceButton, &QPushButton::clicked, this, &MainWindow::loadInstance);
    connect(addInstanceButton, &QPushButton::clicked, this, &MainWindow::addInstance);
    connect(deleteInstanceButton, &QPushButton::clicked, this, &MainWindow::deleteCurrentInstance);
    connect(saveRailButton, &QToolButton::clicked, this, &MainWindow::saveAnnotation);
    connect(loadRailButton, &QToolButton::clicked, this, &MainWindow::loadAnnotation);
    connect(exportAviRailButton, &QToolButton::clicked, this, &MainWindow::exportMarkedAvi);
    connect(exportCsvRailButton, &QToolButton::clicked, this, &MainWindow::exportCsv);
    connect(exitRailButton, &QToolButton::clicked, this, &QWidget::close);
    connect(m_playPauseButton, &QPushButton::clicked, this, &MainWindow::togglePlayPause);
    connect(previousButton, &QPushButton::clicked, this, &MainWindow::previousFrame);
    connect(nextButton, &QPushButton::clicked, this, &MainWindow::nextFrame);
    connect(jumpFrameButton, &QPushButton::clicked, this, &MainWindow::jumpToFrame);
    connect(jumpTimeButton, &QPushButton::clicked, this, &MainWindow::jumpToTime);
    connect(m_slider, &QSlider::sliderPressed, this, &MainWindow::pause);
    connect(m_slider, &QSlider::valueChanged, this, &MainWindow::showFrameFromSlider);
    connect(m_viewModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        renderAllDisplayedInstances();
        refreshCurrentInstanceUi();
    });
    connect(m_speedCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        setPlaybackSpeed(m_speedCombo->currentData().toDouble());
    });
    connect(m_autoPauseEnableCheck, &QCheckBox::toggled, this, [this]() {
        for (AnalyzerInstance* instance : m_instances)
        {
            if (instance != nullptr)
            {
                instance->hasPreviousAutoPauseResult = false;
            }
        }
    });
    connect(m_annotationPanel, &AnnotationPanel::saveCurrentFrameCorrectionsRequested,
            this, &MainWindow::saveCurrentFrameCorrections);
    connect(m_annotationPanel, &AnnotationPanel::deleteAnnotationsRequested,
            this, &MainWindow::deleteAnnotations);
    connect(m_annotationPanel, &AnnotationPanel::deleteCorrectionsRequested,
            this, &MainWindow::deleteCorrections);
    connect(m_annotationPanel, &AnnotationPanel::batchAddCorrectionsRequested,
            this, &MainWindow::batchAddCorrections);
    connect(m_annotationPanel, &AnnotationPanel::autoMatchCorrectionFramesRequested,
            this, &MainWindow::autoMatchCorrectionFrames);
    connect(m_annotationPanel, &AnnotationPanel::batchAddCorrectionsToFramesRequested,
            this, &MainWindow::batchAddCorrectionsToFrames);
    connect(m_instanceList, &QListWidget::itemClicked, this, [this](QListWidgetItem* item) {
        if (item != nullptr)
        {
            selectInstance(item->data(Qt::UserRole).toInt());
        }
    });
    connect(m_instanceList, &QListWidget::itemChanged, this, [this](QListWidgetItem* item) {
        if (item == nullptr)
        {
            return;
        }
        setInstanceVisible(item->data(Qt::UserRole).toInt(), item->checkState() == Qt::Checked);
        updateSplitLayout();
        refreshInstanceList();
    });
    connect(m_annotationPanel, &AnnotationPanel::correctionToolChanged, this, [this](const QString& tool) {
        for (VideoWidget* widget : m_videoWidgets)
        {
            widget->setCorrectionTool(tool);
        }
    });
    connect(m_annotationPanel, &AnnotationPanel::correctionStyleChanged, this, [this](const QColor& color, int lineWidth) {
        for (VideoWidget* widget : m_videoWidgets)
        {
            widget->setCorrectionStyle(color, lineWidth);
        }
    });
    connect(m_annotationPanel, &AnnotationPanel::autoIdentifyRequested,
            this, &MainWindow::autoIdentifyCorrectionTargets);
    connect(m_annotationPanel, &AnnotationPanel::recordActivated,
            this, &MainWindow::jumpToRecordFrame);
}

void MainWindow::buildMenus()
{
    auto* fileMenu = menuBar()->addMenu(QStringLiteral("文件"));
    fileMenu->addAction(QStringLiteral("加载实例"), this, &MainWindow::loadInstance);
    fileMenu->addAction(QStringLiteral("添加实例"), this, &MainWindow::addInstance);
    fileMenu->addSeparator();
    fileMenu->addAction(QStringLiteral("打开视频"), this, &MainWindow::openVideo);
    fileMenu->addAction(QStringLiteral("新增 TCP 监视窗口"), this, &MainWindow::configureTcpReceiver);
    fileMenu->addAction(QStringLiteral("打开 JustFloat 日志"), this, &MainWindow::openJustFloatLogWindow);
    fileMenu->addAction(QStringLiteral("地平线标定"), this, &MainWindow::openHorizonCalibrationWindow);
    fileMenu->addAction(QStringLiteral("信标样本标注"), this, &MainWindow::openBeaconLabelWindow);
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

void MainWindow::loadInstance()
{
    const QString rootDir = QFileDialog::getExistingDirectory(this,
                                                              QStringLiteral("选择新实例文件夹"),
                                                              defaultInstancesRoot());
    if (rootDir.isEmpty())
    {
        return;
    }

    QString algorithmPath;
    QString error;
    algorithmPath = twoBl3ImageSourceFromInstance(rootDir, &error);
    if (!error.isEmpty())
    {
        QMessageBox::critical(this, QStringLiteral("加载实例失败"), error);
        return;
    }
    if (algorithmPath.isEmpty()
        && !seedAlgorithmFolder(rootDir, defaultAlgorithmPath(), &algorithmPath, &error))
    {
        QMessageBox::critical(this, QStringLiteral("新建实例失败"), error);
        return;
    }

    const QString defaultName = QFileInfo(rootDir).fileName().trimmed().isEmpty()
        ? QStringLiteral("实例 %1").arg(m_nextInstanceId)
        : QFileInfo(rootDir).fileName();
    const QString name = QInputDialog::getText(this,
                                               QStringLiteral("实例名称"),
                                               QStringLiteral("名称"),
                                               QLineEdit::Normal,
                                               defaultName);
    AnalyzerInstance* instance = createInstance(rootDir,
                                                algorithmPath,
                                                name.isEmpty() ? defaultName : name,
                                                true,
                                                &error);
    if (instance == nullptr)
    {
        QMessageBox::critical(this, QStringLiteral("新建实例失败"), error);
        return;
    }
    selectInstance(instance->id);
}

void MainWindow::addInstance()
{
    AiInstanceDialog dialog(defaultInstancesRoot(), this);
    if (dialog.exec() != QDialog::Accepted)
    {
        return;
    }

    const QString rootDir = dialog.rootDir();
    const QString algorithmDirPath = QDir(rootDir).absoluteFilePath(QStringLiteral("algorithm"));
    const QString algorithmPath = QDir(algorithmDirPath).absoluteFilePath(QStringLiteral("beacon_image.c"));
    const QString headerPath = QDir(algorithmDirPath).absoluteFilePath(QStringLiteral("beacon_image.h"));

    QString error;
    if (!QDir().mkpath(algorithmDirPath))
    {
        QMessageBox::critical(this,
                              QStringLiteral("添加实例失败"),
                              QStringLiteral("无法创建实例算法目录：%1").arg(algorithmDirPath));
        return;
    }

    if (QFileInfo::exists(algorithmPath))
    {
        const int reply = QMessageBox::question(this,
                                                QStringLiteral("覆盖实例代码"),
                                                QStringLiteral("目录中已经存在 beacon_image.c，是否覆盖？"),
                                                QMessageBox::Yes | QMessageBox::No,
                                                QMessageBox::No);
        if (reply != QMessageBox::Yes)
        {
            return;
        }
    }

    const QString source = dialog.generatedSource();
    const bool hasProcess = source.contains(QStringLiteral("beacon_image_process"));
    const bool hasHeader = source.contains(QStringLiteral("#include \"beacon_image.h\""));
    const QString generatedHeader = dialog.generatedHeader();
    const bool hasTypedResultChannels =
        generatedHeader.contains(QStringLiteral("BEACON_MAX_BEACON_COUNT")) &&
        generatedHeader.contains(QStringLiteral("BEACON_MAX_CAR_LAMP_COUNT")) &&
        generatedHeader.contains(QStringLiteral("beacons")) &&
        generatedHeader.contains(QStringLiteral("beacon_count")) &&
        generatedHeader.contains(QStringLiteral("car_lamps")) &&
        generatedHeader.contains(QStringLiteral("car_lamp_count")) &&
        source.contains(QStringLiteral("beacon_count")) &&
        source.contains(QStringLiteral("beacons"));
    const QString normalizedSource = normalizedSourceForStaticCheck(source);
    const bool hasCenterX = normalizedSource.contains(QStringLiteral("beacon_image_w*0.5f-")) ||
                            normalizedSource.contains(QStringLiteral("94.0f-"));
    const bool hasCenterY = normalizedSource.contains(QStringLiteral("-beacon_image_h*0.5f")) ||
                            normalizedSource.contains(QStringLiteral("-60.0f"));
    if (!hasProcess || !hasHeader || !hasTypedResultChannels || !hasCenterX || !hasCenterY)
    {
        QMessageBox::critical(this,
                              QStringLiteral("AI 自检未通过"),
                              QStringLiteral("生成代码必须包含 beacon_image.h、beacon_image_process，并明确执行图像像素坐标到图像中心坐标的转换。"));
        return;
    }

    if (!writeTextFile(headerPath, generatedHeader, &error) ||
        !writeTextFile(algorithmPath, source, &error) ||
        !copyFileIfNeeded(defaultAlgorithmHeaderPath(QStringLiteral("beacon_image.h")),
                          QDir(algorithmDirPath).absoluteFilePath(QStringLiteral("beacon_image.h")),
                          &error) ||
        !copyFileIfNeeded(defaultAlgorithmHeaderPath(QStringLiteral("beacon_image_config.h")),
                          QDir(algorithmDirPath).absoluteFilePath(QStringLiteral("beacon_image_config.h")),
                          &error))
    {
        QMessageBox::critical(this, QStringLiteral("添加实例失败"), error);
        return;
    }

    AnalyzerInstance* instance = createInstance(rootDir,
                                                algorithmPath,
                                                dialog.instanceName(),
                                                true,
                                                &error);
    if (instance == nullptr)
    {
        QMessageBox::critical(this, QStringLiteral("添加实例失败"), error);
        return;
    }
    selectInstance(instance->id);
    statusBar()->showMessage(QStringLiteral("AI 实例已生成并加载"), 3000);
}

void MainWindow::deleteCurrentInstance()
{
    AnalyzerInstance* instance = currentInstance();
    if (instance == nullptr)
    {
        QMessageBox::information(this,
                                 QStringLiteral("删除实例"),
                                 QStringLiteral("当前没有可删除的实例。"));
        return;
    }

    const int reply = QMessageBox::question(this,
                                            QStringLiteral("删除实例"),
                                            QStringLiteral("确定从当前会话移除实例“%1”吗？\n磁盘上的实例文件夹不会被删除。")
                                                .arg(instance->name),
                                            QMessageBox::Yes | QMessageBox::No,
                                            QMessageBox::No);
    if (reply != QMessageBox::Yes)
    {
        return;
    }

    const int removedId = instance->id;
    saveProject();
    m_instances.removeAll(instance);
    delete instance;

    for (int& instanceId : m_splitSlotInstanceIds)
    {
        if (instanceId == removedId)
        {
            instanceId = -1;
        }
    }

    m_currentInstanceId = m_instances.isEmpty() ? -1 : m_instances.first()->id;
    if (m_currentInstanceId >= 0)
    {
        setInstanceVisible(m_currentInstanceId, true);
    }

    refreshInstanceList();
    updateAnnotationList();
    updateSplitLayout();
    refreshCurrentInstanceUi();
    statusBar()->showMessage(QStringLiteral("实例已移除"), 3000);
}

void MainWindow::importAlgorithmFile()
{
    const QString sourcePath = QFileDialog::getOpenFileName(this,
                                                            QStringLiteral("导入 C 文件"),
                                                            QString(),
                                                            QStringLiteral("C Source (*.c)"));
    if (sourcePath.isEmpty())
    {
        return;
    }

    const QString rootDir = QFileDialog::getExistingDirectory(this,
                                                              QStringLiteral("选择实例文件夹"),
                                                              defaultInstancesRoot());
    if (rootDir.isEmpty())
    {
        return;
    }

    QString algorithmPath;
    QString error;
    if (!seedAlgorithmFolder(rootDir, sourcePath, &algorithmPath, &error))
    {
        QMessageBox::critical(this, QStringLiteral("导入 C 文件失败"), error);
        return;
    }

    const QString defaultName = QFileInfo(rootDir).fileName().trimmed().isEmpty()
        ? QFileInfo(sourcePath).completeBaseName()
        : QFileInfo(rootDir).fileName();
    const QString name = QInputDialog::getText(this,
                                               QStringLiteral("实例名称"),
                                               QStringLiteral("名称"),
                                               QLineEdit::Normal,
                                               defaultName);
    AnalyzerInstance* instance = createInstance(rootDir,
                                                algorithmPath,
                                                name.isEmpty() ? defaultName : name,
                                                true,
                                                &error);
    if (instance == nullptr)
    {
        QMessageBox::critical(this, QStringLiteral("导入 C 文件失败"), error);
        return;
    }
    selectInstance(instance->id);
}

void MainWindow::openVideo()
{
    if (requireCurrentInstance(QStringLiteral("打开视频")) == nullptr)
    {
        return;
    }

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

void MainWindow::configureTcpReceiver()
{
    QVector<TcpInstanceOption> options;
    for (AnalyzerInstance* instance : m_instances)
    {
        if (instance == nullptr)
        {
            continue;
        }
        TcpInstanceOption option;
        option.id = instance->id;
        option.name = instance->name;
        option.runner = &instance->runner;
        option.annotations = &instance->annotations;
        options.push_back(option);
    }

    auto* window = new TcpImageWindow;
    window->setAvailableAddresses(localTcpListenAddresses());
    window->setInstanceOptions(options);
    window->setDefaultSaveDirectory(m_liveSaveDir.isEmpty() ? defaultTcpSaveDir() : m_liveSaveDir);
    window->setSuggestedPort(m_nextTcpPort);
    window->show();
    window->raise();

    if (m_nextTcpPort < 65535)
    {
        ++m_nextTcpPort;
    }
    updateTcpStatusLabel(QStringLiteral("已新增 TCP 监视窗口，可在窗口内选择本机 IP 和端口。"));
}

void MainWindow::openJustFloatLogWindow()
{
    auto* window = new LogReplayWindow;
    window->show();
    window->raise();
    statusBar()->showMessage(QStringLiteral("已打开 JustFloat 日志回放窗口。"), 3000);
}

void MainWindow::openHorizonCalibrationWindow()
{
    auto* window = new HorizonCalibrationWindow;
    window->show();
    window->raise();
    statusBar()->showMessage(QStringLiteral("已打开地平线标定窗口。"), 3000);
}

void MainWindow::openBeaconLabelWindow()
{
    auto* window = new BeaconLabelWindow;
    window->show();
    window->raise();
    statusBar()->showMessage(QStringLiteral("已打开信标样本标注窗口。"), 3000);
}

void MainWindow::loadCompanionFrameTelemetry(const QString& videoPath)
{
    m_frameTelemetry.clear();
    const QFileInfo videoInfo(videoPath);
    const QString sessionPath = videoInfo.dir().absoluteFilePath(
        videoInfo.completeBaseName() + QStringLiteral(".hcal.json"));
    if (!QFileInfo::exists(sessionPath))
    {
        return;
    }

    HorizonCalibrationSession session;
    QString error;
    if (!HorizonCalibration::loadSession(sessionPath, &session, &error))
    {
        statusBar()->showMessage(QStringLiteral("HCAL遥测读取失败：%1").arg(error), 5000);
        return;
    }

    for (const HorizonFrameMetadata& frame : session.frames)
    {
        AlgorithmFrameTelemetry telemetry;
        telemetry.cameraId = frame.cameraId;
        telemetry.rollDeg = static_cast<float>(frame.rollDeg);
        telemetry.pitchDeg = static_cast<float>(frame.pitchDeg);
        telemetry.heightMm = static_cast<float>(frame.heightMm);
        telemetry.attitudeValid = frame.attitudeValid;
        telemetry.heightValid = frame.heightValid;
        m_frameTelemetry.insert(frame.frameIndex, telemetry);
    }
}

AlgorithmFrameTelemetry MainWindow::frameTelemetryForFrame(int frameIndex) const
{
    return m_liveMode ? AlgorithmFrameTelemetry{} : m_frameTelemetry.value(frameIndex);
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
    loadCompanionFrameTelemetry(path);
    m_currentFrame = fallbackFrame;
    m_liveMode = false;
    m_decodedFrameCache.clear();
    for (AnalyzerInstance* instance : m_instances)
    {
        resetInstanceTemporal(instance);
    }

    m_slider->setRange(0, qMax(0, m_reader.frameCount() - 1));
    m_frameSpin->setRange(0, qMax(0, m_reader.frameCount() - 1));
    m_timeSpin->setRange(0.0, frameTime(qMax(0, m_reader.frameCount() - 1)));
    m_annotationPanel->setVideoFrameRange(0, qMax(0, m_reader.frameCount() - 1));

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
    renderAllDisplayedInstances();
    showFrame(qBound(0, m_currentFrame, qMax(0, m_reader.frameCount() - 1)));
    refreshInstanceList();
    statusBar()->showMessage(QStringLiteral("视频已打开"), 3000);
    return true;
}

void MainWindow::saveAnnotation()
{
    if (requireCurrentInstance(QStringLiteral("保存标注")) == nullptr)
    {
        return;
    }
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
    if (requireCurrentInstance(QStringLiteral("读取标注")) == nullptr)
    {
        return;
    }

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
    if (requireCurrentInstance(QStringLiteral("导出标注 AVI")) == nullptr)
    {
        return;
    }
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
    const bool ok = exporter.exportMarkedAvi(m_currentVideoPath, path, m_usedFps, &m_runner, &m_annotations,
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
    if (requireCurrentInstance(QStringLiteral("导出 CSV")) == nullptr)
    {
        return;
    }
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
    const bool ok = exporter.exportResultCsv(m_currentVideoPath, path, m_usedFps, &m_runner,
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
    if (m_liveMode)
    {
        return;
    }
    if (m_reader.isOpen())
    {
        m_globalPlaying = true;
        m_playTimer.start(playbackIntervalMs());
        updatePlayPauseButton();
    }
}

void MainWindow::pause()
{
    m_globalPlaying = false;
    m_playTimer.stop();
    updatePlayPauseButton();
}

void MainWindow::togglePlayPause()
{
    if (m_globalPlaying)
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

    if (m_globalPlaying)
    {
        m_playPauseButton->setText(QStringLiteral("暂停"));
        m_playPauseButton->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    }
    else
    {
        m_playPauseButton->setText(QStringLiteral("播放"));
        m_playPauseButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    }
}

void MainWindow::updateTcpStatusLabel(const QString& eventMessage)
{
    if (m_tcpStatusLabel == nullptr)
    {
        return;
    }
    m_tcpStatusLabel->setText(eventMessage.isEmpty()
                                  ? QStringLiteral("TCP：点击左侧 TCP 按钮新增监视窗口")
                                  : eventMessage);
}

void MainWindow::setPlaybackSpeed(double speed)
{
    if (speed <= 0.0)
    {
        return;
    }

    const bool wasPlaying = m_globalPlaying;
    m_playbackSpeed = speed;
    if (wasPlaying)
    {
        m_playTimer.start(playbackIntervalMs());
    }
}

int MainWindow::playbackIntervalMs() const
{
    const double fps = m_usedFps > 0.0 ? m_usedFps : 50.0;
    return qMax(1, (int)std::round(1000.0 / (fps * m_playbackSpeed)));
}

bool MainWindow::validateAnnotationInput(const QStringList& types,
                                         const QVector<ErrorCircle>& errorCircles,
                                         const QString& actionName) const
{
    if (types.isEmpty())
    {
        QMessageBox::warning(const_cast<MainWindow*>(this),
                             actionName,
                             QStringLiteral("请至少选择一个错误类型。"));
        return false;
    }

    if (typesRequireErrorSource(types) && errorCircles.isEmpty())
    {
        QMessageBox::warning(const_cast<MainWindow*>(this),
                             actionName,
                             QStringLiteral("选中误检、排序错误、目标跳变或其他类型时，必须选择至少一个错误圆并填写对应期望编号。"));
        return false;
    }

    return true;
}

void MainWindow::nextFrame()
{
    if (m_liveMode)
    {
        return;
    }
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
    if (m_liveMode)
    {
        return;
    }
    if (m_reader.isOpen())
    {
        showFrame(qMax(0, m_currentFrame - 1));
    }
}

void MainWindow::jumpToFrame()
{
    if (m_liveMode)
    {
        return;
    }
    showFrame(m_frameSpin->value());
}

void MainWindow::jumpToTime()
{
    if (m_liveMode)
    {
        return;
    }
    showFrame(qBound(0, (int)(m_timeSpin->value() * m_usedFps + 0.5), qMax(0, m_reader.frameCount() - 1)));
}

void MainWindow::showFrameFromSlider(int value)
{
    if (!m_updatingControls)
    {
        if (m_liveMode)
        {
            QSignalBlocker blocker(m_slider);
            m_slider->setValue(m_currentFrame);
            return;
        }
        pause();
        showFrame(value);
    }
}

bool MainWindow::readFrameCached(int frameIndex, QImage* grayImage, QString* errorMessage)
{
    if (grayImage == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("grayImage is null");
        }
        return false;
    }

    if (const QImage* cached = m_decodedFrameCache.object(frameIndex))
    {
        *grayImage = *cached;
        if (errorMessage != nullptr)
        {
            errorMessage->clear();
        }
        return true;
    }

    QImage decoded;
    if (!m_reader.readFrame(frameIndex, &decoded, errorMessage))
    {
        return false;
    }

    const qsizetype sizeKiB = qMax<qsizetype>(1, (decoded.sizeInBytes() + 1023) / 1024);
    if (sizeKiB <= m_decodedFrameCache.maxCost())
    {
        m_decodedFrameCache.insert(frameIndex, new QImage(decoded), static_cast<int>(sizeKiB));
    }
    *grayImage = decoded;
    return true;
}

void MainWindow::showFrame(int frameIndex)
{
    m_liveMode = false;
    if (!m_reader.isOpen())
    {
        return;
    }

    frameIndex = qBound(0, frameIndex, m_reader.frameCount() - 1);
    QImage gray;
    QString error;
    if (!readFrameCached(frameIndex, &gray, &error))
    {
        QMessageBox::critical(this, QStringLiteral("读取帧失败"), error);
        pause();
        return;
    }

    const int previousFrame = m_currentFrame;
    QVector<QPair<AnalyzerInstance*, beacon_result_t>> results;
    for (AnalyzerInstance* instance : m_instances)
    {
        if (instance == nullptr)
        {
            continue;
        }
        const beacon_result_t result = processCausalFrame(instance, frameIndex, gray);
        instance->currentResult = result;
        results.push_back(qMakePair(instance, result));
    }

    m_currentFrame = frameIndex;
    const beacon_result_t result = m_currentResult;
    const QVector<CorrectionShape> savedCorrections = m_annotations.correctionsForFrame(m_currentFrame);
    m_annotationPanel->setCurrentContext(m_currentFrame, frameTime(m_currentFrame), result.count);
    m_annotationPanel->setCurrentFrameCorrections(savedCorrections);
    renderAllDisplayedInstances(gray, results);

    m_updatingControls = true;
    m_slider->setValue(frameIndex);
    m_frameSpin->setValue(frameIndex);
    m_timeSpin->setValue(frameTime(frameIndex));
    m_updatingControls = false;

    updateFrameInfo(result);
    QString autoPauseReason;
    if (autoPauseTriggered(previousFrame, frameIndex, results, &autoPauseReason))
    {
        pause();
        statusBar()->showMessage(autoPauseReason, 5000);
    }
}

bool MainWindow::autoPauseTriggered(int previousFrame,
                                    int currentFrame,
                                    const QVector<QPair<AnalyzerInstance*, beacon_result_t>>& results,
                                    QString* reason)
{
    const bool enabled = m_autoPauseEnableCheck != nullptr && m_autoPauseEnableCheck->isChecked();
    const bool jumpEnabled = m_autoPauseJumpCheck != nullptr && m_autoPauseJumpCheck->isChecked();
    const bool countEnabled = m_autoPauseCountCheck != nullptr && m_autoPauseCountCheck->isChecked();
    const double jumpThreshold = m_autoPauseJumpThresholdSpin != nullptr
        ? m_autoPauseJumpThresholdSpin->value()
        : 0.0;

    bool triggered = false;
    QString triggerReason;
    const bool adjacentForwardFrame = currentFrame == previousFrame + 1;

    if (enabled && m_globalPlaying && adjacentForwardFrame && (jumpEnabled || countEnabled))
    {
        for (const auto& item : results)
        {
            AnalyzerInstance* instance = item.first;
            const beacon_result_t& current = item.second;
            if (instance == nullptr ||
                !instance->hasPreviousAutoPauseResult ||
                instance->previousAutoPauseFrame != previousFrame)
            {
                continue;
            }

            const beacon_result_t& previous = instance->previousAutoPauseResult;
            if (countEnabled && validCircleCount(previous) != validCircleCount(current))
            {
                triggered = true;
                triggerReason = QStringLiteral("自动暂停：%1 相邻帧目标数量变化").arg(instance->name);
                break;
            }

            if (jumpEnabled)
            {
                const int count = qMin<int>(qMin(previous.count, current.count), BEACON_MAX_CIRCLE_COUNT);
                for (int i = 0; i < count; ++i)
                {
                    if (previous.circles[i].valid == 0 || current.circles[i].valid == 0)
                    {
                        continue;
                    }

                    const double distance = circleImageDistance(previous.circles[i], current.circles[i]);
                    if (distance > jumpThreshold)
                    {
                        triggered = true;
                        triggerReason = QStringLiteral("自动暂停：%1 目标 #%2 跳变 %3 px")
                                            .arg(instance->name)
                                            .arg(i)
                                            .arg(distance, 0, 'f', 1);
                        break;
                    }
                }
                if (triggered)
                {
                    break;
                }
            }
        }
    }

    for (const auto& item : results)
    {
        AnalyzerInstance* instance = item.first;
        if (instance == nullptr)
        {
            continue;
        }
        instance->previousAutoPauseResult = item.second;
        instance->previousAutoPauseFrame = currentFrame;
        instance->hasPreviousAutoPauseResult = true;
    }

    if (triggered && reason != nullptr)
    {
        *reason = triggerReason;
    }
    return triggered;
}

void MainWindow::updateFrameInfo(const beacon_result_t& result)
{
    QString text;
    const AlgorithmProcessProfile profile = currentInstance() != nullptr
        ? currentInstance()->currentProfile
        : AlgorithmProcessProfile{};
    const AlgorithmDetectionMetrics metrics = currentInstance() != nullptr
        ? currentInstance()->currentDetectionMetrics
        : AlgorithmDetectionMetrics{};
    const QString profileText = AlgorithmProcessProfiler::format(profile, m_usedFps);
    const bool useLegacyBeacons = BeaconResultUtils::usesLegacyBeacons(result);
    const beacon_circle_t* beacons = useLegacyBeacons ? result.circles : result.beacons;
    const int beaconLimit = useLegacyBeacons
        ? BeaconResultUtils::boundedCount(result.count, BEACON_MAX_CIRCLE_COUNT)
        : BeaconResultUtils::boundedCount(result.beacon_count, BEACON_MAX_BEACON_COUNT);
    for (int i = 0; i < beaconLimit; ++i)
    {
        const beacon_circle_t& circle = beacons[i];
        if (circle.valid == 0)
        {
            continue;
        }
        const int pixelArea = qRound(3.14159265358979323846 * circle.radius * circle.radius);
        text += QStringLiteral("信标 #%1  X=%2  Y=%3  R=%4  PixelArea=%5 px\n")
                    .arg(i)
                    .arg(circle.x, 0, 'f', 2)
                    .arg(circle.y, 0, 'f', 2)
                    .arg(circle.radius, 0, 'f', 2)
                    .arg(pixelArea);
    }

    const int carLampLimit = BeaconResultUtils::boundedCount(result.car_lamp_count, BEACON_MAX_CAR_LAMP_COUNT);
    for (int i = 0; i < carLampLimit; ++i)
    {
        const beacon_rect_t& rect = result.car_lamps[i];
        if (rect.valid == 0)
        {
            continue;
        }
        const double fitArea = rect.length * rect.width;
        QString pixelAreaText = QStringLiteral("--");
        if (metrics.carLampPixelAreasAvailable && i < metrics.carLampPixelAreaCount)
        {
            pixelAreaText = QStringLiteral("%1 px").arg(
                metrics.carLampPixelAreas[static_cast<std::size_t>(i)]);
        }
        text += QStringLiteral("车灯 #%1  CX=%2  CY=%3  L=%4  W=%5  Angle=%6  PixelArea=%7  FitArea=%8\n")
                    .arg(i)
                    .arg(rect.cx, 0, 'f', 2)
                    .arg(rect.cy, 0, 'f', 2)
                    .arg(rect.length, 0, 'f', 2)
                    .arg(rect.width, 0, 'f', 2)
                    .arg(rect.angle, 0, 'f', 2)
                    .arg(pixelAreaText)
                    .arg(fitArea, 0, 'f', 2);
    }

    m_frameInfoLabel->setText(QStringLiteral("当前帧: %1 / %2\n播放时间: %3 s / %4 s\n信标: %5  车灯: %6  总目标: %7")
                                  .arg(m_currentFrame)
                                  .arg(qMax(0, m_reader.frameCount() - 1))
                                  .arg(frameTime(m_currentFrame), 0, 'f', 3)
                                  .arg(frameTime(qMax(0, m_reader.frameCount() - 1)), 0, 'f', 3)
                                  .arg(BeaconResultUtils::beaconCount(result))
                                  .arg(BeaconResultUtils::carLampCount(result))
                                  .arg(BeaconResultUtils::totalTargetCount(result)));
    m_frameInfoLabel->setText(m_frameInfoLabel->text() + QStringLiteral("\n") + profileText);

    if (text.isEmpty())
    {
        text = QStringLiteral("总目标: 0\n无有效目标");
    }
    else
    {
        text.prepend(QStringLiteral("信标: %1  车灯: %2  总目标: %3\n")
                         .arg(BeaconResultUtils::beaconCount(result))
                         .arg(BeaconResultUtils::carLampCount(result))
                         .arg(BeaconResultUtils::totalTargetCount(result)));
    }
    text.prepend(profileText + QStringLiteral("\n"));
    m_resultText->setPlainText(text);
    updateCurrentAnnotationInfo();
}

void MainWindow::updateHoverPixelInfo(int x, int y, int gray, bool valid)
{
    if (m_pixelInfoLabel == nullptr)
    {
        return;
    }

    if (!valid)
    {
        m_pixelInfoLabel->setText(QStringLiteral("鼠标: -"));
        statusBar()->clearMessage();
        return;
    }

    const QString text = QStringLiteral("鼠标 X=%1 Y=%2 灰度=%3").arg(x).arg(y).arg(gray);
    m_pixelInfoLabel->setText(text);
    statusBar()->showMessage(text);
}

void MainWindow::updateCurrentAnnotationInfo()
{
    if (m_currentAnnotationsLabel == nullptr)
    {
        return;
    }
    const AnalyzerInstance* instance = currentInstance();
    if (instance == nullptr)
    {
        m_currentAnnotationsLabel->setText(QStringLiteral("当前帧标注：无实例"));
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
        const QStringList types = record.types.isEmpty() ? QStringList{ record.type } : record.types;
        const QVector<ErrorCircle> circles = record.errorCircles.isEmpty() && record.circleIndex >= 0
            ? QVector<ErrorCircle>{ ErrorCircle{ record.circleIndex, -1 } }
            : record.errorCircles;
        text += QStringLiteral("- %1 %2 %3\n")
                    .arg(annotationTypesDisplayName(types))
                    .arg(errorCirclesDisplayName(circles))
                    .arg(record.description);
    }
    for (const CorrectionShape& shape : corrections)
    {
        text += QStringLiteral("- 图形 %1 %2 %3 %4\n")
                    .arg(annotationTypesDisplayName(shape.errorTypes.isEmpty() ? QStringList{ shape.errorType } : shape.errorTypes))
                    .arg(shape.shapeType)
                    .arg(errorCirclesDisplayName(shape.errorCircles))
                    .arg(shape.description);
    }
    m_currentAnnotationsLabel->setText(text.trimmed());
}

void MainWindow::markCurrentFrameAnnotation(const QStringList& types,
                                            const QVector<ErrorCircle>& errorCircles,
                                            const QString& description)
{
    if (!m_reader.isOpen())
    {
        return;
    }
    if (!validateAnnotationInput(types, errorCircles, QStringLiteral("标记当前帧")))
    {
        return;
    }

    AnnotationRecord record;
    record.type = types.isEmpty() ? QStringLiteral("other") : types.first();
    record.types = types;
    record.startFrame = m_currentFrame;
    record.endFrame = m_currentFrame;
    record.startTimeSec = frameTime(m_currentFrame);
    record.endTimeSec = record.startTimeSec;
    record.circleIndex = errorCircles.isEmpty() ? -1 : errorCircles.first().circleIndex;
    record.errorCircles = errorCircles;
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

void MainWindow::saveSegmentAnnotation(const QStringList& types,
                                       const QVector<ErrorCircle>& errorCircles,
                                       const QString& description)
{
    if (!m_reader.isOpen() || m_segmentStartFrame < 0 || m_segmentEndFrame < 0)
    {
        QMessageBox::warning(this, QStringLiteral("片段不完整"), QStringLiteral("请先设置片段开始和结束"));
        return;
    }
    if (!validateAnnotationInput(types, errorCircles, QStringLiteral("保存片段标注")))
    {
        return;
    }

    const int startFrame = qMin(m_segmentStartFrame, m_segmentEndFrame);
    const int endFrame = qMax(m_segmentStartFrame, m_segmentEndFrame);
    AnnotationRecord record;
    record.type = types.isEmpty() ? QStringLiteral("other") : types.first();
    record.types = types;
    record.startFrame = startFrame;
    record.endFrame = endFrame;
    record.startTimeSec = frameTime(startFrame);
    record.endTimeSec = frameTime(endFrame);
    record.circleIndex = errorCircles.isEmpty() ? -1 : errorCircles.first().circleIndex;
    record.errorCircles = errorCircles;
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

void MainWindow::deleteAnnotations(const QVector<int>& rows)
{
    bool changed = false;
    QVector<int> sortedRows = rows;
    std::sort(sortedRows.begin(), sortedRows.end(), std::greater<int>());
    for (int row : sortedRows)
    {
        changed = m_annotations.removeAt(row) || changed;
    }
    if (changed)
    {
        updateAnnotationList();
        showFrame(m_currentFrame);
    }
}

void MainWindow::deleteCorrections(const QVector<int>& rows)
{
    bool changed = false;
    QVector<int> sortedRows = rows;
    std::sort(sortedRows.begin(), sortedRows.end(), std::greater<int>());
    for (int row : sortedRows)
    {
        changed = m_annotations.removeCorrectionAt(row) || changed;
    }
    if (changed)
    {
        updateAnnotationList();
        showFrame(m_currentFrame);
    }
}

void MainWindow::saveCurrentFrameCorrections(const QVector<CorrectionShape>& corrections)
{
    if (!m_reader.isOpen())
    {
        return;
    }

    m_annotations.removeCorrectionsForFrame(m_currentFrame);
    for (CorrectionShape correction : corrections)
    {
        correction.frame = m_currentFrame;
        if (correction.name.trimmed().isEmpty())
        {
            correction.name = annotationTypeDisplayName(correction.errorType);
        }
        m_annotations.addCorrection(correction);
    }

    m_annotationPanel->setCurrentFrameCorrections(m_annotations.correctionsForFrame(m_currentFrame), true);
    updateAnnotationList();
    showFrame(m_currentFrame);
    statusBar()->showMessage(QStringLiteral("当前帧纠错已保存"), 3000);
}

bool MainWindow::collectBatchCorrections(const QVector<int>& correctionRows,
                                         QVector<CorrectionShape>* corrections,
                                         QString* errorMessage) const
{
    if (corrections == nullptr)
    {
        return false;
    }

    corrections->clear();
    QVector<int> rows = correctionRows;
    std::sort(rows.begin(), rows.end());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    if (rows.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("请先选择当前帧已保存的纠错条目。");
        }
        return false;
    }

    const QVector<CorrectionShape>& allCorrections = m_annotations.corrections();
    for (int row : rows)
    {
        if (row < 0 || row >= allCorrections.size())
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("选中的纠错条目已失效，请刷新后重新选择。");
            }
            return false;
        }

        const CorrectionShape& correction = allCorrections[row];
        if (correction.frame != m_currentFrame)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("只能批量添加当前帧已保存的纠错条目。");
            }
            return false;
        }
        corrections->push_back(correction);
    }

    return true;
}

bool MainWindow::correctionsHaveMixedBatchTypes(const QVector<CorrectionShape>& corrections) const
{
    bool hasMissed = false;
    bool hasNonMissed = false;
    for (const CorrectionShape& correction : corrections)
    {
        if (isMissedCorrectionType(correction.errorType))
        {
            hasMissed = true;
        }
        else
        {
            hasNonMissed = true;
        }
    }
    return hasMissed && hasNonMissed;
}

bool MainWindow::correctionsAreAllMissed(const QVector<CorrectionShape>& corrections) const
{
    if (corrections.isEmpty())
    {
        return false;
    }
    for (const CorrectionShape& correction : corrections)
    {
        if (!isMissedCorrectionType(correction.errorType))
        {
            return false;
        }
    }
    return true;
}

bool MainWindow::processFrameForBatch(int frame, beacon_result_t* result, QString* errorMessage) const
{
    if (result == nullptr)
    {
        return false;
    }
    if (!m_reader.isOpen() || frame < 0 || frame >= m_reader.frameCount())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("帧号超出视频有效范围。");
        }
        return false;
    }

    QImage gray;
    QString readError;
    if (!m_reader.readFrame(frame, &gray, &readError))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = readError;
        }
        return false;
    }

    MainWindow* self = const_cast<MainWindow*>(this);
    *result = self->processCausalFrame(self->currentInstance(), frame, gray);
    return true;
}

bool MainWindow::batchCorrectionsMatchAdjacent(const QVector<CorrectionShape>& corrections,
                                               const beacon_result_t& previous,
                                               const beacon_result_t& next,
                                               double positionThreshold) const
{
    for (const CorrectionShape& correction : corrections)
    {
        QVector<int> sourceIndices;
        for (const ErrorCircle& circle : correction.errorCircles)
        {
            if (circle.circleIndex >= 0 && !sourceIndices.contains(circle.circleIndex))
            {
                sourceIndices.push_back(circle.circleIndex);
            }
        }
        if (sourceIndices.isEmpty())
        {
            return false;
        }

        for (int circleIndex : sourceIndices)
        {
            if (circleIndex >= BEACON_MAX_CIRCLE_COUNT ||
                circleIndex >= previous.count ||
                circleIndex >= next.count)
            {
                return false;
            }

            const beacon_circle_t& previousCircle = previous.circles[circleIndex];
            const beacon_circle_t& nextCircle = next.circles[circleIndex];
            if (previousCircle.valid == 0 || nextCircle.valid == 0)
            {
                return false;
            }

            const QPointF previousPoint = FrameRenderer::algorithmToImagePoint(previousCircle.x, previousCircle.y);
            const QPointF nextPoint = FrameRenderer::algorithmToImagePoint(nextCircle.x, nextCircle.y);
            if (QLineF(previousPoint, nextPoint).length() > positionThreshold)
            {
                return false;
            }
        }
    }
    return true;
}

bool MainWindow::buildMissedBatchCorrections(const QVector<CorrectionShape>& baseCorrections,
                                             int startFrame,
                                             int endFrame,
                                             double overlapPixelThreshold,
                                             QVector<CorrectionShape>* matchedCorrections,
                                             QVector<int>* matchedFrames,
                                             QVector<int>* failedFrames,
                                             QString* errorMessage) const
{
    if (matchedCorrections == nullptr || matchedFrames == nullptr || failedFrames == nullptr)
    {
        return false;
    }

    matchedCorrections->clear();
    matchedFrames->clear();
    failedFrames->clear();

    QVector<MissedBatchTarget> baseTargets;
    if (!prepareMissedBatchTargets(baseCorrections, &baseTargets, errorMessage))
    {
        return false;
    }

    auto scanDirection = [&](int direction) {
        QVector<MissedBatchTarget> previousTargets = baseTargets;
        int previousFrame = m_currentFrame;
        while (true)
        {
            const int nextFrame = previousFrame + direction;
            if (nextFrame < 0 || nextFrame >= m_reader.frameCount())
            {
                break;
            }
            if (direction < 0 && nextFrame < startFrame)
            {
                break;
            }
            if (direction > 0 && nextFrame > endFrame)
            {
                break;
            }

            QImage gray;
            QString readError;
            if (!m_reader.readFrame(nextFrame, &gray, &readError))
            {
                if (errorMessage != nullptr)
                {
                    *errorMessage = readError;
                }
                failedFrames->push_back(nextFrame);
                break;
            }

            QVector<MissedBatchTarget> nextTargets;
            QVector<CorrectionShape> frameCorrections;
            if (!matchMissedTargetsInImage(gray,
                                           previousTargets,
                                           qMax(0.0, overlapPixelThreshold),
                                           &nextTargets,
                                           &frameCorrections))
            {
                failedFrames->push_back(nextFrame);
                break;
            }

            if (nextFrame >= startFrame && nextFrame <= endFrame)
            {
                matchedFrames->push_back(nextFrame);
                for (CorrectionShape correction : frameCorrections)
                {
                    correction.frame = nextFrame;
                    matchedCorrections->push_back(correction);
                }
            }

            previousTargets = nextTargets;
            previousFrame = nextFrame;
        }
    };

    if (startFrame < m_currentFrame)
    {
        scanDirection(-1);
    }
    if (endFrame > m_currentFrame)
    {
        scanDirection(1);
    }

    std::sort(matchedFrames->begin(), matchedFrames->end());
    matchedFrames->erase(std::unique(matchedFrames->begin(), matchedFrames->end()), matchedFrames->end());
    return true;
}

void MainWindow::appendCorrectionsToFrames(const QVector<CorrectionShape>& corrections,
                                           const QVector<int>& frames,
                                           const QString& actionName)
{
    QVector<int> targetFrames = frames;
    std::sort(targetFrames.begin(), targetFrames.end());
    targetFrames.erase(std::unique(targetFrames.begin(), targetFrames.end()), targetFrames.end());
    if (corrections.isEmpty() || targetFrames.isEmpty())
    {
        QMessageBox::information(this, actionName, QStringLiteral("没有可添加的目标帧。"));
        return;
    }

    for (int frame : targetFrames)
    {
        if (frame < 0 || frame >= m_reader.frameCount())
        {
            QMessageBox::warning(this, actionName, QStringLiteral("目标帧号超出视频有效范围。"));
            return;
        }
    }

    int addedCount = 0;
    for (int frame : targetFrames)
    {
        for (CorrectionShape correction : corrections)
        {
            correction = batchCorrectionConfigOnly(correction);
            correction.frame = frame;
            if (correction.name.trimmed().isEmpty())
            {
                correction.name = annotationTypeDisplayName(correction.errorType);
            }
            m_annotations.addCorrection(correction);
            ++addedCount;
        }
    }

    m_annotationPanel->setCurrentFrameCorrections(m_annotations.correctionsForFrame(m_currentFrame), true);
    updateAnnotationList();
    showFrame(m_currentFrame);
    QMessageBox::information(this,
                             actionName,
                             QStringLiteral("批量添加完成：已向 %1 个目标帧追加 %2 条纠错。")
                                 .arg(targetFrames.size())
                                 .arg(addedCount));
}

void MainWindow::appendResolvedCorrections(const QVector<CorrectionShape>& corrections,
                                           const QString& actionName)
{
    if (corrections.isEmpty())
    {
        QMessageBox::information(this, actionName, QStringLiteral("没有可添加的目标帧。"));
        return;
    }

    QVector<int> targetFrames;
    int addedCount = 0;
    for (CorrectionShape correction : corrections)
    {
        correction = batchCorrectionConfigOnly(correction);
        if (correction.frame < 0 || correction.frame >= m_reader.frameCount())
        {
            QMessageBox::warning(this, actionName, QStringLiteral("目标帧号超出视频有效范围。"));
            return;
        }
        if (!targetFrames.contains(correction.frame))
        {
            targetFrames.push_back(correction.frame);
        }
        if (correction.name.trimmed().isEmpty())
        {
            correction.name = annotationTypeDisplayName(correction.errorType);
        }
        m_annotations.addCorrection(correction);
        ++addedCount;
    }

    std::sort(targetFrames.begin(), targetFrames.end());
    m_annotationPanel->setCurrentFrameCorrections(m_annotations.correctionsForFrame(m_currentFrame), true);
    updateAnnotationList();
    showFrame(m_currentFrame);
    QMessageBox::information(this,
                             actionName,
                             QStringLiteral("批量添加完成：已向 %1 个目标帧追加 %2 条纠错。")
                                 .arg(targetFrames.size())
                                 .arg(addedCount));
}

void MainWindow::batchAddCorrections(const QVector<int>& correctionRows,
                                     int startFrame,
                                     int endFrame,
                                     double overlapPixelThreshold)
{
    if (!m_reader.isOpen())
    {
        return;
    }
    if (startFrame > endFrame)
    {
        QMessageBox::warning(this,
                             QStringLiteral("手动批量添加"),
                             QStringLiteral("起始帧不能大于结束帧，请修正后再操作。"));
        return;
    }
    if (startFrame < 0 || endFrame >= m_reader.frameCount())
    {
        QMessageBox::warning(this,
                             QStringLiteral("手动批量添加"),
                             QStringLiteral("目标帧号超出视频有效范围。"));
        return;
    }

    QVector<CorrectionShape> corrections;
    QString error;
    if (!collectBatchCorrections(correctionRows, &corrections, &error))
    {
        QMessageBox::warning(this, QStringLiteral("手动批量添加"), error);
        return;
    }
    if (correctionsHaveMixedBatchTypes(corrections))
    {
        QMessageBox::warning(this,
                             QStringLiteral("批量操作"),
                             QStringLiteral("选中条目同时包含漏检和非漏检类型，不可混用批量操作。"));
        return;
    }
    if (correctionsAreAllMissed(corrections))
    {
        QVector<CorrectionShape> matchedCorrections;
        QVector<int> matchedFrames;
        QVector<int> failedFrames;
        if (!buildMissedBatchCorrections(corrections,
                                         startFrame,
                                         endFrame,
                                         overlapPixelThreshold,
                                         &matchedCorrections,
                                         &matchedFrames,
                                         &failedFrames,
                                         &error))
        {
            QMessageBox::warning(this, QStringLiteral("手动批量添加"), error);
            return;
        }
        if (!failedFrames.isEmpty())
        {
            QMessageBox::warning(this,
                                 QStringLiteral("手动批量添加"),
                                 QStringLiteral("漏检匹配在帧 %1 失败，已停止对应方向检索。")
                                     .arg(framesText(failedFrames)));
        }
        if (matchedCorrections.isEmpty())
        {
            if (failedFrames.isEmpty())
            {
                QMessageBox::information(this, QStringLiteral("手动批量添加"), QStringLiteral("没有可添加的目标帧。"));
            }
            return;
        }
        appendResolvedCorrections(matchedCorrections, QStringLiteral("手动批量添加"));
        return;
    }

    QVector<int> frames;
    for (int frame = startFrame; frame <= endFrame; ++frame)
    {
        frames.push_back(frame);
    }
    appendCorrectionsToFrames(corrections, frames, QStringLiteral("手动批量添加"));
}

void MainWindow::autoMatchCorrectionFrames(const QVector<int>& correctionRows,
                                           int backwardMaxFrames,
                                           int forwardMaxFrames,
                                           double positionThreshold,
                                           double overlapPixelThreshold)
{
    if (!m_reader.isOpen())
    {
        return;
    }
    m_pendingAutoBatchRows.clear();
    m_pendingAutoMatchedCorrections.clear();

    QVector<CorrectionShape> corrections;
    QString error;
    if (!collectBatchCorrections(correctionRows, &corrections, &error))
    {
        QMessageBox::warning(this, QStringLiteral("自动批量添加"), error);
        return;
    }
    if (correctionsHaveMixedBatchTypes(corrections))
    {
        QMessageBox::warning(this,
                             QStringLiteral("批量操作"),
                             QStringLiteral("选中条目同时包含漏检和非漏检类型，不可混用批量操作。"));
        return;
    }
    if (correctionsAreAllMissed(corrections))
    {
        QVector<CorrectionShape> matchedCorrections;
        QVector<int> matchedFrames;
        QVector<int> failedFrames;
        if (!buildMissedBatchCorrections(corrections,
                                         0,
                                         m_reader.frameCount() - 1,
                                         overlapPixelThreshold,
                                         &matchedCorrections,
                                         &matchedFrames,
                                         &failedFrames,
                                         &error))
        {
            QMessageBox::warning(this, QStringLiteral("自动批量添加"), error);
            return;
        }

        m_pendingAutoBatchRows = normalizedRows(correctionRows);
        m_pendingAutoMatchedCorrections = matchedCorrections;
        m_annotationPanel->setAutoMatchedBatchFrames(matchedFrames);
        QString status = QStringLiteral("自动识别匹配帧完成：%1 帧").arg(matchedFrames.size());
        if (!failedFrames.isEmpty())
        {
            status += QStringLiteral("；停止帧：%1").arg(framesText(failedFrames));
        }
        statusBar()->showMessage(status, 3000);
        return;
    }

    for (const CorrectionShape& correction : corrections)
    {
        bool hasSource = false;
        for (const ErrorCircle& circle : correction.errorCircles)
        {
            if (circle.circleIndex >= 0)
            {
                hasSource = true;
                break;
            }
        }
        if (!hasSource)
        {
            QMessageBox::warning(this,
                                 QStringLiteral("自动批量添加"),
                                 QStringLiteral("自动匹配要求选中的每个纠错条目都包含错误源光点。"));
            return;
        }
    }

    beacon_result_t baseResult;
    if (!processFrameForBatch(m_currentFrame, &baseResult, &error))
    {
        QMessageBox::warning(this, QStringLiteral("自动批量添加"), error);
        return;
    }

    QVector<int> matchedFrames;
    auto scanDirection = [&](int direction, int maxFrames) {
        beacon_result_t previousResult = baseResult;
        int previousFrame = m_currentFrame;
        for (int step = 1; step <= maxFrames; ++step)
        {
            const int nextFrame = previousFrame + direction;
            if (nextFrame < 0 || nextFrame >= m_reader.frameCount())
            {
                break;
            }

            beacon_result_t nextResult;
            QString frameError;
            if (!processFrameForBatch(nextFrame, &nextResult, &frameError))
            {
                break;
            }
            if (!batchCorrectionsMatchAdjacent(corrections, previousResult, nextResult, positionThreshold))
            {
                break;
            }

            if (direction < 0)
            {
                matchedFrames.prepend(nextFrame);
            }
            else
            {
                matchedFrames.push_back(nextFrame);
            }
            previousResult = nextResult;
            previousFrame = nextFrame;
        }
    };

    scanDirection(-1, qMax(0, backwardMaxFrames));
    scanDirection(1, qMax(0, forwardMaxFrames));
    m_annotationPanel->setAutoMatchedBatchFrames(matchedFrames);
    statusBar()->showMessage(QStringLiteral("自动识别匹配帧完成：%1 帧").arg(matchedFrames.size()), 3000);
}

void MainWindow::batchAddCorrectionsToFrames(const QVector<int>& correctionRows, const QVector<int>& frames)
{
    if (!m_reader.isOpen())
    {
        return;
    }

    QVector<CorrectionShape> corrections;
    QString error;
    if (!collectBatchCorrections(correctionRows, &corrections, &error))
    {
        QMessageBox::warning(this, QStringLiteral("自动批量添加"), error);
        return;
    }
    if (correctionsHaveMixedBatchTypes(corrections))
    {
        QMessageBox::warning(this,
                             QStringLiteral("批量操作"),
                             QStringLiteral("选中条目同时包含漏检和非漏检类型，不可混用批量操作。"));
        return;
    }
    if (correctionsAreAllMissed(corrections))
    {
        const QVector<int> rows = normalizedRows(correctionRows);
        if (m_pendingAutoBatchRows != rows || m_pendingAutoMatchedCorrections.isEmpty())
        {
            QMessageBox::warning(this,
                                 QStringLiteral("自动批量添加"),
                                 QStringLiteral("请先点击“自动识别匹配帧”生成匹配帧列表。"));
            return;
        }

        const QVector<int> targetFrames = normalizedRows(frames);
        QVector<CorrectionShape> resolvedCorrections;
        for (const CorrectionShape& correction : m_pendingAutoMatchedCorrections)
        {
            if (targetFrames.contains(correction.frame))
            {
                resolvedCorrections.push_back(correction);
            }
        }
        appendResolvedCorrections(resolvedCorrections, QStringLiteral("自动批量添加"));
        return;
    }

    appendCorrectionsToFrames(corrections, frames, QStringLiteral("自动批量添加"));
}

void MainWindow::addCorrectionShape(const QString& shapeType, const QVector<QPointF>& points)
{
    if (!m_reader.isOpen() || points.isEmpty() || currentInstance() == nullptr)
    {
        return;
    }

    m_annotationPanel->applyDrawnCorrectionShape(shapeType, points);
    renderInstance(currentInstance());
    updateCurrentAnnotationInfo();
}

void MainWindow::autoIdentifyCorrectionTargets()
{
    if (!m_reader.isOpen())
    {
        return;
    }

    CorrectionShape shape;
    if (!m_annotationPanel->activeDraftShape(&shape) || !isClosedCorrectionShape(shape))
    {
        QMessageBox::information(this,
                                 QStringLiteral("自动识别"),
                                 QStringLiteral("请先在当前帧绘制一个封闭纠错图形（圆、矩形或自由闭合）。"));
        return;
    }

    QVector<int> matchedIndices;
    for (int i = 0; i < m_currentResult.count && i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        const beacon_circle_t& circle = m_currentResult.circles[i];
        if (circle.valid == 0)
        {
            continue;
        }

        const QPointF imagePoint = FrameRenderer::algorithmToImagePoint(circle.x, circle.y);
        if (shapeContainsPoint(shape, imagePoint))
        {
            matchedIndices.push_back(i);
        }
    }

    if (matchedIndices.isEmpty())
    {
        QMessageBox::information(this,
                                 QStringLiteral("自动识别"),
                                 QStringLiteral("封闭区域内没有已识别的光点目标。"));
        return;
    }

    m_annotationPanel->applyAutoIdentifiedErrorCircles(matchedIndices);
    showFrame(m_currentFrame);
    statusBar()->showMessage(QStringLiteral("已自动选择 %1 个错误圆").arg(matchedIndices.size()), 3000);
}

void MainWindow::openAlgorithmLocation()
{
    AnalyzerInstance* instance = requireCurrentInstance(QStringLiteral("打开算法位置"));
    if (instance == nullptr)
    {
        return;
    }

    const QString algorithmPath = instance->algorithmPath.isEmpty()
        ? defaultAlgorithmPath()
        : instance->algorithmPath;
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
    const AnalyzerInstance* instance = currentInstance();
    if (instance == nullptr)
    {
        m_annotationPanel->setAnnotations({}, {});
        updateCurrentAnnotationInfo();
        return;
    }

    m_annotationPanel->setAnnotations(instance->annotations.records(), instance->annotations.corrections());
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
    const int previousInstanceId = m_currentInstanceId;
    for (AnalyzerInstance* instance : m_instances)
    {
        if (instance != nullptr)
        {
            m_currentInstanceId = instance->id;
            saveProject();
        }
    }
    m_currentInstanceId = previousInstanceId;
    QMainWindow::closeEvent(event);
}

void MainWindow::restoreLastSession()
{
    if (m_instances.isEmpty())
    {
        return;
    }

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
    AnalyzerInstance* instance = currentInstance();
    if (instance == nullptr || !m_reader.isOpen() || m_currentVideoPath.isEmpty())
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
    algorithm.insert(QStringLiteral("source_path"), instance->algorithmPath);
    algorithm.insert(QStringLiteral("instance_name"), instance->name);
    algorithm.insert(QStringLiteral("note"), QStringLiteral("simple threshold + connected components"));
    root.insert(QStringLiteral("algorithm"), algorithm);

    QJsonArray annotations;
    for (const AnnotationRecord& record : m_annotations.records())
    {
        QJsonObject item;
        item.insert(QStringLiteral("type"), record.type);
        item.insert(QStringLiteral("types"), stringListToJson(record.types));
        item.insert(QStringLiteral("start_frame"), record.startFrame);
        item.insert(QStringLiteral("end_frame"), record.endFrame);
        item.insert(QStringLiteral("start_time_sec"), record.startTimeSec);
        item.insert(QStringLiteral("end_time_sec"), record.endTimeSec);
        item.insert(QStringLiteral("circle_index"), record.circleIndex);
        item.insert(QStringLiteral("error_circles"), errorCirclesToJson(record.errorCircles));
        item.insert(QStringLiteral("description"), record.description);
        annotations.append(item);
    }
    root.insert(QStringLiteral("annotations"), annotations);

    QJsonArray corrections;
    for (const CorrectionShape& shape : m_annotations.corrections())
    {
        QJsonObject item;
        item.insert(QStringLiteral("name"), shape.name);
        item.insert(QStringLiteral("shape_type"), shape.shapeType);
        item.insert(QStringLiteral("frame"), shape.frame);
        item.insert(QStringLiteral("error_type"), shape.errorType);
        item.insert(QStringLiteral("error_types"), stringListToJson(shape.errorTypes));
        item.insert(QStringLiteral("expected_index"), shape.expectedIndex);
        item.insert(QStringLiteral("error_circles"), errorCirclesToJson(shape.errorCircles));
        item.insert(QStringLiteral("description"), shape.description);
        item.insert(QStringLiteral("line_color"), shape.lineColor.name(QColor::HexRgb));
        item.insert(QStringLiteral("line_width"), shape.lineWidth);
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
    const AnalyzerInstance* instance = currentInstance();
    if (instance != nullptr && !instance->rootDir.isEmpty())
    {
        return QDir(instance->rootDir).absoluteFilePath(info.completeBaseName() + QStringLiteral(".bia_project.json"));
    }
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
    if (m_liveMode && !m_liveSaveDir.isEmpty())
    {
        return QDir(m_liveSaveDir).absoluteFilePath(QStringLiteral("tcp_frame%1%2").arg(qMax(0, m_liveFrameIndex)).arg(suffix));
    }
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

