#include "AlgorithmRunner.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QProcess>
#include <QStandardPaths>

#include <cstring>

namespace
{
QString projectAlgorithmIncludeDir()
{
#ifdef BEACON_SOURCE_DIR
    return QDir(QStringLiteral(BEACON_SOURCE_DIR)).absoluteFilePath(QStringLiteral("algorithm"));
#else
    return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../algorithm"));
#endif
}

QString compilerPath()
{
    const QString fromPath = QStandardPaths::findExecutable(QStringLiteral("gcc"));
    if (!fromPath.isEmpty())
    {
        return fromPath;
    }

    const QStringList candidates = {
        QStringLiteral("C:/code/msys64/mingw64/bin/gcc.exe"),
        QStringLiteral("C:/msys64/mingw64/bin/gcc.exe")
    };
    for (const QString& candidate : candidates)
    {
        if (QFileInfo::exists(candidate))
        {
            return candidate;
        }
    }
    return QString();
}

QString librarySuffix()
{
#ifdef Q_OS_WIN
    return QStringLiteral(".dll");
#elif defined(Q_OS_MACOS)
    return QStringLiteral(".dylib");
#else
    return QStringLiteral(".so");
#endif
}

QString dynamicLibraryPath(const QString& sourcePath, const QString& buildDir)
{
    QFileInfo sourceInfo(sourcePath);
    const QByteArray key = (sourceInfo.absoluteFilePath() +
                            QString::number(sourceInfo.lastModified().toMSecsSinceEpoch()) +
                            QString::number(QDateTime::currentMSecsSinceEpoch())).toUtf8();
    const QString hash = QString::fromLatin1(QCryptographicHash::hash(key, QCryptographicHash::Sha1).toHex().left(12));
    return QDir(buildDir).absoluteFilePath(QStringLiteral("beacon_algorithm_%1%2").arg(hash, librarySuffix()));
}

bool copyGrayToAlgorithmImage(const QImage& grayImage,
                              unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    memset(image, 0, BEACON_IMAGE_H * BEACON_IMAGE_W);

    if (grayImage.width() != BEACON_IMAGE_W || grayImage.height() != BEACON_IMAGE_H)
    {
        return false;
    }

    const QImage normalized = grayImage.format() == QImage::Format_Grayscale8
        ? grayImage
        : grayImage.convertToFormat(QImage::Format_Grayscale8);

    for (int y = 0; y < BEACON_IMAGE_H; ++y)
    {
        const unsigned char* source = normalized.constScanLine(y);
        memcpy(image[y], source, BEACON_IMAGE_W);
    }

    return true;
}

constexpr double McuEstimateMinScale = 80.0;
constexpr double McuEstimateMaxScale = 240.0;
}

double AlgorithmProcessProfiler::milliseconds(const AlgorithmProcessProfile& profile)
{
    return profile.valid ? (double)profile.algorithmNanoseconds / 1000000.0 : 0.0;
}

double AlgorithmProcessProfiler::estimatedMcuMillisecondsMin(const AlgorithmProcessProfile& profile)
{
    return milliseconds(profile) * McuEstimateMinScale;
}

double AlgorithmProcessProfiler::estimatedMcuMillisecondsMax(const AlgorithmProcessProfile& profile)
{
    return milliseconds(profile) * McuEstimateMaxScale;
}

QString AlgorithmProcessProfiler::format(const AlgorithmProcessProfile& profile, double fps)
{
    if (!profile.valid)
    {
        return QStringLiteral("算法耗时: --");
    }

    const double pcMs = milliseconds(profile);
    const double mcuMinMs = estimatedMcuMillisecondsMin(profile);
    const double mcuMaxMs = estimatedMcuMillisecondsMax(profile);
    const double frameBudgetMs = fps > 0.0 ? 1000.0 / fps : 20.0;
    const double budgetMinPercent = frameBudgetMs > 0.0 ? mcuMinMs * 100.0 / frameBudgetMs : 0.0;
    const double budgetMaxPercent = frameBudgetMs > 0.0 ? mcuMaxMs * 100.0 / frameBudgetMs : 0.0;
    return QStringLiteral("PC算法耗时: %1 ms | %2MHz板端粗估: %3-%4 ms | 帧预算 %5-%6%")
        .arg(pcMs, 0, 'f', 3)
        .arg(TargetCoreMhz, 0, 'f', 0)
        .arg(mcuMinMs, 0, 'f', 1)
        .arg(mcuMaxMs, 0, 'f', 1)
        .arg(budgetMinPercent, 0, 'f', 1)
        .arg(budgetMaxPercent, 0, 'f', 1);
}

QString AlgorithmProcessProfiler::formatCompact(const AlgorithmProcessProfile& profile)
{
    if (!profile.valid)
    {
        return QStringLiteral("耗时 --");
    }

    return QStringLiteral("PC %1 ms | 板估@%2MHz %3-%4 ms")
        .arg(milliseconds(profile), 0, 'f', 3)
        .arg(TargetCoreMhz, 0, 'f', 0)
        .arg(estimatedMcuMillisecondsMin(profile), 0, 'f', 1)
        .arg(estimatedMcuMillisecondsMax(profile), 0, 'f', 1);
}

AlgorithmRunner::AlgorithmRunner()
{
    beacon_image_init();
    beacon_image_reset_temporal();
}

AlgorithmRunner::~AlgorithmRunner()
{
    if (m_library.isLoaded())
    {
        m_library.unload();
    }
}

bool AlgorithmRunner::loadSourceFile(const QString& sourcePath, const QString& buildDir, QString* errorMessage)
{
    const QFileInfo sourceInfo(sourcePath);
    if (!sourceInfo.isFile())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("C 文件不存在：%1").arg(sourcePath);
        }
        return false;
    }

    QDir().mkpath(buildDir);
    const QString compiler = compilerPath();
    if (compiler.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("未找到 gcc，无法为实例编译独立 C 文件。");
        }
        return false;
    }

    if (m_library.isLoaded())
    {
        m_library.unload();
    }
    m_initFn = nullptr;
    m_resetTemporalFn = nullptr;
    m_processFn = nullptr;
    m_binaryFn = nullptr;

    const QString outputPath = dynamicLibraryPath(sourceInfo.absoluteFilePath(), buildDir);
    QStringList arguments;
    arguments << QStringLiteral("-shared")
              << QStringLiteral("-O2")
              << QStringLiteral("-std=c11")
              << QStringLiteral("-I") << sourceInfo.absolutePath()
              << QStringLiteral("-I") << projectAlgorithmIncludeDir()
              << QStringLiteral("-o") << outputPath
              << sourceInfo.absoluteFilePath()
              << QStringLiteral("-lm");

    QProcess compilerProcess;
    compilerProcess.start(compiler, arguments);
    if (!compilerProcess.waitForFinished(60000) || compilerProcess.exitStatus() != QProcess::NormalExit ||
        compilerProcess.exitCode() != 0)
    {
        if (errorMessage != nullptr)
        {
            const QString output = QString::fromLocal8Bit(compilerProcess.readAllStandardError()) +
                                   QString::fromLocal8Bit(compilerProcess.readAllStandardOutput());
            *errorMessage = QStringLiteral("编译 C 文件失败：%1").arg(output.trimmed());
        }
        return false;
    }

    m_library.setFileName(outputPath);
    if (!m_library.load())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("加载算法库失败：%1").arg(m_library.errorString());
        }
        return false;
    }

    m_initFn = reinterpret_cast<InitFn>(m_library.resolve("beacon_image_init"));
    m_resetTemporalFn = reinterpret_cast<ResetTemporalFn>(m_library.resolve("beacon_image_reset_temporal"));
    m_processFn = reinterpret_cast<ProcessFn>(m_library.resolve("beacon_image_process"));
    m_binaryFn = reinterpret_cast<BinaryFn>(m_library.resolve("beacon_image_debug_binary"));
    if (m_processFn == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("算法库未导出 beacon_image_process。");
        }
        m_library.unload();
        return false;
    }

    if (m_initFn != nullptr)
    {
        m_initFn();
    }
    if (m_resetTemporalFn != nullptr)
    {
        m_resetTemporalFn();
    }
    m_sourcePath = sourceInfo.absoluteFilePath();
    return true;
}

QString AlgorithmRunner::sourcePath() const
{
    return m_sourcePath;
}

bool AlgorithmRunner::usesDynamicLibrary() const
{
    return m_processFn != nullptr;
}

void AlgorithmRunner::resetTemporal() const
{
    if (m_resetTemporalFn != nullptr)
    {
        m_resetTemporalFn();
        return;
    }
    if (m_processFn == nullptr)
    {
        beacon_image_reset_temporal();
    }
}

beacon_result_t AlgorithmRunner::process(const QImage& grayImage) const
{
    beacon_result_t result;
    unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W];
    memset(&result, 0, sizeof(result));
    m_lastProcessProfile = {};

    if (!copyGrayToAlgorithmImage(grayImage, image))
    {
        return result;
    }

    QElapsedTimer timer;
    timer.start();
    if (m_processFn != nullptr)
    {
        m_processFn(image, &result);
    }
    else
    {
        beacon_image_process(image, &result);
    }
    m_lastProcessProfile.valid = true;
    m_lastProcessProfile.algorithmNanoseconds = timer.nsecsElapsed();
    return result;
}

AlgorithmProcessProfile AlgorithmRunner::lastProcessProfile() const
{
    return m_lastProcessProfile;
}

QImage AlgorithmRunner::binaryImage(const QImage& grayImage) const
{
    unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W];
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
    memset(binary, 0, sizeof(binary));

    if (!copyGrayToAlgorithmImage(grayImage, image))
    {
        return QImage();
    }

    if (m_binaryFn != nullptr)
    {
        m_binaryFn(image, binary);
    }
    else if (m_processFn == nullptr)
    {
        beacon_image_debug_binary(image, binary);
    }
    else
    {
        return QImage();
    }

    QImage output(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    for (int y = 0; y < BEACON_IMAGE_H; ++y)
    {
        memcpy(output.scanLine(y), binary[y], BEACON_IMAGE_W);
    }

    return output;
}
