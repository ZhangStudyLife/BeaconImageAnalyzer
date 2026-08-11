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

QString twoBl3HostAdapterPath()
{
    return QDir(projectAlgorithmIncludeDir())
        .absoluteFilePath(QStringLiteral("2bl3_host/two_bl3_desktop_adapter.c"));
}

QString twoBl3HostAdapterPathForImageDirectory(const QString& imageDirectory)
{
    const QDir imageDir(imageDirectory);
    const QStringList candidates = {
        imageDir.absoluteFilePath(QStringLiteral("../host/two_bl3_desktop_adapter.c")),
        imageDir.absoluteFilePath(QStringLiteral("../two_bl3_desktop_adapter.c")),
        twoBl3HostAdapterPath()
    };

    for (const QString& candidate : candidates)
    {
        if (QFileInfo(candidate).isFile())
        {
            return QFileInfo(candidate).absoluteFilePath();
        }
    }
    return QString();
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
    clearDynamicFunctions();

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

    if (!resolveDynamicFunctions(errorMessage))
    {
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
    setCarLampMode(m_carLampMode);
    m_sourcePath = sourceInfo.absoluteFilePath();
    return true;
}

bool AlgorithmRunner::loadTwoBl3Firmware(const QString& imageDirectory,
                                         const QString& buildDir,
                                         QString* errorMessage)
{
    const QDir imageDir(imageDirectory);
    const QString imageSource = imageDir.absoluteFilePath(QStringLiteral("image.c"));
    const QString parameterSource = imageDir.absoluteFilePath(QStringLiteral("image_params.c"));
    const QString horizonSource = imageDir.absoluteFilePath(QStringLiteral("image_horizon.c"));
    const QString downHorizonSource = imageDir.absoluteFilePath(
        QStringLiteral("image_down_horizon.c"));
    const QString imageHeader = imageDir.absoluteFilePath(QStringLiteral("image.h"));
    const QString adapterSource = twoBl3HostAdapterPathForImageDirectory(imageDirectory);
    const bool sourcesAvailable = QFileInfo(imageSource).isFile()
                                  && QFileInfo(parameterSource).isFile()
                                  && QFileInfo(imageHeader).isFile()
                                  && QFileInfo(adapterSource).isFile();
    const QString packagedLibrary = QDir(QCoreApplication::applicationDirPath())
                                        .absoluteFilePath(QStringLiteral("two_bl3_diagnostic%1")
                                                              .arg(librarySuffix()));
    if (!sourcesAvailable)
    {
        return loadTwoBl3Library(packagedLibrary, errorMessage);
    }

    const QString compiler = compilerPath();
    if (compiler.isEmpty())
    {
        return loadTwoBl3Library(packagedLibrary, errorMessage);
    }

    QDir().mkpath(buildDir);
    if (m_library.isLoaded())
    {
        m_library.unload();
    }
    clearDynamicFunctions();

    const QString outputPath = dynamicLibraryPath(imageSource, buildDir);
    const QString codeDirectory = QDir(imageDirectory).absoluteFilePath(QStringLiteral(".."));
    const QString adapterIncludeDirectory = QFileInfo(adapterSource).absolutePath();
    QStringList arguments;
    arguments << QStringLiteral("-shared")
              << QStringLiteral("-O2")
              << QStringLiteral("-std=c11")
              << QStringLiteral("-I") << adapterIncludeDirectory
              << QStringLiteral("-I") << codeDirectory
              << QStringLiteral("-o") << QDir::toNativeSeparators(outputPath)
              << imageSource
              << parameterSource;
    if (QFileInfo(horizonSource).isFile())
    {
        arguments << horizonSource;
    }
    if (QFileInfo(downHorizonSource).isFile())
    {
        arguments << downHorizonSource;
    }
    arguments << adapterSource
              << QStringLiteral("-lm");

    QProcess compilerProcess;
    compilerProcess.start(compiler, arguments);
    if (!compilerProcess.waitForFinished(60000)
        || compilerProcess.exitStatus() != QProcess::NormalExit
        || compilerProcess.exitCode() != 0)
    {
        if (errorMessage != nullptr)
        {
            const QString output = QString::fromLocal8Bit(compilerProcess.readAllStandardError())
                                   + QString::fromLocal8Bit(compilerProcess.readAllStandardOutput());
            *errorMessage = QStringLiteral("编译 2BL3 诊断算法失败：%1").arg(output.trimmed());
        }
        return false;
    }

    m_library.setFileName(outputPath);
    if (!m_library.load())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("加载 2BL3 诊断算法失败：%1").arg(m_library.errorString());
        }
        return false;
    }
    if (!resolveDynamicFunctions(errorMessage))
    {
        return false;
    }
    if (m_initFn == nullptr || m_resetTemporalFn == nullptr || !supportsParameterTuning())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("2BL3 诊断算法缺少初始化或参数调试接口。");
        }
        m_library.unload();
        clearDynamicFunctions();
        return false;
    }
    m_initFn();
    m_resetTemporalFn();
    setCarLampMode(m_carLampMode);
    m_sourcePath = imageSource;
    return true;
}

bool AlgorithmRunner::loadTwoBl3Library(const QString& libraryPath, QString* errorMessage)
{
    if (!QFileInfo(libraryPath).isFile())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("未找到 2BL3 固件源码或预编译诊断库：%1")
                                .arg(libraryPath);
        }
        return false;
    }
    if (m_library.isLoaded())
    {
        m_library.unload();
    }
    clearDynamicFunctions();
    m_library.setFileName(libraryPath);
    if (!m_library.load())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("加载预编译 2BL3 诊断库失败：%1")
                                .arg(m_library.errorString());
        }
        return false;
    }
    if (!resolveDynamicFunctions(errorMessage))
    {
        return false;
    }
    if (m_initFn == nullptr || m_resetTemporalFn == nullptr || !supportsParameterTuning())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("预编译 2BL3 诊断库缺少必要调试接口。");
        }
        m_library.unload();
        clearDynamicFunctions();
        return false;
    }
    m_initFn();
    m_resetTemporalFn();
    setCarLampMode(m_carLampMode);
    m_sourcePath = libraryPath;
    return true;
}

QString AlgorithmRunner::defaultTwoBl3ImageDirectory()
{
    const QString configured = qEnvironmentVariable("BEACON_2BL3_IMAGE_DIR").trimmed();
    if (!configured.isEmpty())
    {
        return QDir(configured).absolutePath();
    }
#ifdef BEACON_SOURCE_DIR
    return QDir(QStringLiteral(BEACON_SOURCE_DIR)).absoluteFilePath(
        QStringLiteral("../HDUASC-SmartCar-21st-FlyOverMinefield/"
                       "CYT2BL3_Image/project/code/Image"));
#else
    return QString();
#endif
}

void AlgorithmRunner::clearDynamicFunctions()
{
    m_initFn = nullptr;
    m_resetTemporalFn = nullptr;
    m_processFn = nullptr;
    m_binaryFn = nullptr;
    m_carLampPixelAreasFn = nullptr;
    m_setTelemetryFn = nullptr;
    m_setCarLampModeFn = nullptr;
    m_horizonCurveFn = nullptr;
    m_secondaryHorizonCurveFn = nullptr;
    m_horizonRegionFn = nullptr;
    m_processedFrameCountFn = nullptr;
    m_buildIdFn = nullptr;
    m_parameterCountFn = nullptr;
    m_parameterInfoFn = nullptr;
    m_parameterGetFn = nullptr;
    m_parameterSetFn = nullptr;
}

bool AlgorithmRunner::resolveDynamicFunctions(QString* errorMessage)
{
    m_initFn = reinterpret_cast<InitFn>(m_library.resolve("beacon_image_init"));
    m_resetTemporalFn = reinterpret_cast<ResetTemporalFn>(m_library.resolve("beacon_image_reset_temporal"));
    m_processFn = reinterpret_cast<ProcessFn>(m_library.resolve("beacon_image_process"));
    m_binaryFn = reinterpret_cast<BinaryFn>(m_library.resolve("beacon_image_debug_binary"));
    m_carLampPixelAreasFn = reinterpret_cast<CarLampPixelAreasFn>(
        m_library.resolve("beacon_image_debug_car_lamp_pixel_areas"));
    m_setTelemetryFn = reinterpret_cast<SetTelemetryFn>(
        m_library.resolve("beacon_image_set_telemetry"));
    m_setCarLampModeFn = reinterpret_cast<SetCarLampModeFn>(
        m_library.resolve("beacon_image_set_car_lamp_mode"));
    m_horizonCurveFn = reinterpret_cast<HorizonCurveFn>(
        m_library.resolve("beacon_image_debug_horizon"));
    m_secondaryHorizonCurveFn = reinterpret_cast<HorizonCurveFn>(
        m_library.resolve("beacon_image_debug_horizon_secondary"));
    m_horizonRegionFn = reinterpret_cast<HorizonRegionFn>(
        m_library.resolve("beacon_image_debug_horizon_region"));
    m_processedFrameCountFn = reinterpret_cast<ProcessedFrameCountFn>(
        m_library.resolve("beacon_image_debug_processed_frame_count"));
    m_buildIdFn = reinterpret_cast<BuildIdFn>(m_library.resolve("beacon_image_debug_build_id"));
    m_parameterCountFn = reinterpret_cast<ParameterCountFn>(
        m_library.resolve("beacon_image_debug_parameter_count"));
    m_parameterInfoFn = reinterpret_cast<ParameterInfoFn>(
        m_library.resolve("beacon_image_debug_parameter_info"));
    m_parameterGetFn = reinterpret_cast<ParameterGetFn>(
        m_library.resolve("beacon_image_debug_parameter_get"));
    m_parameterSetFn = reinterpret_cast<ParameterSetFn>(
        m_library.resolve("beacon_image_debug_parameter_set"));
    if (m_processFn == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("算法库未导出 beacon_image_process。");
        }
        m_library.unload();
        clearDynamicFunctions();
        return false;
    }
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
        setCarLampMode(m_carLampMode);
        return;
    }
    if (m_processFn == nullptr)
    {
        beacon_image_reset_temporal();
    }
}

bool AlgorithmRunner::supportsCarLampMode() const
{
    return m_setCarLampModeFn != nullptr;
}

void AlgorithmRunner::setCarLampMode(CarLampMode mode) const
{
    m_carLampMode = (mode == CarLampMode::Dual) ?
        CarLampMode::Dual : CarLampMode::Single;
    if (m_setCarLampModeFn != nullptr)
    {
        m_setCarLampModeFn(static_cast<quint8>(m_carLampMode));
    }
}

CarLampMode AlgorithmRunner::carLampMode() const
{
    return m_carLampMode;
}

void AlgorithmRunner::setFrameTelemetry(const AlgorithmFrameTelemetry& telemetry) const
{
    if (m_setTelemetryFn == nullptr)
    {
        return;
    }
    m_setTelemetryFn(telemetry.cameraId,
                     telemetry.rollDeg,
                     telemetry.pitchDeg,
                     telemetry.heightMm,
                     telemetry.attitudeValid ? 1U : 0U,
                     telemetry.heightValid ? 1U : 0U);
}

beacon_result_t AlgorithmRunner::process(const QImage& grayImage) const
{
    beacon_result_t result;
    unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W];
    memset(&result, 0, sizeof(result));
    m_lastProcessProfile = {};
    m_lastDetectionMetrics = {};

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
    if (m_carLampPixelAreasFn != nullptr)
    {
        unsigned short areas[BEACON_MAX_CAR_LAMP_COUNT] = {};
        int count = m_carLampPixelAreasFn(areas);
        if (count > BEACON_MAX_CAR_LAMP_COUNT)
        {
            count = BEACON_MAX_CAR_LAMP_COUNT;
        }
        m_lastDetectionMetrics.carLampPixelAreasAvailable = true;
        m_lastDetectionMetrics.carLampPixelAreaCount = static_cast<unsigned char>(count);
        for (int i = 0; i < count; ++i)
        {
            m_lastDetectionMetrics.carLampPixelAreas[static_cast<std::size_t>(i)] = areas[i];
        }
    }
    return result;
}

AlgorithmHorizonCurve AlgorithmRunner::horizonCurve() const
{
    AlgorithmHorizonCurve curve;
    if (m_horizonCurveFn != nullptr)
    {
        curve.valid = m_horizonCurveFn(curve.y.data(), curve.columnValid.data()) != 0U;
    }
    if (m_secondaryHorizonCurveFn != nullptr)
    {
        curve.secondaryValid = m_secondaryHorizonCurveFn(
            curve.secondaryY.data(), curve.secondaryColumnValid.data()) != 0U;
    }
    if (m_horizonRegionFn != nullptr)
    {
        curve.closedRegion = m_horizonRegionFn(curve.columnState.data()) != 0U;
    }
    return curve;
}

quint32 AlgorithmRunner::processedFrameCount() const
{
    return m_processedFrameCountFn != nullptr ? m_processedFrameCountFn() : 0U;
}

AlgorithmProcessProfile AlgorithmRunner::lastProcessProfile() const
{
    return m_lastProcessProfile;
}

AlgorithmDetectionMetrics AlgorithmRunner::lastDetectionMetrics() const
{
    return m_lastDetectionMetrics;
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

bool AlgorithmRunner::supportsParameterTuning() const
{
    return m_buildIdFn != nullptr && m_parameterCountFn != nullptr
           && m_parameterInfoFn != nullptr && m_parameterGetFn != nullptr
           && m_parameterSetFn != nullptr;
}

quint32 AlgorithmRunner::algorithmBuildId() const
{
    return m_buildIdFn != nullptr ? m_buildIdFn() : 0U;
}

QVector<AlgorithmParameterInfo> AlgorithmRunner::parameterInfos() const
{
    QVector<AlgorithmParameterInfo> result;
    if (!supportsParameterTuning())
    {
        return result;
    }
    const quint16 count = m_parameterCountFn();
    result.reserve(count);
    for (quint16 index = 0; index < count; ++index)
    {
        AlgorithmParameterInfo info;
        if (m_parameterInfoFn(index, &info.id, &info.type, &info.minimum, &info.maximum) == 0)
        {
            result.push_back(info);
        }
    }
    return result;
}

bool AlgorithmRunner::parameterValue(quint8 type, quint16 id, quint32* valueBits) const
{
    return valueBits != nullptr && m_parameterGetFn != nullptr
           && m_parameterGetFn(type, id, valueBits) == 0;
}

bool AlgorithmRunner::setParameterValue(quint8 type,
                                        quint16 id,
                                        quint32 valueBits,
                                        quint32* actualBits) const
{
    return actualBits != nullptr && m_parameterSetFn != nullptr
           && m_parameterSetFn(type, id, valueBits, actualBits) == 0;
}
