#include "AlgorithmRunner.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
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
}

AlgorithmRunner::AlgorithmRunner()
{
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
    m_imageUpdateFn = nullptr;
    m_dynamicBeaconCount = nullptr;
    m_dynamicBeacons = nullptr;
    m_dynamicCarLampCount = nullptr;
    m_dynamicCarLamps = nullptr;
    m_dynamicFrameBuffer = nullptr;
    m_dynamicFinishFlag = nullptr;

    const QString outputPath = dynamicLibraryPath(sourceInfo.absoluteFilePath(), buildDir);
    QStringList arguments;
    arguments << QStringLiteral("-shared")
              << QStringLiteral("-O2")
              << QStringLiteral("-std=c11")
              << QStringLiteral("-DBEACON_DESKTOP_STANDALONE")
              << QStringLiteral("-I") << sourceInfo.absolutePath()
              << QStringLiteral("-I") << projectAlgorithmIncludeDir()
              << QStringLiteral("-o") << outputPath
              << sourceInfo.absoluteFilePath()
              << QDir(projectAlgorithmIncludeDir()).absoluteFilePath(QStringLiteral("desktop_mt9v03x_stub.c"))
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

    m_initFn = reinterpret_cast<InitFn>(m_library.resolve("image_init"));
    m_imageUpdateFn = reinterpret_cast<ImageUpdateFn>(m_library.resolve("image_update"));
    m_dynamicBeaconCount = reinterpret_cast<ImageCircleCountPtr>(m_library.resolve("g_image_beacon_count"));
    m_dynamicBeacons = reinterpret_cast<ImageCirclePtr>(m_library.resolve("g_image_beacons"));
    m_dynamicCarLampCount = reinterpret_cast<ImageCarLampCountPtr>(m_library.resolve("g_image_car_lamp_count"));
    m_dynamicCarLamps = reinterpret_cast<ImageCarLampPtr>(m_library.resolve("g_image_car_lamps"));
    m_dynamicFrameBuffer = reinterpret_cast<unsigned char*>(m_library.resolve("mt9v03x_image"));
    m_dynamicFinishFlag = reinterpret_cast<unsigned char*>(m_library.resolve("mt9v03x_finish_flag"));

    if (m_dynamicFrameBuffer == nullptr)
    {
        using ImageGetFrameBufferFn = unsigned char* (*)();
        const auto getFrameBuffer =
            reinterpret_cast<ImageGetFrameBufferFn>(m_library.resolve("image_get_frame_buffer"));
        if (getFrameBuffer != nullptr)
        {
            m_dynamicFrameBuffer = getFrameBuffer();
        }
    }

    if (m_imageUpdateFn == nullptr ||
        m_dynamicBeaconCount == nullptr ||
        m_dynamicBeacons == nullptr ||
        m_dynamicFrameBuffer == nullptr ||
        m_dynamicFinishFlag == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("算法库需导出 image_update、g_image_beacons、g_image_beacon_count、mt9v03x_image、mt9v03x_finish_flag。");
        }
        m_library.unload();
        return false;
    }

    if (m_initFn != nullptr)
    {
        m_initFn();
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
    return m_imageUpdateFn != nullptr &&
           m_dynamicBeaconCount != nullptr &&
           m_dynamicBeacons != nullptr &&
           m_dynamicFrameBuffer != nullptr &&
           m_dynamicFinishFlag != nullptr;
}

beacon_result_t AlgorithmRunner::process(const QImage& grayImage) const
{
    beacon_result_t result;
    unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W];
    memset(&result, 0, sizeof(result));

    if (!copyGrayToAlgorithmImage(grayImage, image))
    {
        return result;
    }

    if (m_imageUpdateFn != nullptr && m_dynamicBeaconCount != nullptr && m_dynamicBeacons != nullptr &&
             m_dynamicFrameBuffer != nullptr && m_dynamicFinishFlag != nullptr)
    {
        const int imageSize = BEACON_IMAGE_H * BEACON_IMAGE_W;
        memcpy(m_dynamicFrameBuffer, image[0], imageSize);
        *m_dynamicFinishFlag = 1U;
        m_imageUpdateFn();

        const int beaconCopyCount = qMin((int)*m_dynamicBeaconCount, BEACON_MAX_CIRCLE_COUNT);
        for (int i = 0; i < beaconCopyCount; ++i)
        {
            result.beacons[i] = m_dynamicBeacons[i];
            result.circles[i] = result.beacons[i];
        }
        result.count = (unsigned char)beaconCopyCount;
        result.beacon_count = result.count;

        if (m_dynamicCarLampCount != nullptr && m_dynamicCarLamps != nullptr)
        {
            const int carLampCopyCount = qMin((int)*m_dynamicCarLampCount, BEACON_MAX_CAR_LAMP_COUNT);
            for (int i = 0; i < carLampCopyCount; ++i)
            {
                result.car_lamps[i] = m_dynamicCarLamps[i];
            }
            result.car_lamp_count = (unsigned char)carLampCopyCount;
        }
    }
    else
    {
        const int imageSize = BEACON_IMAGE_H * BEACON_IMAGE_W;
        memcpy(mt9v03x_image[0], image[0], imageSize);
        mt9v03x_finish_flag = 1U;
        image_update();

        const int beaconCopyCount = qMin((int)g_image_beacon_count, BEACON_MAX_CIRCLE_COUNT);
        for (int i = 0; i < beaconCopyCount; ++i)
        {
            result.beacons[i] = g_image_beacons[i];
            result.circles[i] = result.beacons[i];
        }
        result.count = (unsigned char)beaconCopyCount;
        result.beacon_count = result.count;

        const int carLampCopyCount = qMin((int)g_image_car_lamp_count, BEACON_MAX_CAR_LAMP_COUNT);
        for (int i = 0; i < carLampCopyCount; ++i)
        {
            result.car_lamps[i] = g_image_car_lamps[i];
        }
        result.car_lamp_count = (unsigned char)carLampCopyCount;
    }
    return result;
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

    constexpr unsigned char Threshold = 200U;
    for (int y = 0; y < BEACON_IMAGE_H; ++y)
    {
        for (int x = 0; x < BEACON_IMAGE_W; ++x)
        {
            binary[y][x] = image[y][x] >= Threshold ? 255U : 0U;
        }
    }

    QImage output(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    for (int y = 0; y < BEACON_IMAGE_H; ++y)
    {
        memcpy(output.scanLine(y), binary[y], BEACON_IMAGE_W);
    }

    return output;
}
