#include "FusionRunner.h"

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
    const QFileInfo sourceInfo(sourcePath);
    const QByteArray key = (sourceInfo.absoluteFilePath() +
                            QString::number(sourceInfo.lastModified().toMSecsSinceEpoch()) +
                            QString::number(QDateTime::currentMSecsSinceEpoch())).toUtf8();
    const QString hash = QString::fromLatin1(QCryptographicHash::hash(key, QCryptographicHash::Sha1).toHex().left(12));
    return QDir(buildDir).absoluteFilePath(QStringLiteral("beacon_fusion_%1%2").arg(hash, librarySuffix()));
}
}

FusionRunner::FusionRunner()
{
    beacon_fusion_init();
}

FusionRunner::~FusionRunner()
{
    if (m_library.isLoaded())
    {
        m_library.unload();
    }
}

bool FusionRunner::loadSourceFile(const QString& sourcePath, const QString& buildDir, QString* errorMessage)
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
            *errorMessage = QStringLiteral("未找到 gcc，无法编译融合分析 C 文件。");
        }
        return false;
    }

    if (m_library.isLoaded())
    {
        m_library.unload();
    }
    m_initFn = nullptr;
    m_analyzeFn = nullptr;

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
            *errorMessage = QStringLiteral("编译融合分析 C 文件失败：%1").arg(output.trimmed());
        }
        return false;
    }

    m_library.setFileName(outputPath);
    if (!m_library.load())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("加载融合分析库失败：%1").arg(m_library.errorString());
        }
        return false;
    }

    m_initFn = reinterpret_cast<InitFn>(m_library.resolve("beacon_fusion_init"));
    m_analyzeFn = reinterpret_cast<AnalyzeFn>(m_library.resolve("beacon_fusion_analyze"));
    if (m_analyzeFn == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("融合分析库未导出 beacon_fusion_analyze。");
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

QString FusionRunner::sourcePath() const
{
    return m_sourcePath;
}

bool FusionRunner::usesDynamicLibrary() const
{
    return m_analyzeFn != nullptr;
}

beacon_fusion_result_t FusionRunner::analyze(const beacon_result_t cameraResults[BEACON_CAMERA_COUNT]) const
{
    beacon_fusion_result_t result;
    memset(&result, 0, sizeof(result));

    if (cameraResults == nullptr)
    {
        return result;
    }

    if (m_analyzeFn != nullptr)
    {
        m_analyzeFn(cameraResults, &result);
    }
    else
    {
        beacon_fusion_analyze(cameraResults, &result);
    }
    return result;
}
