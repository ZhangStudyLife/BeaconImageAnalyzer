#include "CarPlanRunner.h"

#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QStandardPaths>
#include <QTextStream>

#include <cstring>

namespace
{
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
    return QDir(buildDir).absoluteFilePath(QStringLiteral("car_plan_%1%2").arg(hash, librarySuffix()));
}

bool writeTextFile(const QString& path, const QString& text, QString* errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法写入 %1：%2").arg(path, file.errorString());
        }
        return false;
    }

    QTextStream stream(&file);
    stream << text;
    return true;
}

QString resolveCarPlanSourcePath(const QString& path, QString* errorMessage)
{
    const QFileInfo info(path);
    if (info.isDir())
    {
        const QString sourcePath = QDir(info.absoluteFilePath()).absoluteFilePath(QStringLiteral("car_plan.c"));
        if (QFileInfo::exists(sourcePath))
        {
            return QFileInfo(sourcePath).absoluteFilePath();
        }
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("目录中没有 car_plan.c：%1").arg(info.absoluteFilePath());
        }
        return QString();
    }

    if (info.isFile())
    {
        return info.absoluteFilePath();
    }

    if (errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("CarPlan 源文件不存在：%1").arg(path);
    }
    return QString();
}

QString typedefShim()
{
    return QStringLiteral(
        "#ifndef ZF_COMMON_TYPEDEF_H\n"
        "#define ZF_COMMON_TYPEDEF_H\n"
        "#include <stdint.h>\n"
        "typedef uint8_t uint8;\n"
        "typedef uint16_t uint16;\n"
        "typedef uint32_t uint32;\n"
        "typedef uint64_t uint64;\n"
        "typedef int8_t int8;\n"
        "typedef int16_t int16;\n"
        "typedef int32_t int32;\n"
        "typedef int64_t int64;\n"
        "#endif\n");
}

QString headfileShim()
{
    return QStringLiteral(
        "#ifndef ZF_COMMON_HEADFILE_H\n"
        "#define ZF_COMMON_HEADFILE_H\n"
        "#include \"zf_common_typedef.h\"\n"
        "#include <math.h>\n"
        "#include <string.h>\n"
        "#endif\n");
}

QString wrapperSource()
{
    return QStringLiteral(
        "#include \"car_plan.h\"\n"
        "#include \"image_data.h\"\n"
        "#include <string.h>\n"
        "\n"
        "typedef struct { unsigned char valid; float x; float y; float area; } host_beacon_t;\n"
        "typedef struct { unsigned char valid; float cx; float cy; float angle; float width; float length; } host_car_lamp_t;\n"
        "typedef struct { host_beacon_t beacons[2]; host_car_lamp_t car_lamp; } host_camera_frame_t;\n"
        "typedef struct {\n"
        "    unsigned char valid;\n"
        "    unsigned char camera;\n"
        "    unsigned char beacon_index;\n"
        "    float target_strafe_mps;\n"
        "    float target_forward_mps;\n"
        "    float dist_px;\n"
        "    float along;\n"
        "    float perp;\n"
        "} host_car_plan_result_t;\n"
        "\n"
        "struct image_data image_data[IMAGE_CAMERA_COUNT];\n"
        "\n"
        "static void clear_host_image_data(void)\n"
        "{\n"
        "    unsigned char camera;\n"
        "    for (camera = 0; camera < (unsigned char)IMAGE_CAMERA_COUNT; camera++)\n"
        "    {\n"
        "        image_data_clear(&image_data[camera]);\n"
        "    }\n"
        "}\n"
        "\n"
        "void car_plan_instance_reset(void)\n"
        "{\n"
        "    clear_host_image_data();\n"
        "    CarPlan_Reset();\n"
        "}\n"
        "\n"
        "unsigned char car_plan_instance_update(const host_camera_frame_t frames[3], host_car_plan_result_t *out)\n"
        "{\n"
        "    car_plan_result_t result;\n"
        "    unsigned char camera;\n"
        "    unsigned char i;\n"
        "\n"
        "    if (frames == 0 || out == 0)\n"
        "    {\n"
        "        return 0;\n"
        "    }\n"
        "\n"
        "    clear_host_image_data();\n"
        "    memset(out, 0, sizeof(*out));\n"
        "    for (camera = 0; camera < 3; camera++)\n"
        "    {\n"
        "        for (i = 0; i < 2; i++)\n"
        "        {\n"
        "            image_data[camera].beacon_data[i].valid = frames[camera].beacons[i].valid;\n"
        "            image_data[camera].beacon_data[i].x = frames[camera].beacons[i].x;\n"
        "            image_data[camera].beacon_data[i].y = frames[camera].beacons[i].y;\n"
        "            image_data[camera].beacon_data[i].area = frames[camera].beacons[i].area;\n"
        "        }\n"
        "        image_data[camera].car_lamp_data[0].valid = frames[camera].car_lamp.valid;\n"
        "        image_data[camera].car_lamp_data[0].cx = frames[camera].car_lamp.cx;\n"
        "        image_data[camera].car_lamp_data[0].cy = frames[camera].car_lamp.cy;\n"
        "        image_data[camera].car_lamp_data[0].angle = frames[camera].car_lamp.angle;\n"
        "        image_data[camera].car_lamp_data[0].width = frames[camera].car_lamp.width;\n"
        "        image_data[camera].car_lamp_data[0].length = frames[camera].car_lamp.length;\n"
        "    }\n"
        "\n"
        "    CarPlan_Update(&result);\n"
        "    out->valid = result.valid;\n"
        "    out->camera = result.camera;\n"
        "    out->beacon_index = result.beacon_index;\n"
        "    out->target_strafe_mps = result.target_strafe_mps;\n"
        "    out->target_forward_mps = result.target_forward_mps;\n"
        "    out->dist_px = result.dist_px;\n"
        "    out->along = result.along;\n"
        "    out->perp = result.perp;\n"
        "    return result.valid;\n"
        "}\n");
}
}

CarPlanRunner::~CarPlanRunner()
{
    if (m_library.isLoaded())
    {
        m_library.unload();
    }
}

bool CarPlanRunner::loadSourcePath(const QString& path, const QString& buildDir, QString* errorMessage)
{
    const QString sourcePath = resolveCarPlanSourcePath(path, errorMessage);
    if (sourcePath.isEmpty())
    {
        return false;
    }

    const QString compiler = compilerPath();
    if (compiler.isEmpty())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("未找到 gcc，无法编译 CarPlan 实例。");
        }
        return false;
    }

    QDir().mkpath(buildDir);
    const QFileInfo sourceInfo(sourcePath);
    const QDir build(buildDir);
    const QString shimTypedefPath = build.absoluteFilePath(QStringLiteral("zf_common_typedef.h"));
    const QString shimHeadfilePath = build.absoluteFilePath(QStringLiteral("zf_common_headfile.h"));
    const QString wrapperPath = build.absoluteFilePath(QStringLiteral("car_plan_host_wrapper.c"));
    if (!writeTextFile(shimTypedefPath, typedefShim(), errorMessage) ||
        !writeTextFile(shimHeadfilePath, headfileShim(), errorMessage) ||
        !writeTextFile(wrapperPath, wrapperSource(), errorMessage))
    {
        return false;
    }

    if (m_library.isLoaded())
    {
        m_library.unload();
    }
    m_sourcePath.clear();
    m_resetFn = nullptr;
    m_updateFn = nullptr;

    const QString imageDir = QDir(sourceInfo.absolutePath()).absoluteFilePath(QStringLiteral("../Image"));
    const QString outputPath = dynamicLibraryPath(sourcePath, buildDir);
    QStringList arguments;
    arguments << QStringLiteral("-shared")
              << QStringLiteral("-O2")
              << QStringLiteral("-std=c11")
              << QStringLiteral("-I") << sourceInfo.absolutePath()
              << QStringLiteral("-I") << imageDir
              << QStringLiteral("-I") << buildDir
              << QStringLiteral("-o") << outputPath
              << sourcePath
              << wrapperPath
              << QStringLiteral("-lm");

    QProcess compilerProcess;
    compilerProcess.start(compiler, arguments);
    if (!compilerProcess.waitForFinished(60000) ||
        compilerProcess.exitStatus() != QProcess::NormalExit ||
        compilerProcess.exitCode() != 0)
    {
        if (errorMessage != nullptr)
        {
            const QString output = QString::fromLocal8Bit(compilerProcess.readAllStandardError()) +
                                   QString::fromLocal8Bit(compilerProcess.readAllStandardOutput());
            *errorMessage = QStringLiteral("编译 CarPlan 失败：%1").arg(output.trimmed());
        }
        return false;
    }

    m_library.setFileName(outputPath);
    if (!m_library.load())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("加载 CarPlan 动态库失败：%1").arg(m_library.errorString());
        }
        return false;
    }

    m_resetFn = reinterpret_cast<ResetFn>(m_library.resolve("car_plan_instance_reset"));
    m_updateFn = reinterpret_cast<UpdateFn>(m_library.resolve("car_plan_instance_update"));
    if (m_resetFn == nullptr || m_updateFn == nullptr)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("CarPlan 动态库缺少适配导出函数。");
        }
        m_library.unload();
        m_resetFn = nullptr;
        m_updateFn = nullptr;
        return false;
    }

    m_sourcePath = sourcePath;
    m_resetFn();
    return true;
}

bool CarPlanRunner::isLoaded() const
{
    return m_updateFn != nullptr;
}

QString CarPlanRunner::sourcePath() const
{
    return m_sourcePath;
}

void CarPlanRunner::reset() const
{
    if (m_resetFn != nullptr)
    {
        m_resetFn();
    }
}

bool CarPlanRunner::update(const JustFloatLogRow& row, CarPlanResult* result) const
{
    if (result != nullptr)
    {
        *result = {};
    }
    if (m_updateFn == nullptr)
    {
        return false;
    }

    HostCameraFrame frames[3];
    std::memset(frames, 0, sizeof(frames));
    for (int camera = 0; camera < 3; ++camera)
    {
        for (int i = 0; i < 2; ++i)
        {
            const JustFloatBeacon& beacon = row.cameras[camera].beacons[i];
            frames[camera].beacons[i].valid = beacon.valid ? 1 : 0;
            frames[camera].beacons[i].x = beacon.x;
            frames[camera].beacons[i].y = beacon.y;
            frames[camera].beacons[i].area = beacon.area;
        }

        const JustFloatCarLamp& lamp = row.cameras[camera].carLamp;
        frames[camera].carLamp.valid = lamp.valid ? 1 : 0;
        frames[camera].carLamp.cx = lamp.cx;
        frames[camera].carLamp.cy = lamp.cy;
        frames[camera].carLamp.angle = lamp.angle;
        frames[camera].carLamp.width = lamp.width;
        frames[camera].carLamp.length = lamp.length;
    }

    HostResult hostResult;
    std::memset(&hostResult, 0, sizeof(hostResult));
    m_updateFn(frames, &hostResult);

    if (result != nullptr)
    {
        result->available = true;
        result->valid = hostResult.valid != 0;
        result->camera = hostResult.camera;
        result->beaconIndex = hostResult.beaconIndex;
        result->targetStrafeMps = hostResult.targetStrafeMps;
        result->targetForwardMps = hostResult.targetForwardMps;
        result->distPx = hostResult.distPx;
        result->along = hostResult.along;
        result->perp = hostResult.perp;
    }
    return true;
}
