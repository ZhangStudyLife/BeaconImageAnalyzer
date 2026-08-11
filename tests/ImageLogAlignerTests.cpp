#include "ImageLogAligner.h"

#include <QFile>
#include <QTemporaryDir>
#include <QtTest>

namespace
{
QString logRow(int rowIndex, int imageSequence, bool varyingAttitude)
{
    QStringList fields(JustFloatLog::SingleLampRoiChannelCount, QStringLiteral("0"));
    fields[0] = QString::number(rowIndex * 10);
    for (int camera = 0; camera < 3; ++camera)
    {
        fields[1 + camera * 2] = QStringLiteral("-999");
        fields[2 + camera * 2] = QStringLiteral("-999");
    }
    const double roll = varyingAttitude ? imageSequence * 0.11 : 2.0;
    const double pitch = varyingAttitude ? imageSequence * -0.07 : -1.0;
    const double height = varyingAttitude ? 850.0 + imageSequence * 0.8 : 1000.0;
    fields[7] = QString::number(roll, 'f', 4);
    fields[8] = QString::number(pitch, 'f', 4);
    fields[9] = QString::number(height, 'f', 3);
    for (int index = 13; index <= 30; ++index)
    {
        fields[index] = (index % 3 == 0) ? QStringLiteral("0") : QStringLiteral("-999");
    }
    const quint32 stamp = 0x80U | (static_cast<quint32>(imageSequence) & 0x7fU);
    fields[33] = QString::number(stamp | (stamp << 8U) | (stamp << 16U));
    return fields.join(QLatin1Char(','));
}

bool createLog(const QString& path, bool varyingAttitude, JustFloatLog* log, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        if (error != nullptr) { *error = file.errorString(); }
        return false;
    }
    file.write(JustFloatLog::csvHeader(JustFloatLogLayout::SingleLampRoiV1).toUtf8());
    file.write("\n");
    for (int imageSequence = 0; imageSequence < 450; ++imageSequence)
    {
        for (int duplicate = 0; duplicate < 2; ++duplicate)
        {
            file.write(logRow(imageSequence * 2 + duplicate,
                              imageSequence,
                              varyingAttitude).toUtf8());
            file.write("\n");
        }
    }
    file.close();
    return JustFloatLog::loadCsv(path, log, error);
}

QVector<ImageFrameSidecarRecord> videoFrames(bool varyingAttitude)
{
    QVector<ImageFrameSidecarRecord> frames;
    for (int index = 0; index < 100; ++index)
    {
        const int logSequence = 200 + index;
        ImageFrameSidecarRecord frame;
        frame.videoFrameIndex = index;
        frame.hostTimeMs = 1700000000000LL + index * 20;
        frame.bimgSequence = 800U + index;
        // The full source sequence and I33 low seven bits describe the same frame.
        frame.sourceFrameSequence = 5064U + index;
        frame.captureTimeMs = 40000U + index * 20U;
        frame.sourceFrameValid = true;
        frame.captureTimeValid = true;
        frame.sourceCameraId = 0U;
        frame.physicalBoardId = 0U;
        frame.rollDeg = varyingAttitude ? static_cast<float>(logSequence * 0.11) : 2.0f;
        frame.pitchDeg = varyingAttitude ? static_cast<float>(logSequence * -0.07) : -1.0f;
        frame.heightMm = varyingAttitude ? static_cast<float>(850.0 + logSequence * 0.8)
                                         : 1000.0f;
        frame.attitudeValid = true;
        frame.heightValid = true;
        frames.push_back(frame);
    }
    return frames;
}
}

class ImageLogAlignerTests : public QObject
{
    Q_OBJECT

private slots:
    void alignsFullSequenceAcrossSevenBitWrap();
    void marksStaticSignatureAsLowConfidence();
    void appliesManual128FrameShift();
};

void ImageLogAlignerTests::alignsFullSequenceAcrossSevenBitWrap()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    JustFloatLog log;
    QString error;
    QVERIFY2(createLog(directory.filePath(QStringLiteral("varying.csv")), true, &log, &error),
             qPrintable(error));

    const ImageLogAlignmentResult result = ImageLogAligner::align(videoFrames(true), log);
    QCOMPARE(result.sourceCameraId, 0);
    QCOMPARE(result.sequenceOffset, qint64(4864));
    QCOMPARE(result.matchedVideoFrameCount, 100);
    QCOMPARE(result.coverage, 1.0);
    QCOMPARE(result.confidence, ImageLogAlignmentConfidence::High);
    QCOMPARE(result.videoToLogRow[0] / 2, 200);
    QCOMPARE(result.videoToLogRow[99] / 2, 299);
}

void ImageLogAlignerTests::marksStaticSignatureAsLowConfidence()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    JustFloatLog log;
    QString error;
    QVERIFY2(createLog(directory.filePath(QStringLiteral("static.csv")), false, &log, &error),
             qPrintable(error));

    const ImageLogAlignmentResult result = ImageLogAligner::align(videoFrames(false), log);
    QCOMPARE(result.coverage, 1.0);
    QVERIFY(result.candidateCount > 1);
    QCOMPARE(result.confidence, ImageLogAlignmentConfidence::Low);
}

void ImageLogAlignerTests::appliesManual128FrameShift()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    JustFloatLog log;
    QString error;
    QVERIFY2(createLog(directory.filePath(QStringLiteral("manual.csv")), true, &log, &error),
             qPrintable(error));

    const ImageLogAlignmentResult automatic = ImageLogAligner::align(videoFrames(true), log);
    const ImageLogAlignmentResult shifted = ImageLogAligner::align(videoFrames(true), log, 1);
    QCOMPARE(shifted.sequenceOffset, automatic.automaticSequenceOffset + 128);
    QCOMPARE(shifted.matchedVideoFrameCount, 100);
    QCOMPARE(shifted.videoToLogRow[0] / 2, 72);
}

QTEST_MAIN(ImageLogAlignerTests)

#include "ImageLogAlignerTests.moc"
