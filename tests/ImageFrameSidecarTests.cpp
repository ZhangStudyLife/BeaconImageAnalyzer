#include "ImageFrameSidecar.h"

#include <QFile>
#include <QTemporaryDir>
#include <QtTest>

namespace
{
bool writeUtf8File(const QString& path, const QString& text)
{
    QFile file(path);
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate) &&
           file.write(text.toUtf8()) == text.toUtf8().size();
}

QString validRow(quint64 videoFrameIndex)
{
    return QStringLiteral("%1,1723456789012,100,200,300,1,1,0,0,12.5,-3.25,1024,1,1")
        .arg(videoFrameIndex);
}
}

class ImageFrameSidecarTests : public QObject
{
    Q_OBJECT

private slots:
    void roundTripsRecords();
    void rejectsBadHeader();
    void rejectsNonContinuousIndices();
    void rejectsOutOfRangeValues();
    void rejectsInvalidSourceBoardMapping();
};

void ImageFrameSidecarTests::roundTripsRecords()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString videoPath = directory.filePath(QString::fromUtf8("同步录像.avi"));
    QCOMPARE(imageFrameSidecarPathForVideo(videoPath),
             videoPath + QStringLiteral(".frames.csv"));

    ImageFrameSidecarRecord first;
    first.videoFrameIndex = 0;
    first.hostTimeMs = 1723456789012LL;
    first.bimgSequence = 0xfedcba98U;
    first.sourceFrameSequence = 0x89abcdefU;
    first.captureTimeMs = 0x76543210U;
    first.sourceFrameValid = true;
    first.captureTimeValid = true;
    first.sourceCameraId = 0U;
    first.physicalBoardId = 0U;
    first.rollDeg = 12.5f;
    first.pitchDeg = -3.25f;
    first.heightMm = 1024.5f;
    first.attitudeValid = true;
    first.heightValid = true;

    ImageFrameSidecarRecord second = first;
    second.videoFrameIndex = 1;
    second.hostTimeMs += 20;
    second.bimgSequence += 1U;
    second.sourceFrameSequence += 1U;
    second.captureTimeMs += 20U;
    second.sourceCameraId = 2U;
    second.physicalBoardId = 1U;
    second.sourceFrameValid = false;
    second.captureTimeValid = false;
    second.attitudeValid = false;
    second.heightValid = false;

    ImageFrameSidecarWriter writer;
    QString error;
    QVERIFY2(writer.start(videoPath, &error), qPrintable(error));
    QVERIFY(writer.isActive());
    QCOMPARE(writer.sidecarPath(), imageFrameSidecarPathForVideo(videoPath));
    QVERIFY2(writer.append(first, &error), qPrintable(error));
    QVERIFY2(writer.append(second, &error), qPrintable(error));
    QCOMPARE(writer.rowCount(), quint64(2));
    QVERIFY2(writer.finish(&error), qPrintable(error));
    QVERIFY(!writer.isActive());

    QVector<ImageFrameSidecarRecord> loaded;
    QVERIFY2(loadImageFrameSidecar(writer.sidecarPath(), &loaded, &error), qPrintable(error));
    QCOMPARE(loaded.size(), 2);
    QCOMPARE(loaded[0].videoFrameIndex, first.videoFrameIndex);
    QCOMPARE(loaded[0].hostTimeMs, first.hostTimeMs);
    QCOMPARE(loaded[0].bimgSequence, first.bimgSequence);
    QCOMPARE(loaded[0].sourceFrameSequence, first.sourceFrameSequence);
    QCOMPARE(loaded[0].captureTimeMs, first.captureTimeMs);
    QCOMPARE(loaded[0].sourceFrameValid, first.sourceFrameValid);
    QCOMPARE(loaded[0].captureTimeValid, first.captureTimeValid);
    QCOMPARE(loaded[0].sourceCameraId, first.sourceCameraId);
    QCOMPARE(loaded[0].physicalBoardId, first.physicalBoardId);
    QCOMPARE(loaded[0].rollDeg, first.rollDeg);
    QCOMPARE(loaded[0].pitchDeg, first.pitchDeg);
    QCOMPARE(loaded[0].heightMm, first.heightMm);
    QCOMPARE(loaded[0].attitudeValid, first.attitudeValid);
    QCOMPARE(loaded[0].heightValid, first.heightValid);
    QCOMPARE(loaded[1].videoFrameIndex, second.videoFrameIndex);
    QCOMPARE(loaded[1].sourceCameraId, second.sourceCameraId);
    QCOMPARE(loaded[1].physicalBoardId, second.physicalBoardId);
    QCOMPARE(loaded[1].sourceFrameValid, second.sourceFrameValid);
    QCOMPARE(loaded[1].captureTimeValid, second.captureTimeValid);
}

void ImageFrameSidecarTests::rejectsBadHeader()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("bad-header.frames.csv"));
    QString header = imageFrameSidecarCsvHeader();
    header.replace(QStringLiteral("bimg_sequence"), QStringLiteral("stream_sequence"));
    QVERIFY(writeUtf8File(path, header + QLatin1Char('\n') + validRow(0) + QLatin1Char('\n')));

    QVector<ImageFrameSidecarRecord> output{ImageFrameSidecarRecord{}};
    QString error;
    QVERIFY(!loadImageFrameSidecar(path, &output, &error));
    QVERIFY2(error.contains(QStringLiteral("header mismatch")), qPrintable(error));
    QCOMPARE(output.size(), 1);
}

void ImageFrameSidecarTests::rejectsNonContinuousIndices()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("non-continuous.frames.csv"));

    QString text = imageFrameSidecarCsvHeader() + QLatin1Char('\n') +
                   validRow(0) + QLatin1Char('\n') +
                   validRow(2) + QLatin1Char('\n');
    QVERIFY(writeUtf8File(path, text));
    QVector<ImageFrameSidecarRecord> output;
    QString error;
    QVERIFY(!loadImageFrameSidecar(path, &output, &error));
    QVERIFY2(error.contains(QStringLiteral("missing video_frame_index 1")), qPrintable(error));

    text = imageFrameSidecarCsvHeader() + QLatin1Char('\n') +
           validRow(0) + QLatin1Char('\n') +
           validRow(0) + QLatin1Char('\n');
    QVERIFY(writeUtf8File(path, text));
    error.clear();
    QVERIFY(!loadImageFrameSidecar(path, &output, &error));
    QVERIFY2(error.contains(QStringLiteral("duplicate video_frame_index 0")), qPrintable(error));
}

void ImageFrameSidecarTests::rejectsOutOfRangeValues()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("out-of-range.frames.csv"));
    const QString row = QStringLiteral(
        "0,1723456789012,100,200,300,1,1,0,0,181,-3.25,1024,1,1");
    QVERIFY(writeUtf8File(path,
                         imageFrameSidecarCsvHeader() + QLatin1Char('\n') +
                             row + QLatin1Char('\n')));

    QVector<ImageFrameSidecarRecord> output;
    QString error;
    QVERIFY(!loadImageFrameSidecar(path, &output, &error));
    QVERIFY2(error.contains(QStringLiteral("roll_deg")), qPrintable(error));
}

void ImageFrameSidecarTests::rejectsInvalidSourceBoardMapping()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("bad-source.frames.csv"));
    const QString centerCamera = QStringLiteral(
        "0,1723456789012,100,200,300,1,1,1,0,12.5,-3.25,1024,1,1");
    QVERIFY(writeUtf8File(path,
                         imageFrameSidecarCsvHeader() + QLatin1Char('\n') +
                             centerCamera + QLatin1Char('\n')));

    QVector<ImageFrameSidecarRecord> output;
    QString error;
    QVERIFY(!loadImageFrameSidecar(path, &output, &error));
    QVERIFY2(error.contains(QStringLiteral("source_camera_id")), qPrintable(error));

    const QString wrongBoard = QStringLiteral(
        "0,1723456789012,100,200,300,1,1,2,0,12.5,-3.25,1024,1,1");
    QVERIFY(writeUtf8File(path,
                         imageFrameSidecarCsvHeader() + QLatin1Char('\n') +
                             wrongBoard + QLatin1Char('\n')));
    error.clear();
    QVERIFY(!loadImageFrameSidecar(path, &output, &error));
    QVERIFY2(error.contains(QStringLiteral("map camera")), qPrintable(error));
}

QTEST_MAIN(ImageFrameSidecarTests)

#include "ImageFrameSidecarTests.moc"
