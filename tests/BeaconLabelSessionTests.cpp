#include "BeaconLabelSession.h"

#include <QFileInfo>
#include <QTemporaryDir>
#include <QtTest>

class BeaconLabelSessionTests : public QObject
{
    Q_OBJECT

private slots:
    void defaultPathUsesVideoBaseName();
    void saveAndLoadRoundTrip();
    void rejectInconsistentFrameState();
};

void BeaconLabelSessionTests::defaultPathUsesVideoBaseName()
{
    const QString path = BeaconLabelSessionIO::defaultSessionPath(
        QStringLiteral("C:/recordings/front.raw.avi"));
    QVERIFY(path.endsWith(QStringLiteral("front.raw.beacon-label.json")));
}

void BeaconLabelSessionTests::saveAndLoadRoundTrip()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());

    BeaconLabelSession session;
    session.sessionPath = directory.filePath(QStringLiteral("front.beacon-label.json"));
    session.videoPath = directory.filePath(QStringLiteral("front.avi"));
    session.imageSize = QSize(188, 120);
    session.frameCount = 100;
    session.videoFps = 50.0;
    session.sampleStride = 5;

    BeaconFrameLabel annotated;
    annotated.state = BeaconLabelFrameState::Annotated;
    annotated.points = {QPointF(12.5, 30.25), QPointF(90.0, 60.0)};
    session.frames.insert(0, annotated);

    BeaconFrameLabel noBeacon;
    noBeacon.state = BeaconLabelFrameState::NoBeacon;
    session.frames.insert(5, noBeacon);

    BeaconFrameLabel ignored;
    ignored.state = BeaconLabelFrameState::Ignored;
    session.frames.insert(10, ignored);

    QString error;
    QVERIFY2(BeaconLabelSessionIO::save(session, &error), qPrintable(error));
    QVERIFY(QFileInfo::exists(session.sessionPath));

    BeaconLabelSession loaded;
    QVERIFY2(BeaconLabelSessionIO::load(session.sessionPath, &loaded, &error), qPrintable(error));
    QCOMPARE(loaded.sessionPath, QFileInfo(session.sessionPath).absoluteFilePath());
    QCOMPARE(loaded.videoPath, QFileInfo(session.videoPath).absoluteFilePath());
    QCOMPARE(loaded.imageSize, QSize(188, 120));
    QCOMPARE(loaded.frameCount, 100);
    QCOMPARE(loaded.videoFps, 50.0);
    QCOMPARE(loaded.sampleStride, 5);
    QCOMPARE(loaded.frames.size(), 3);
    QCOMPARE(loaded.frames.value(0).state, BeaconLabelFrameState::Annotated);
    QCOMPARE(loaded.frames.value(0).points, annotated.points);
    QCOMPARE(loaded.frames.value(5).state, BeaconLabelFrameState::NoBeacon);
    QCOMPARE(loaded.frames.value(10).state, BeaconLabelFrameState::Ignored);
}

void BeaconLabelSessionTests::rejectInconsistentFrameState()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());

    BeaconLabelSession session;
    session.sessionPath = directory.filePath(QStringLiteral("invalid.beacon-label.json"));
    session.videoPath = directory.filePath(QStringLiteral("invalid.avi"));
    session.imageSize = QSize(188, 120);
    session.frameCount = 10;
    session.videoFps = 50.0;
    session.sampleStride = 5;

    BeaconFrameLabel invalid;
    invalid.state = BeaconLabelFrameState::Annotated;
    session.frames.insert(0, invalid);

    QString error;
    QVERIFY(!BeaconLabelSessionIO::save(session, &error));
    QVERIFY(!error.isEmpty());
}

QTEST_MAIN(BeaconLabelSessionTests)
#include "BeaconLabelSessionTests.moc"
