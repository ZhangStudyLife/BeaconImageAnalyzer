#ifndef VIDEO_READER_H
#define VIDEO_READER_H

#include <QImage>
#include <QByteArray>
#include <QString>
#include <QVector>

#include <opencv2/videoio.hpp>

struct VideoFrameChunk
{
    qint64 dataOffset = 0;
    quint32 size = 0;
};

class VideoReader
{
public:
    bool open(const QString& path, QString* errorMessage = nullptr);
    bool isOpen() const;
    bool readFrame(int frameIndex, QImage* grayImage, QString* errorMessage = nullptr) const;

    QString filePath() const;
    QString codecName() const;
    int width() const;
    int height() const;
    int frameCount() const;
    double videoFps() const;
    int bitCount() const;
    QString backendName() const;

private:
    enum class BackendMode
    {
        None,
        OpenCv,
        DibAvi
    };

    static QImage matToGrayImage(const cv::Mat& frame);
    static QString fourccToString(int fourcc);
    static quint16 readU16(const QByteArray& data, qsizetype offset);
    static quint32 readU32(const QByteArray& data, qsizetype offset);
    static qint32 readI32(const QByteArray& data, qsizetype offset);
    static bool isUncompressedDibAvi(const QString& path);

    bool openOpenCv(const QString& path, const QVector<int>& backends, QString* lastError);
    bool openDibAvi(const QString& path, QString* errorMessage);
    bool readDibFrame(int frameIndex, QImage* grayImage, QString* errorMessage) const;

    QString m_filePath;
    QString m_codecName;
    QString m_backendName;
    BackendMode m_backendMode = BackendMode::None;
    int m_width = 0;
    int m_height = 0;
    int m_frameCount = 0;
    double m_videoFps = 0.0;
    int m_bitCount = 0;
    mutable cv::VideoCapture m_capture;

    int m_sourceStride = 0;
    bool m_bottomUp = true;
    QVector<VideoFrameChunk> m_dibChunks;
};

#endif
