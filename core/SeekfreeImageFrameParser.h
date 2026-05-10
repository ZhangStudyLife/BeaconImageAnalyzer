#ifndef SEEKFREE_IMAGE_FRAME_PARSER_H
#define SEEKFREE_IMAGE_FRAME_PARSER_H

#include <QByteArray>
#include <QImage>
#include <QVector>

struct SeekfreeImageFrame
{
    QImage image;
    quint8 cameraType = 0;
    quint16 width = 0;
    quint16 height = 0;
};

class SeekfreeImageFrameParser
{
public:
    QVector<SeekfreeImageFrame> append(const QByteArray& data);
    void clear();

private:
    static qsizetype imagePayloadSize(quint8 cameraType, quint16 width, quint16 height);
    static QImage grayImageFromPayload(const QByteArray& payload, quint16 width, quint16 height);

    QByteArray m_buffer;
};

#endif
