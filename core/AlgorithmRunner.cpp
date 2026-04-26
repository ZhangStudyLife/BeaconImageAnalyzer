#include "AlgorithmRunner.h"

#include <cstring>

namespace
{
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
    beacon_image_init();
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

    beacon_image_process(image, &result);
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

    beacon_image_debug_binary(image, binary);

    QImage output(BEACON_IMAGE_W, BEACON_IMAGE_H, QImage::Format_Grayscale8);
    for (int y = 0; y < BEACON_IMAGE_H; ++y)
    {
        memcpy(output.scanLine(y), binary[y], BEACON_IMAGE_W);
    }

    return output;
}
