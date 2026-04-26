#ifndef ALGORITHM_RUNNER_H
#define ALGORITHM_RUNNER_H

#include "beacon_image.h"

#include <QImage>

class AlgorithmRunner
{
public:
    AlgorithmRunner();
    beacon_result_t process(const QImage& grayImage) const;
    QImage binaryImage(const QImage& grayImage) const;
};

#endif
