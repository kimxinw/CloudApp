#pragma once

#include <QByteArray>
#include <QtGlobal>

enum class ImagePixelFormat : quint8
{
    Mono8
};

struct ImageFrame
{
    int cameraId = -1;
    quint64 sequence = 0;
    quint64 deviceFrameId = 0;
    quint64 deviceTimestamp = 0;

    int width = 0;
    int height = 0;
    ImagePixelFormat pixelFormat = ImagePixelFormat::Mono8;

    QByteArray pixels;
};
