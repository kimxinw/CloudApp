#pragma once

#include "imageframe.h"

#include <QMutex>
#include <QObject>
#include <QString>
#include <QWaitCondition>

#include <deque>
#include <thread>
#include <vector>

class ImageWriter final : public QObject
{
    Q_OBJECT

public:
    enum class EnqueueResult
    {
        Accepted,
        QueueFull,
        Closed
    };

    struct Stats
    {
        quint64 acceptedFrames = 0;
        quint64 savedFrames = 0;
        quint64 failedFrames = 0;
        quint64 droppedFrames = 0;
        qsizetype pendingFrames = 0;
    };

    explicit ImageWriter(QString camera1Directory,
                         QString camera2Directory,
                         qsizetype maxFrames = 64,
                         qsizetype maxQueuedBytes = 256 * 1024 * 1024,
                         int workerCount = 2,
                         QObject *parent = nullptr);
    ~ImageWriter() override;

    ImageWriter(const ImageWriter &) = delete;
    ImageWriter &operator=(const ImageWriter &) = delete;

    EnqueueResult enqueue(ImageFrame frame);
    void seal();
    void wait();

    Stats stats() const;
    bool isFinished() const;

signals:
    void queueOverflow();
    void allImagesSaved();

private:
    void workerLoop();
    bool saveFrame(const ImageFrame &frame) const;

    QString m_cameraDirectories[2];
    const qsizetype m_maxFrames;
    const qsizetype m_maxQueuedBytes;

    mutable QMutex m_mutex;
    QWaitCondition m_notEmpty;
    std::deque<ImageFrame> m_queue;
    std::vector<std::thread> m_workers;

    qsizetype m_queuedBytes = 0;
    qsizetype m_activeWrites = 0;
    bool m_accepting = true;
    bool m_finished = false;
    bool m_overflowSignaled = false;
    int m_runningWorkers = 0;

    quint64 m_acceptedFrames = 0;
    quint64 m_savedFrames = 0;
    quint64 m_failedFrames = 0;
    quint64 m_droppedFrames = 0;
};
