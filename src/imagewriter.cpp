#include "imagewriter.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMutexLocker>

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

ImageWriter::ImageWriter(QString camera1Directory,
                         QString camera2Directory,
                         qsizetype maxFrames,
                         qsizetype maxQueuedBytes,
                         int workerCount,
                         QObject *parent)
    : QObject(parent),
      m_maxFrames(maxFrames),
      m_maxQueuedBytes(maxQueuedBytes)
{
    if (m_maxFrames <= 0 || m_maxQueuedBytes <= 0) {
        throw std::invalid_argument("图像保存队列容量必须大于零");
    }

    m_cameraDirectories[0] = std::move(camera1Directory);
    m_cameraDirectories[1] = std::move(camera2Directory);

    if (!QDir().mkpath(m_cameraDirectories[0])
        || !QDir().mkpath(m_cameraDirectories[1])) {
        throw std::runtime_error("无法创建图像保存目录");
    }

    workerCount = std::max(1, workerCount);
    m_runningWorkers = workerCount;
    m_workers.reserve(static_cast<size_t>(workerCount));

    for (int i = 0; i < workerCount; ++i) {
        m_workers.emplace_back([this]() {
            workerLoop();
        });
    }
}

ImageWriter::~ImageWriter()
{
    seal();
    wait();
}

ImageWriter::EnqueueResult ImageWriter::enqueue(ImageFrame frame)
{
    bool notifyOverflow = false;

    {
        QMutexLocker locker(&m_mutex);

        if (!m_accepting) {
            return EnqueueResult::Closed;
        }

        const qsizetype frameBytes = frame.pixels.size();
        if (static_cast<qsizetype>(m_queue.size()) >= m_maxFrames
            || frameBytes > m_maxQueuedBytes - m_queuedBytes) {
            ++m_droppedFrames;
            if (!m_overflowSignaled) {
                m_overflowSignaled = true;
                notifyOverflow = true;
            }
        } else {
            m_queuedBytes += frameBytes;
            m_queue.emplace_back(std::move(frame));
            ++m_acceptedFrames;
            m_notEmpty.wakeOne();
            return EnqueueResult::Accepted;
        }
    }

    if (notifyOverflow) {
        emit queueOverflow();
    }

    return EnqueueResult::QueueFull;
}

void ImageWriter::seal()
{
    QMutexLocker locker(&m_mutex);
    if (!m_accepting) {
        return;
    }

    m_accepting = false;
    m_notEmpty.wakeAll();
}

void ImageWriter::wait()
{
    for (std::thread &worker : m_workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    m_workers.clear();
}

ImageWriter::Stats ImageWriter::stats() const
{
    QMutexLocker locker(&m_mutex);

    Stats result;
    result.acceptedFrames = m_acceptedFrames;
    result.savedFrames = m_savedFrames;
    result.failedFrames = m_failedFrames;
    result.droppedFrames = m_droppedFrames;
    result.pendingFrames = static_cast<qsizetype>(m_queue.size()) + m_activeWrites;
    return result;
}

bool ImageWriter::isFinished() const
{
    QMutexLocker locker(&m_mutex);
    return m_finished;
}

void ImageWriter::workerLoop()
{
    for (;;) {
        ImageFrame frame;

        {
            QMutexLocker locker(&m_mutex);
            while (m_queue.empty() && m_accepting) {
                m_notEmpty.wait(&m_mutex);
            }

            if (m_queue.empty() && !m_accepting) {
                break;
            }

            frame = std::move(m_queue.front());
            m_queue.pop_front();
            m_queuedBytes -= frame.pixels.size();
            ++m_activeWrites;
        }

        const bool saved = saveFrame(frame);

        {
            QMutexLocker locker(&m_mutex);
            --m_activeWrites;
            if (saved) {
                ++m_savedFrames;
            } else {
                ++m_failedFrames;
            }
        }
    }

    bool notifyFinished = false;
    {
        QMutexLocker locker(&m_mutex);
        --m_runningWorkers;
        if (m_runningWorkers == 0) {
            m_finished = true;
            notifyFinished = true;
        }
    }

    if (notifyFinished) {
        emit allImagesSaved();
    }
}

bool ImageWriter::saveFrame(const ImageFrame &frame) const
{
    if (frame.cameraId < 0 || frame.cameraId > 1
        || frame.pixelFormat != ImagePixelFormat::Mono8
        || frame.width <= 0 || frame.height <= 0) {
        return false;
    }

    const qsizetype requiredBytes = static_cast<qsizetype>(frame.width) * frame.height;
    if (requiredBytes <= 0 || frame.pixels.size() < requiredBytes) {
        return false;
    }

    const QString fileName = QStringLiteral("%1.png")
                                 .arg(frame.sequence, 4, 10, QLatin1Char('0'));
    const QString fullPath = QDir(m_cameraDirectories[frame.cameraId]).filePath(fileName);
    if (QFileInfo::exists(fullPath)) {
        return false;
    }

    cv::Mat image(frame.height,
                  frame.width,
                  CV_8UC1,
                  const_cast<char *>(frame.pixels.constData()));

    try {
        const QByteArray nativePath = QFile::encodeName(fullPath);
        return cv::imwrite(nativePath.constData(),
                           image,
                           {cv::IMWRITE_PNG_COMPRESSION, 1});
    } catch (const cv::Exception &) {
        return false;
    } catch (...) {
        return false;
    }
}
