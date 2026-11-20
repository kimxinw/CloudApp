#include "opencvset.h"
#include "ui_opencvset.h"
#include <QMessageBox>
#include <QDir>
#include <QScrollBar>
#include <QtConcurrent>
#include <functional>
#include <map>

#include <filesystem>
#include <QThread>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <chrono>
#include <QDebug>
#include <QSettings>
#include <QPointer>
#include <QScopeGuard>
#include <QLineF>
#include <random>
#include <delectseam.h>
#include <cstring>
#include <limits>

bool camera_is_Init=false;
bool camera_is_open=false;
bool laser_is_open = false;
bool alxe_is_connect = false;

bool robot_is_connect = false;
bool robot_is_Init = false;
int connectID = 0;//新松机械臂控制程序需要使用

GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;
CGXDevicePointer objDevicePtr;
CGXStreamPointer objStreamPtr;
CGXDevicePointer objDevicePtr1;
CGXStreamPointer objStreamPtr1;

CImageDataPointer image;
CGXFeatureControlPointer featureControl;
CImageDataPointer image1;
CGXFeatureControlPointer featureControl1;


int picNumber=0;
int nolaser = 0;
int laser = 0;
int plat = 0;
int work = 0;
int callCount = 0;
int picNumber1=0;
int nolaser1 = 0;
int laser1 = 0;
int plat1 = 0;
int work1 = 0;
int callCount1 = 0;

int handtoeyesnumber=0;

double x_b=0;//手眼标定时小车位置
double y_b=0;//手眼标定时大车位置


double h = 1960.0;

cv::Size board_size(11, 8);
float square_size = 20.0f;

double step_count = 10;
long sudu=2;
long time_jiange = 200;

Eigen::Vector3f axis(0.0f,0.0f,0.1f);
float angle=0;
float z0=0;
float alxepositionstart=0;

OpenCVSet::OpenCVSet(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::OpenCVSet),
    alxeSocket(nullptr),
    alxeTimer(nullptr),
    positionTimer(nullptr),
    workTimer(nullptr),
    calibrator(nullptr),
    weldSearch(nullptr),
    wedlprocessor(nullptr),
    m_pCaptureHandler(nullptr),
    m_pCaptureHandler1(nullptr),
    m_bCallbackMode(false),
    m_bGrabbing(false)
{
    m_frameSequences[0].store(0);
    m_frameSequences[1].store(0);
    ui->setupUi(this);
    calibrator = new Cloud_selve();
    weldSearch = new Weld_search();
    wedlprocessor = new WeldSeamExtractor();

    // 初始化 arc_clouds
    m_arc_clouds = std::make_unique<std::vector<pcl::PointCloud<pcl::PointXYZ>>>();
    // 1. 创建渲染器和渲染窗口
    // 必须使用 vtkGenericOpenGLRenderWindow，它是 QVTKOpenGLNativeWidget 所必需的[citation:3][citation:9]。
    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    renderWindow->AddRenderer(renderer);

    // 2. 创建 PCLVisualizer，并将其与上面创建的渲染器和窗口绑定
    // 注意构造函数最后一个参数设置为 'false'，表示不立即显示，等待嵌入[citation:1][citation:3]。
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));

    // 3. 将 PCLVisualizer 的渲染窗口设置到 Qt 控件上
    // 这里的 "qvtkWidget" 是你在 UI 中为 QVTKOpenGLNativeWidget 设置的 objectName。
    ui->widget->setRenderWindow(viewer->getRenderWindow());

    // 4. 设置交互器，使鼠标操作（旋转、缩放）能正常工作
    viewer->setupInteractor(ui->widget->interactor(), ui->widget->renderWindow());

    // 5. 可选：设置一些 PCL 可视化器的初始属性
    viewer->setBackgroundColor(0.1, 0.1, 0.1); // 设置背景色为深灰色



    alxeTimer= new QTimer(this);
    alxeTimer->setInterval(1000);
    connect(alxeTimer, &QTimer::timeout, this, &OpenCVSet::updateConnectionStatus);

    positionTimer=new QTimer(this);
    positionTimer->setInterval(1000);
    connect(positionTimer, &QTimer::timeout, this, &OpenCVSet::updatePosition);

    alxeTimer->start();

    workTimer=new QTimer(this);
    workTimer->setInterval(time_jiange);
    connect(workTimer,&QTimer::timeout,this,&OpenCVSet::takePhoto);

    OpenCVSet::loadSettings();

}

OpenCVSet::~OpenCVSet()
{
    m_isDestorying = true;
    if (workTimer) {
        workTimer->stop();
    }
    StopCallbackCapture();
    if (m_imageWriter) {
        m_imageWriter->seal();
        m_imageWriter->wait();
        m_imageWriter.reset();
    }

    delete calibrator;
    delete weldSearch;
    delete wedlprocessor;
    if(robot_is_connect)
    {
        Disconnect(connectID);
    }

    if(robot_is_Init)
    {
        RobotRelease();
    }

    if (workTimer) {
        workTimer->stop();
        workTimer->disconnect();
    }
    if (alxeTimer) {
        alxeTimer->stop();
        alxeTimer->disconnect();
    }

    if (positionTimer) {
        positionTimer->stop();
        positionTimer->disconnect();
    }


    if (viewer)
    {
        viewer->close(); // 关闭 PCL 可视化器
    }
    if (ui->widget->interactor())
    {
        ui->widget->interactor()->SetRenderWindow(nullptr);
    }



    QCoreApplication::removePostedEvents(this);

    if (!objStreamPtr.IsNull()) {
        try { objStreamPtr->StopGrab(); } catch(...) {}
    }

    if (!objDevicePtr.IsNull()) {
        try {
            objDevicePtr->GetRemoteFeatureControl()->GetCommandFeature("AcquisitionStop")->Execute();
            objDevicePtr->Close();
        } catch(...) {}
    }

    if (!objStreamPtr1.IsNull()) {
        try { objStreamPtr1->StopGrab(); } catch(...) {}
    }

    if (!objDevicePtr1.IsNull()) {
        try {
            objDevicePtr1->GetRemoteFeatureControl()->GetCommandFeature("AcquisitionStop")->Execute();
            objDevicePtr1->Close();
        } catch(...) {}
    }

    if(laser_is_open) {
        on_pushButton_14_clicked();
    }

    if (alxeSocket) {
        alxeSocket->disconnectFromHost();
        alxeSocket->waitForDisconnected(1000);
        delete alxeSocket;
    }

    delete alxeTimer;
    delete positionTimer;
    delete workTimer;

    if(camera_is_Init) {
        IGXFactory::GetInstance().Uninit();
    }
    saveSettings();
    delete ui;

}


// 回调处理类的实现
void CSampleCaptureEventHandler::DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam)
{
    if (m_pMainWindow) {
        m_pMainWindow->OnImageCaptured(objImageDataPointer, m_cameraID);
    }
}

bool OpenCVSet::InitCallbackCapture()
{
    try {
        // 确保相机设备已经打开
        if (objDevicePtr.IsNull() || objDevicePtr1.IsNull()) {
            addlog("相机设备未打开");
            return false;
        }

        // ========== 相机1：停止现有采集 ==========
        if (!objStreamPtr.IsNull()) {
            try {
                if (!featureControl.IsNull()) {
                    CCommandFeaturePointer pCmd = featureControl->GetCommandFeature("AcquisitionStop");
                    if (!pCmd.IsNull()) {
                        pCmd->Execute();
                    }
                }
                objStreamPtr->StopGrab();
                objStreamPtr->UnregisterCaptureCallback();
            } catch (...) {}
        }

        // 设置最优包长
        try {
            GX_DEVICE_CLASS_LIST objDeviceClass = objDevicePtr->GetDeviceInfo().GetDeviceClass();
            if (GX_DEVICE_CLASS_GEV == objDeviceClass)
            {
                CGXFeatureControlPointer objFeatureControlPtr = objDevicePtr->GetRemoteFeatureControl();
                if (!objFeatureControlPtr.IsNull() && objFeatureControlPtr->IsImplemented("GevSCPSPacketSize"))
                {
                    int nPacketSize = objStreamPtr->GetOptimalPacketSize();
                    if (nPacketSize > 0) {
                        objFeatureControlPtr->GetIntFeature("GevSCPSPacketSize")->SetValue(nPacketSize);
                        addlog(QString("相机1设置最优包长: %1").arg(nPacketSize));
                    }
                }
            }
        } catch (...) {
            addlog("相机1设置最优包长失败，使用默认值");
        }

        // 注册回调
        if (m_pCaptureHandler) {
            delete m_pCaptureHandler;
            m_pCaptureHandler = nullptr;
        }
        m_pCaptureHandler = new CSampleCaptureEventHandler(this, 0);
        objStreamPtr->RegisterCaptureCallback(m_pCaptureHandler, NULL);
        objStreamPtr->StartGrab();

        // ========== 相机2：同样处理 ==========
        if (!objStreamPtr1.IsNull()) {
            try {
                if (!featureControl1.IsNull()) {
                    CCommandFeaturePointer pCmd = featureControl1->GetCommandFeature("AcquisitionStop");
                    if (!pCmd.IsNull()) {
                        pCmd->Execute();
                    }
                }
                objStreamPtr1->StopGrab();
                objStreamPtr1->UnregisterCaptureCallback();
            } catch (...) {}
        }

        try {
            GX_DEVICE_CLASS_LIST objDeviceClass = objDevicePtr1->GetDeviceInfo().GetDeviceClass();
            if (GX_DEVICE_CLASS_GEV == objDeviceClass)
            {
                CGXFeatureControlPointer objFeatureControlPtr1 = objDevicePtr1->GetRemoteFeatureControl();
                if (!objFeatureControlPtr1.IsNull() && objFeatureControlPtr1->IsImplemented("GevSCPSPacketSize"))
                {
                    int nPacketSize = objStreamPtr1->GetOptimalPacketSize();
                    if (nPacketSize > 0) {
                        objFeatureControlPtr1->GetIntFeature("GevSCPSPacketSize")->SetValue(nPacketSize);
                        addlog(QString("相机2设置最优包长: %1").arg(nPacketSize));
                    }
                }
            }
        } catch (...) {
            addlog("相机2设置最优包长失败，使用默认值");
        }

        if (m_pCaptureHandler1) {
            delete m_pCaptureHandler1;
            m_pCaptureHandler1 = nullptr;
        }
        m_pCaptureHandler1 = new CSampleCaptureEventHandler(this, 1);
        objStreamPtr1->RegisterCaptureCallback(m_pCaptureHandler1, NULL);
        objStreamPtr1->StartGrab();

        addlog("回调采集初始化成功");
        return true;

    } catch (const std::exception& e) {
        addlog(QString("初始化回调采集失败: %1").arg(e.what()));
        return false;
    } catch (...) {
        addlog("初始化回调采集失败: 未知错误");
        return false;
    }
}


// 启动回调采集
void OpenCVSet::StartCallbackCapture()
{
    if (m_bGrabbing) return;

    try {
        // 发送开采命令 - 使用你的全局变量 featureControl 和 featureControl1
        if (!featureControl.IsNull()) {
            CCommandFeaturePointer pCmd = featureControl->GetCommandFeature("AcquisitionStart");
            if (!pCmd.IsNull()) {
                pCmd->Execute();
                addlog("相机1开采命令已发送");
            } else {
                addlog("相机1获取AcquisitionStart命令失败");
            }
        } else {
            addlog("相机1功能控制对象为空");
        }

        if (!featureControl1.IsNull()) {
            CCommandFeaturePointer pCmd = featureControl1->GetCommandFeature("AcquisitionStart");
            if (!pCmd.IsNull()) {
                pCmd->Execute();
                addlog("相机2开采命令已发送");
            } else {
                addlog("相机2获取AcquisitionStart命令失败");
            }
        } else {
            addlog("相机2功能控制对象为空");
        }

        m_bGrabbing = true;
        m_bCallbackMode = true;
        m_captureTimer.start();

        addlog("回调采集已启动");

    } catch (const std::exception& e) {
        addlog(QString("启动回调采集失败: %1").arg(e.what()));
    } catch (...) {
        addlog("启动回调采集失败: 未知错误");
    }
}

// 停止回调采集
void OpenCVSet::StopCallbackCapture()
{
    m_acceptFrames.store(false, std::memory_order_release);

    if (m_bGrabbing || m_pCaptureHandler || m_pCaptureHandler1) {
        try {
            if (!featureControl.IsNull()) {
                CCommandFeaturePointer command = featureControl->GetCommandFeature("AcquisitionStop");
                if (!command.IsNull()) {
                    command->Execute();
                }
            }
        } catch (...) {
            addlog("停止相机1采集失败");
        }

        try {
            if (!featureControl1.IsNull()) {
                CCommandFeaturePointer command = featureControl1->GetCommandFeature("AcquisitionStop");
                if (!command.IsNull()) {
                    command->Execute();
                }
            }
        } catch (...) {
            addlog("停止相机2采集失败");
        }
    }

    if (!objStreamPtr.IsNull() && m_pCaptureHandler) {
        try {
            objStreamPtr->StopGrab();
        } catch (...) {
            addlog("停止相机1数据流失败");
        }
        try {
            objStreamPtr->UnregisterCaptureCallback();
        } catch (...) {
            addlog("注销相机1回调失败");
        }
    }

    if (!objStreamPtr1.IsNull() && m_pCaptureHandler1) {
        try {
            objStreamPtr1->StopGrab();
        } catch (...) {
            addlog("停止相机2数据流失败");
        }
        try {
            objStreamPtr1->UnregisterCaptureCallback();
        } catch (...) {
            addlog("注销相机2回调失败");
        }
    }

    {
        QMutexLocker locker(&m_callbackWaitMutex);
        while (m_callbacksInFlight.load(std::memory_order_acquire) != 0) {
            m_callbacksFinished.wait(&m_callbackWaitMutex);
        }
    }

    delete m_pCaptureHandler;
    m_pCaptureHandler = nullptr;
    delete m_pCaptureHandler1;
    m_pCaptureHandler1 = nullptr;

    const bool wasCapturing = m_bGrabbing || m_bCallbackMode;
    m_bGrabbing = false;
    m_bCallbackMode = false;

    if (m_imageWriter) {
        m_imageWriter->seal();
    }

    if (wasCapturing) {
        const quint64 frameCount1 = m_frameSequences[0].load();
        const quint64 frameCount2 = m_frameSequences[1].load();
        addlog("回调采集已停止，正在排空图像保存队列");
        qDebug() << "采集时长:" << m_captureTimer.elapsed() << "ms";
        qDebug() << "相机1采集帧数:" << frameCount1;
        qDebug() << "相机2采集帧数:" << frameCount2;
    }
}

// 图像回调处理
void OpenCVSet::OnImageCaptured(CImageDataPointer& objImageDataPointer, int cameraID)
{
    m_callbacksInFlight.fetch_add(1, std::memory_order_acq_rel);
    const auto callbackGuard = qScopeGuard([this]() {
        const int previous = m_callbacksInFlight.fetch_sub(1, std::memory_order_acq_rel);
        if (previous == 1) {
            QMutexLocker locker(&m_callbackWaitMutex);
            m_callbacksFinished.wakeAll();
        }
    });

    if (!m_acceptFrames.load(std::memory_order_acquire)) {
        return;
    }

    const quint64 callbackGeneration = m_captureGeneration.load(std::memory_order_acquire);
    auto requestStop = [this, callbackGeneration](const QString &reason) {
        if (!m_capturePipelineFailed.exchange(true)) {
            QMetaObject::invokeMethod(this,
                                      "handleCapturePipelineFailure",
                                      Qt::QueuedConnection,
                                      Q_ARG(quint64, callbackGeneration),
                                      Q_ARG(QString, reason));
        }
    };

    try {
        if (objImageDataPointer.IsNull() || cameraID < 0 || cameraID > 1) {
            requestStop("收到无效的相机回调数据，已停止本次扫描");
            return;
        }

        if (objImageDataPointer->GetStatus() != GX_FRAME_STATUS_SUCCESS) {
            requestStop("相机返回不完整图像，已停止本次扫描");
            return;
        }

        if (objImageDataPointer->GetPixelFormat() != GX_PIXEL_FORMAT_MONO8) {
            requestStop("收到非 Mono8 图像，已停止本次扫描");
            return;
        }

        const uint64_t width = objImageDataPointer->GetWidth();
        const uint64_t height = objImageDataPointer->GetHeight();
        const uint64_t payloadSize = objImageDataPointer->GetPayloadSize();
        if (width == 0 || height == 0
            || width > static_cast<uint64_t>(std::numeric_limits<int>::max())
            || height > static_cast<uint64_t>(std::numeric_limits<int>::max())
            || width > std::numeric_limits<uint64_t>::max() / height
            || payloadSize < width * height
            || payloadSize > static_cast<uint64_t>(std::numeric_limits<qsizetype>::max())
            || objImageDataPointer->GetBuffer() == nullptr) {
            requestStop("收到尺寸或缓冲区长度无效的图像，已停止本次扫描");
            return;
        }

        ImageFrame frame;
        frame.cameraId = cameraID;
        frame.sequence = m_frameSequences[cameraID].fetch_add(1) + 1;
        frame.deviceFrameId = objImageDataPointer->GetFrameID();
        frame.deviceTimestamp = objImageDataPointer->GetTimeStamp();
        frame.width = static_cast<int>(width);
        frame.height = static_cast<int>(height);
        frame.pixelFormat = ImagePixelFormat::Mono8;
        frame.pixels.resize(static_cast<qsizetype>(payloadSize));
        std::memcpy(frame.pixels.data(), objImageDataPointer->GetBuffer(),
                    static_cast<size_t>(payloadSize));

        if (!m_imageWriter) {
            requestStop("图像保存器不存在，已停止本次扫描");
            return;
        }

        const ImageWriter::EnqueueResult result = m_imageWriter->enqueue(std::move(frame));
        if (result != ImageWriter::EnqueueResult::Accepted) {
            m_capturePipelineFailed.store(true);
        }
    } catch (const std::exception &exception) {
        requestStop(QString("处理相机回调失败：%1").arg(exception.what()));
    } catch (...) {
        requestStop("处理相机回调时发生未知错误，已停止本次扫描");
    }
}

void OpenCVSet::handleImageQueueOverflow()
{
    if (sender() != m_imageWriter.get()) {
        return;
    }

    handleCapturePipelineFailure(
        m_captureGeneration.load(),
        "图像保存队列已达到上限，为避免内存继续增长，本次扫描已停止");
}

void OpenCVSet::handleCapturePipelineFailure(quint64 generation, const QString &reason)
{
    if (m_isDestorying || generation != m_captureGeneration.load()) {
        return;
    }

    m_capturePipelineFailed.store(true);
    addlog(reason);

    if (workTimer) {
        workTimer->stop();
    }
    StopCallbackCapture();
}

void OpenCVSet::handleAllImagesSaved()
{
    if (m_isDestorying || !m_imageWriter || sender() != m_imageWriter.get()) {
        return;
    }

    const ImageWriter::Stats stats = m_imageWriter->stats();
    const quint64 camera1Frames = m_frameSequences[0].load();
    const quint64 camera2Frames = m_frameSequences[1].load();
    const bool success = !m_capturePipelineFailed.load()
                         && stats.pendingFrames == 0
                         && stats.savedFrames > 0
                         && stats.failedFrames == 0
                         && stats.droppedFrames == 0
                         && stats.acceptedFrames == stats.savedFrames
                         && stats.acceptedFrames == camera1Frames + camera2Frames
                         && camera1Frames > 0
                         && camera1Frames == camera2Frames;

    m_imagesReadyForReconstruction = success;
    ui->pushButton_19->setEnabled(success);

    addlog(QString("图像保存完成：接收=%1，成功=%2，失败=%3，丢弃=%4")
               .arg(stats.acceptedFrames)
               .arg(stats.savedFrames)
               .arg(stats.failedFrames)
               .arg(stats.droppedFrames));

    if (camera1Frames != camera2Frames) {
        addlog(QString("双相机帧数不一致：相机1=%1，相机2=%2")
                   .arg(camera1Frames)
                   .arg(camera2Frames));
    }

    addlog(success ? "图像已全部保存，可以开始点云重建"
                   : "本次扫描数据不完整，禁止点云重建");
}

// 在可视化窗口中添加圆弧
void addArcToVisualizer(pcl::visualization::PCLVisualizer::Ptr& viewer,
                        const pcl::PointCloud<pcl::PointXYZ>& v_cloud,
                        const std::string& arc_id,
                        double color_r = 255.0,
                        double color_g = 0.0,
                        double color_b = 0.0) {

    if (v_cloud.size() < 2) {
        std::cout << "警告: 点云点数不足，无法绘制圆弧" << std::endl;
        return;
    }

    // 逐段绘制相邻点之间的线段
    for (size_t i = 0; i < v_cloud.size() - 1; ++i) {
        std::string line_id = arc_id + "_line_" + std::to_string(i);

        const pcl::PointXYZ& p1 = v_cloud.points[i];
        const pcl::PointXYZ& p2 = v_cloud.points[i + 1];

        // 绘制线段
        viewer->addLine<pcl::PointXYZ>(p1, p2, color_r, color_g, color_b, line_id);
    }

}

double pointToPoint(const pcl::PointXYZ p_start,const pcl::PointXYZ p_end)
{
    return std::sqrt((p_start.x-p_end.x)*(p_start.x-p_end.x)+(p_start.y-p_end.y)*(p_start.y-p_end.y)+(p_start.z-p_end.z)*(p_start.z-p_end.z));
}


void OpenCVSet::loadSettings()
{
    QSettings settings("OpenCVSet", "Settings");

    ui->lineEdit->setText(settings.value("meter", "100").toString());
    ui->h_edi->setText(settings.value("h", "50").toString());
    z0 = static_cast<float>(settings.value("z0", 0.0).toDouble());
    angle = static_cast<float>(settings.value("angle", 0.0).toDouble());

    axis = Eigen::Vector3f(
        static_cast<float>(settings.value("axis1", 0.0).toDouble()),
        static_cast<float>(settings.value("axis2", 0.0).toDouble()),
        static_cast<float>(settings.value("axis3", 0.1).toDouble())
        );
    alxepositionstart=settings.value("alxepositionstart",0).toFloat();

    x_b=settings.value("x_b",0).toDouble();
    y_b=settings.value("y_b",0).toDouble();

}

void OpenCVSet::saveSettings()
{
    QSettings settings("OpenCVSet", "Settings");

    settings.setValue("meter", ui->lineEdit->text());
    settings.setValue("h", ui->h_edi->text());
    settings.setValue("z0", z0);
    settings.setValue("angle", angle);
    settings.setValue("axis1", axis.x());
    settings.setValue("axis2", axis.y());
    settings.setValue("axis3", axis.z());
    settings.setValue("alxepositionstart",alxepositionstart);
    settings.setValue("x_b",x_b);
    settings.setValue("y_b",y_b);

}


void OpenCVSet::takePhoto()
{
    if (m_bCallbackMode) {  // 如果当前是回调模式
       // 回调模式下，实际拍照已经在回调函数中自动进行了
       // 这个函数只负责计数和停止控制


       callCount++;
       long expectedTotal = ui->lineEdit->text().toLong()*2;  // 预期拍照总数

       // 检查是否达到预期数量
       if (callCount >= expectedTotal) {
           workTimer->stop();           // 停止工作定时器
           StopCallbackCapture();       // 停止回调采集
           addlog(QString("已完成 %1 次触发，停止采集").arg(callCount));
           positionTimer->start();      // 启动位置定时器（可能是移动到一个位置）
       }
       return;  // 回调模式下不执行后面的同步采集代码
   }
    QElapsedTimer totalTimer;
    totalTimer.start();

    if (m_isDestorying) return;
    if (!workTimer || !camera_is_open) {
        addlog("相机未连接，无法执行拍照操作");
        return;
    }

    callCount++;
    long expectedTotal = ui->lineEdit->text().toLong();

    if (callCount >= expectedTotal) {
        workTimer->stop();
        addlog(QString("已拍摄照片数目足够，停止拍照"));
        if (callCount == expectedTotal) {
            addlog(QString("第 %1 次拍照开始（最后一张）").arg(callCount));
        } else {
            addlog(QString("超过预期，不执行拍照"));
            return;
        }
        positionTimer->start();
    }


    uint32_t nTimeout = 2000;
    CImageDataPointer image = objStreamPtr->DQBuf(nTimeout);
    CImageDataPointer image1 = objStreamPtr1->DQBuf(nTimeout);

    if (image.IsNull() || image1.IsNull()) {
        addlog("获取图像失败");
        if (!image1.IsNull()) objStreamPtr1->QBuf(image1);
        if (!image.IsNull()) objStreamPtr->QBuf(image);
        return;
    }

    // 阶段2: 图像保存
    std::string path = "./work/" + QString("%1").arg(work, 4, 10, QChar('0')).toStdString() + ".png";
    std::string path1 = "./work1/" + QString("%1").arg(work, 4, 10, QChar('0')).toStdString() + ".png";

    GX_SAVE_IMAGE_INFO stSaveImageInfo = {0};
    stSaveImageInfo.emCfaMethod = GX_CFA_METHOD_BALANCE;
    stSaveImageInfo.emImgFormat = GX_IMAGE_FORMAT_PNG;
    stSaveImageInfo.emSrcFormat = (GX_PIXEL_FORMAT_ENTRY)image->GetPixelFormat();
    stSaveImageInfo.nHeight = image->GetHeight();
    stSaveImageInfo.nWidth = image->GetWidth();
    stSaveImageInfo.nImgQuality = 70;
    stSaveImageInfo.pImageBuffer = (unsigned char*)(image->GetBuffer());
    stSaveImageInfo.pImgPath = (char*)path.c_str();

    GX_SAVE_IMAGE_INFO stSaveImageInfo1 = {0};
    stSaveImageInfo1.emCfaMethod = GX_CFA_METHOD_BALANCE;
    stSaveImageInfo1.emImgFormat = GX_IMAGE_FORMAT_PNG;
    stSaveImageInfo1.emSrcFormat = (GX_PIXEL_FORMAT_ENTRY)image1->GetPixelFormat();
    stSaveImageInfo1.nHeight = image1->GetHeight();
    stSaveImageInfo1.nWidth = image1->GetWidth();
    stSaveImageInfo1.nImgQuality = 70;
    stSaveImageInfo1.pImageBuffer = (unsigned char*)(image1->GetBuffer());
    stSaveImageInfo1.pImgPath = (char*)path1.c_str();

    // SaveImage 返回 void，直接调用，假设成功
    IGXFactory::GetInstance().SaveImage(&stSaveImageInfo);
    IGXFactory::GetInstance().SaveImage(&stSaveImageInfo1);

    objStreamPtr->QBuf(image);
    objStreamPtr1->QBuf(image1);

    // 假设保存成功，增加计数
    work++;

    // 输出详细性能数据
    qint64 totalTime = totalTimer.elapsed();
    qDebug() << "总耗时:" << totalTime << "ms";
}

void OpenCVSet::takePhoto2(QString &path,int camera_number,int number)
{
    // 确保目录存在
    QDir dir;
    if (!dir.exists(path))
    {
        dir.mkpath(path);
    }
    CGXDevicePointer objDevicePtr_now;
    CGXStreamPointer objStreamPtr_now;
    CImageDataPointer image_now;
    CGXFeatureControlPointer featureControl_now;
    int number_now=number;

    if(camera_number==0)
    {
        objDevicePtr_now=objDevicePtr;
        objStreamPtr_now=objStreamPtr;
        image_now=image;
        featureControl_now=featureControl;
    }
    else
    {
        objDevicePtr_now=objDevicePtr1;
        objStreamPtr_now=objStreamPtr1;
        image_now=image1;
        featureControl_now=featureControl1;
    }
    // 检查流是否已经在采集
    bool isGrabbing = false;
    if (!objStreamPtr_now.IsNull())
    {
        isGrabbing = objStreamPtr_now->GetFeatureControl()->GetBoolFeature("StreamIsGrabbing")->GetValue();
    }

    if (!isGrabbing)
    {
        addlog("流未启动，重新启动采集");
        // 只在第一次或流停止时才重新启动
        if (objStreamPtr_now == nullptr)
        {
            int32_t nStreamCount = objDevicePtr_now->GetStreamCount();
            if (0 < nStreamCount)
            {
                objStreamPtr_now = objDevicePtr_now->OpenStream(0);
            }
            else
            {
                addlog("获取流数量为 0");
                return;
            }
        }

        objStreamPtr_now->StartGrab();
        objDevicePtr_now->GetRemoteFeatureControl()->GetCommandFeature("AcquisitionStart")->Execute();
        addlog("采集已启动");

        // 等待相机稳定
        QThread::msleep(100);
    }
    if (featureControl_now->IsImplemented("TriggerSource"))
    {
        featureControl_now->GetEnumFeature("TriggerSource")->SetValue("Software");
    }

    // 发送软触发信号获取新图像
    try
    {
        CGXFeatureControlPointer featureControl = objDevicePtr_now->GetRemoteFeatureControl();
        if (featureControl->IsImplemented("TriggerSoftware"))
        {
            featureControl->GetCommandFeature("TriggerSoftware")->Execute();
            addlog("已发送软触发信号");
        }
        else
        {
            addlog("相机不支持软触发，使用连续采集模式");
        }
    }
    catch (const std::exception& e)
    {
        addlog("触发异常");
    }

    // 等待新图像采集完成
    QThread::msleep(50);

    // 获取图像（增加超时时间）
    uint32_t nTimeout = 2000;
    CImageDataPointer currentImage = objStreamPtr_now->DQBuf(nTimeout);

    if (currentImage == nullptr)
    {
        addlog("获取图像失败（超时）");
        return;
    }

    // 检查图像是否是新采集的
    uint64_t frameID = currentImage->GetFrameID();

    // 使用PNG格式
    std::string path1 =path.toStdString()+ "/" + QString("%1").arg(number_now, 4, 10, QChar('0')).toStdString() + ".png";

    GX_SAVE_IMAGE_INFO stCurrentSaveInfo;
    stCurrentSaveInfo.emCfaMethod = GX_CFA_METHOD_BALANCE;
    stCurrentSaveInfo.emImgFormat = GX_IMAGE_FORMAT_PNG;
    stCurrentSaveInfo.emSrcFormat = (GX_PIXEL_FORMAT_ENTRY)currentImage->GetPixelFormat();
    stCurrentSaveInfo.nHeight = currentImage->GetHeight();
    stCurrentSaveInfo.nWidth = currentImage->GetWidth();
    stCurrentSaveInfo.nImgQuality = 99;
    stCurrentSaveInfo.pImageBuffer = (unsigned char*)(currentImage->GetBuffer());
    stCurrentSaveInfo.pImgPath = (char*)path1.c_str();

    // 保存图片
    IGXFactory::GetInstance().SaveImage(&stCurrentSaveInfo);

    // 归还缓冲区
    objStreamPtr_now->QBuf(currentImage);
}


//数据转换
void floatToBytes(float value, uint8_t* output) {
    memcpy(output, &value, 4);
}

//日志添加
void OpenCVSet::addlog(const QString &log)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    ui->logEdit->append(QString("[%1] %2").arg(timestamp).arg(log));
    ui->logEdit_2->append(QString("[%1] %2").arg(timestamp).arg(log));

    // 自动滚动到底部
    QScrollBar *scrollbar = ui->logEdit->verticalScrollBar();
    scrollbar->setValue(scrollbar->maximum());
    QScrollBar *scrollbar_2 = ui->logEdit_2->verticalScrollBar();
    scrollbar_2->setValue(scrollbar_2->maximum());
}


//连接相机
void OpenCVSet::on_pushButton_clicked()
{
    IGXFactory::GetInstance().Init();
    camera_is_Init = true;

    IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);

    if(vectorDeviceInfo.size() == 2)
    {
        GxIAPICPP::gxstring strSN = vectorDeviceInfo[0].GetSN();
        GxIAPICPP::gxstring strSN1 = vectorDeviceInfo[1].GetSN();
        if(vectorDeviceInfo[0].GetIP()=="192.168.3.22")
        {
            objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(strSN, GX_ACCESS_EXCLUSIVE);
            addlog("相机1已打开111");
            objDevicePtr1 = IGXFactory::GetInstance().OpenDeviceBySN(strSN1, GX_ACCESS_EXCLUSIVE);
            addlog("相机2已打开111");
        }
        else
        {
            objDevicePtr1 = IGXFactory::GetInstance().OpenDeviceBySN(strSN, GX_ACCESS_EXCLUSIVE);
            addlog("相机1已打开");
            objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(strSN1, GX_ACCESS_EXCLUSIVE);
            addlog("相机2已打开");
        }

        camera_is_open = true;
        ui->cameralabel->setText("已连接");
        ui->cameralabel->setStyleSheet("color: rgb(0, 255, 0);");
        ui->comboBox->addItem("0");
        ui->comboBox->addItem("1");
    }
    else
    {
        addlog("未找到设备,或设备不全");
        ui->cameralabel->setStyleSheet("color: rgb(255, 0, 0);");
        return;
    }

    int32_t nStreamCount = objDevicePtr->GetStreamCount();
    int32_t nStreamCount1 = objDevicePtr1->GetStreamCount();

    if (0 < nStreamCount)
    {
        objStreamPtr = objDevicePtr->OpenStream(0);

        // 配置相机为软触发模式
        featureControl = objDevicePtr->GetRemoteFeatureControl();

        // 设置触发模式为开启
        if (featureControl->IsImplemented("TriggerMode"))
        {
            featureControl->GetEnumFeature("TriggerMode")->SetValue("On");
            addlog("触发模式: 开启");
        }

        // 设置触发源为IO触发
        if (featureControl->IsImplemented("TriggerSource"))
        {
            featureControl->GetEnumFeature("TriggerSource")->SetValue("Line0");
            addlog("相机1触发源: IO触发");
        }

        // 启动流
        objStreamPtr->StartGrab();

        // 启动采集（但不触发）
        featureControl->GetCommandFeature("AcquisitionStart")->Execute();
        addlog("相机1已就绪，等待触发信号");

    }
    else
    {
        addlog("相机1获取流数量为 0");
    }

    if (0 < nStreamCount1)
    {
        objStreamPtr1 = objDevicePtr1->OpenStream(0);

        // 配置相机为软触发模式
        featureControl1 = objDevicePtr1->GetRemoteFeatureControl();

        // 设置触发模式为开启
        if (featureControl1->IsImplemented("TriggerMode"))
        {
            featureControl1->GetEnumFeature("TriggerMode")->SetValue("On");
            addlog("触发模式: 开启");
        }

        // 设置触发源为IO触发
        if (featureControl1->IsImplemented("TriggerSource"))
        {
            featureControl1->GetEnumFeature("TriggerSource")->SetValue("Line0");
            addlog("相机2触发源: IO触发");
        }

        // 启动流
        objStreamPtr1->StartGrab();

        // 启动采集（但不触发）
        featureControl1->GetCommandFeature("AcquisitionStart")->Execute();
        addlog("相机2已就绪，等待触发信号");
    }
    else
    {
        addlog("相机2获取流数量为 0");
    }
}

//通过照片进行相机内参标定计算
void OpenCVSet::on_pushButton_2_clicked()
{
    QString calibDir;
    QString outfilename;

    if(ui->comboBox->currentText()=="0")
    {
        calibDir = "./calib_images";
        outfilename="camera_calib.yaml";
    }
    else
    {
        calibDir = "./calib_images1";
        outfilename="camera_calib1.yaml";
    }

    if (calibrator) {
        qDebug() << "开始相机标定...";
        calibrator->calibrateCameraFromImages(calibDir, 11, 8, 20.0f,outfilename);
        QMessageBox::information(this, "完成", "标定完成！\n结果已保存到 camera_calib.yaml");
    } else {
        QMessageBox::warning(this, "错误", "标定器未初始化");
    }
}

//激光平面标定
void OpenCVSet::on_pushButton_3_clicked()
{

    if (calibrator) {
        qDebug() << "开始激光平面标定...";
        if(ui->comboBox->currentText()=="0")
        {
            calibrator->calibrateLaserPlane("./camera_calib.yaml","./nolaser/0000.png","./laser/0000.png","./nolaser/0001.png","./laser/0001.png",11,8,20.0f,"light_plane.yaml");
        }else
        {
            calibrator->calibrateLaserPlane("./camera_calib1.yaml","./nolaser1/0000.png","./laser1/0000.png","./nolaser1/0001.png","./laser1/0001.png",11,8,20.0f,"light_plane1.yaml");
        }

        QMessageBox::information(this, "完成", "标定完成！");
    } else {
        QMessageBox::warning(this, "错误", "标定器未初始化");
    }
}

//连接轨道
void OpenCVSet::on_pushButton_4_clicked()
{
    // 创建新的连接
    if (!alxeSocket) {
        alxeSocket = new QTcpSocket(this);
        connect(alxeSocket, &QTcpSocket::connected, this, &OpenCVSet::connected);
        connect(alxeSocket, &QTcpSocket::disconnected, this, &OpenCVSet::disconnected);
        connect(alxeSocket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
                this, &OpenCVSet::socketError);
    }

    // 尝试连接
    alxeSocket->connectToHost("192.168.3.168", 502);
    addlog("正在连接到 192.168.3.168; 502");

    // 设置连接超时
    if (!alxeSocket->waitForConnected(5000)) {
        addlog("连接超时");
    }

}

//轨道连接
void OpenCVSet::connected()
{
    addlog("已连接到服务器：192.168.3.168  502");
    alxe_is_connect = true;
}

//轨道断开连接
void OpenCVSet::disconnected()
{
    addlog("已断开与服务器的连接");
    alxe_is_connect = false;
    positionTimer->stop();
}

//轨道连接报错
void OpenCVSet::socketError(QAbstractSocket::SocketError error)
{
    if (alxeSocket) {
        addlog(QString("Socket错误: %1").arg(alxeSocket->errorString()));
        QMessageBox::warning(this, "连接错误", alxeSocket->errorString());
    }

}

//更新连接状态
void OpenCVSet::updateConnectionStatus()
{
    QString statusText;
    QString statusColor;

    if (alxeSocket) {
        switch (alxeSocket->state()) {
        case QTcpSocket::UnconnectedState:
            statusText = "未连接";
            statusColor = "red";
            break;
        case QTcpSocket::HostLookupState:
            statusText = "正在查找主机...";
            statusColor = "orange";
            break;
        case QTcpSocket::ConnectingState:
            statusText = "正在连接...";
            statusColor = "orange";
            break;
        case QTcpSocket::ConnectedState:
            statusText = "已连接";
            statusColor = "green";
            break;
        case QTcpSocket::ClosingState:
            statusText = "正在关闭...";
            statusColor = "orange";
            break;
        default:
            statusText = "未知状态";
            statusColor = "gray";
            break;
        }
    } else {
        statusText = "未连接";
        statusColor = "red";
    }

    ui->alxelabel->setText(QString("<font color='%1'> %2</font>")
                                 .arg(statusColor)
                                 .arg(statusText));
}

//更新位置信息
void OpenCVSet::updatePosition()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }

    float exAxis[6]; //固定长度
    RPY rpy;
    int mWord = 0;
    int errorCode =ReadPOSE_RPY(&rpy,exAxis ,connectID);
    int res = Read_PowerOn (connectID);
    if(errorCode!=0)
    {
        addlog("大车、小车位置读取失败");
        positionTimer->stop();
        return;
    }
    ui->alxelineEdit->setText(QString::number(exAxis[0]));//大车位置
    ui->xalxelineEdit->setText(QString::number(exAxis[1]));//小车位置

    if (res == 1)
    {
        ui->power->setText("已上电");
    }
    else if(res == 0)
    {//伺服上电关闭
        ui->power->setText("未上电");
    }
    else
    {
        addlog("查询失败");
    }

}


//相对移动开始
void OpenCVSet::PCL2Camera()
{
    if (!alxeSocket || alxeSocket->state() != QTcpSocket::ConnectedState) {
        QMessageBox::warning(this, "警告", "未连接到服务器！");
        return;
    }

    static quint16 transactionId = 1;

    float value = ui->lineEdit->text().toFloat()*2;
    uint8_t bytes[4];

    // 浮点数转完全小端序字节
    memcpy(bytes, &value, 4);

    quint16 protocolAddr = 40023 - 40001;  //

    // 构建报文
    QByteArray sendData;

    // Modbus TCP 头部
    sendData.append(static_cast<char>((transactionId >> 8) & 0xFF));   // 事务ID高字节
    sendData.append(static_cast<char>(transactionId & 0xFF));          // 事务ID低字节
    sendData.append(static_cast<char>(0x00));                           // 协议ID高字节
    sendData.append(static_cast<char>(0x00));                           // 协议ID低字节
    sendData.append(static_cast<char>(0x00));                           // 长度占位高字节
    sendData.append(static_cast<char>(0x00));                           // 长度占位低字节

    // PDU 部分
    sendData.append(static_cast<char>(0x01));                           // 单元ID（从站地址）
    sendData.append(static_cast<char>(0x10));                           // 功能码：写多个寄存器
    sendData.append(static_cast<char>((protocolAddr >> 8) & 0xFF));    // 起始地址高字节
    sendData.append(static_cast<char>(protocolAddr & 0xFF));            // 起始地址低字节
    sendData.append(static_cast<char>(0x00));                           // 寄存器数量高字节
    sendData.append(static_cast<char>(0x02));                           // 寄存器数量低字节
    sendData.append(static_cast<char>(0x04));                           // 数据字节数
    sendData.append(static_cast<char>(bytes[3]));                       // 数据字节0
    sendData.append(static_cast<char>(bytes[2]));                       // 数据字节1
    sendData.append(static_cast<char>(bytes[1]));                       // 数据字节2
    sendData.append(static_cast<char>(bytes[0]));                       // 数据字节3

    // 计算并更新长度字段
    quint16 length = sendData.size() - 6;
    sendData[4] = static_cast<char>((length >> 8) & 0xFF);
    sendData[5] = static_cast<char>(length & 0xFF);

    // 发送报文
    alxeSocket->write(sendData);

    // 等待响应
    if (alxeSocket->waitForReadyRead(3000)) {
        QByteArray response = alxeSocket->readAll();

        if (response.size() >= 12 && response[7] == 0x10) {
            addlog("✓ 相对位置写入成功");
        } else {
            addlog("✗ 相对位置写入失败");
        }
    } else {
        addlog("✗ 响应超时");
    }

    transactionId+=2;
    // 发送归零结束命令
    QString hexStr2 = QString("%1 00 00 00 06 01 06 00 1D 00 00")
                          .arg(transactionId++, 4, 16, QChar('0'))
                          .toUpper();
    QByteArray sendData2 = QByteArray::fromHex(hexStr2.remove(' ').toLatin1());

    alxeSocket->write(sendData2);
    // 等待响应确认
    if (alxeSocket->waitForReadyRead(3000)) {
        QByteArray response = alxeSocket->readAll();

        // 检查响应是否正确
        if (response.size() >= 12 && response[7] == 0x06) {
            addlog("移动命令接收成功");
        } else {
            addlog("移动命令接收失败");
            return;
        }
    } else {
        addlog("等待归零启动响应超时");
        return;
    }

    Sleep(100);  // 增加到2秒或更长
    QString hexStr3 = QString("%1 00 00 00 06 01 06 00 1D 00 01")
                          .arg(transactionId++, 4, 16, QChar('0'))
                          .toUpper();
    QByteArray sendData3 = QByteArray::fromHex(hexStr3.remove(' ').toLatin1());

    alxeSocket->write(sendData3);
    if (alxeSocket->waitForReadyRead(3000)) {
        QByteArray response = alxeSocket->readAll();

        // 检查响应是否正确
        if (response.size() >= 12 && response[7] == 0x06) {
            addlog("移动命令接收成功");
        } else {
            addlog("移动命令接收失败");
            return;
        }
    } else {
        addlog("等待归零启动响应超时");
        return;
    }
    Sleep(100);
    QString hexStr4 = QString("%1 00 00 00 06 01 06 00 1D 00 00")
                          .arg(transactionId++, 4, 16, QChar('0'))
                          .toUpper();
    QByteArray sendData4 = QByteArray::fromHex(hexStr4.remove(' ').toLatin1());

    alxeSocket->write(sendData4);
}

//拍照
void OpenCVSet::on_pushButton_11_clicked()
{
    if (!camera_is_open) {
        addlog("相机未连接，无法执行拍照操作");
        return;
    }
    int i=ui->comboBox->currentText().toInt();
    QString path1;
    std::string path;
    if(i==0)
    {
        path1="./calib_images";
        path ="./calib_images/" + QString("%1").arg(picNumber, 4, 10, QChar('0')).toStdString() + ".png";
        OpenCVSet::takePhoto2(path1,i,picNumber);
        picNumber++;
        ui->pic_number_lab->setText(QString::number(picNumber));

    }else if(i==1)
    {
        path1="./calib_images1";
        path ="./calib_images1/" + QString("%1").arg(picNumber1, 4, 10, QChar('0')).toStdString() + ".png";
        OpenCVSet::takePhoto2(path1,i,picNumber1);
        picNumber1++;
        ui->pic_number_lab->setText(QString::number(picNumber1));
    }
    // 显示图片
    ui->piclabel->clear();
    QString absolutePath = QDir::current().absoluteFilePath(QString::fromStdString(path));
    QPixmap originalPixmap(absolutePath);

    if (!originalPixmap.isNull())
    {
        ui->piclabel->setPixmap(originalPixmap);
        ui->piclabel->setScaledContents(true);
        ui->piclabel->update();
        addlog("图片显示成功");
    }
    else
    {
        addlog("QPixmap加载图片失败");
    }
}

//拍照无激光
void OpenCVSet::on_pushButton_12_clicked()
{
    if(!camera_is_open)
    {
        addlog("相机未连接，无法执行拍照操作");
        return;
    }
    int i=ui->comboBox->currentText().toInt();
    // 确保目录存在
    QString path1;
    std::string path;
    if(i==0)
    {
        path1="./nolaser";
        path ="./nolaser/" + QString("%1").arg(nolaser, 4, 10, QChar('0')).toStdString() + ".png";
        OpenCVSet::takePhoto2(path1,i,nolaser);
        nolaser++;
        ui->nolaser_lab->setText(QString::number(nolaser));

    }else if(i==1)
    {
        path1="./nolaser1";
        path ="./nolaser1/" + QString("%1").arg(nolaser1, 4, 10, QChar('0')).toStdString() + ".png";
        OpenCVSet::takePhoto2(path1,i,nolaser1);
        nolaser1++;
        ui->nolaser_lab->setText(QString::number(nolaser1));
    }
    // 显示图片
    ui->piclabel->clear();
    QString absolutePath = QDir::current().absoluteFilePath(QString::fromStdString(path));
    QPixmap originalPixmap(absolutePath);

    if (!originalPixmap.isNull())
    {
        ui->piclabel->setPixmap(originalPixmap);
        ui->piclabel->setScaledContents(true);
        ui->piclabel->update();
        addlog("图片显示成功");
    }
    else
    {
        addlog("QPixmap加载图片失败");
    }
}

//拍照有激光
void OpenCVSet::on_pushButton_13_clicked()
{
    if(!camera_is_open)
    {
        addlog("相机未连接，无法执行拍照操作");
        return;
    }
    int i=ui->comboBox->currentText().toInt();
    // 确保目录存在
    QString path1;
    std::string path;
    if(i==0)
    {
        path1="./laser";
        path ="./laser/" + QString("%1").arg(laser, 4, 10, QChar('0')).toStdString() + ".png";
        OpenCVSet::takePhoto2(path1,i,laser);
        laser++;
        ui->laser_lab->setText(QString::number(laser));

    }else if(i==1)
    {
        path1="./laser1";
        path ="./laser1/" + QString("%1").arg(laser1, 4, 10, QChar('0')).toStdString() + ".png";
        OpenCVSet::takePhoto2(path1,i,laser1);
        laser1++;
        ui->laser_lab->setText(QString::number(laser1));
    }
    // 显示图片
    ui->piclabel->clear();
    QString absolutePath = QDir::current().absoluteFilePath(QString::fromStdString(path));
    QPixmap originalPixmap(absolutePath);

    if (!originalPixmap.isNull())
    {
        ui->piclabel->setPixmap(originalPixmap);
        ui->piclabel->setScaledContents(true);
        ui->piclabel->update();
        addlog("图片显示成功");
    }
    else
    {
        addlog("QPixmap加载图片失败");
    }

}

//相机重新标定
void OpenCVSet::on_pushButton_16_clicked()
{
    std::string folder_path;
    // 要删除的文件夹路径
    if(ui->comboBox->currentText()=="0")
    {
        picNumber=0;
        folder_path = "./calib_images/";
        ui->pic_number_lab->setText(QString::number(picNumber));
    }else
    {
        picNumber1=0;
        folder_path = "./calib_images1/";
        ui->pic_number_lab->setText(QString::number(picNumber1));
    }

    // 遍历文件夹里所有文件
    for (const auto& entry : std::filesystem::directory_iterator(folder_path))
    {
        QString path = QString::fromStdString(entry.path().string());
        // 获取文件路径
        std::filesystem::remove(entry.path());

        addlog("图片已删除："+path);
    }
    addlog("图片已删除完成，现在可以开始重新标定");
}

//激光平面重新标定
void OpenCVSet::on_pushButton_17_clicked()
{
    // 要删除的文件夹路径
    std::string folder_path1;
    std::string folder_path2;

    // 要删除的文件夹路径
    if(ui->comboBox->currentText()=="0")
    {
        laser=0;
        nolaser = 0;
        folder_path1 = "./laser/";
        folder_path2 = "./nolaser/";
        ui->laser_lab->setText(QString::number(laser));
        ui->nolaser_lab->setText(QString::number(nolaser));
    }else
    {
        laser1=0;
        nolaser1 = 0;
        folder_path1 = "./laser1/";
        folder_path2 = "./nolaser1/";
        ui->laser_lab->setText(QString::number(laser1));
        ui->nolaser_lab->setText(QString::number(nolaser1));
    }

    // 遍历文件夹里所有文件
    for (const auto& entry : std::filesystem::directory_iterator(folder_path1))
    {
        QString path = QString::fromStdString(entry.path().string());
        // 获取文件路径
        std::filesystem::remove(entry.path());

        addlog("图片已删除："+path);
    }
    for (const auto& entry2 : std::filesystem::directory_iterator(folder_path2))
    {
        QString path2 = QString::fromStdString(entry2.path().string());
        // 获取文件路径
        std::filesystem::remove(entry2.path());

        addlog("图片已删除："+path2);
    }
    addlog("图片已删除完成，现在可以开始激光平面重新标定");
}

//轨道标定
void OpenCVSet::on_pushButton_10_clicked()
{
    if (calibrator) {
        if(ui->comboBox->currentText()=="0")
        {
            calibrator->calibrateMotion("./camera_calib.yaml","./platform/0000.png","./platform/0001.png",100,11,8,20.0f,"motion_calib.yaml",100);
        }else
        {
           calibrator->calibrateMotion("./camera_calib1.yaml","./platform1/0000.png","./platform1/0001.png",100,11,8,20.0f,"motion_calib1.yaml",100);
        }

        QMessageBox::information(this, "完成", "标定完成！");
    } else {
        QMessageBox::warning(this, "错误", "标定器未初始化");
    }
}


//打开激光
void OpenCVSet::on_pushButton_14_clicked()
{
    if (!alxeSocket || alxeSocket->state() != QTcpSocket::ConnectedState) {
        QMessageBox::warning(this, "警告", "未连接到服务器！");
        return;
    }

    static quint16 transactionId = 1;

    if(!laser_is_open)
    {
        // 发送归零启动命令
        QString hexStr1 = QString("%1 00 00 00 06 01 06 00 1F 00 01")
                              .arg(transactionId++, 4, 16, QChar('0'))
                              .toUpper();
        QByteArray sendData1 = QByteArray::fromHex(hexStr1.remove(' ').toLatin1());

        alxeSocket->write(sendData1);

        // 等待响应确认
        if (alxeSocket->waitForReadyRead(3000)) {
            QByteArray response = alxeSocket->readAll();

            // 检查响应是否正确
            if (response.size() >= 12 && response[7] == 0x06) {
                addlog("激光打开成功");
                laser_is_open = true;
                ui->pushButton_14->setText("关闭激光");
            }
            else
            {
                addlog("激光打开失败");
                return;
            }
        }
        else {
            addlog("激光打开响应超时");
            return;
        }
    }
    else
    {
        // 发送归零启动命令
        QString hexStr1 = QString("%1 00 00 00 06 01 06 00 1F 00 00")
                              .arg(transactionId++, 4, 16, QChar('0'))
                              .toUpper();
        QByteArray sendData1 = QByteArray::fromHex(hexStr1.remove(' ').toLatin1());

        alxeSocket->write(sendData1);

        // 等待响应确认
        if (alxeSocket->waitForReadyRead(3000)) {
            QByteArray response = alxeSocket->readAll();

            // 检查响应是否正确
            if (response.size() >= 12 && response[7] == 0x06) {
                addlog("激光关闭成功");
                laser_is_open = false;
                ui->pushButton_14->setText("打开激光");
            }
            else
            {
                addlog("激光关闭失败");
                return;
            }
        }
        else
        {
            addlog("激光关闭响应超时");
            return;
        }
    }
}

//外部轴标定拍照
void OpenCVSet::on_pushButton_9_clicked()
{
    if(!camera_is_open)
    {
        addlog("相机未连接，无法执行拍照操作");
        return;
    }

    int i=ui->comboBox->currentText().toInt();
    QString path1;
    std::string path;
    if(i==0)
    {
        path1="./platform";
        path ="./platform/" + QString("%1").arg(plat, 4, 10, QChar('0')).toStdString() + ".png";
        OpenCVSet::takePhoto2(path1,i,plat);
        plat++;
        ui->plat_lab->setText(QString::number(plat));

    }else if(i==1)
    {
        path1="./platform1";
        path ="./platform1/" + QString("%1").arg(plat1, 4, 10, QChar('0')).toStdString() + ".png";
        OpenCVSet::takePhoto2(path1,i,plat1);
        plat1++;
        ui->plat_lab->setText(QString::number(plat1));
    }

    ui->piclabel->clear();
    QString absolutePath = QDir::current().absoluteFilePath(QString::fromStdString(path));
    QPixmap originalPixmap(absolutePath);

    if (!originalPixmap.isNull())
    {
        ui->piclabel->setPixmap(originalPixmap);
        ui->piclabel->setScaledContents(true);
        ui->piclabel->update();
        addlog("图片显示成功");
    }
    else
    {
        addlog("QPixmap加载图片失败");
    }

}

//平台重新标定
void OpenCVSet::on_pushButton_18_clicked()
{
    std::string folder_path;
    // 要删除的文件夹路径
    if(ui->comboBox->currentText()=="0")
    {
        plat=0;
        folder_path = "./platform/";
        ui->plat_lab->setText(QString::number(plat));
    }else
    {
        plat1=0;
        folder_path = "./platform1/";
        ui->plat_lab->setText(QString::number(plat1));
    }

    // 遍历文件夹里所有文件
    for (const auto& entry : std::filesystem::directory_iterator(folder_path))
    {
        QString path = QString::fromStdString(entry.path().string());
        // 获取文件路径
        std::filesystem::remove(entry.path());

        addlog("图片已删除："+path);
    }
    addlog("图片已删除完成，现在可以开始重新平台标定");
}


void OpenCVSet::clearWork()
{
    work=0;
    callCount = 0;
    ui->plat_lab->setText(QString::number(plat));

    // 要删除的文件夹路径
    std::vector<std::string> folders = {
        "./work/",
        "./work1/",
        "./debug_frames/",
        "./debug_frames1/"
    };

    for (const auto& folder_path : folders) {
        try {
            // 检查文件夹是否存在
            if (!std::filesystem::exists(folder_path)) {
                addlog(QString("文件夹不存在，跳过：%1").arg(QString::fromStdString(folder_path)));
                continue;
            }

            // 检查是否是目录
            if (!std::filesystem::is_directory(folder_path)) {
                addlog(QString("路径不是目录：%1").arg(QString::fromStdString(folder_path)));
                continue;
            }

            // 遍历并删除文件
            for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
                if (std::filesystem::is_regular_file(entry.path())) {
                    QString path = QString::fromStdString(entry.path().string());
                    std::filesystem::remove(entry.path());
                    addlog("图片已删除：" + path);
                }
            }

        } catch (const std::filesystem::filesystem_error& e) {
            addlog(QString("删除文件时出错：%1").arg(e.what()));
        } catch (const std::exception& e) {
            addlog(QString("未知错误：%1").arg(e.what()));
        }
    }

    addlog("图片已删除完成，现在开始重新扫描");
}

//扫描拍摄图片
void OpenCVSet::on_pushButton_15_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }

    if(!camera_is_open)
    {
        addlog("相机未连接");
        return;
    }

    if(ui->lineEdit->text().toInt()<0)
    {
        addlog("移动距离不可为负数，反方向扫描未写");
        return;
    }

    float exAxis[6]; //固定长度
    RPY rpy;
    int mWord = 0;
    int errorCode1 = ReadPOSE_RPY(&rpy,exAxis,connectID);
    if(errorCode1 != 0)
    {
        addlog("外部轴位置读取失败");
        return;
    }
    alxepositionstart = exAxis[0];



    if (m_bCallbackMode || m_bGrabbing || m_pCaptureHandler || m_pCaptureHandler1) {
        StopCallbackCapture();
    }

    if (m_imageWriter) {
        if (!m_imageWriter->isFinished()) {
            addlog("上一次图像仍在保存，暂时不能开始新扫描");
            return;
        }
        m_imageWriter->wait();
        m_imageWriter.reset();
    }

    OpenCVSet::clearWork();
    callCount = 0;
    m_frameSequences[0].store(0);
    m_frameSequences[1].store(0);
    m_capturePipelineFailed.store(false);
    const quint64 captureGeneration = m_captureGeneration.fetch_add(1) + 1;
    m_imagesReadyForReconstruction = false;
    ui->pushButton_19->setEnabled(false);

    try {
        const QString root = QDir::currentPath();
        m_imageWriter = std::make_unique<ImageWriter>(
            QDir(root).filePath("work"),
            QDir(root).filePath("work1"),
            64,
            256 * 1024 * 1024,
            2);

        connect(m_imageWriter.get(), &ImageWriter::queueOverflow,
                this, &OpenCVSet::handleImageQueueOverflow,
                Qt::QueuedConnection);
        connect(m_imageWriter.get(), &ImageWriter::allImagesSaved,
                this, &OpenCVSet::handleAllImagesSaved,
                Qt::QueuedConnection);
    } catch (const std::exception &exception) {
        addlog(QString("初始化图像保存器失败：%1").arg(exception.what()));
        return;
    }

    if (featureControl->IsImplemented("TriggerSource"))
    {
        featureControl->GetEnumFeature("TriggerSource")->SetValue("Line0");
        addlog("触发源: IO触发");
    }
    if (featureControl1->IsImplemented("TriggerSource"))
    {
        featureControl1->GetEnumFeature("TriggerSource")->SetValue("Line0");
        addlog("触发源: IO触发");
    }

    if (InitCallbackCapture()) {
        OpenCVSet::on_pushButton_42_clicked();
        PCL2Camera();
        m_acceptFrames.store(true, std::memory_order_release);
        StartCallbackCapture();

        if (!m_bGrabbing) {
            handleCapturePipelineFailure(captureGeneration,
                                         "回调采集启动失败，本次扫描已取消");
            return;
        }

        workTimer->start();
        addlog("已切换到回调采集模式");
    } else {
        handleCapturePipelineFailure(captureGeneration,
                                     "回调采集初始化失败，本次扫描已取消");
    }
}

void OpenCVSet::on_pushButton_19_clicked()
{
    if (!m_imagesReadyForReconstruction || !m_imageWriter) {
        QMessageBox::warning(this, "无法重建",
                             "图像尚未全部保存，或本次扫描存在保存失败/丢帧。");
        return;
    }

    const ImageWriter::Stats imageStats = m_imageWriter->stats();
    if (imageStats.pendingFrames != 0
        || imageStats.failedFrames != 0
        || imageStats.droppedFrames != 0) {
        QMessageBox::warning(this, "无法重建", "图像保存状态不完整。");
        return;
    }

    if (calibrator) {
        // 使用红色通道（默认）

        double y_q=alxepositionstart-2561.22;
        LaserScanConfig config_red;
        config_red.column_sampling_step =1;
        config_red.color_channel = 0;  // 红色
        config_red.output_filename = "cloud0";
        config_red.save_debug_images = false;
        calibrator->generatePointCloud("./work","./camera_calib.yaml","./light_plane.yaml","./motion_calib.yaml",config_red,y_q,-0.0084270448906421363, -0.99946858401195582,
                                       0.031488640611291849);
        config_red.color_channel = 0;  // 绿色
        config_red.output_filename = "cloud1";
        config_red.debug_dir="./debug_frames1";
        calibrator->generatePointCloud("./work1","./camera_calib1.yaml","./light_plane1.yaml","./motion_calib1.yaml",config_red,y_q,0.0024286257862979415, -0.99752762244646898,
                                       0.070233497941401601);


    } else {
        QMessageBox::warning(this, "错误", "标定器未初始化");
        return;
    }
    // // 检查 weldSearch 是否有效
    // if (!weldSearch) {
    //     QMessageBox::warning(this, "错误", "标定器未初始化");
    //     return;
    // }
    // QString path = QDir::currentPath();
    // QString cloudPath = path + "/cloud1.pcd";
    // weldSearch->filterPointCloud(
    //     cloudPath.toStdString(),
    //     cloudPath.toStdString(),
    //     "x",
    //     -650,
    //     300,
    //     false
    //     );
    // QString cloudPath1 = path + "/cloud0.pcd";
    // weldSearch->filterPointCloud(
    //     cloudPath1.toStdString(),
    //     cloudPath1.toStdString(),
    //     "x",
    //     -940,
    //     550,
    //     false
    //     );
    // addlog("平台过滤完成");
    QMessageBox::information(this, "完成", "点云生成成功");

}

//参数设置
void OpenCVSet::on_pushButton_20_clicked()
{
    if(!camera_is_open)
    {
        addlog("相机未连接");
        return;
    }
    GX_OPERATE_HANDLE hHandle = NULL;
    //1. 创建窗口操作句柄
    CGXDevicePointer objDevicePtr_now;

    if(ui->comboBox->currentText()=="0")
    {
        objDevicePtr_now=objDevicePtr;
    }else
    {
        objDevicePtr_now=objDevicePtr1;
    }

    objDevicePtr_now->GXCreateWnd(GX_WND_PROP, NULL, &hHandle);
    //2. 设置属性窗口显示位置位于相对屏幕左上角，宽、高均为 700
    objDevicePtr_now->GXSetShowPosition(hHandle, 0, 0, 700, 700);
    //3.设置属性窗口显示模式为阻塞（当程序是控制台时需要设置
    // BLOCK_SHOW_MODE，GUI 程序则请勿设置）
    objDevicePtr_now->GXSetShowMode(hHandle, BLOCK_SHOW_MODE);
    //4. 设置属性窗口标题
    objDevicePtr_now->GXSetWndTitle(hHandle, "TITLE");
    //5. 显示窗口
    objDevicePtr_now->GXShowWnd(hHandle, true);
    //6. 销毁窗口
    objDevicePtr_now->GXDestroyWnd(hHandle);
}


void OpenCVSet::on_pushButton_21_clicked()
{
    QString path = QDir::currentPath();
    QString cloudPath = path+"/cloud.pcd";

    qDebug()<<cloudPath;
    // 1. 先加载点云到临时对象
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (pcl::io::loadPCDFile(cloudPath.toStdString(), cloud) == -1)
    {
        QMessageBox::warning(this, "警告", "点云加载失败!");
        return;
    }


    if (cloud.empty())
    {
        QMessageBox::warning(this, "警告", "点云为空!");
        return;
    }

    if (viewer->contains("cloud_display"))  // 检查是否存在指定ID的点云
    {
        viewer->removePointCloud("cloud_display");  // 移除旧的
        viewer->removeAllShapes();
        qDebug() << "已移除旧点云";
    }

    viewer->addPointCloud(cloud.makeShared(),"cloud_display");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_display");

    // 7. 最后强制渲染一次，确保内容显示出来
    ui->widget->renderWindow()->Render();
}

void OpenCVSet::on_pushButton_22_clicked()
{
    double z = h-ui->h_edi->text().toDouble();
    QString path = QDir::currentPath();
    QString cloudPath = path + "/cloud.pcd";
    QString saveCloudPath = path + "/cloud_fitle.pcd";

    // 检查 weldSearch 是否有效
    if (!weldSearch) {
        QMessageBox::warning(this, "错误", "标定器未初始化");
        return;
    }
    weldSearch->filterPointCloud(
        cloudPath.toStdString(),
        saveCloudPath.toStdString(),
        "z",
        0.0,
        z,
        false
        );
    addlog("平台过滤完成");
}



//打开处理过的点云
void OpenCVSet::on_pushButton_23_clicked()
{
    QString path = QDir::currentPath();
    QString cloudPath = path+"/cloud_fitle.pcd";

    qDebug()<<cloudPath;
    // 1. 先加载点云到临时对象
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (pcl::io::loadPCDFile(cloudPath.toStdString(), cloud) == -1)
    {
        QMessageBox::warning(this, "警告", "点云加载失败!");
        return;
    }

    if (cloud.empty())
    {
        QMessageBox::warning(this, "警告", "点云为空!");
        return;
    }

    if (viewer->contains("cloud_display"))  // 检查是否存在指定ID的点云
    {
        viewer->removePointCloud("cloud_display");
        viewer->removeAllShapes();
        qDebug() << "已移除旧点云";
    }

    viewer->addPointCloud(cloud.makeShared(),"cloud_display");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1,
        "cloud_display"  // 这里必须和上面的ID一致
        );
    ui->widget->renderWindow()->Render();
}


void OpenCVSet::on_pushButton_25_clicked()
{
    QString path = QDir::currentPath();
    QString cloudPath = path+"/cloud_fitle.pcd";

    weldSearch->filterGroundFace(cloudPath.toStdString(),cloudPath.toStdString(),axis,angle,z0);
    addlog("焊缝轮廓提取成功");
}

// 绘制圆弧时，如果是优弧，需要特殊处理
void drawArcOnImage(cv::Mat& image, const Arc2D& arc,
                    std::function<cv::Point(double, double)> transform,
                    cv::Scalar color, int thickness) {
    const int num_points = 200;
    double start = arc.start_angle;
    double end = arc.end_angle;

    // 对于优弧，可能需要反向绘制
    // if (arc.isMajorArc()) {
    //     // 绘制优弧时，确保绘制较长的那段
    //     if (!arc.is_clockwise) {
    //         if (end < start) end += 2 * M_PI;
    //     } else {
    //         if (end > start) end -= 2 * M_PI;
    //     }
    // }

    double step = (end - start) / num_points;

    std::vector<cv::Point> points;
    for (int i = 0; i <= num_points; ++i) {
        double angle = start + i * step;
        double x = arc.cx + arc.radius * std::cos(angle);
        double y = arc.cy + arc.radius * std::sin(angle);
        points.push_back(transform(x, y));
    }

    for (size_t i = 1; i < points.size(); ++i) {
        cv::line(image, points[i-1], points[i], color, thickness);
    }
}

// 绘制菱形标记
void drawDiamond(cv::Mat& image, cv::Point center, int size,
                 cv::Scalar fill_color, cv::Scalar border_color) {
    std::vector<cv::Point> diamond;
    diamond.push_back(cv::Point(center.x, center.y - size));
    diamond.push_back(cv::Point(center.x + size, center.y));
    diamond.push_back(cv::Point(center.x, center.y + size));
    diamond.push_back(cv::Point(center.x - size, center.y));

    cv::fillPoly(image, std::vector<std::vector<cv::Point>>{diamond}, fill_color);
    cv::polylines(image, diamond, true, border_color, 1);
}

// 计算圆弧文字位置（沿法线方向偏移）
cv::Point calculateTextPositionForArc(const WeldArc2D& arc,
                                      std::function<cv::Point(double, double)> transform) {
    // 使用圆弧中点角度
    double mid_angle = (arc.arc.start_angle + arc.arc.end_angle) / 2;
    // 沿半径向外偏移30%
    double text_radius = arc.arc.radius * 1.3;
    double x = arc.arc.cx + text_radius * std::cos(mid_angle);
    double y = arc.arc.cy + text_radius * std::sin(mid_angle);
    return transform(x, y);
}

// 绘制带背景的文字
void drawTextWithBackground(cv::Mat& image, cv::Point position,
                            const std::string& text, cv::Scalar color) {
    int font_face = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 0.6;
    int thickness = 1;
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

    // 文字背景
    cv::rectangle(image,
                  cv::Point(position.x - text_size.width/2 - 1, position.y - text_size.height/2 - 1),
                  cv::Point(position.x + text_size.width/2 + 1, position.y + text_size.height/2 + 1),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    // 绘制文字
    cv::putText(image, text,
                cv::Point(position.x - text_size.width/2, position.y + text_size.height/2),
                font_face, font_scale, cv::Scalar(255,255,255), thickness);
}

// 绘制网格
void drawGrid(cv::Mat& image, std::function<cv::Point(double, double)> transform,
              double min_x, double max_x, double min_y, double max_y) {
    // 计算合适的网格间距
    double range_x = max_x - min_x;
    double range_y = max_y - min_y;

    int num_grid_x = 10;
    int num_grid_y = 10;

    double step_x = range_x / num_grid_x;
    double step_y = range_y / num_grid_y;

    // 绘制垂直线
    for (int i = 0; i <= num_grid_x; ++i) {
        double x = min_x + i * step_x;
        cv::Point p1 = transform(x, min_y);
        cv::Point p2 = transform(x, max_y);
        cv::line(image, p1, p2, cv::Scalar(240, 240, 240), 1);
    }

    // 绘制水平线
    for (int i = 0; i <= num_grid_y; ++i) {
        double y = min_y + i * step_y;
        cv::Point p1 = transform(min_x, y);
        cv::Point p2 = transform(max_x, y);
        cv::line(image, p1, p2, cv::Scalar(240, 240, 240), 1);
    }
}

// 添加坐标轴信息
void addCoordinateInfo(cv::Mat& image, std::function<cv::Point(double, double)> transform,
                       double min_x, double max_x, double min_y, double max_y) {
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.3;
    int thickness = 1;

    // 显示坐标范围
    std::string info = "X: [" + std::to_string(min_x).substr(0, 6) + ", " +
                       std::to_string(max_x).substr(0, 6) + "]  Y: [" +
                       std::to_string(min_y).substr(0, 6) + ", " +
                       std::to_string(max_y).substr(0, 6) + "]";

    cv::putText(image, info, cv::Point(10, 20), font_face, font_scale, cv::Scalar(100, 100, 100), thickness);
}

// 添加组合图例
void addCombinedLegend(cv::Mat& image, int num_lines, int num_arcs) {
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.35;
    int thickness = 1;
    int y_offset = 40;

    // 直线焊缝图例
    cv::line(image, cv::Point(10, y_offset + 5), cv::Point(40, y_offset + 5), cv::Scalar(0, 0, 255), 2);
    cv::putText(image, "Line Weld (L" + std::to_string(num_lines) + ")",
                cv::Point(50, y_offset + 10), font_face, font_scale, cv::Scalar(0, 0, 0), thickness);

    // 圆弧焊缝图例
    cv::line(image, cv::Point(10, y_offset + 25), cv::Point(40, y_offset + 25), cv::Scalar(255, 100, 0), 2);
    cv::putText(image, "Arc Weld (A" + std::to_string(num_arcs) + ")",
                cv::Point(50, y_offset + 30), font_face, font_scale, cv::Scalar(0, 0, 0), thickness);

    // 起点标记图例
    cv::circle(image, cv::Point(15, y_offset + 50), 4, cv::Scalar(0, 255, 0), -1);
    cv::putText(image, "Start Point", cv::Point(50, y_offset + 55), font_face, font_scale, cv::Scalar(0, 0, 0), thickness);

    // 终点标记图例
    cv::circle(image, cv::Point(15, y_offset + 70), 4, cv::Scalar(0, 0, 255), -1);
    cv::putText(image, "End Point", cv::Point(50, y_offset + 75), font_face, font_scale, cv::Scalar(0, 0, 0), thickness);

    // 圆心标记图例
    cv::circle(image, cv::Point(15, y_offset + 90), 3, cv::Scalar(255, 0, 0), -1);
    cv::putText(image, "Arc Center", cv::Point(50, y_offset + 95), font_face, font_scale, cv::Scalar(0, 0, 0), thickness);

    // 中点标记图例
    drawDiamond(image, cv::Point(25, y_offset + 115), 4, cv::Scalar(0, 255, 255), cv::Scalar(0, 100, 100));
    cv::putText(image, "Arc Middle Point", cv::Point(50, y_offset + 120), font_face, font_scale, cv::Scalar(0, 0, 0), thickness);
}

// 可视化焊缝和圆弧（正确处理负坐标）
void visualizeWeldSeamsAndArcs(const std::vector<Point2D>& points,
                               const std::vector<WeldSeam2D>& seams,
                               const std::vector<WeldArc2D>& arcs,
                               QVector<QLineF>* seam_hit_lines = nullptr) {
    if (points.empty()) {
        qDebug() << "点云为空，无法可视化";
        return;
    }

    if (seam_hit_lines) {
        seam_hit_lines->clear();
    }

    // 1. 计算点云的边界范围
    double min_x = points[0].x, max_x = points[0].x;
    double min_y = points[0].y, max_y = points[0].y;

    for (const auto& p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    // 同时考虑焊缝端点的范围
    for (const auto& seam : seams) {
        min_x = std::min({min_x, seam.start.x, seam.end.x});
        max_x = std::max({max_x, seam.start.x, seam.end.x});
        min_y = std::min({min_y, seam.start.y, seam.end.y});
        max_y = std::max({max_y, seam.start.y, seam.end.y});
    }

    // 考虑圆弧的范围
    for (const auto& arc : arcs) {
        min_x = std::min({min_x, arc.start.x, arc.middle.x, arc.end.x});
        max_x = std::max({max_x, arc.start.x, arc.middle.x, arc.end.x});
        min_y = std::min({min_y, arc.start.y, arc.middle.y, arc.end.y});
        max_y = std::max({max_y, arc.start.y, arc.middle.y, arc.end.y});

        // 考虑圆弧可能的外接矩形范围
        double cx = arc.arc.cx;
        double cy = arc.arc.cy;
        double r = arc.arc.radius;
        min_x = std::min(min_x, cx - r);
        max_x = std::max(max_x, cx + r);
        min_y = std::min(min_y, cy - r);
        max_y = std::max(max_y, cy + r);
    }

    // 添加边距（10%）
    double margin_x = (max_x - min_x) * 0.1;
    double margin_y = (max_y - min_y) * 0.1;
    min_x -= margin_x;
    max_x += margin_x;
    min_y -= margin_y;
    max_y += margin_y;

    // 2. 计算缩放比例（图像大小 800x800）
    const int image_size = 800;
    double scale_x = image_size / (max_x - min_x);
    double scale_y = 600 / (max_y - min_y);
    double scale = std::min(scale_x, scale_y);  // 保持纵横比

    // 重新计算边界以保持纵横比
    double width = (max_x - min_x) * scale;
    double height = (max_y - min_y) * scale;
    double offset_x = (image_size - width) / 2;
    double offset_y = (image_size - height) / 2;

    // 3. 坐标转换函数（世界坐标 -> 图像坐标）
    auto transform = [&](double x, double y) -> cv::Point {
        int img_x = int((x - min_x) * scale + offset_x);
        int img_y = int((y - min_y) * scale + offset_y);  // 注意Y轴翻转
        return cv::Point(img_x, img_y);
    };

    // 4. 创建白色背景图像
    cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));

    // 5. 绘制网格（可选，便于观察坐标）
    // drawGrid(image, transform, min_x, max_x, min_y, max_y);

    // 6. 绘制所有边界点
    for (const auto& p : points) {
        cv::circle(image, transform(p.x, p.y), 2, cv::Scalar(200, 200, 200), -1);
    }

    // 7. 绘制直线焊缝
    std::vector<cv::Scalar> line_colors = {
        cv::Scalar(0, 0, 255),    // 红色
    };

    // 8. 绘制圆弧焊缝（使用不同的颜色组）
    std::vector<cv::Scalar> arc_colors = {
        cv::Scalar(0, 100, 255),   // 橙色
    };

    // 绘制直线焊缝
    for (size_t i = 0; i < seams.size(); ++i) {
        const auto& seam = seams[i];
        cv::Scalar color = line_colors[i % line_colors.size()];

        cv::Point start_pt = transform(seam.start.x, seam.start.y);
        cv::Point end_pt = transform(seam.end.x, seam.end.y);
        cv::Point center_pt = transform((seam.start.x+seam.end.x)/2+1, (seam.start.y+seam.end.y)/2);

        if (seam_hit_lines) {
            seam_hit_lines->append(
                QLineF(QPointF(start_pt.x, start_pt.y),
                       QPointF(end_pt.x, end_pt.y)));
        }

        // 绘制焊缝线段（加粗）
        cv::line(image, start_pt, end_pt, color, 3);

        // 绘制起点（绿色圆）
        cv::circle(image, start_pt, 6, cv::Scalar(0, 255, 0), -1);
        cv::circle(image, start_pt, 6, cv::Scalar(0, 100, 0), 2);

        // 绘制终点（红色圆）
        cv::circle(image, end_pt, 6, cv::Scalar(0, 0, 255), -1);
        cv::circle(image, end_pt, 6, cv::Scalar(0, 0, 100), 2);

        // 标注序号（带背景框）
        std::string text =std::to_string(i + 1);
        drawTextWithBackground(image, center_pt, text, color);
    }

    // 绘制圆弧焊缝
    for (size_t i = 0; i < arcs.size(); ++i) {
        const auto& arc = arcs[i];
        cv::Scalar color = arc_colors[i % arc_colors.size()];

        cv::Point start_pt = transform(arc.start.x, arc.start.y);
        cv::Point middle_pt = transform(arc.middle.x, arc.middle.y);
        cv::Point end_pt = transform(arc.end.x, arc.end.y);
        cv::Point center_pt = transform(arc.arc.cx, arc.arc.cy);

        // 绘制圆弧（通过采样点）
        drawArcOnImage(image, arc.arc, transform, color, 3);

        // 绘制圆心（蓝色小圆点）
        cv::circle(image, center_pt, 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(image, center_pt, 5, cv::Scalar(100, 0, 0), 1);

        // 绘制圆心到中点的连线（虚线效果，表示半径）
        cv::line(image, center_pt, middle_pt, cv::Scalar(200, 200, 200), 1, cv::LINE_4);

        // 绘制起点（绿色菱形）
        drawDiamond(image, start_pt, 6, cv::Scalar(0, 255, 0), cv::Scalar(0, 100, 0));

        // 绘制中间点（黄色菱形）
        drawDiamond(image, middle_pt, 6, cv::Scalar(0, 255, 255), cv::Scalar(0, 100, 100));

        // 绘制终点（红色菱形）
        drawDiamond(image, end_pt, 6, cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 100));

        // 标注序号
        std::string text = "A" + std::to_string(i + 1);
        // 计算文字位置（在圆弧中点附近沿法线方向偏移）
        cv::Point text_pt = calculateTextPositionForArc(arc, transform);
        drawTextWithBackground(image, text_pt, text, color);

    }

    // 10. 显示图像
    // cv::namedWindow("Weld Seams and Arcs Detection", cv::WINDOW_NORMAL);
    // cv::resizeWindow("Weld Seams and Arcs Detection", 800, 600);
    // cv::imshow("Weld Seams and Arcs Detection", image);

    // 11. 保存图像（可选）
    cv::imwrite("weld_seams_and_arcs_result.png", image);
    qDebug() << "可视化结果已保存到 weld_seams_and_arcs_result.png";

    cv::waitKey(0);
}

// 已有的可视化函数（向后兼容）
void visualizeWeldSeams(const std::vector<Point2D>& points,
                        const std::vector<WeldSeam2D>& seams) {
    std::vector<WeldArc2D> empty_arcs;
    visualizeWeldSeamsAndArcs(points, seams, empty_arcs);
}


void OpenCVSet::on_pushButton_26_clicked()
{
    QString path = QDir::currentPath();
    QString cloudPath = path + "/cloud_fitle.pcd";

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(cloudPath.toStdString(), cloud) == -1) {
        addlog("点云加载失败!");
        return;
    }

    if (cloud.empty()) {
        addlog("点云为空!");
        return;
    }
    // 功能1: 提取直线和圆弧
    auto [seams, arcs] = WeldSeamExtractor::extractSeamsAndArcs(
        cloud,
        2.0,    // distance_threshold
        150,    // min_inliers
        600,    // max_iterations
        50,     // min_points_to_fit
        5.0,    // max_gap
        1.0,    // arc_dist_threshold
        150,    // min_arc_inliers
        100.0,   // arc_min_radius
        200.0,   // arc_max_radius
        0.35,   // arc_outlier_ratio
        0.01,   // arc_angle_tolerance
        2       // arc_min_continuous
        );
    arcs.clear();
    std::vector<Point2D> points_2d;
    for (const auto& point : cloud.points) {
        // 根据你的投影平面选择坐标
        // 如果是投影到XY平面（Z=0）
        points_2d.emplace_back(point.x, point.y);
    }
    qDebug() << "共检测到" << seams.size() << "条直焊缝";
    qDebug() <<"检测到"<<arcs.size()<<"条圆弧";

    if (seams.empty()) {
        QMessageBox::warning(this, "提示", "未检测到可选择的直线焊缝任务");
        return;
    }


    QVector<QLineF> seam_hit_lines;
    visualizeWeldSeamsAndArcs(points_2d, seams, arcs, &seam_hit_lines);

    QString pic_path = path + "/weld_seams_and_arcs_result.png";

    QStringList seam_items;
    for (int i = 0; i < static_cast<int>(seams.size()); ++i) {
        const WeldSeam2D& seam = seams[i];
        seam_items.append(
            QString("焊缝 %1  长度 %2 mm  (%3, %4) → (%5, %6)")
                .arg(i + 1)
                .arg(seam.length, 0, 'f', 1)
                .arg(seam.start.x, 0, 'f', 1)
                .arg(seam.start.y, 0, 'f', 1)
                .arg(seam.end.x, 0, 'f', 1)
                .arg(seam.end.y, 0, 'f', 1));
    }

    DelectSeam ds(this);
    ds.addpic(pic_path);
    ds.setSeamItems(seam_items);
    ds.setSeamHitLines(seam_hit_lines);
    ds.setArcDeletionVisible(false);

    if (ds.exec() != QDialog::Accepted) {
        addlog("已取消焊缝任务选择");
        return;
    }

    const QVector<int> selected_indices = ds.selectedSeamIndices();
    if (selected_indices.isEmpty()) {
        QMessageBox::warning(this, "提示", "请至少选择一条焊缝任务");
        return;
    }

    std::vector<WeldSeam2D> selected_seams;
    selected_seams.reserve(selected_indices.size());
    for (int index : selected_indices) {
        if (index >= 0 && index < static_cast<int>(seams.size())) {
            selected_seams.push_back(seams[index]);
        }
    }

    seams = std::move(selected_seams);
    addlog(QString("已选择 %1 条焊缝任务").arg(seams.size()));

    std::vector<std::vector<pcl::PointXYZ>> arc_clouds;
    auto result3d = WeldSeamExtractor::convertAndGroupTo3D(
        seams, arcs, points_2d, z0, axis, angle,
        20.0, 10.0, 20.0, 20.0, &arc_clouds, 30,
        true);

    //平焊缝手眼转换
    for(int i=0;i<result3d.grouped_seams.size();i++)
    {
        for(int j=0;j<result3d.grouped_seams[i].size();j++)
        {
            cv::Point3f p_in(result3d.grouped_seams[i][j].start.x,result3d.grouped_seams[i][j].start.y,result3d.grouped_seams[i][j].start.z);
            qDebug()<<p_in.x<<","<<p_in.y<<","<<p_in.z;
            cv::Point3f p_out;
            calibrator->transformCameraToBase("./handtoeye.yaml",p_in,p_out);
            qDebug()<<p_out.x<<","<<p_out.y<<","<<p_out.z;

            cv::Point3f p_in2(result3d.grouped_seams[i][j].end.x,result3d.grouped_seams[i][j].end.y,result3d.grouped_seams[i][j].end.z);
            qDebug()<<p_in2.x<<","<<p_in2.y<<","<<p_in2.z;
            cv::Point3f p_out2;
            calibrator->transformCameraToBase("./handtoeye.yaml",p_in2,p_out2);
            qDebug()<<p_out2.x<<","<<p_out2.y<<","<<p_out2.z;

            result3d.grouped_seams[i][j].start.x=p_out.x;
            result3d.grouped_seams[i][j].start.y=p_out.y;
            result3d.grouped_seams[i][j].start.z=p_out.z;
            result3d.grouped_seams[i][j].end.x=p_out2.x;
            result3d.grouped_seams[i][j].end.y=p_out2.y;
            result3d.grouped_seams[i][j].end.z=p_out2.z;
        }
    }

    //立焊缝手眼转换
    for(int i=0;i<result3d.vertical_seams.size();i++)
    {
        cv::Point3f p_in(result3d.vertical_seams[i].start.x,result3d.vertical_seams[i].start.y,result3d.vertical_seams[i].start.z);
        cv::Point3f p_out;
        calibrator->transformCameraToBase("./handtoeye.yaml",p_in,p_out);
        cv::Point3f p_in2(result3d.vertical_seams[i].end.x,result3d.vertical_seams[i].end.y,result3d.vertical_seams[i].end.z);
        cv::Point3f p_out2;
        calibrator->transformCameraToBase("./handtoeye.yaml",p_in,p_out);
        result3d.vertical_seams[i].start.x=p_out.x;
        result3d.vertical_seams[i].start.y=p_out.y;
        result3d.vertical_seams[i].start.z=p_out.z;
        result3d.vertical_seams[i].end.x=p_out2.x;
        result3d.vertical_seams[i].end.y=p_out2.y;
        result3d.vertical_seams[i].end.z=p_out2.z;
    }


    // 功能3: 生成JSON文件
    bool success = WeldSeamExtractor::generateJSON(
        result3d.grouped_seams,
        result3d.grouped_seams_with_arc,
        result3d.grouped_arcs,
        result3d.vertical_seams,
        "weld_seams_output.json",
        WeldType::FILLET,
        WeldPosition::FLAT,
        1.0
        );

    if (success) {
        addlog("JSON文件生成成功!");
    } else {
        addlog("JSON文件生成失败!");
    }
    weldSearch->process_welding_system("weld_seams_output.json","./weldseam");


    //焊缝展示
    QString cloudPath2 = path+"/cloud.pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    if (pcl::io::loadPCDFile(cloudPath2.toStdString(), cloud2) == -1)
    {
        QMessageBox::warning(this, "警告", "点云加载失败!");
        return;
    }
    if (cloud2.empty())
    {
        QMessageBox::warning(this, "警告", "点云为空!");
        return;
    }

    if (viewer->contains("cloud_display"))  // 检查是否存在指定ID的点云
    {
        viewer->removePointCloud("cloud_display");  // 移除旧的
        viewer->removeAllShapes();
        qDebug() << "已移除旧点云";
    }
    viewer->addPointCloud(cloud2.makeShared(),"cloud_display");

    int a=0;
    for(int i = 0; i < result3d.grouped_seams.size(); i++)
    {
        for(int j=0;j<result3d.grouped_seams[i].size();j++)
        {
            pcl::PointXYZ start_point(result3d.grouped_seams[i][j].start.x,result3d.grouped_seams[i][j].start.y,result3d.grouped_seams[i][j].start.z);
            pcl::PointXYZ end_point(result3d.grouped_seams[i][j].end.x,result3d.grouped_seams[i][j].end.y,result3d.grouped_seams[i][j].end.z);
            viewer->addLine(start_point, end_point, 255.0, 0.0, 0.0, "line_id_"+QString::number(a).toStdString());
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5, "line_id_"+QString::number(a).toStdString());
            a++;
        }

    }
    for(int i = 0; i < result3d.vertical_seams.size(); i++)
    {
        viewer->addLine(result3d.vertical_seams[i].start, result3d.vertical_seams[i].end, 255.0, 0.0, 0.0, "line_id_"+QString::number(a).toStdString());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5, "line_id_"+QString::number(a).toStdString());
        a++;
    }

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1,
        "cloud_display"  // 这里必须和上面的ID一致
        );

    ui->widget->renderWindow()->Render();

}


void mergePointClouds(QString path1, QString path2,int i)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile(path1.toStdString(), *cloud1);
    pcl::io::loadPCDFile(path2.toStdString(), *cloud2);

    qDebug() << "Original cloud1 size:" << cloud1->size();
    qDebug() << "Original cloud2 size:" << cloud2->size();

    // 1. 计算点云1的Y值范围
    if (cloud1->empty()) {
        qDebug() << "Cloud1 is empty!";
        return;
    }

    float minX = cloud1->points[0].x;
    float maxX = cloud1->points[0].x;

    for (const auto& point : cloud1->points) {
        if (point.x < minX) minX = point.x;
        if (point.x > maxX) maxX = point.x;
    }

    qDebug() << "Cloud1 Y range: [" << minX << ", " << maxX << "]";

    // 2. 过滤点云2，移除Y值在点云1范围内的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    int removedCount = 0;

    for (const auto& point : cloud2->points) {
        // 如果Y值不在点云1的范围内，则保留
        if (point.x < minX+i || point.x > maxX-i) {
            filteredCloud2->push_back(point);
        } else {
            removedCount++;
        }
    }

    qDebug() << "Filtered cloud2 size:" << filteredCloud2->size();
    qDebug() << "Removed points in Y range:" << removedCount;

    // 3. 创建合并后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
    *merged = *cloud1;  // 先拷贝第一个

    // 4. 分批添加过滤后的第二个点云
    int batchSize = 10000;
    for (size_t i = 0; i < filteredCloud2->size(); i += batchSize) {
        size_t end = std::min(i + batchSize, filteredCloud2->size());
        for (size_t j = i; j < end; ++j) {
            merged->push_back(filteredCloud2->points[j]);
        }
    }

    qDebug() << "Merged size:" << merged->size();

    // 5. 保存最终结果
    QString cloudPath = QDir::currentPath() + "/cloud.pcd";
    if (pcl::io::savePCDFileBinary(cloudPath.toStdString(), *merged) == 0) {
        qDebug() << "Save successful!";
    } else {
        qDebug() << "Save failed, trying ASCII...";
        if (pcl::io::savePCDFile(cloudPath.toStdString(), *merged) == 0) {
            qDebug() << "ASCII save successful!";
        }
    }
}


void OpenCVSet::on_pushButton_27_clicked()
{
    if (calibrator) {
        calibrator->transformCam2ToCam1("./cloud1.pcd",
                                        "./cloud1_0.pcd",
                                        "./cam1_to_cam.yaml");
        QString path = QDir::currentPath();
        QString cloudPath1 = path + "/cloud0.pcd";
        QString cloudPath2 = path + "/cloud1_0.pcd";
        mergePointClouds(cloudPath1,cloudPath2,ui->lineEdit->text().toInt()/6);
        QMessageBox::information(this, "完成", "拼接完成！");
    } else {
        QMessageBox::warning(this, "错误", "标定器未初始化");
    }
}


void OpenCVSet::on_pushButton_24_clicked()
{
    std::string folder_path;
    handtoeyesnumber=0;

    folder_path = "./EyesToHand/";

    // 遍历文件夹里所有文件
    for (const auto& entry : std::filesystem::directory_iterator(folder_path))
    {
        QString path = QString::fromStdString(entry.path().string());
        // 获取文件路径
        std::filesystem::remove(entry.path());

        addlog("图片已删除："+path);
    }
    addlog("图片已删除完成，现在可以开始重新标定");
}


void OpenCVSet::on_pushButton_28_clicked()
{
    if (!camera_is_open) {
        addlog("相机未连接，无法执行拍照操作");
        return;
    }
    QString path1;
    std::string path;
    path1="./EyesToHand";
    path ="./EyesToHand/" + QString("%1").arg(handtoeyesnumber, 4, 10, QChar('0')).toStdString() + ".png";
    OpenCVSet::takePhoto2(path1,0,handtoeyesnumber);
    ui->pic_number_lab->setText(QString::number(handtoeyesnumber));
    handtoeyesnumber++;

    // 显示图片
    ui->piclabel->clear();
    QString absolutePath = QDir::current().absoluteFilePath(QString::fromStdString(path));
    QPixmap originalPixmap(absolutePath);

    if (!originalPixmap.isNull())
    {
        ui->piclabel->setPixmap(originalPixmap);
        ui->piclabel->setScaledContents(true);
        ui->piclabel->update();
        addlog("图片显示成功");
    }
    else
    {
        addlog("QPixmap加载图片失败");
    }
}


struct PointPair {
    cv::Point2f pixel;      // 图像中的亚像素坐标
    cv::Point3f world;      // 机器人基座/世界坐标系下的三维坐标
};

// cv::Size board_size(11, 8);
void OpenCVSet::on_pushButton_29_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }

    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("外部轴位置读取失败");
        return;
    }

    x_b=pJoint[7];
    y_b=pJoint[6];

    // ============================================================
    // 参数配置
    // ============================================================
    std::string cameraIntrinFile = "camera_calib.yaml";
    std::string imagePathLow = "./EyesToHand/0000.png";    // 低位图片
    std::string imagePathHigh = "./EyesToHand/0001.png";  // 高位图片（垫高30mm）
    std::string savePath = "handtoeye.yaml";

    // ============================================================
    // 定义角点索引（6个点，分布均匀覆盖棋盘格）
    // 9x6 棋盘格，索引 = row * 9 + col
    // ============================================================
    std::vector<int> selectedCorners = {
        0,          // (0,0)   左上角
        10,          // (0,10)   右上角
        77,         // (7,0)   左下角
        87         // (7,10)   右下角
    };

    std::vector<cv::Point3f> robotPointsLow = {
        cv::Point3f(ui->p1_x1->text().toDouble(), ui->p1_y1->text().toDouble(), ui->p1_z1->text().toDouble()),   // 左上角
        cv::Point3f(ui->p2_x1->text().toDouble(), ui->p2_y1->text().toDouble(), ui->p2_z1->text().toDouble()),   // 右上角
        cv::Point3f(ui->p3_x1->text().toDouble(), ui->p3_y1->text().toDouble(), ui->p3_z1->text().toDouble()),   // 左下角
        cv::Point3f(ui->p4_x1->text().toDouble(), ui->p4_y1->text().toDouble(), ui->p4_z1->text().toDouble()),   // 右下角
    };

    std::vector<cv::Point3f> robotPointsHigh = {
        cv::Point3f(ui->p1_x2->text().toDouble(), ui->p1_y2->text().toDouble(), ui->p1_z2->text().toDouble()),   // 左上角
        cv::Point3f(ui->p2_x2->text().toDouble(), ui->p2_y2->text().toDouble(), ui->p2_z2->text().toDouble()),   // 右上角
        cv::Point3f(ui->p3_x2->text().toDouble(), ui->p3_y2->text().toDouble(), ui->p3_z2->text().toDouble()),   // 左下角
        cv::Point3f(ui->p4_x2->text().toDouble(), ui->p4_y2->text().toDouble(), ui->p4_z2->text().toDouble()),   // 右下角
    };

    // ============================================================
    // 执行双高度标定
    // ============================================================
    bool success = calibrator->calibrateHandToEyeTwoHeights(
        cameraIntrinFile,
        imagePathLow,
        imagePathHigh,
        savePath,
        board_size,
        selectedCorners,
        robotPointsLow,
        robotPointsHigh,
        true  // 显示可视化
        );

    if (success) {
        addlog("标定成功！");
    } else {
        addlog("标定失败！");
    }
}

//连接机械臂
void OpenCVSet::on_pushButton_38_clicked()
{
    if(!robot_is_Init)
    {
        int errorCode = RobotInit();
        if(errorCode == 0)
        {
            robot_is_Init=true;
            addlog("机械臂接口初始化成功");
        }
        else
        {
            addlog("机械臂接口初始化失败");
            return;
        }
    }
    const char* connectIP = "192.168.3.150";
    short port = 9999;
    connectID= ConnectUDP(connectIP,port);
    if(connectID>0)
    {
        robot_is_connect = true;
        ui->robotlabel->setText("已连接");
        ui->robotlabel->setStyleSheet("color: rgb(0, 255, 0);");
        QMessageBox::information(this,"提示","机械臂连接成功");
        addlog("机械臂连接成功");
        positionTimer->start();
    }
    else
    {
        QMessageBox::information(this,"提示","机械臂连接失败");
        addlog("机械臂连接失败");
    }

}


void OpenCVSet::on_pushButton_39_clicked()
{
    if(!robot_is_connect)
    {
        QMessageBox::information(this,"提示","请先连接机械臂");
        return;
    }
    int errorCode1 = ClearAlarm(connectID);
    if (errorCode1 == 0)
    {
        addlog("报警清除成功");
    }
}


void OpenCVSet::on_pushButton_35_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接！");
        return;
    }

    RPY rpy;
    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("机器人关节坐标读取失败，请检查机械臂连接状态");
        return;
    }

    BYTE ToolIndex = 1; //要查询的工具坐标系号
    float Px,Py,Pz;
    float Rx,Ry,Rz;
    int errorCode2 = ReadTool(ToolIndex,Px,Py,Pz,Rx,Ry,Rz,connectID);
    if(errorCode2!=0)
    {
        addlog("机器人工具坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    RPY toolCoor;
    toolCoor.px=Px;
    toolCoor.py=Py;
    toolCoor.pz=Pz;
    toolCoor.rx=Rx;
    toolCoor.ry=Ry;
    toolCoor.rz=Rz;

    BYTE userIndex = 1;
    RPY userCoord;
    int errorCode3= ReadUserCoord_RPY(userIndex, &userCoord, connectID);
    if(errorCode3!=0)
    {
        addlog("机器人用户坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    int errorCode4 = Joint2Pos(pJoint, &rpy, toolCoor, userCoord, connectID);
    if(errorCode4!=0)
    {
        addlog("机器人坐标系转换失败，请检查机械臂连接状态");
        return;
    }

    ui->p1_x1->setText(QString::number(rpy.px));
    ui->p1_y1->setText(QString::number(rpy.py));
    ui->p1_z1->setText(QString::number(rpy.pz));
    addlog("机械臂TCP坐标接收成功");
}

void OpenCVSet::on_pushButton_34_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    RPY rpy;
    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("机器人关节坐标读取失败，请检查机械臂连接状态");
        return;
    }

    BYTE ToolIndex = 1; //要查询的工具坐标系号
    float Px,Py,Pz;
    float Rx,Ry,Rz;
    int errorCode2 = ReadTool(ToolIndex,Px,Py,Pz,Rx,Ry,Rz,connectID);
    if(errorCode2!=0)
    {
        addlog("机器人工具坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    RPY toolCoor;
    toolCoor.px=Px;
    toolCoor.py=Py;
    toolCoor.pz=Pz;
    toolCoor.rx=Rx;
    toolCoor.ry=Ry;
    toolCoor.rz=Rz;

    BYTE userIndex = 1;
    RPY userCoord;
    int errorCode3= ReadUserCoord_RPY(userIndex, &userCoord, connectID);
    if(errorCode3!=0)
    {
        addlog("机器人用户坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    int errorCode4 = Joint2Pos(pJoint, &rpy, toolCoor, userCoord, connectID);
    if(errorCode4!=0)
    {
        addlog("机器人坐标系转换失败，请检查机械臂连接状态");
        return;
    }
    ui->p2_x1->setText(QString::number(rpy.px));
    ui->p2_y1->setText(QString::number(rpy.py));
    ui->p2_z1->setText(QString::number(rpy.pz));
    addlog("机械臂TCP坐标接收成功");
}

void OpenCVSet::on_pushButton_36_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    RPY rpy;
    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("机器人关节坐标读取失败，请检查机械臂连接状态");
        return;
    }

    BYTE ToolIndex = 1; //要查询的工具坐标系号
    float Px,Py,Pz;
    float Rx,Ry,Rz;
    int errorCode2 = ReadTool(ToolIndex,Px,Py,Pz,Rx,Ry,Rz,connectID);
    if(errorCode2!=0)
    {
        addlog("机器人工具坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    RPY toolCoor;
    toolCoor.px=Px;
    toolCoor.py=Py;
    toolCoor.pz=Pz;
    toolCoor.rx=Rx;
    toolCoor.ry=Ry;
    toolCoor.rz=Rz;

    BYTE userIndex = 1;
    RPY userCoord;
    int errorCode3= ReadUserCoord_RPY(userIndex, &userCoord, connectID);
    if(errorCode3!=0)
    {
        addlog("机器人用户坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    int errorCode4 = Joint2Pos(pJoint, &rpy, toolCoor, userCoord, connectID);
    if(errorCode4!=0)
    {
        addlog("机器人坐标系转换失败，请检查机械臂连接状态");
        return;
    }
    ui->p3_x1->setText(QString::number(rpy.px));
    ui->p3_y1->setText(QString::number(rpy.py));
    ui->p3_z1->setText(QString::number(rpy.pz));
    addlog("机械臂TCP坐标读取成功");
}

void OpenCVSet::on_pushButton_37_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    RPY rpy;
    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("机器人关节坐标读取失败，请检查机械臂连接状态");
        return;
    }

    BYTE ToolIndex = 1; //要查询的工具坐标系号
    float Px,Py,Pz;
    float Rx,Ry,Rz;
    int errorCode2 = ReadTool(ToolIndex,Px,Py,Pz,Rx,Ry,Rz,connectID);
    if(errorCode2!=0)
    {
        addlog("机器人工具坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    RPY toolCoor;
    toolCoor.px=Px;
    toolCoor.py=Py;
    toolCoor.pz=Pz;
    toolCoor.rx=Rx;
    toolCoor.ry=Ry;
    toolCoor.rz=Rz;

    BYTE userIndex = 1;
    RPY userCoord;
    int errorCode3= ReadUserCoord_RPY(userIndex, &userCoord, connectID);
    if(errorCode3!=0)
    {
        addlog("机器人用户坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    int errorCode4 = Joint2Pos(pJoint, &rpy, toolCoor, userCoord, connectID);
    if(errorCode4!=0)
    {
        addlog("机器人坐标系转换失败，请检查机械臂连接状态");
        return;
    }
    ui->p4_x1->setText(QString::number(rpy.px));
    ui->p4_y1->setText(QString::number(rpy.py));
    ui->p4_z1->setText(QString::number(rpy.pz));
    addlog("机械臂TCP坐标获取成功");

}

void OpenCVSet::on_pushButton_30_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    RPY rpy;
    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("机器人关节坐标读取失败，请检查机械臂连接状态");
        return;
    }

    BYTE ToolIndex = 1; //要查询的工具坐标系号
    float Px,Py,Pz;
    float Rx,Ry,Rz;
    int errorCode2 = ReadTool(ToolIndex,Px,Py,Pz,Rx,Ry,Rz,connectID);
    if(errorCode2!=0)
    {
        addlog("机器人工具坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    RPY toolCoor;
    toolCoor.px=Px;
    toolCoor.py=Py;
    toolCoor.pz=Pz;
    toolCoor.rx=Rx;
    toolCoor.ry=Ry;
    toolCoor.rz=Rz;

    BYTE userIndex = 1;
    RPY userCoord;
    int errorCode3= ReadUserCoord_RPY(userIndex, &userCoord, connectID);
    if(errorCode3!=0)
    {
        addlog("机器人用户坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    int errorCode4 = Joint2Pos(pJoint, &rpy, toolCoor, userCoord, connectID);
    if(errorCode4!=0)
    {
        addlog("机器人坐标系转换失败，请检查机械臂连接状态");
        return;
    }
    ui->p1_x2->setText(QString::number(rpy.px));
    ui->p1_y2->setText(QString::number(rpy.py));
    ui->p1_z2->setText(QString::number(rpy.pz));
    addlog("机械臂TCP坐标获取成功");

}

void OpenCVSet::on_pushButton_31_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    RPY rpy;
    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("机器人关节坐标读取失败，请检查机械臂连接状态");
        return;
    }

    BYTE ToolIndex = 1; //要查询的工具坐标系号
    float Px,Py,Pz;
    float Rx,Ry,Rz;
    int errorCode2 = ReadTool(ToolIndex,Px,Py,Pz,Rx,Ry,Rz,connectID);
    if(errorCode2!=0)
    {
        addlog("机器人工具坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    RPY toolCoor;
    toolCoor.px=Px;
    toolCoor.py=Py;
    toolCoor.pz=Pz;
    toolCoor.rx=Rx;
    toolCoor.ry=Ry;
    toolCoor.rz=Rz;

    BYTE userIndex = 1;
    RPY userCoord;
    int errorCode3= ReadUserCoord_RPY(userIndex, &userCoord, connectID);
    if(errorCode3!=0)
    {
        addlog("机器人用户坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    int errorCode4 = Joint2Pos(pJoint, &rpy, toolCoor, userCoord, connectID);
    if(errorCode4!=0)
    {
        addlog("机器人坐标系转换失败，请检查机械臂连接状态");
        return;
    }
    ui->p2_x2->setText(QString::number(rpy.px));
    ui->p2_y2->setText(QString::number(rpy.py));
    ui->p2_z2->setText(QString::number(rpy.pz));
    addlog("机械臂TCP坐标获取成功");
}


void OpenCVSet::on_pushButton_32_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    RPY rpy;
    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("机器人关节坐标读取失败，请检查机械臂连接状态");
        return;
    }

    BYTE ToolIndex = 1; //要查询的工具坐标系号
    float Px,Py,Pz;
    float Rx,Ry,Rz;
    int errorCode2 = ReadTool(ToolIndex,Px,Py,Pz,Rx,Ry,Rz,connectID);
    if(errorCode2!=0)
    {
        addlog("机器人工具坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    RPY toolCoor;
    toolCoor.px=Px;
    toolCoor.py=Py;
    toolCoor.pz=Pz;
    toolCoor.rx=Rx;
    toolCoor.ry=Ry;
    toolCoor.rz=Rz;

    BYTE userIndex = 1;
    RPY userCoord;
    int errorCode3= ReadUserCoord_RPY(userIndex, &userCoord, connectID);
    if(errorCode3!=0)
    {
        addlog("机器人用户坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    int errorCode4 = Joint2Pos(pJoint, &rpy, toolCoor, userCoord, connectID);
    if(errorCode4!=0)
    {
        addlog("机器人坐标系转换失败，请检查机械臂连接状态");
        return;
    }
    ui->p3_x2->setText(QString::number(rpy.px));
    ui->p3_y2->setText(QString::number(rpy.py));
    ui->p3_z2->setText(QString::number(rpy.pz));
    addlog("机械臂TCP坐标获取成功");
}


void OpenCVSet::on_pushButton_33_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    RPY rpy;
    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("机器人关节坐标读取失败，请检查机械臂连接状态");
        return;
    }

    BYTE ToolIndex = 1; //要查询的工具坐标系号
    float Px,Py,Pz;
    float Rx,Ry,Rz;
    int errorCode2 = ReadTool(ToolIndex,Px,Py,Pz,Rx,Ry,Rz,connectID);
    if(errorCode2!=0)
    {
        addlog("机器人工具坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    RPY toolCoor;
    toolCoor.px=Px;
    toolCoor.py=Py;
    toolCoor.pz=Pz;
    toolCoor.rx=Rx;
    toolCoor.ry=Ry;
    toolCoor.rz=Rz;

    BYTE userIndex = 1;
    RPY userCoord;
    int errorCode3= ReadUserCoord_RPY(userIndex, &userCoord, connectID);
    if(errorCode3!=0)
    {
        addlog("机器人用户坐标系读取失败，请检查机械臂连接状态");
        return;
    }

    int errorCode4 = Joint2Pos(pJoint, &rpy, toolCoor, userCoord, connectID);
    if(errorCode4!=0)
    {
        addlog("机器人坐标系转换失败，请检查机械臂连接状态");
        return;
    }
    ui->p4_x2->setText(QString::number(rpy.px));
    ui->p4_y2->setText(QString::number(rpy.py));
    ui->p4_z2->setText(QString::number(rpy.pz));
    addlog("机械臂TCP坐标获取成功");

}



// 定义焊缝路径结构体
struct SeamPath {
    int seam_id;
    std::vector<double> start_point;  // [x, y, z]
    std::vector<double> end_point;    // [x, y, z]
    std::vector<double> direction;    // [dx, dy, dz]
    std::vector<int> weld_position;       // ✅
    std::vector<float> angle_deg;         // ✅
    std::vector<int> start_angle;         // ✅
    std::vector<int> end_angle;
};

// 定义完整数据
struct PathData {
    std::vector<SeamPath> path;
};

// 反序列化 JSON → 结构体
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SeamPath, seam_id, start_point, end_point, direction,weld_position,angle_deg,start_angle,end_angle)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathData, path)

bool robot_is_moving=false;

void OpenCVSet::on_pushButton_40_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    std::ifstream file("./weldseam/welding_path.json");
    if (!file.is_open()) {
        addlog("无法打开文件！");
        return;
    }
    nlohmann::json data2;
    file>>data2;
    PathData data = data2.get<PathData>();
    qDebug()<<"读取出焊缝"<<data.path.size();

    robot_is_moving=true;

    QFutureWatcher<void> *watcher = new QFutureWatcher<void>(this);
    connect(watcher, &QFutureWatcher<void>::finished, this, [=]() {
        watcher->deleteLater();
        robot_is_moving=false;
    });

    QFuture<void> future = QtConcurrent::run([this,data]() {
        for(int i=0;i<data.path.size();i++)
        {
            ClearBuffer(connectID);
            robot_is_moving=true;
            if(data.path[i].weld_position[0]==0)
            {
                RPY start_safe;
                start_safe.px=data.path[i].end_point[0]+20*data.path[i].direction[1];
                start_safe.py=data.path[i].end_point[1]-20*data.path[i].direction[0];
                start_safe.pz=data.path[i].end_point[2]+130;
                start_safe.rx=-45.0;
                start_safe.ry=-6.0;
                start_safe.rz= atan2(-data.path[i].direction[1], -data.path[i].direction[0])* 180.0 / M_PI;
                if(start_safe.rz<0)
                {
                    start_safe.rz+=360;
                }
                float exAxis[6] = {-start_safe.py+400,284.358,0, 0,0,0};

                RPY start_safe2;
                start_safe2.px=start_safe.px-40*data.path[i].direction[0];
                start_safe2.py=start_safe.py-40*data.path[i].direction[1];
                start_safe2.pz=start_safe.pz;
                start_safe2.rx=-45.0;
                start_safe2.ry=-6.0;
                start_safe2.rz= start_safe.rz;
                if(start_safe2.rz<0)
                {
                    start_safe2.rz+=360;
                }
                float exAxis_2[6] = {-start_safe2.py+400,284.358,0, 0,0,0};


                RPY start;
                start.px=start_safe.px+data.path[i].direction[0]*100;
                start.py=start_safe.py+data.path[i].direction[1]*100;
                start.pz=start_safe.pz;
                start.rx=start_safe.rx;
                start.ry=start_safe.ry;
                start.rz=start_safe.rz;
                float exAxis1[6]= {-start_safe.py+400,284.358,0, 0,0,0};

                RPY end;
                end.px=data.path[i].start_point[0];
                end.py=data.path[i].start_point[1];
                end.pz=data.path[i].start_point[2]+40;
                end.rx=-45.0;
                end.ry=-6.0;
                end.rz=atan2(-data.path[i].direction[1], -data.path[i].direction[0])* 180.0 / M_PI;
                if(end.rz<0)
                {
                    end.rz+=360;
                }
                float exAxis2[6] = {-end.py+400,284.358,0,0,0,0};


                RPY end_safe;
                end_safe.px=data.path[i].start_point[0]+20*data.path[i].direction[1];
                end_safe.py=data.path[i].start_point[1]-20*data.path[i].direction[0];
                end_safe.pz=data.path[i].start_point[2]+200;
                end_safe.rx=-45.0;
                end_safe.ry=-6.0;
                end_safe.rz=end.rz;
                float exAxis3[6] = {-end.py+400,284.358,0,0,0,0};


                float X = -4*data.path[i].direction[1];
                float Y = 4*data.path[i].direction[0];
                float Z = 0;
                //末点前40mm点位计算
                float X2 = 70*data.path[i].direction[0];
                float Y2 = 70*data.path[i].direction[1];
                float Z2 = 70*data.path[i].direction[2];
                //起终点类型
                qDebug()<<1;
                int start_type=data.path[i].end_angle[0];
                int end_type=data.path[i].start_angle[0];
                qDebug()<<2;
                WriteUserVarFLOAT (1,X,connectID);
                Sleep(200);
                WriteUserVarFLOAT (2,Y,connectID);
                Sleep(200);
                WriteUserVarFLOAT (3,Z,connectID);
                Sleep(200);
                WriteUserVarFLOAT (4,X2,connectID);
                Sleep(200);
                WriteUserVarFLOAT (5,Y2,connectID);
                Sleep(200);
                WriteUserVarFLOAT (6,Z2,connectID);
                Sleep(200);
                WriteUserVarINT (1,start_type,connectID);
                Sleep(200);
                WriteUserVarINT (2,end_type,connectID);
                Sleep(200);
                WriteUserVarRPYS(11,start_safe,exAxis,2,connectID);
                Sleep(200);
                WriteUserVarRPYS(12,start,exAxis1,2,connectID);
                Sleep(200);
                WriteUserVarRPYS(13,end,exAxis2,2,connectID);
                Sleep(200);
                WriteUserVarRPYS(14,end_safe,exAxis3,2,connectID);
                Sleep(200);
                WriteUserVarRPYS(10,start_safe2,exAxis_2,2,connectID);
                addlog("点位添加成功");

                char jobName[] = "WELDSEAM";
                int errorCode2 = OpenJob(jobName,connectID);
                addlog(QString::number(errorCode2));
                if (errorCode2 != 0)
                {
                    addlog("平焊程序未打开");
                    return;
                }
                Sleep(1000);
                int res = Send_Start (connectID);
                if(res!=0)
                {
                    addlog("工作命令发送失败");
                    return;
                }
                Sleep(1000);
                while(robot_is_moving)
                {
                    JobInfo *jobinfo = new JobInfo;
                    ReadCurJobInfo(jobinfo, connectID);
                    if(jobinfo->status == 0)
                    {
                        addlog("运行完成");
                        robot_is_moving=false;
                    }
                    else if(jobinfo->status == 1)
                    {
                        addlog("运行暂停");
                        robot_is_moving=false;
                        return;
                    }
                    else if(jobinfo->status == 3)
                    {
                        addlog("运行出错");
                        robot_is_moving=false;
                        return;
                    }
                    Sleep(1000);
                }
            }
            else{
                double rad = data.path[i].angle_deg[0] * M_PI / 180.0;

                double neg_dx = cos(rad);   // = -dx
                double neg_dy = sin(rad);   // = -dy

                double dx = -neg_dx;        // = data.path[i].direction[0]
                double dy = -neg_dy;        // = data.path[i].direction[1]

                RPY P_safe;
                P_safe.px=data.path[i].start_point[0]+20*dy;
                P_safe.py=data.path[i].start_point[1]-20*dx;
                P_safe.pz=data.path[i].start_point[2]+200;
                P_safe.rx=-45.0;
                P_safe.ry=0.0;
                P_safe.rz= data.path[i].angle_deg[0];
                float exAxis[6] = {-P_safe.py+400,284.358,0, 0,0,0};


                RPY P21;
                P21.px=data.path[i].start_point[0]+15*dy;
                P21.py=data.path[i].start_point[1]-15*dx;
                P21.pz=data.path[i].start_point[2]+100;
                P21.rx=-45.0;
                P21.ry=0.0;
                P21.rz= data.path[i].angle_deg[0]+45;
                float exAxis1[6] = {-P21.py+400,284.358,0, 0,0,0};

                RPY P22;
                int a = data.path[i].angle_deg[0];
                int kx, ky;
                if ((a > 90 && a <= 180) || (a > -180 && a <= -90)) {
                    kx = 40;
                    ky = 15;
                } else {
                    kx = 15;
                    ky = 40;
                }
                P22.px = data.path[i].start_point[0] + ky * dy;
                P22.py = data.path[i].start_point[1] - kx * dx;
                P22.pz = data.path[i].start_point[2] + 100;
                P22.rx = -45.0;
                P22.ry = 0.0;
                P22.rz = data.path[i].angle_deg[0]+45;
                float exAxis2[6] = {-P22.py + 400, 284.358, 0, 0, 0, 0};


                RPY P23;
                int kx2, ky2;
                if ((a > 90 && a <= 180) || (a > -180 && a <= -90)) {
                    kx2 = 20;
                    ky2 = 150;
                } else {
                    kx2 = 150;
                    ky2 = 20;
                }
                P23.px = data.path[i].start_point[0] + ky2 * dy;
                P23.py = data.path[i].start_point[1] - kx2 * dx;
                P23.pz = data.path[i].start_point[2] + 100;
                P23.rx = -45.0;
                P23.ry = 0.0;
                P23.rz = data.path[i].angle_deg[0] - 45;
                float exAxis3[6] = {-P23.py + 400, 284.358, 0, 0, 0, 0};


                RPY P24;
                int kx3, ky3;
                if ((a > 90 && a <= 180) || (a > -180 && a <= -90)) {
                    kx3 = 20;
                    ky3 = 180;
                } else {
                    kx3 = 180;
                    ky3 = 20;
                }
                P24.px = data.path[i].start_point[0] + ky3 * dy;
                P24.py = data.path[i].start_point[1] - kx3 * dx;
                P24.pz = data.path[i].start_point[2] + 100;
                P24.rx = -45.0;
                P24.ry = 0.0;
                P24.rz = data.path[i].angle_deg[0] - 45;
                float exAxis4[6] = {-P24.py + 400, 284.358, 0, 0, 0, 0};

                WriteUserVarFLOAT (1,dy*4,connectID);
                Sleep(200);
                WriteUserVarFLOAT (2,-dx*4,connectID);
                Sleep(200);
                WriteUserVarRPYS(20,P_safe,exAxis,2,connectID);
                Sleep(200);
                WriteUserVarRPYS(21,P21,exAxis1,2,connectID);
                Sleep(200);
                WriteUserVarRPYS(22,P22,exAxis2,2,connectID);
                Sleep(200);
                WriteUserVarRPYS(23,P23,exAxis3,2,connectID);
                Sleep(200);
                WriteUserVarRPYS(24,P24,exAxis4,2,connectID);
                addlog("立焊点位添加成功");

                char jobName[] = "WELDSEAM_L";
                int errorCode2 = OpenJob(jobName,connectID);
                addlog(QString::number(errorCode2));
                if (errorCode2 != 0)
                {
                    addlog("立焊程序未打开");
                    return;
                }
                Sleep(1000);
                int res = Send_Start (connectID);
                if(res!=0)
                {
                    addlog("工作命令发送失败");
                    return;
                }
                Sleep(1000);
                while(robot_is_moving)
                {
                    JobInfo *jobinfo = new JobInfo;
                    ReadCurJobInfo(jobinfo, connectID);
                    if(jobinfo->status == 0)
                    {
                        addlog("运行完成");
                        robot_is_moving=false;
                    }
                    else if(jobinfo->status == 1)
                    {
                        addlog("运行暂停");
                        robot_is_moving=false;
                        return;
                    }
                    else if(jobinfo->status == 3)
                    {
                        addlog("运行出错");
                        robot_is_moving=false;
                        return;
                    }
                    Sleep(2000);
                }
            }
        }
    });
}

// 作业状态
// typedef struct JobInfo
// {
//     char name[33];       // 作业名
//     char parentName[33]; // 父作业名
//     int lineNum;         // 执行行号
//     int status;          // 作业状态（0：停止，1：暂停，2：运行中：3：报警）
//     int posNum;          // 正在运动的P变量编号或者最后一次运动完成的P变量编号
// } JobInfo;

//机械臂停止运动
void OpenCVSet::on_pushButton_47_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }

    addlog("还没写，看上方运行情况");

}

//机械臂上电
void OpenCVSet::on_pushButton_48_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }


    int res=Send_SafeDoorOFF(connectID);

    if (res == 0)
    {
        addlog("软安全门已关闭");
    }
    Sleep(1000);

    Send_Reset(connectID);
    Sleep(1000);
    ClearBuffer(connectID);


    int res1 = Send_PowerOn (connectID);
    if (res1 == 0)
    {
        addlog("上电成功");
    }
}

//速率修改
void OpenCVSet::on_pushButton_49_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    float speedScale = ui->v_robot->text().toFloat();
    int errorCode = SetSpeedScale(speedScale, connectID);
    if (errorCode == 0)
    {
        addlog("速度修改成功");
    }
}

//外部轴移动
void OpenCVSet::on_pushButton_42_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }

    float exAxis[6]; //固定长度
    RPY rpy;

    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("读取失败");
        return;
    }
    addlog(QString::number(pJoint[6]));

    pJoint[6]-=ui->lineEdit->text().toDouble();
    addlog(QString::number(pJoint[6]));

    rpy.px = pJoint[0];
    rpy.py = pJoint[1];
    rpy.pz = pJoint[2];
    rpy.rx = pJoint[3];
    rpy.ry = pJoint[4];
    rpy.rz = pJoint[5];

    exAxis[0] = pJoint[6];
    exAxis[1] = pJoint[7];
    exAxis[2] = 0;
    exAxis[3] = 0;
    exAxis[4] = 0;
    exAxis[5] = 0;


    WriteUserVarRPYS(1,rpy,exAxis,1,connectID);


    char jobName[] = "SEARCHMOVING";
    int errorCode2 = OpenJob(jobName,connectID);
    addlog(QString::number(errorCode2));
    if (errorCode2 != 0)
    {
        addlog("外部轴移动文件未打开");
        return;
    }
    Sleep(1000);
    int res = Send_Start (connectID);
    if(res!=0)
    {
        addlog("移动命令发送失败");
        return;
    }

}

//回扫描位置
void OpenCVSet::on_pushButton_6_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }

    addlog("先别动");
    return;


    float exAxis[6]; //固定长度
    RPY rpy;

    float pJoint[12];
    int JointNum = 8;
    int errorCode = ReadJoint(pJoint, JointNum,connectID);
    if(errorCode!=0)
    {
        addlog("读取失败");
        return;
    }

    rpy.px = pJoint[0];
    rpy.py = pJoint[1];
    rpy.pz = pJoint[2];
    rpy.rx = pJoint[3];
    rpy.ry = pJoint[4];
    rpy.rz = pJoint[5];

    exAxis[0] = pJoint[6];
    exAxis[1] = 284.358;
    exAxis[2] = 0;
    exAxis[3] = 0;
    exAxis[4] = 0;
    exAxis[5] = 0;


    WriteUserVarRPYS(1,rpy,exAxis,1,connectID);


    char jobName[] = "SAFEPOSE";
    int errorCode2 = OpenJob(jobName,connectID);
    addlog(QString::number(errorCode2));
    if (errorCode2 != 0)
    {
        addlog("外部轴移动文件未打开");
        return;
    }
    Sleep(1000);
    int res = Send_Start (connectID);
    if(res!=0)
    {
        addlog("移动命令发送失败");
        return;
    }

}

//测试按钮，没事别动
void OpenCVSet::on_pushButton_5_clicked()
{
    if(!robot_is_connect)
    {
        addlog("机械臂未连接");
        return;
    }
    addlog("测试按钮，没事别动");
}


//生成圆弧
void OpenCVSet::on_pushButton_7_clicked()
{
    QString path = QDir::currentPath();
    QString cloudPath = path + "/cloud_fitle.pcd";

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(cloudPath.toStdString(), cloud) == -1) {
        addlog("点云加载失败!");
        return;
    }

    if (cloud.empty()) {
        addlog("点云为空!");
        return;
    }
    // 功能1: 提取直线和圆弧
    auto [seams, arcs] = WeldSeamExtractor::extractSeamsAndArcs(
        cloud,
        2.0,    // distance_threshold
        150,    // min_inliers
        200,    // max_iterations
        50,     // min_points_to_fit
        5.0,    // max_gap
        1.0,    // arc_dist_threshold
        150,    // min_arc_inliers
        100.0,   // arc_min_radius
        200.0,   // arc_max_radius
        0.35,   // arc_outlier_ratio
        0.01,   // arc_angle_tolerance
        10       // arc_min_continuous
        );
    seams.clear();
    std::vector<Point2D> points_2d;
    for (const auto& point : cloud.points) {
        // 根据你的投影平面选择坐标
        // 如果是投影到XY平面（Z=0）
        points_2d.emplace_back(point.x, point.y);
    }
    qDebug() << "共检测到" << seams.size() << "条直焊缝";
    qDebug() <<"检测到"<<arcs.size()<<"条圆弧";


    std::vector<std::vector<pcl::PointXYZ>> arc_clouds;

    // ===== 第一次调用：正常顺时针排序 =====
    auto result3d = WeldSeamExtractor::convertAndGroupTo3D(
        seams, arcs, points_2d, z0, axis, angle,
        20.0, 10.0, 20.0, 20.0, &arc_clouds, 30,
        true);   // do_clockwise_sort = true

    // 保存第一次的立焊缝
    auto saved_vertical_seams = result3d.vertical_seams;

    visualizeWeldSeamsAndArcs(points_2d, seams, arcs);

    QString pic_path = path + "/weld_seams_and_arcs_result.png";

    DelectSeam ds;
    ds.show();
label:
    ds.addpic(pic_path);
    int result = ds.exec();


    if (result / 10000 == 1) {
        if (result % 10000 >= seams.size()) {
            QMessageBox::information(&ds, "提示", "未检测到该焊缝，请检查数据", QMessageBox::Ok);
            goto label;
        } else {
            seams.erase(seams.begin() + result % 10000);
            arc_clouds.clear();
            // ===== 重新调用：跳过顺时针排序，保持起终点方向 =====
            result3d = WeldSeamExtractor::convertAndGroupTo3D(
                seams, arcs, points_2d, z0, axis, angle,
                20.0, 10.0, 20.0, 20.0, &arc_clouds, 30,
                false);
            result3d.vertical_seams = saved_vertical_seams;  // 恢复立焊缝
            visualizeWeldSeamsAndArcs(points_2d, seams, arcs);
            goto label;
        }
    } else if (result / 20000 == 1) {
        if (result % 20000 >= arcs.size()) {
            QMessageBox::information(&ds, "提示", "未检测到该焊缝，请检查数据", QMessageBox::Ok);
            goto label;
        } else {
            arcs.erase(arcs.begin() + result % 20000);
            arc_clouds.clear();
            result3d = WeldSeamExtractor::convertAndGroupTo3D(
                seams, arcs, points_2d, z0, axis, angle,
                20.0, 10.0, 20.0, 20.0, &arc_clouds, 30,
                false);
            result3d.vertical_seams = saved_vertical_seams;  // 恢复立焊缝
            visualizeWeldSeamsAndArcs(points_2d, seams, arcs);
            goto label;
        }
    }

    //平焊缝手眼转换
    for(int i=0;i<result3d.grouped_seams.size();i++)
    {
        for(int j=0;j<result3d.grouped_seams[i].size();j++)
        {
            cv::Point3f p_in(result3d.grouped_seams[i][j].start.x,result3d.grouped_seams[i][j].start.y,result3d.grouped_seams[i][j].start.z);
            qDebug()<<p_in.x<<","<<p_in.y<<","<<p_in.z;
            cv::Point3f p_out;
            calibrator->transformCameraToBase("./handtoeye.yaml",p_in,p_out);
            qDebug()<<p_out.x<<","<<p_out.y<<","<<p_out.z;

            cv::Point3f p_in2(result3d.grouped_seams[i][j].end.x,result3d.grouped_seams[i][j].end.y,result3d.grouped_seams[i][j].end.z);
            qDebug()<<p_in2.x<<","<<p_in2.y<<","<<p_in2.z;
            cv::Point3f p_out2;
            calibrator->transformCameraToBase("./handtoeye.yaml",p_in2,p_out2);
            qDebug()<<p_out2.x<<","<<p_out2.y<<","<<p_out2.z;

            result3d.grouped_seams[i][j].start.x=p_out.x;
            result3d.grouped_seams[i][j].start.y=p_out.y;
            result3d.grouped_seams[i][j].start.z=p_out.z;
            result3d.grouped_seams[i][j].end.x=p_out2.x;
            result3d.grouped_seams[i][j].end.y=p_out2.y;
            result3d.grouped_seams[i][j].end.z=p_out2.z;
        }
    }

    //立焊缝手眼转换
    for(int i=0;i<result3d.vertical_seams.size();i++)
    {
        cv::Point3f p_in(result3d.vertical_seams[i].start.x,result3d.vertical_seams[i].start.y,result3d.vertical_seams[i].start.z);
        cv::Point3f p_out;
        calibrator->transformCameraToBase("./handtoeye.yaml",p_in,p_out);
        cv::Point3f p_in2(result3d.vertical_seams[i].end.x,result3d.vertical_seams[i].end.y,result3d.vertical_seams[i].end.z);
        cv::Point3f p_out2;
        calibrator->transformCameraToBase("./handtoeye.yaml",p_in,p_out);
        result3d.vertical_seams[i].start.x=p_out.x;
        result3d.vertical_seams[i].start.y=p_out.y;
        result3d.vertical_seams[i].start.z=p_out.z;
        result3d.vertical_seams[i].end.x=p_out2.x;
        result3d.vertical_seams[i].end.y=p_out2.y;
        result3d.vertical_seams[i].end.z=p_out2.z;
    }


    // 功能3: 生成JSON文件
    bool success = WeldSeamExtractor::generateJSON(
        result3d.grouped_seams,
        result3d.grouped_seams_with_arc,
        result3d.grouped_arcs,
        result3d.vertical_seams,
        "weld_seams_output.json",
        WeldType::FILLET,
        WeldPosition::FLAT,
        1.0
        );

    if (success) {
        addlog("JSON文件生成成功!");
    } else {
        addlog("JSON文件生成失败!");
    }
    weldSearch->process_welding_system("weld_seams_output.json","./weldseam");


    //焊缝展示
    QString cloudPath2 = path+"/cloud.pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    if (pcl::io::loadPCDFile(cloudPath2.toStdString(), cloud2) == -1)
    {
        QMessageBox::warning(this, "警告", "点云加载失败!");
        return;
    }
    if (cloud2.empty())
    {
        QMessageBox::warning(this, "警告", "点云为空!");
        return;
    }

    if (viewer->contains("cloud_display"))  // 检查是否存在指定ID的点云
    {
        viewer->removePointCloud("cloud_display");  // 移除旧的
        viewer->removeAllShapes();
        qDebug() << "已移除旧点云";
    }
    viewer->addPointCloud(cloud2.makeShared(),"cloud_display");

    int a=0;
    for(int i = 0; i < result3d.grouped_seams.size(); i++)
    {
        for(int j=0;j<result3d.grouped_seams[i].size();j++)
        {
            pcl::PointXYZ start_point(result3d.grouped_seams[i][j].start.x,result3d.grouped_seams[i][j].start.y,result3d.grouped_seams[i][j].start.z);
            pcl::PointXYZ end_point(result3d.grouped_seams[i][j].end.x,result3d.grouped_seams[i][j].end.y,result3d.grouped_seams[i][j].end.z);
            viewer->addLine(start_point, end_point, 255.0, 0.0, 0.0, "line_id_"+QString::number(a).toStdString());
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5, "line_id_"+QString::number(a).toStdString());
            a++;
        }

    }
    for(int i = 0; i < result3d.vertical_seams.size(); i++)
    {
        viewer->addLine(result3d.vertical_seams[i].start, result3d.vertical_seams[i].end, 255.0, 0.0, 0.0, "line_id_"+QString::number(a).toStdString());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5, "line_id_"+QString::number(a).toStdString());
        a++;
    }

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1,
        "cloud_display"  // 这里必须和上面的ID一致
        );

    ui->widget->renderWindow()->Render();
}

