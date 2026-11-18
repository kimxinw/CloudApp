#ifndef OPENCVSET_H
#define OPENCVSET_H

#define NOMINMAX

#include <QMainWindow>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <QTcpSocket>
#include <QTimer>
#include <QElapsedTimer>
#include <QMutex>
#include <QQueue>
#include <QThread>
#include <GalaxyIncludes.h>
#include "cloud_selve.h"
#include "weld_search.h"
#include "Robot_Datatype.h"
#include "RobotSDK.h"

// #include <eigen-master/Eigen/Dense>

#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

namespace Ui {
class OpenCVSet;
}


// 前向声明
class OpenCVSet;

struct ImageTask {
    unsigned char* buffer;
    size_t bufferSize;
    int width;
    int height;
    int cameraID;
    int frameNumber;
    GX_PIXEL_FORMAT_ENTRY pixelFormat;
};

struct WeldDate{
    int id;
    int group_id;
    double start_point[3];
    double end_point[3];
    double direction[3];
    std::string weld_type;
    std::string weld_position;
};

// 大恒相机回调处理类
class CSampleCaptureEventHandler : public ICaptureEventHandler
{
public:
    CSampleCaptureEventHandler(OpenCVSet* pMainWindow, int cameraID)
        : m_pMainWindow(pMainWindow), m_cameraID(cameraID) {}

    void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam) override;

private:
    OpenCVSet* m_pMainWindow;
    int m_cameraID;  // 0=相机1, 1=相机2
};


class OpenCVSet : public QMainWindow
{
    Q_OBJECT

public:
    explicit OpenCVSet(QWidget *parent = nullptr);
    ~OpenCVSet();

    // 回调采集相关函数
    bool InitCallbackCapture();           // 初始化回调采集
    void StartCallbackCapture();          // 启动回调采集
    void StopCallbackCapture();           // 停止回调采集
    void OnImageCaptured(CImageDataPointer& objImageDataPointer, int cameraID);  // 图像回调处理

private slots:
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void addlog(const QString &log);
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void connected();
    void disconnected();
    void socketError(QAbstractSocket::SocketError error);
    void updateConnectionStatus();
    void updatePosition();
    void PCL2Camera();
    void on_pushButton_11_clicked();
    void on_pushButton_12_clicked();
    void on_pushButton_16_clicked();
    void on_pushButton_10_clicked();
    void on_pushButton_13_clicked();
    void on_pushButton_17_clicked();
    void on_pushButton_14_clicked();
    void on_pushButton_9_clicked();
    void on_pushButton_18_clicked();
    void on_pushButton_15_clicked();
    void takePhoto();
    void takePhoto2(QString &path,int camera_number,int number);
    void on_pushButton_19_clicked();
    void clearWork();
    void on_pushButton_20_clicked();
    void on_pushButton_21_clicked();
    void on_pushButton_22_clicked();
    void on_pushButton_23_clicked();
    void on_pushButton_25_clicked();
    void loadSettings();
    void saveSettings();
    void on_pushButton_26_clicked();
    void ProcessSavedImage();
    void on_pushButton_27_clicked();

    void on_pushButton_24_clicked();

    void on_pushButton_28_clicked();

    void on_pushButton_29_clicked();

    void on_pushButton_38_clicked();

    void on_pushButton_39_clicked();

    void on_pushButton_35_clicked();

    void on_pushButton_30_clicked();

    void on_pushButton_34_clicked();

    void on_pushButton_36_clicked();

    void on_pushButton_37_clicked();

    void on_pushButton_31_clicked();

    void on_pushButton_32_clicked();

    void on_pushButton_33_clicked();

    void on_pushButton_40_clicked();

    void on_pushButton_47_clicked();

    void on_pushButton_48_clicked();

    void on_pushButton_49_clicked();

    void on_pushButton_42_clicked();


    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

private:
    Ui::OpenCVSet *ui;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    QTcpSocket *alxeSocket;//TCP通讯
    QTimer *alxeTimer;
    QTimer *positionTimer;
    QTimer *workTimer;
    bool m_isDestorying = false;
    Cloud_selve *calibrator;
    Weld_search *weldSearch;
    WeldSeamExtractor *wedlprocessor;                    //焊缝处理类

    // 新增成员变量，用于缓存焊缝数据
    std::vector<WeldSeam3D> m_seams3d;
    std::vector<WeldLSeam3D> m_Lseams3d;
    std::vector<WeldArc3D> m_arc3d;
    std::vector<std::vector<WeldSeam3D>> m_grouped_seams;
    std::vector<std::vector<WeldArc3D>> m_grouped_arcs;
    std::unique_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>>> m_arc_clouds;

    // 回调采集相关
    CSampleCaptureEventHandler* m_pCaptureHandler;   // 回调处理器（相机1）
    CSampleCaptureEventHandler* m_pCaptureHandler1;  // 回调处理器（相机2）
    bool m_bCallbackMode;                            // 是否使用回调模式
    bool m_bGrabbing;                                // 是否正在采集

    // 图像计数
    int m_frameCount1;                               // 相机1采集帧数
    int m_frameCount2;                               // 相机2采集帧数
    int m_savedCount1;                               // 相机1已保存数量
    int m_savedCount2;                               // 相机2已保存数量

    // 性能统计
    QElapsedTimer m_captureTimer;
    QList<qint64> m_captureTimes;                   // 记录每次采集耗时


    void SaveImageTask(const ImageTask& task);  // 实际保存函数
    size_t GetImageBufferSize(CImageDataPointer& image);  // 计算缓冲区大小

    QQueue<ImageTask> m_imageQueue1;
    QQueue<ImageTask> m_imageQueue2;
    QMutex m_mutex1;
    QMutex m_mutex2;
};

#endif // OPENCVSET_H
