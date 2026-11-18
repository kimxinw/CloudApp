#ifndef CLOUD_SELVE_H
#define CLOUD_SELVE_H

#include "cloud_selve_global.h"
#include <QString>
#include <vector>
#include <opencv2/opencv.hpp>

// 运动标定结果结构体
struct CLOUD_SELVE_EXPORT MotionCalibResult
{
    cv::Vec3d direction;        // 运动方向（单位向量）
    double total_displacement;  // 总位移（毫米）
    cv::Vec3d displacement_vec; // 位移向量（毫米）
    double actual_movement_mm;  // 实际移动距离（毫米）
    bool is_linear_movement;    // 是否是直线运动
    double linearity_error;      // 线性误差（如果有多个点）
    double step_size_mm;         // 步长（毫米/步）
    int step_count;              // 步数
};

// 激光扫描参数配置
struct CLOUD_SELVE_EXPORT LaserScanConfig
{
    std::string extraction_method = "gravity";  // 提取方法: "gravity" 或 "steger"
    std::string output_format = "pcd";           // 输出格式: "pcd", "ply", "xyz"
    std::string output_filename = "cloud";       // 输出文件名（不含扩展名）
    std::string debug_dir = "./debug_frames";    // 调试图像保存目录
    bool save_debug_images = true;               // 是否保存调试图像
    int laser_threshold = 30;                    // 激光提取阈值
    int color_channel = 0;                       // 颜色通道: 0=红色, 1=绿色, 2=蓝色, -1=灰度
    int column_sampling_step=1;                    //采样间隔
};

// 双目相机外参标定结果
struct CLOUD_SELVE_EXPORT StereoCalibResult
{
    cv::Mat R;           // 旋转矩阵 (3x3)
    cv::Mat t;           // 平移向量 (3x1)
    double reproj_error; // 重投影误差
};


class CLOUD_SELVE_EXPORT Cloud_selve
{
public:
    Cloud_selve();
    ~Cloud_selve();

    /*****************标定接口********************/

    /**
     * @brief 相机标定.
     * @param imageDir (in) 存放标定图片的目录路径
     * @param board_width (in) 棋盘格宽度方向的内角点数量
     * @param board_height (in) 棋盘格高度方向的内角点数量
     * @param square_size_mm (in) 棋盘格方格边长(毫米)
     * @param filename (in) 输出文件的文件名
     * @param 功能: 在当前工作目录生成相机标定文件 camera_calib.yaml
    */
    void calibrateCameraFromImages(const QString& imageDir,
                                   int board_width,
                                   int board_height,
                                   float square_size_mm,
                                   QString filename);

    /**
     * @brief 激光平面标定.
     * @param cameraCalibFile (in) 相机标定文件路径 (camera_calib.yaml)
     * @param calibImgPose1 (in) 位姿1无激光图片路径(标定板)
     * @param laserImgPose1 (in) 位姿1有激光图片路径
     * @param calibImgPose2 (in) 位姿2无激光图片路径(标定板)
     * @param laserImgPose2 (in) 位姿2有激光图片路径
     * @param board_width (in) 棋盘格宽度方向的内角点数量
     * @param board_height (in) 棋盘格高度方向的内角点数量
     * @param square_size_mm (in) 棋盘格方格边长(毫米)
     * @param filename (in) 输出文件名
     * @param 功能: 在当前工作目录生成激光平面标定文件 light_plane.yaml
    */
    void calibrateLaserPlane(const QString& cameraCalibFile,
                             const QString& calibImgPose1,
                             const QString& laserImgPose1,
                             const QString& calibImgPose2,
                             const QString& laserImgPose2,
                             int board_width,
                             int board_height,
                             float square_size_mm,
                             QString filename);

    /**
     * @brief 轨道运动标定（两点法）.
     * @param cameraCalibFile (in) 相机标定文件路径 (camera_calib.yaml)
     * @param imgBefore (in) 运动前图片路径
     * @param imgAfter (in) 运动后图片路径
     * @param movement_mm (in) 实际移动距离（毫米）
     * @param board_width (in) 棋盘格宽度方向的内角点数量
     * @param board_height (in) 棋盘格高度方向的内角点数量
     * @param square_size_mm (in) 棋盘格方格边长(毫米)
     * @param filename (in) 输出文件名
     * @param step_count (in) 移动步数（可选，默认为0）
     * @param 功能: 在当前工作目录生成外部轴标定文件 motion_calib.yaml
    */
    void calibrateMotion(const QString& cameraCalibFile,
                         const QString& imgBefore,
                         const QString& imgAfter,
                         double movement_mm,
                         int board_width,
                         int board_height,
                         float square_size_mm,
                         QString filename,
                         int step_count = 0);

    /**
     * @brief 轨道运动标定（多点法）.
     * @param cameraCalibFile (in) 相机标定文件路径 (camera_calib.yaml)
     * @param images (in) 运动过程图片集合
     * @param movement_mm (in) 实际移动距离（毫米）
     * @param board_width (in) 棋盘格宽度方向的内角点数量
     * @param board_height (in) 棋盘格高度方向的内角点数量
     * @param square_size_mm (in) 棋盘格方格边长(毫米)
     * @param filename (in) 输出文件名
     * @param step_count (in) 移动步数（可选，默认为0）
     * @param 功能: 在当前工作目录生成外部轴标定文件 motion_calib.yaml
    */
    void calibrateMotionMultiPoint(const QString& cameraCalibFile,
                                   const std::vector<QString>& images,
                                   const std::vector<double>& movements_mm,
                                   int board_width,
                                   int board_height,
                                   float square_size_mm,
                                   QString filename,
                                   int step_count = 0);

    /**
     * @brief 激光扫描生成点云
     * @param imageDir (in) 包含激光图像的目录路径
     * @param cameraCalibFile (in) 相机标定文件路径
     * @param laserPlaneFile (in) 光平面标定文件路径
     * @param motionCalibFile (in) 运动标定文件路径
     * @param config (in) 扫描配置参数
     * @param alxeposition (in) 当前外部轴位置
     * @param A (in) 外部轴x偏移量
     * @param B (in) 外部轴y偏移量
     * @param C (in) 外部轴z偏移量
     * @param return (out) 是否成功
    */
    bool generatePointCloud(const QString& imageDir,
                            const QString& cameraCalibFile,
                            const QString& laserPlaneFile,
                            const QString& motionCalibFile,
                            const LaserScanConfig& config = LaserScanConfig(),
                            const double& alxeposition = 0,
                            const double& A=0,
                            const double& B=1,
                            const double& C=0);

    /**
     * @brief 激光扫描生成点云
     * @param imageDir (in) 包含激光图像的目录路径
     * @param cameraCalibFile (in) 相机标定文件路径
     * @param laserPlaneFile (in) 光平面标定文件路径
     * @param motionCalibFile (in) 运动标定文件路径
     * @param alxeposition (in) 当前外部轴位置
     * @param A (in) 外部轴x偏移量
     * @param B (in) 外部轴y偏移量
     * @param C (in) 外部轴z偏移量
     * @param return (out) 是否成功
    */
    bool generatePointCloudSimple(const QString& imageDir,
                                  const QString& cameraCalibFile,
                                  const QString& laserPlaneFile,
                                  const QString& motionCalibFile,
                                  const double& alxeposition = 0,
                                  const double& A=0,
                                  const double& B=1,
                                  const double& C=0);

    /**
     * @brief 双目相机外参标定
     * @param cameraCalibFile1 (in) 相机1的标定文件路径
     * @param cameraCalibFile2 (in) 相机2的标定文件路径
     * @param imageDir1 (in) 相机1的标定图片目录
     * @param imageDir2 (in) 相机2的标定图片目录
     * @param board_width (in) 棋盘格宽度方向的内角点数量
     * @param board_height (in) 棋盘格高度方向的内角点数量
     * @param square_size_mm (in) 棋盘格方格边长(毫米)
     * @param filename (in) 输出文件名
     * @param 功能: 生成双目相机外参标定文件 stereo_calib.yaml
    */
    void calibrateStereoCamera(const QString& cameraCalibFile1,
                               const QString& cameraCalibFile2,
                               const QString& imageDir1,
                               const QString& imageDir2,
                               int board_width,
                               int board_height,
                               float square_size_mm,
                               QString filename);


    /**
     * @brief 将点云从相机2坐标系转换到相机1坐标系
     * @param cloudFileFromCam2 (in) 相机2扫描出的点云文件路径
     * @param outputCloudFile (out) 转换到相机1坐标系后的点云文件路径
     * @param stereoCalibFile (in) 双目外参标定文件路径（相机2→相机1）
     * @return 是否成功
    */
    bool transformCam2ToCam1(const QString& cloudFileFromCam2,
                             const QString& outputCloudFile,
                             const QString& stereoCalibFile);


    /**
     * @brief 手眼标定（Eye-to-Hand）- 基于两张不同高度的标定板图片 + 机器人坐标点对
     *
     * 改进点：
     *   1. 拍摄两张不同高度的标定板图片（例如高度差 20mm 或 50mm）
     *   2. 每张图片选取 6 个分布均匀的角点
     *   3. 总计 12 个点，在三维空间中形成立体分布，提升深度方向标定精度
     *
     * @param cameraIntrinFile  相机内参标定文件路径（YAML格式）
     * @param imagePathLow      低位置标定板图片路径
     * @param imagePathHigh     高位置标定板图片路径
     * @param savePath          标定结果保存路径（YAML格式）
     * @param boardSize         棋盘格内角点数，例如 Size(9, 6)
     * @param selectedCorners   每张图片选取的角点索引数组（6个点）
     * @param robotPointsLow    低位置图片对应的机器人世界坐标（与 selectedCorners 一一对应）
     * @param robotPointsHigh   高位置图片对应的机器人世界坐标（与 selectedCorners 一一对应）
     * @param visualize         是否显示标定板图片并标记角点（默认 true）
     * @return true  标定成功
     * @return false 标定失败
    */
    bool calibrateHandToEyeTwoHeights(const std::string& cameraIntrinFile,
                                      const std::string& imagePathLow,
                                      const std::string& imagePathHigh,
                                      const std::string& savePath,
                                      const cv::Size& boardSize,
                                      const std::vector<int>& selectedCorners,
                                      const std::vector<cv::Point3f>& robotPointsLow,
                                      const std::vector<cv::Point3f>& robotPointsHigh,
                                      bool visualize = true) ;

    /**
     * @brief 加载手眼标定文件，获取变换矩阵
     *
     * @param calibFile      标定文件路径（handtoeye.yaml）
     * @param R              输出的旋转矩阵（3x3）
     * @param t              输出的平移向量（3x1）
     * @return true          加载成功
     * @return false         加载失败
    */
    bool loadHandToEyeCalibration(const std::string& calibFile,
                                  cv::Mat& R,
                                  cv::Mat& t);

    /**
     * @brief 将相机坐标系下的3D点转换到机器人基座坐标系
     *
     * 转换公式：P_base = R * P_camera + t
     *
     * @param calibFile      标定文件路径（handtoeye.yaml）
     * @param pointsCamera   输入：相机坐标系下的3D点（cv::Point3f 或 cv::Point3d）
     * @param pointsBase     输出：机器人基座坐标系下的3D点
     * @return true          转换成功
     * @return false         转换失败
    */
    bool transformCameraToBase(const std::string& calibFile,
                               const std::vector<cv::Point3f>& pointsCamera,
                               std::vector<cv::Point3f>& pointsBase);

    /**
     * @brief 将单个相机坐标系下的3D点转换到机器人基座坐标系（重载版本）
     *
     * @param calibFile      标定文件路径（handtoeye.yaml）
     * @param pointCamera    输入：相机坐标系下的3D点
     * @param pointBase      输出：机器人基座坐标系下的3D点
     * @return true          转换成功
     * @return false         转换失败
    */
    bool transformCameraToBase(const std::string& calibFile,
                               const cv::Point3f& pointCamera,
                               cv::Point3f& pointBase);
};

#endif // CLOUD_SELVE_H
