#include "cloud_selve.h"
#include <QDir>
#include <QDebug>
#include <fstream>
#include <QFileInfo>
#include <sstream>
#include <iomanip>
#include <cctype>

Cloud_selve::Cloud_selve()
{
}

Cloud_selve::~Cloud_selve()
{
}

// 运动标定器内部类
class MotionCalibrator {
private:
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Size board_size;
    float square_size;

    void addLog(const QString& msg) {
        qDebug() << msg;
    }

public:
    MotionCalibrator(const cv::Mat& cam_mat, const cv::Mat& dist_coeff,
                     const cv::Size& board_sz, float square_sz)
        : camera_matrix(cam_mat), dist_coeffs(dist_coeff),
        board_size(board_sz), square_size(square_sz) {}

    // 检测棋盘格并计算外参
    bool computeBoardPose(const cv::Mat& img, cv::Mat& R, cv::Mat& tvec) {
        std::vector<cv::Point2f> corners;
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, board_size, corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH +
                                                   cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!found) {
            return false;
        }

        cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

        std::vector<cv::Point3f> objp;
        for (int i = 0; i < board_size.height; ++i) {
            for (int j = 0; j < board_size.width; ++j) {
                objp.push_back(cv::Point3f(j * square_size, i * square_size, 0));
            }
        }

        cv::Mat rvec;
        cv::solvePnP(objp, corners, camera_matrix, dist_coeffs, rvec, tvec);
        cv::Rodrigues(rvec, R);

        return true;
    }

    // 获取标定板中心点
    cv::Point3f getBoardCenter(const cv::Mat& img) {
        cv::Mat R, tvec;
        if (!computeBoardPose(img, R, tvec)) {
            return cv::Point3f(0, 0, 0);
        }

        float center_x = (board_size.width - 1) * square_size / 2.0f;
        float center_y = (board_size.height - 1) * square_size / 2.0f;
        cv::Mat center_board = (cv::Mat_<double>(3,1) << center_x, center_y, 0);
        cv::Mat center_cam = R * center_board + tvec;

        return cv::Point3f(center_cam.at<double>(0),
                           center_cam.at<double>(1),
                           center_cam.at<double>(2));
    }

    // 两点法标定
    MotionCalibResult calibrate(const cv::Mat& img_before,
                                const cv::Mat& img_after,
                                double movement_mm,
                                int step_count = 0) {
        MotionCalibResult result;
        result.actual_movement_mm = movement_mm;
        result.is_linear_movement = true;
        result.step_count = step_count;
        result.step_size_mm = (step_count > 0) ? movement_mm / step_count : 0;

        cv::Point3f center_before = getBoardCenter(img_before);
        cv::Point3f center_after = getBoardCenter(img_after);

        if (center_before.x == 0 && center_before.y == 0 && center_before.z == 0) {
            throw std::runtime_error("无法计算运动前的位置");
        }
        if (center_after.x == 0 && center_after.y == 0 && center_after.z == 0) {
            throw std::runtime_error("无法计算运动后的位置");
        }

        cv::Vec3d displacement(
            center_after.x - center_before.x,
            center_after.y - center_before.y,
            center_after.z - center_before.z
            );

        result.displacement_vec = displacement;
        result.total_displacement = cv::norm(displacement);
        result.direction = displacement / result.total_displacement;

        double error_pct = fabs(result.total_displacement - movement_mm) / movement_mm * 100;

        addLog("\n========== 运动分析 ==========");
        addLog(QString("视觉测量的位移: %1 mm").arg(result.total_displacement));
        addLog(QString("实际移动距离: %1 mm").arg(movement_mm));
        addLog(QString("相对误差: %1 %").arg(error_pct));

        if (error_pct > 5.0) {
            addLog("⚠️ 警告: 视觉测量与实际移动距离偏差较大！");
        } else {
            addLog("✓ 视觉测量结果可信");
        }

        return result;
    }

    // 多点标定
    MotionCalibResult calibrateMultiPoint(
        const std::vector<cv::Mat>& images,
        const std::vector<double>& movements_mm,
        int step_count = 0) {

        if (images.size() != movements_mm.size() || images.size() < 2) {
            throw std::runtime_error("数据不匹配");
        }

        std::vector<cv::Point3f> centers;
        std::vector<double> cumulative_movements;

        for (size_t i = 0; i < images.size(); i++) {
            cv::Point3f center = getBoardCenter(images[i]);
            if (center.x == 0 && center.y == 0 && center.z == 0) {
                throw std::runtime_error("无法计算位置 " + std::to_string(i));
            }
            centers.push_back(center);

            double cum_mov = 0;
            for (size_t j = 1; j <= i; j++) {
                cum_mov += movements_mm[j];
            }
            cumulative_movements.push_back(cum_mov);
        }

        cv::Point3f origin = centers[0];

        // 使用最小二乘法拟合方向
        cv::Mat A = cv::Mat::zeros(images.size() - 1, 3, CV_64F);
        cv::Mat B = cv::Mat::zeros(images.size() - 1, 1, CV_64F);

        for (size_t i = 1; i < centers.size(); i++) {
            cv::Vec3d displacement(
                centers[i].x - origin.x,
                centers[i].y - origin.y,
                centers[i].z - origin.z
                );

            double distance = cv::norm(displacement);
            double movement = cumulative_movements[i];

            if (movement > 0) {
                cv::Vec3d dir = displacement / distance;
                A.at<double>(i-1, 0) = dir[0];
                A.at<double>(i-1, 1) = dir[1];
                A.at<double>(i-1, 2) = dir[2];
                B.at<double>(i-1, 0) = distance / movement;
            }
        }

        MotionCalibResult result;
        result.direction = cv::Vec3d(1, 0, 0);
        result.actual_movement_mm = cumulative_movements.back();
        result.step_count = step_count;
        result.step_size_mm = (step_count > 0) ? result.actual_movement_mm / step_count : 0;

        // 简单平均方向
        cv::Vec3d avg_dir(0, 0, 0);
        int valid_count = 0;
        for (size_t i = 1; i < centers.size(); i++) {
            cv::Vec3d displacement(
                centers[i].x - origin.x,
                centers[i].y - origin.y,
                centers[i].z - origin.z
                );
            double distance = cv::norm(displacement);
            if (distance > 0) {
                avg_dir += displacement / distance;
                valid_count++;
            }
        }
        if (valid_count > 0) {
            avg_dir /= valid_count;
            result.direction = avg_dir / cv::norm(avg_dir);
        }

        // 计算线性误差
        result.linearity_error = 0;
        for (size_t i = 1; i < centers.size(); i++) {
            cv::Vec3d displacement(
                centers[i].x - origin.x,
                centers[i].y - origin.y,
                centers[i].z - origin.z
                );
            cv::Vec3d predicted = cumulative_movements[i] * result.direction;
            double error = cv::norm(displacement - predicted);
            result.linearity_error += error * error;
        }
        result.linearity_error = sqrt(result.linearity_error / (centers.size() - 1));

        result.total_displacement = cv::norm(cv::Vec3d(
            centers.back().x - origin.x,
            centers.back().y - origin.y,
            centers.back().z - origin.z
            ));

        result.displacement_vec = cv::Vec3d(
            centers.back().x - origin.x,
            centers.back().y - origin.y,
            centers.back().z - origin.z
            );

        result.is_linear_movement = (result.linearity_error < 0.5);

        addLog(QString("\n多点标定线性误差: %1 mm").arg(result.linearity_error));
        if (result.is_linear_movement) {
            addLog("✓ 运动轨迹线性度良好");
        } else {
            addLog("⚠️ 运动轨迹线性度较差");
        }

        return result;
    }
};

struct CalibrationParams {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Vec4f light_plane;
    cv::Vec3d motion_direction;
    double step_size_mm;
    cv::Size image_size;
    bool validate() const {
        return !camera_matrix.empty() &&
               !dist_coeffs.empty() &&
               light_plane[0] != 0 &&
               step_size_mm > 0;
    }
};

class LaserScannerInternal {
private:
    CalibrationParams params;
    std::vector<cv::Point3f> point_cloud;
    int laser_threshold;
    int color_channel;
    int column_sampling_step;

    void addLog(const QString& msg) {
        qDebug() << msg;
    }

    std::vector<cv::Point2f> extractLaserCenterline(const cv::Mat& img, int threshold) {
        auto t_total_start = std::chrono::high_resolution_clock::now();

        // 1. 颜色通道提取
        auto t1 = std::chrono::high_resolution_clock::now();
        cv::Mat gray;
        if (img.channels() == 3) {
            std::vector<cv::Mat> channels;
            cv::split(img, channels);
            if (color_channel >= 0 && color_channel <= 2) {
                gray = channels[color_channel].clone();
            } else {
                gray = channels[0].clone();
            }
            cv::Mat enhanced;
            cv::normalize(gray, enhanced, 0, 255, cv::NORM_MINMAX);
            gray = enhanced;
        } else {
            gray = img.clone();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto dt_channel = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        // 2. 高斯滤波
        cv::Mat blurred;
        cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0.8);
        auto t3 = std::chrono::high_resolution_clock::now();
        auto dt_blur = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

        // 3. ROI和阈值计算
        int width = blurred.cols;
        int height = blurred.rows;
        cv::Rect roi(0, height * 0.3, width, height * 0.5);
        if (roi.y + roi.height > height) {
            roi.height = height - roi.y;
        }
        cv::Mat roi_img = blurred(roi);
        cv::Scalar mean_val = cv::mean(roi_img);
        int dynamic_threshold = std::max(threshold, (int)mean_val[0] + 15);
        int min_threshold = 25;
        auto t4 = std::chrono::high_resolution_clock::now();
        auto dt_roi = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();

        // 4. 列扫描（主要循环）
        std::vector<cv::Point2f> center_points;
        for (int col = roi.x; col < roi.x + roi.width; col += column_sampling_step) {
            float sum_intensity = 0;
            float sum_y = 0;
            int valid_count = 0;
            int top_bound = -1, bottom_bound = -1;
            float max_intensity = 0;

            for (int row = roi.y; row < roi.y + roi.height; row++) {
                if (row >= blurred.rows || col >= blurred.cols) continue;
                int intensity = blurred.at<uchar>(row, col);
                if (intensity > dynamic_threshold || (valid_count > 0 && intensity > min_threshold)) {
                    sum_intensity += intensity;
                    sum_y += row * intensity;
                    valid_count++;
                    if (intensity > max_intensity) max_intensity = intensity;
                    if (top_bound == -1) top_bound = row;
                    bottom_bound = row;
                }
            }
            int line_height = bottom_bound - top_bound;
            if (valid_count > 3 && sum_intensity > 0 && line_height > 0 && line_height < 80) {
                float center_y = sum_y / sum_intensity;
                if (max_intensity > dynamic_threshold * 0.5) {
                    center_points.push_back(cv::Point2f(col, center_y));
                }
            }
        }
        auto t5 = std::chrono::high_resolution_clock::now();
        auto dt_scan = std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();

        auto dt_total = std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t_total_start).count();

        // 每100帧输出一次性能统计
        static int frame_count = 0;
        if (++frame_count % 100 == 0) {
            addLog(QString("性能: 通道=%1ms, 滤波=%2ms, ROI=%3ms, 扫描=%4ms, 总计=%5ms, 点数=%6")
                       .arg(dt_channel).arg(dt_blur).arg(dt_roi).arg(dt_scan).arg(dt_total).arg(center_points.size()));
        }

        return center_points;
    }


    cv::Point3f pixelTo3D(const cv::Point2f& pt) {
        std::vector<cv::Point2f> pts_in = {pt};
        std::vector<cv::Point2f> pts_undistorted;
        cv::undistortPoints(pts_in, pts_undistorted, params.camera_matrix, params.dist_coeffs);

        cv::Point2f pt_norm = pts_undistorted[0];
        cv::Point3f ray_dir(pt_norm.x, pt_norm.y, 1.0f);

        cv::Point3f normal(params.light_plane[0], params.light_plane[1], params.light_plane[2]);
        double d = params.light_plane[3];  // 使用临时变量，不修改成员变量

        // 如果法向量Z分量为负，说明方向反了，取反
        if (normal.z < 0) {
            normal = -normal;
            d = -d;  // 只修改临时变量
        }

        double denominator = normal.x * ray_dir.x + normal.y * ray_dir.y + normal.z * ray_dir.z;
        if (fabs(denominator) < 1e-10) {
            return cv::Point3f(0, 0, 0);
        }

        double t = -d / denominator;
        return cv::Point3f(t * ray_dir.x, t * ray_dir.y, t * ray_dir.z);
    }

    // 保存为PCD文件
    void saveAsPCD(const std::string& filename) {
        if (point_cloud.empty()) {
            addLog("点云为空");
            return;
        }

        std::ofstream file(filename);
        if (!file.is_open()) {
            addLog(QString("无法创建PCD文件: %1").arg(QString::fromStdString(filename)));
            return;
        }

        file << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
        file << "VERSION 0.7" << std::endl;
        file << "FIELDS x y z" << std::endl;
        file << "SIZE 4 4 4" << std::endl;
        file << "TYPE F F F" << std::endl;
        file << "COUNT 1 1 1" << std::endl;
        file << "WIDTH " << point_cloud.size() << std::endl;
        file << "HEIGHT 1" << std::endl;
        file << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
        file << "POINTS " << point_cloud.size() << std::endl;
        file << "DATA ascii" << std::endl;

        for (const auto& pt : point_cloud) {
            file << pt.x << " " << pt.y << " " << pt.z << std::endl;
        }

        file.close();
        addLog(QString("✓ PCD文件已保存: %1 (点数: %2)").arg(QString::fromStdString(filename)).arg(point_cloud.size()));
    }

    // 保存为PLY文件
    void saveAsPLY(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            addLog(QString("无法创建PLY文件: %1").arg(QString::fromStdString(filename)));
            return;
        }

        file << "ply" << std::endl;
        file << "format ascii 1.0" << std::endl;
        file << "element vertex " << point_cloud.size() << std::endl;
        file << "property float x" << std::endl;
        file << "property float y" << std::endl;
        file << "property float z" << std::endl;
        file << "end_header" << std::endl;

        for (const auto& pt : point_cloud) {
            file << pt.x << " " << pt.y << " " << pt.z << std::endl;
        }

        file.close();
        addLog(QString("✓ PLY文件已保存: %1").arg(QString::fromStdString(filename)));
    }

    // 保存为XYZ文件
    void saveAsXYZ(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            addLog(QString("无法创建XYZ文件: %1").arg(QString::fromStdString(filename)));
            return;
        }

        for (const auto& pt : point_cloud) {
            file << pt.x << " " << pt.y << " " << pt.z << std::endl;
        }

        file.close();
        addLog(QString("✓ XYZ文件已保存: %1").arg(QString::fromStdString(filename)));
    }

public:
    // 修改构造函数，添加颜色通道参数
    LaserScannerInternal(int threshold = 30, int channel = 0, int column_step = 1)
        : laser_threshold(threshold), color_channel(channel), column_sampling_step(column_step) {
        if (column_sampling_step < 1) column_sampling_step = 1;
    }

    // 设置颜色通道
    void setColorChannel(int channel) {
        color_channel = channel;
        std::string channel_name;
        if (color_channel == 0) channel_name = "红色";
        else if (color_channel == 1) channel_name = "绿色";
        else if (color_channel == 2) channel_name = "蓝色";
        else channel_name = "灰度";
        addLog(QString("切换激光提取通道为: %1").arg(QString::fromStdString(channel_name)));
    }
    // 设置列采样步长（每隔多少列提取一个激光点）
    void setColumnSamplingStep(int step) {
        column_sampling_step = step < 1 ? 1 : step;
        addLog(QString("设置列采样步长为: %1 (每隔%2列提取一个点)").arg(column_sampling_step).arg(column_sampling_step - 1));
    }

    // 加载标定参数
    bool loadCalibration(const std::string& cam_file,
                         const std::string& plane_file,
                         const std::string& motion_file) {
        // 加载相机参数
        cv::FileStorage fs_cam(cam_file, cv::FileStorage::READ);
        if (!fs_cam.isOpened()) {
            addLog("无法打开相机标定文件: " + QString::fromStdString(cam_file));
            return false;
        }
        fs_cam["camera_matrix"] >> params.camera_matrix;
        fs_cam["distortion_coefficients"] >> params.dist_coeffs;
        if (fs_cam["image_width"].isInt()) {
            fs_cam["image_width"] >> params.image_size.width;
            fs_cam["image_height"] >> params.image_size.height;
        }
        fs_cam.release();

        // 加载光平面参数
        cv::FileStorage fs_plane(plane_file, cv::FileStorage::READ);
        if (!fs_plane.isOpened()) {
            addLog("无法打开光平面标定文件: " + QString::fromStdString(plane_file));
            return false;
        }
        fs_plane["light_plane"] >> params.light_plane;
        fs_plane.release();

        // 加载运动参数
        cv::FileStorage fs_motion(motion_file, cv::FileStorage::READ);
        if (!fs_motion.isOpened()) {
            addLog("无法打开运动标定文件: " + QString::fromStdString(motion_file));
            return false;
        }

        cv::Vec3d dir;
        fs_motion["motion_direction"] >> dir;
        params.motion_direction = dir;

        // 加载步长
        if (fs_motion["step_size_mm_per_step"].isReal()) {
            fs_motion["step_size_mm_per_step"] >> params.step_size_mm;
        } else if (fs_motion["step_size_mm"].isReal()) {
            fs_motion["step_size_mm"] >> params.step_size_mm;
        } else {
            double total_displacement = 0, step_count = 0;
            if (fs_motion["total_displacement_mm"].isReal()) {
                fs_motion["total_displacement_mm"] >> total_displacement;
            }
            if (fs_motion["step_count"].isReal()) {
                fs_motion["step_count"] >> step_count;
            }
            if (total_displacement > 0 && step_count > 0) {
                params.step_size_mm = total_displacement / step_count;
            } else {
                addLog("错误: 运动标定文件中缺少步长信息");
                return false;
            }
        }
        fs_motion.release();

        if (!params.validate()) {
            addLog("标定参数验证失败");
            return false;
        }

        addLog("✓ 标定参数加载成功");
        addLog(QString("  相机分辨率: %1x%2").arg(params.image_size.width).arg(params.image_size.height));
        addLog(QString("  步长: %1 mm/帧").arg(params.step_size_mm));

        return true;
    }

    // 处理单张图像
    int processImage(const cv::Mat& img, int frame_index, const double& alxeposition, const double& A,const double& B, const double& C,
                     bool save_debug = false, const std::string& debug_dir = "") {
        if (img.empty()) return -1;

        // 提取激光中心线（使用成员变量中的颜色通道）
        std::vector<cv::Point2f> laser_pts_2d = extractLaserCenterline(img, laser_threshold);

        if (laser_pts_2d.empty()) return 0;

        // 保存调试图像
        if (save_debug && !debug_dir.empty()) {
            cv::Mat debug_img = img.clone();
            if (img.channels() == 1) {
                cv::cvtColor(img, debug_img, cv::COLOR_GRAY2BGR);
            }

            // 在调试图像上绘制提取的激光点
            for (const auto& pt : laser_pts_2d) {
                cv::circle(debug_img, pt, 2, cv::Scalar(0, 0, 255), -1);
            }

            // 添加颜色通道信息
            std::string channel_info;
            if (color_channel == 0) channel_info = "Red Channel";
            else if (color_channel == 1) channel_info = "Green Channel";
            else if (color_channel == 2) channel_info = "Blue Channel";
            else channel_info = "Gray";

            std::string text = "Frame " + std::to_string(frame_index) +
                               " Points: " + std::to_string(laser_pts_2d.size()) +
                               " [" + channel_info + "]";
            cv::putText(debug_img, text, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            std::string filename = debug_dir + "/frame_" + std::to_string(frame_index) + ".png";
            cv::imwrite(filename, debug_img);
        }

        // 计算平台移动距离
        float move_y = alxeposition*B+frame_index * params.step_size_mm;

        // 转换为3D点
        int points_added = 0;
        for (const auto& pt_2d : laser_pts_2d) {
            cv::Point3f pt_cam = pixelTo3D(pt_2d);
            if (pt_cam.x == 0 && pt_cam.y == 0 && pt_cam.z == 0) continue;

            cv::Point3f pt_scanned(pt_cam.x+alxeposition*A, move_y+pt_cam.y, pt_cam.z+alxeposition*C);
            point_cloud.push_back(pt_scanned);
            points_added++;
        }

        return points_added;
    }

    // 批量处理图像文件夹（处理所有帧，帧采样步长不再使用）
    bool processImageFolder(const std::string& folder_path,
                            const std::string& file_pattern,
                            const double& alxeposition,
                            const double& A,
                            const double& B,
                            const double& C,
                            bool save_debug = false,
                            const std::string& debug_dir = ""
                            ) {
        point_cloud.clear();

        std::string search_path = folder_path + "/" + file_pattern;
        std::vector<cv::String> filenames;
        cv::glob(search_path, filenames, false);

        if (filenames.empty()) {
            addLog("在 " + QString::fromStdString(folder_path) + " 中未找到匹配的文件");
            return false;
        }

        std::sort(filenames.begin(), filenames.end());

        addLog(QString("\n开始扫描..."));
        addLog(QString("找到 %1 个图像文件").arg(filenames.size()));
        addLog(QString("列采样步长: %1 (每帧提取约 %2 个点)").arg(column_sampling_step).arg(3800 / column_sampling_step));
        addLog(QString("颜色通道: %1").arg(color_channel == 0 ? "红色" : (color_channel == 1 ? "绿色" : (color_channel == 2 ? "蓝色" : "灰度"))));
        addLog("----------------------------------------");

        auto start_time = std::chrono::high_resolution_clock::now();

        int total_points = 0;
        int frames_processed = 0;

        for (size_t i = 0; i < filenames.size(); i++) {
            cv::Mat img = cv::imread(filenames[i]);
            if (img.empty()) continue;

            // 使用原始帧序号 i 作为移动距离计算
            int points = processImage(img, i,alxeposition,A,B,C, save_debug, debug_dir);
            if (points > 0) {
                total_points += points;
                frames_processed++;

                if ((i + 1) % 50 == 0 || (i + 1) == filenames.size()) {
                    addLog(QString("  已处理 %1/%2 帧, 累计点数: %3")
                               .arg(i + 1).arg(filenames.size()).arg(total_points));
                }
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

        addLog("----------------------------------------");
        addLog(QString("扫描完成!"));
        addLog(QString("有效帧数: %1 / %2").arg(frames_processed).arg(filenames.size()));
        addLog(QString("总点数: %1").arg(total_points));
        addLog(QString("耗时: %1 秒").arg(duration.count()));

        return true;
    }

    // 保存点云
    void savePointCloud(const std::string& filename, const std::string& format) {
        if (point_cloud.empty()) {
            addLog("点云为空，无法保存");
            return;
        }

        if (format == "pcd") {
            saveAsPCD(filename);
        } else if (format == "ply") {
            saveAsPLY(filename);
        } else if (format == "xyz") {
            saveAsXYZ(filename);
        } else {
            addLog("不支持的格式: " + QString::fromStdString(format));
        }
    }

    const std::vector<cv::Point3f>& getPointCloud() const {
        return point_cloud;
    }

    void clearPointCloud() {
        point_cloud.clear();
    }
};


// 激光平面标定辅助函数
cv::Vec4f fitPlane(const std::vector<cv::Point3f>& points) {
    if (points.size() < 3) {
        std::cerr << "点数不足，无法拟合平面" << std::endl;
        return cv::Vec4f(0,0,0,0);
    }

    // 构建矩阵A，每行是 [x, y, z, 1]
    cv::Mat A(points.size(), 4, CV_64F);
    for (size_t i = 0; i < points.size(); i++) {
        A.at<double>(i, 0) = points[i].x;
        A.at<double>(i, 1) = points[i].y;
        A.at<double>(i, 2) = points[i].z;
        A.at<double>(i, 3) = 1.0;
    }

    // 进行SVD分解求解最小二乘问题 A * n = 0
    cv::SVD svd(A, cv::SVD::FULL_UV);
    cv::Mat n = svd.vt.row(svd.vt.rows - 1);  // 最小奇异值对应的右奇异向量

    // 归一化，使得 (A,B,C) 为单位向量，方便计算距离
    double norm_abc = sqrt(n.at<double>(0,0)*n.at<double>(0,0) +
                           n.at<double>(0,1)*n.at<double>(0,1) +
                           n.at<double>(0,2)*n.at<double>(0,2));

    return cv::Vec4f(
        n.at<double>(0,0) / norm_abc,
        n.at<double>(0,1) / norm_abc,
        n.at<double>(0,2) / norm_abc,
        n.at<double>(0,3) / norm_abc
        );
}


// 激光平面标定辅助函数
cv::Rect getBoardROI(const std::vector<cv::Point2f>& corners, const cv::Size& board_size) {
    if (corners.empty()) return cv::Rect();

    cv::Point2f top_left = corners[0];
    cv::Point2f top_right = corners[board_size.width - 1];
    cv::Point2f bottom_left = corners[(board_size.height - 1) * board_size.width];
    cv::Point2f bottom_right = corners[board_size.width * board_size.height - 1];

    int expand = 10;
    int x_min = (std::min)({top_left.x, top_right.x, bottom_left.x, bottom_right.x}) - expand;
    int y_min = (std::min)({top_left.y, top_right.y, bottom_left.y, bottom_right.y}) - expand;
    int x_max = (std::max)({top_left.x, top_right.x, bottom_left.x, bottom_right.x}) + expand;
    int y_max = (std::max)({top_left.y, top_right.y, bottom_left.y, bottom_right.y}) + expand;

    x_min = (std::max)(0, x_min);
    y_min = (std::max)(0, y_min);
    x_max = (std::min)(x_max, 3840);
    y_max = (std::min)(y_max, 2748);

    return cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
}

// 激光平面标定辅助函数
cv::Point3f getBoardCenterFromPose(const cv::Mat& img,
                                   const cv::Mat& camera_matrix,
                                   const cv::Mat& dist_coeffs,
                                   const cv::Size& board_size,
                                   float square_size)
{
    std::vector<cv::Point2f> corners;
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(gray, board_size, corners);
    if (!found) return cv::Point3f(0,0,0);

    cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

    std::vector<cv::Point3f> objp;
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            objp.push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }

    cv::Mat rvec, tvec;
    cv::solvePnP(objp, corners, camera_matrix, dist_coeffs, rvec, tvec);

    float center_x = (board_size.width - 1) * square_size / 2.0f;
    float center_y = (board_size.height - 1) * square_size / 2.0f;

    cv::Mat center_board = (cv::Mat_<double>(3,1) << center_x, center_y, 0);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat center_cam = R * center_board + tvec;

    return cv::Point3f(center_cam.at<double>(0), center_cam.at<double>(1), center_cam.at<double>(2));
}

// 激光平面标定辅助函数
std::vector<cv::Point2f> extractLaserCenterline(const cv::Mat& img_laser,
                                                const std::vector<cv::Point2f>& corners,
                                                cv::Size board_size,
                                                int threshold = 30) {
    cv::Mat gray;
    if (img_laser.channels() == 3)
        cv::cvtColor(img_laser, gray, cv::COLOR_BGR2GRAY);
    else
        gray = img_laser.clone();

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.0);

    std::vector<cv::Point2f> center_points;

    // 获取标定板ROI
    cv::Rect roi = getBoardROI(corners, board_size);

    // 保存ROI可视化
    cv::Mat roi_debug = gray.clone();
    cv::rectangle(roi_debug, roi, cv::Scalar(255, 255, 255), 2);
    cv::imwrite("./debug_roi.png", roi_debug);
    qDebug() << "已保存ROI可视化 debug_roi.png";

    // 动态阈值
    cv::Mat roi_img = blurred(roi);
    cv::Scalar mean_val = cv::mean(roi_img);
    int dynamic_threshold = (std::max)(threshold, (int)mean_val[0] + 30);
    qDebug() << "动态阈值:" << dynamic_threshold << "ROI均值:" << mean_val[0];

    // 按行扫描（横向激光应该按列，但先确认激光线方向）
    int points_found = 0;

    for (int col = roi.x; col < roi.x + roi.width; col++) {
        float sum_intensity = 0;
        float sum_y = 0;
        int valid_count = 0;

        for (int row = roi.y; row < roi.y + roi.height; row++) {
            int intensity = blurred.at<uchar>(row, col);
            if (intensity > dynamic_threshold) {
                sum_intensity += intensity;
                sum_y += row * intensity;
                valid_count++;
            }
        }

        if (valid_count > 1 && sum_intensity > 0) {
            float center_y = sum_y / sum_intensity;
            center_points.push_back(cv::Point2f(col, center_y));
            points_found++;
        }
    }



    qDebug() << "提取到" << points_found << "个激光点";
    return center_points;
}

//相机标定实现
void Cloud_selve::calibrateCameraFromImages(const QString& imageDir,
                                            int board_width,
                                            int board_height,
                                            float square_size_mm,
                                            QString filename)
{
    // 1. 检查目录
    QDir dir(imageDir);
    if (!dir.exists()) {
        qDebug() << "目录不存在:" << imageDir;
        return;
    }

    // 2. 棋盘格参数
    cv::Size board_size(board_width, board_height);

    // 3. 准备世界坐标系中的3D点
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            objp.push_back(cv::Point3f(j * square_size_mm, i * square_size_mm, 0));
        }
    }

    // 4. 存储角点检测结果
    std::vector<std::vector<cv::Point2f>> all_image_points;
    std::vector<std::vector<cv::Point3f>> all_object_points;

    // 5. 遍历图片
    std::string image_path = (imageDir + "/*.png").toStdString();
    std::vector<cv::String> fn;
    cv::glob(image_path, fn, false);

    cv::Size image_size;
    int valid_count = 0;

    for (auto &fname : fn) {
        qDebug() << "处理图片:" << QString::fromStdString(fname);
        cv::Mat img = cv::imread(fname);
        if (img.empty()) {
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        image_size = gray.size();

        std::vector<cv::Point2f> corners;
        bool ret = cv::findChessboardCorners(gray, board_size, corners,
                                             cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

        if (ret) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            all_image_points.push_back(corners);
            all_object_points.push_back(objp);

            cv::drawChessboardCorners(img, board_size, corners, ret);
            cv::imshow("检测到的角点", img);
            cv::waitKey(200);
            valid_count++;
        } else {
            qDebug() << "未找到棋盘格:" << QString::fromStdString(fname);
        }
    }
    cv::destroyAllWindows();

    if (all_image_points.empty()) {
        qDebug() << "错误: 没有检测到棋盘格！";
        return;
    }

    qDebug() << "成功检测到" << valid_count << "张有效图片";

    if (valid_count < 10) {
        qDebug() << "有效图片小于10张，不符合标定要求";
        return;
    }

    // 6. 执行相机标定
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(all_object_points, all_image_points, image_size,
                                     camera_matrix, dist_coeffs, rvecs, tvecs);

    qDebug() << "标定完成！重投影误差:" << rms;

    // 7. 保存结果到文件
    cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_coefficients" << dist_coeffs;
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "rms_error" << rms;
    fs.release();

    qDebug() << "结果已保存到 camera_calib.yaml";
}

void Cloud_selve::calibrateLaserPlane(const QString& cameraCalibFile,
                                      const QString& calibImgPose1,
                                      const QString& laserImgPose1,
                                      const QString& calibImgPose2,
                                      const QString& laserImgPose2,
                                      int board_width,
                                      int board_height,
                                      float square_size_mm,
                                      QString filename)
{
    // 1. 加载相机参数
    cv::FileStorage fs(cameraCalibFile.toStdString(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        qDebug()<<"错误: 找不到相机标定文件,请先完成相机标定!";
        return;
    }

    cv::Mat camera_matrix, dist_coeffs;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    fs.release();

    // 2. 读取图片
    cv::Mat calib_img_pose1 = cv::imread(calibImgPose1.toStdString());
    cv::Mat laser_img_pose1 = cv::imread(laserImgPose1.toStdString());
    cv::Mat calib_img_pose2 = cv::imread(calibImgPose2.toStdString());
    cv::Mat laser_img_pose2 = cv::imread(laserImgPose2.toStdString());

    if (calib_img_pose1.empty() || laser_img_pose1.empty() ||
        calib_img_pose2.empty() || laser_img_pose2.empty()) {
        qDebug()<<"错误：标定图片不存在,检查图片是否正确";
        return;
    }

    // 3. 准备标定板3D点（两个位置都用Z=0）
    cv::Size board_size(board_width, board_height);
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            objp.push_back(cv::Point3f(j * square_size_mm, i * square_size_mm, 0));
        }
    }

    // 4. 检测角点并计算外参
    std::vector<cv::Point2f> corners1, corners2;
    cv::Mat gray1, gray2;
    cv::cvtColor(calib_img_pose1, gray1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(calib_img_pose2, gray2, cv::COLOR_BGR2GRAY);

    bool found1 = cv::findChessboardCorners(gray1, board_size, corners1,
                                            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
    bool found2 = cv::findChessboardCorners(gray2, board_size, corners2,
                                            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!found1 || !found2) {
        qDebug()<<"错误: 无法检测到棋盘格角点";
        return;
    }

    // 亚像素优化
    cv::cornerSubPix(gray1, corners1, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    cv::cornerSubPix(gray2, corners2, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

    // 求解外参
    cv::Mat rvec1, tvec1, rvec2, tvec2;
    cv::solvePnP(objp, corners1, camera_matrix, dist_coeffs, rvec1, tvec1);
    cv::solvePnP(objp, corners2, camera_matrix, dist_coeffs, rvec2, tvec2);

    // 打印tvec Z值，用于验证
    qDebug() << "位置1 tvec Z:" << tvec1.at<double>(2);
    qDebug() << "位置2 tvec Z:" << tvec2.at<double>(2);
    qDebug() << "Z差值:" << (tvec2.at<double>(2) - tvec1.at<double>(2)) << "mm";

    // 5. 提取激光条纹中心线
    std::vector<cv::Point2f> laser_pts1_2d = extractLaserCenterline(laser_img_pose1, corners1, board_size, 30);
    std::vector<cv::Point2f> laser_pts2_2d = extractLaserCenterline(laser_img_pose2, corners2, board_size, 30);

    if (laser_pts1_2d.empty() || laser_pts2_2d.empty()) {
        qDebug()<<"错误: 未提取到激光点，请检查激光是否打开且清晰可见";
        return;
    }

    // 6. 计算激光点在相机坐标系下的3D坐标（使用棋盘格平面方程）
    std::vector<cv::Point3f> all_points;

    // 处理位姿1
    cv::Mat R1;
    cv::Rodrigues(rvec1, R1);
    cv::Mat normal1 = R1.col(2);                          // 棋盘格平面法向量
    cv::Mat plane_pt1 = R1 * (cv::Mat_<double>(3,1) << 0, 0, 0) + tvec1;  // 棋盘格原点在相机坐标系下的位置

    for (const auto& pt : laser_pts1_2d) {
        std::vector<cv::Point2f> in = {pt};
        std::vector<cv::Point2f> undist;
        cv::undistortPoints(in, undist, camera_matrix, dist_coeffs);

        cv::Point3f ray(undist[0].x, undist[0].y, 1.0f);

        // 射线与棋盘格平面的交点
        double denominator = ray.x * normal1.at<double>(0) +
                             ray.y * normal1.at<double>(1) +
                             ray.z * normal1.at<double>(2);
        if (fabs(denominator) < 1e-10) continue;

        double numerator = normal1.dot(plane_pt1);
        double t = numerator / denominator;

        cv::Point3f p3d = ray * t;
        all_points.push_back(p3d);
    }

    // 处理位姿2
    cv::Mat R2;
    cv::Rodrigues(rvec2, R2);
    cv::Mat normal2 = R2.col(2);
    cv::Mat plane_pt2 = R2 * (cv::Mat_<double>(3,1) << 0, 0, 0) + tvec2;

    for (const auto& pt : laser_pts2_2d) {
        std::vector<cv::Point2f> in = {pt};
        std::vector<cv::Point2f> undist;
        cv::undistortPoints(in, undist, camera_matrix, dist_coeffs);

        cv::Point3f ray(undist[0].x, undist[0].y, 1.0f);

        double denominator = ray.x * normal2.at<double>(0) +
                             ray.y * normal2.at<double>(1) +
                             ray.z * normal2.at<double>(2);
        if (fabs(denominator) < 1e-10) continue;

        double numerator = normal2.dot(plane_pt2);
        double t = numerator / denominator;

        cv::Point3f p3d = ray * t;
        all_points.push_back(p3d);
    }

    if (all_points.size() < 10) {
        qDebug()<<"错误: 3D点数量不足，无法拟合光平面";
        return;
    }

    // 7. 保存3D点用于调试
    std::ofstream debug_pts("debug_points.xyz");
    for (const auto& pt : all_points) {
        debug_pts << pt.x << " " << pt.y << " " << pt.z << std::endl;
    }
    debug_pts.close();

    // 8. 拟合光平面
    cv::Vec4f light_plane = fitPlane(all_points);

    // 计算拟合误差
    double rms_error = 0;
    for (const auto& pt : all_points) {
        double distance = fabs(light_plane[0] * pt.x + light_plane[1] * pt.y +
                               light_plane[2] * pt.z + light_plane[3]);
        rms_error += distance * distance;
    }
    rms_error = sqrt(rms_error / all_points.size());

    // 9. 保存结果
    cv::FileStorage fs_out(filename.toStdString(), cv::FileStorage::WRITE);
    fs_out << "light_plane" << light_plane;
    fs_out << "fitting_rms" << rms_error;
    fs_out << "num_points" << (int)all_points.size();
    fs_out.release();

    qDebug() << "激光平面标定完成，结果已保存到" << filename;
    qDebug() << "平面拟合RMS误差:" << rms_error;
    qDebug() << "激光平面参数 (a,b,c,d):"
             << light_plane[0] << ", " << light_plane[1] << ", "
             << light_plane[2] << ", " << light_plane[3];
}


// 轨道运动标定实现
void Cloud_selve::calibrateMotion(const QString& cameraCalibFile,
                                  const QString& imgBefore,
                                  const QString& imgAfter,
                                  double movement_mm,
                                  int board_width,
                                  int board_height,
                                  float square_size_mm,
                                  QString filename,
                                  int step_count)
{
    // 1. 加载相机参数
    cv::FileStorage fs_cam(cameraCalibFile.toStdString(), cv::FileStorage::READ);
    if (!fs_cam.isOpened()) {
        qDebug() << "错误: 找不到相机标定文件" << cameraCalibFile;
        qDebug() << "请先完成相机标定！";
        return;
    }

    cv::Mat camera_matrix, dist_coeffs;
    fs_cam["camera_matrix"] >> camera_matrix;
    fs_cam["distortion_coefficients"] >> dist_coeffs;
    fs_cam.release();

    qDebug() << "✓ 相机参数加载成功";

    // 2. 读取图片
    cv::Mat img_before = cv::imread(imgBefore.toStdString());
    cv::Mat img_after = cv::imread(imgAfter.toStdString());

    if (img_before.empty() || img_after.empty()) {
        qDebug() << "错误: 无法读取图片";
        qDebug() << "需要图片:" << imgBefore << "和" << imgAfter;
        return;
    }

    // 3. 检查移动距离
    if (movement_mm <= 0) {
        qDebug() << "错误: 移动距离需要大于0";
        return;
    }

    // 4. 创建标定器并执行标定
    cv::Size board_size(board_width, board_height);
    MotionCalibrator calibrator(camera_matrix, dist_coeffs, board_size, square_size_mm);

    try {
        MotionCalibResult result = calibrator.calibrate(img_before, img_after, movement_mm, step_count);

        qDebug() << "========================================";
        qDebug() << "\n【运动方向向量】";
        qDebug() << "  X:" << result.direction[0];
        qDebug() << "  Y:" << result.direction[1];
        qDebug() << "  Z:" << result.direction[2];

        if (fabs(result.direction[0]) > 0.95) {
            qDebug() << "  → 运动方向: 沿X轴";
        } else if (fabs(result.direction[1]) > 0.95) {
            qDebug() << "  → 运动方向: 沿Y轴";
        } else if (fabs(result.direction[2]) > 0.95) {
            qDebug() << "  → 运动方向: 沿Z轴";
        } else {
            qDebug() << "  → 运动方向: 复合方向";
        }

        qDebug() << "\n【位移信息】";
        qDebug() << "  视觉测量位移:" << result.total_displacement << "mm";
        qDebug() << "  实际移动距离:" << movement_mm << "mm";
        qDebug() << "  位移向量: ("
                 << result.displacement_vec[0] << ", "
                 << result.displacement_vec[1] << ", "
                 << result.displacement_vec[2] << ")";

        if (step_count > 0) {
            qDebug() << "\n【步长信息】";
            qDebug() << "  移动步数:" << step_count;
            qDebug() << "  每步移动距离:" << result.step_size_mm << "mm/步";
        }

        // 5. 保存标定结果
        cv::FileStorage fs_out(filename.toStdString(), cv::FileStorage::WRITE);
        fs_out << "motion_direction" << result.direction;
        fs_out << "displacement_vector" << result.displacement_vec;
        fs_out << "total_displacement_mm" << result.total_displacement;
        fs_out << "actual_movement_mm" << movement_mm;
        fs_out << "step_size_mm_per_step" << result.step_size_mm;
        fs_out << "step_count" << step_count;
        fs_out << "calibration_date" << std::time(nullptr);
        fs_out.release();

        qDebug() << "\n✓ 标定结果已保存到 motion_calib.yaml";

    } catch (const std::exception& e) {
        qDebug() << "❌ 标定失败:" << e.what();
        qDebug() << "请检查图片是否清晰，棋盘格是否完整可见";
    }
}

// 多点运动标定实现
void Cloud_selve::calibrateMotionMultiPoint(const QString& cameraCalibFile,
                                            const std::vector<QString>& images,
                                            const std::vector<double>& movements_mm,
                                            int board_width,
                                            int board_height,
                                            float square_size_mm,
                                            QString filename,
                                            int step_count)
{
    // 1. 加载相机参数
    cv::FileStorage fs_cam(cameraCalibFile.toStdString(), cv::FileStorage::READ);
    if (!fs_cam.isOpened()) {
        qDebug() << "错误: 找不到相机标定文件" << cameraCalibFile;
        return;
    }

    cv::Mat camera_matrix, dist_coeffs;
    fs_cam["camera_matrix"] >> camera_matrix;
    fs_cam["distortion_coefficients"] >> dist_coeffs;
    fs_cam.release();

    qDebug() << "✓ 相机参数加载成功";

    // 2. 读取所有图片
    std::vector<cv::Mat> img_list;
    for (const auto& path : images) {
        cv::Mat img = cv::imread(path.toStdString());
        if (img.empty()) {
            qDebug() << "错误: 无法读取图片" << path;
            return;
        }
        img_list.push_back(img);
    }

    if (img_list.size() != movements_mm.size() || img_list.size() < 2) {
        qDebug() << "错误: 图片数量与移动距离数量不匹配";
        return;
    }

    // 3. 执行多点标定
    cv::Size board_size(board_width, board_height);
    MotionCalibrator calibrator(camera_matrix, dist_coeffs, board_size, square_size_mm);

    try {
        MotionCalibResult result = calibrator.calibrateMultiPoint(img_list, movements_mm, step_count);

        qDebug() << "========================================";
        qDebug() << "\n【多点标定结果】";
        qDebug() << "  运动方向: ("
                 << result.direction[0] << ", "
                 << result.direction[1] << ", "
                 << result.direction[2] << ")";
        qDebug() << "  总位移:" << result.total_displacement << "mm";
        qDebug() << "  线性误差:" << result.linearity_error << "mm";
        qDebug() << "  运动线性度:" << (result.is_linear_movement ? "良好" : "较差");

        if (step_count > 0) {
            qDebug() << "  步长:" << result.step_size_mm << "mm/步";
        }

        // 4. 保存结果
        cv::FileStorage fs_out(filename.toStdString(), cv::FileStorage::WRITE);
        fs_out << "motion_direction" << result.direction;
        fs_out << "displacement_vector" << result.displacement_vec;
        fs_out << "total_displacement_mm" << result.total_displacement;
        fs_out << "linearity_error_mm" << result.linearity_error;
        fs_out << "is_linear_movement" << result.is_linear_movement;
        fs_out << "step_size_mm_per_step" << result.step_size_mm;
        fs_out << "step_count" << step_count;
        fs_out << "num_points" << (int)images.size();
        fs_out << "calibration_date" << std::time(nullptr);
        fs_out.release();

        qDebug() << "\n✓ 多点标定结果已保存到 motion_calib.yaml";

    } catch (const std::exception& e) {
        qDebug() << "❌ 多点标定失败:" << e.what();
    }
}

bool Cloud_selve::generatePointCloud(const QString& imageDir,
                                     const QString& cameraCalibFile,
                                     const QString& laserPlaneFile,
                                     const QString& motionCalibFile,
                                     const LaserScanConfig& config,
                                     const double& alxeposition,
                                     const double& A,
                                     const double& B,
                                     const double& C)
{
    // 检查目录
    QDir dir(imageDir);
    if (!dir.exists()) {
        qDebug() << "目录不存在:" << imageDir;
        return false;
    }

    // 创建调试目录
    if (config.save_debug_images && !config.debug_dir.empty()) {
        std::filesystem::create_directory(config.debug_dir);
    }

    // 创建扫描器，传入颜色通道参数
    LaserScannerInternal scanner(config.laser_threshold, config.color_channel,config.column_sampling_step);

    // 加载标定参数
    if (!scanner.loadCalibration(cameraCalibFile.toStdString(),
                                 laserPlaneFile.toStdString(),
                                 motionCalibFile.toStdString())) {
        qDebug() << "标定参数加载失败，请先完成标定！";
        return false;
    }

    // 处理图像
    bool success = scanner.processImageFolder(imageDir.toStdString(),
                                              "*.png",
                                              alxeposition,
                                              A,
                                              B,
                                              C,
                                              config.save_debug_images,
                                              config.debug_dir);

    if (success) {
        std::string full_filename = config.output_filename + "." + config.output_format;
        scanner.savePointCloud(full_filename, config.output_format);
        qDebug() << "点云生成成功，文件名为" << QString::fromStdString(full_filename);
        return true;
    } else {
        qDebug() << "点云生成失败！";
        return false;
    }
}


bool Cloud_selve::generatePointCloudSimple(const QString& imageDir,
                                           const QString& cameraCalibFile,
                                           const QString& laserPlaneFile,
                                           const QString& motionCalibFile,
                                           const double& alxeposition,
                                           const double& A,
                                           const double& B,
                                           const double& C)
{
    LaserScanConfig config;
    return generatePointCloud(imageDir, cameraCalibFile, laserPlaneFile, motionCalibFile, config , alxeposition,A,B,C);
}


void Cloud_selve::calibrateStereoCamera(const QString& cameraCalibFile1,
                                        const QString& cameraCalibFile2,
                                        const QString& imageDir1,
                                        const QString& imageDir2,
                                        int board_width,
                                        int board_height,
                                        float square_size_mm,
                                        QString filename)
{
    // 1. 加载两个相机的内参
    cv::Mat K1, D1, K2, D2;

    cv::FileStorage fs1(cameraCalibFile1.toStdString(), cv::FileStorage::READ);
    if (!fs1.isOpened()) {
        qDebug() << "Failed to open camera calib file 1:" << cameraCalibFile1;
        return;
    }
    fs1["camera_matrix"] >> K1;
    fs1["distortion_coefficients"] >> D1;
    fs1.release();

    cv::FileStorage fs2(cameraCalibFile2.toStdString(), cv::FileStorage::READ);
    if (!fs2.isOpened()) {
        qDebug() << "Failed to open camera calib file 2:" << cameraCalibFile2;
        return;
    }
    fs2["camera_matrix"] >> K2;
    fs2["distortion_coefficients"] >> D2;
    fs2.release();

    // 2. 获取两个相机的标定图片
    QDir dir1(imageDir1);
    QDir dir2(imageDir2);
    QStringList filters;
    filters << "*.jpg" << "*.png" << "*.bmp";

    QStringList imageFiles1 = dir1.entryList(filters, QDir::Files);
    QStringList imageFiles2 = dir2.entryList(filters, QDir::Files);

    if (imageFiles1.size() != imageFiles2.size() || imageFiles1.empty()) {
        qDebug() << "Image count mismatch or empty";
        qDebug() << "Camera1 images:" << imageFiles1.size();
        qDebug() << "Camera2 images:" << imageFiles2.size();
        return;
    }

    // 3. 准备标定数据
    std::vector<std::vector<cv::Point2f>> corners1, corners2;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    cv::Size boardSize(board_width, board_height);

    // 获取第一张图片的尺寸（假设所有图片尺寸相同）
    cv::Mat firstImg = cv::imread((imageDir1 + "/" + imageFiles1[0]).toStdString());
    cv::Size imageSize = firstImg.size();

    for (int i = 0; i < imageFiles1.size(); i++) {
        cv::Mat img1 = cv::imread((imageDir1 + "/" + imageFiles1[i]).toStdString());
        cv::Mat img2 = cv::imread((imageDir2 + "/" + imageFiles2[i]).toStdString());

        if (img1.empty() || img2.empty()) {
            qDebug() << "Failed to load image pair:" << i;
            continue;
        }

        std::vector<cv::Point2f> corner1, corner2;
        bool found1 = cv::findChessboardCorners(img1, boardSize, corner1);
        bool found2 = cv::findChessboardCorners(img2, boardSize, corner2);

        if (found1 && found2) {
            cv::Mat gray1, gray2;
            cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
            cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

            cv::cornerSubPix(gray1, corner1, cv::Size(11,11), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::cornerSubPix(gray2, corner2, cv::Size(11,11), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            corners1.push_back(corner1);
            corners2.push_back(corner2);

            // 生成3D物体点
            std::vector<cv::Point3f> objp;
            for (int r = 0; r < board_height; r++) {
                for (int c = 0; c < board_width; c++) {
                    objp.push_back(cv::Point3f(c * square_size_mm, r * square_size_mm, 0));
                }
            }
            objectPoints.push_back(objp);
        }
    }

    if (objectPoints.size() < 3) {
        qDebug() << "Not enough valid image pairs, found:" << objectPoints.size();
        return;
    }

    // 4. 双目相机标定
    cv::Mat R, t, E, F;
    double reprojError = cv::stereoCalibrate(objectPoints, corners1, corners2,
                                             K1, D1, K2, D2,
                                             imageSize,  // 使用获取的图像尺寸
                                             R, t, E, F,
                                             cv::CALIB_FIX_INTRINSIC);

    qDebug() << "Stereo calibration reprojection error:" << reprojError;

    // 5. 保存结果
    cv::FileStorage fsOut(filename.toStdString(), cv::FileStorage::WRITE);
    fsOut << "R" << R;
    fsOut << "t" << t;
    fsOut << "E" << E;
    fsOut << "F" << F;
    fsOut << "reprojection_error" << reprojError;
    fsOut.release();

    qDebug() << "Stereo calibration saved to:" << filename;
}


bool Cloud_selve::transformCam2ToCam1(const QString& cloudFileFromCam2,
                                      const QString& outputCloudFile,
                                      const QString& stereoCalibFile)
{
    // 1. 读取外参（相机2到相机1的变换）
    cv::Mat R, t;
    cv::FileStorage fs(stereoCalibFile.toStdString(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        qDebug() << "Failed to open stereo calibration file:" << stereoCalibFile;
        return false;
    }
    fs["R"] >> R;
    fs["t"] >> t;
    fs.release();

    qDebug() << "Loaded transformation matrix R (rows x cols):" << R.rows << "x" << R.cols;
    qDebug() << "Loaded translation vector t (rows x cols):" << t.rows << "x" << t.cols;

    // 2. 读取点云文件（支持 PCD 和 XYZ 格式）
    std::vector<cv::Point3f> sourcePoints;

    std::ifstream infile(cloudFileFromCam2.toStdString());
    if (!infile.is_open()) {
        qDebug() << "Failed to open point cloud file:" << cloudFileFromCam2;
        return false;
    }

    std::string line;
    bool isPCD = false;
    bool isBinaryPCD = false;
    bool dataSection = false;
    int pointCount = 0;
    int pointsRead = 0;

    // 先检查文件头，判断格式
    while (std::getline(infile, line)) {
        // 去除行尾的回车符
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        // 检测 PCD 文件
        if (line.find("# .PCD") != std::string::npos) {
            isPCD = true;
            qDebug() << "Detected PCD format";
        }

        // 读取点数
        if (line.find("POINTS") != std::string::npos) {
            std::stringstream ss(line);
            std::string token;
            ss >> token >> pointCount;
            qDebug() << "Expected points:" << pointCount;
        }

        // 检测数据段开始
        if (line.find("DATA") != std::string::npos) {
            dataSection = true;
            if (line.find("binary") != std::string::npos) {
                isBinaryPCD = true;
                qDebug() << "Binary PCD format detected (not supported yet)";
                qDebug() << "Please convert PCD to ASCII format using pcl_convert_pcd_ascii";
                infile.close();
                return false;
            } else {
                qDebug() << "ASCII PCD format detected";
            }
            break;
        }

        // 对于简单 XYZ 格式，没有头信息就直接是数据
        if (!isPCD && !line.empty() && (isdigit(line[0]) || line[0] == '-' || line[0] == '.')) {
            dataSection = true;
            // 回退一行，让后面的循环读取这个数据行
            infile.seekg(-static_cast<int>(line.length()) - 1, std::ios::cur);
            break;
        }
    }

    // 读取点云数据
    double x, y, z;
    while (infile >> x >> y >> z) {
        sourcePoints.push_back(cv::Point3f(x, y, z));
        pointsRead++;

        // 如果知道总点数，可以显示进度
        if (pointCount > 0 && pointsRead % 10000 == 0) {
            qDebug() << "Read" << pointsRead << "/" << pointCount << "points...";
        }
    }
    infile.close();

    qDebug() << "Loaded" << sourcePoints.size() << "points from" << cloudFileFromCam2;

    if (sourcePoints.empty()) {
        qDebug() << "Point cloud is empty";
        qDebug() << "Please check if the file format is ASCII PCD or XYZ";
        return false;
    }

    // 3. 转换每个点：P_cam1 = R * P_cam2 + t
    std::vector<cv::Point3f> transformedPoints;
    transformedPoints.reserve(sourcePoints.size());

    qDebug() << "Transforming points...";
    for (size_t i = 0; i < sourcePoints.size(); i++) {
        const auto& point = sourcePoints[i];
        cv::Mat pt = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
        cv::Mat transformed =  R*pt + t;
        // 确保获取值的方式正确
        double x_out = transformed.at<double>(0, 0);  // 明确指定行列
        double y_out = transformed.at<double>(1, 0);
        double z_out = transformed.at<double>(2, 0);

        transformedPoints.push_back(cv::Point3f(x_out, y_out, z_out));

        // 显示进度
        if ((i + 1) % 10000 == 0) {
            qDebug() << "Transformed" << (i + 1) << "/" << sourcePoints.size() << "points...";
        }
    }

    // 4. 保存转换后的点云（保持 PCD 格式）
    std::ofstream outfile(outputCloudFile.toStdString());
    if (!outfile.is_open()) {
        qDebug() << "Failed to open output file:" << outputCloudFile;
        return false;
    }

    // 保存为 ASCII PCD 格式
    outfile << "# .PCD v0.7 - Point Cloud Data file format\n";
    outfile << "VERSION 0.7\n";
    outfile << "FIELDS x y z\n";
    outfile << "SIZE 4 4 4\n";
    outfile << "TYPE F F F\n";
    outfile << "COUNT 1 1 1\n";
    outfile << "WIDTH " << transformedPoints.size() << "\n";
    outfile << "HEIGHT 1\n";
    outfile << "VIEWPOINT 0 0 0 1 0 0 0\n";
    outfile << "POINTS " << transformedPoints.size() << "\n";
    outfile << "DATA ascii\n";

    for (const auto& p : transformedPoints) {
        outfile << std::fixed << std::setprecision(6) << p.x << " " << p.y << " " << p.z << "\n";
    }
    outfile.close();

    qDebug() << "Transformed point cloud saved to:" << outputCloudFile;
    qDebug() << "Converted" << transformedPoints.size() << "points from camera2 to camera1 coordinate system";

    return true;
}


bool Cloud_selve::calibrateHandToEyeTwoHeights(const std::string& cameraIntrinFile,
                                               const std::string& imagePathLow,
                                               const std::string& imagePathHigh,
                                               const std::string& savePath,
                                               const cv::Size& boardSize,
                                               const std::vector<int>& selectedCorners,
                                               const std::vector<cv::Point3f>& robotPointsLow,
                                               const std::vector<cv::Point3f>& robotPointsHigh,
                                               bool visualize)
{

    if (selectedCorners.size() < 4) {
        qDebug() << "[ERROR] 每张图片至少需要 4 个角点（推荐 6 个）" ;
        return false;
    }

    if (selectedCorners.size() != robotPointsLow.size() ||
        selectedCorners.size() != robotPointsHigh.size()) {
        qDebug() << "[ERROR] 角点数量与机器人坐标数量不匹配！" ;
        qDebug() << "  selectedCorners.size() = " << selectedCorners.size() ;
        qDebug() << "  robotPointsLow.size() = " << robotPointsLow.size() ;
        qDebug() << "  robotPointsHigh.size() = " << robotPointsHigh.size() ;
        return false;
    }

    // ============================================================
    // 第一步：加载相机内参
    // ============================================================
    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fsIntrin(cameraIntrinFile, cv::FileStorage::READ);
    if (!fsIntrin.isOpened()) {
        qDebug() << "[ERROR] 无法打开相机内参文件: " << cameraIntrinFile;
        return false;
    }
    fsIntrin["camera_matrix"] >> cameraMatrix;
    fsIntrin["distortion_coefficients"] >> distCoeffs;
    fsIntrin.release();

    if (cameraMatrix.empty() || distCoeffs.empty()) {
        qDebug() << "[ERROR] 相机内参文件格式错误" ;
        return false;
    }

    qDebug() << "[INFO] 相机内参加载成功";

    // ============================================================
    // 第二步：提取两张图片的棋盘格角点
    // ============================================================
    auto extractCorners = [&](const std::string& imagePath, const std::string& heightDesc) -> std::vector<cv::Point2f> {
        cv::Mat image = cv::imread(imagePath);
        if (image.empty()) {
            qDebug() << "[ERROR] 无法读取图片: " << imagePath;
            return {};
        }

        cv::Mat gray;
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = findChessboardCorners(gray, boardSize, corners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (!found) {
            qDebug() << "[ERROR] 未在图片中找到棋盘格角点: " << imagePath;
            return {};
        }

        // 亚像素精确定位
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001);
        cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

        qDebug() << "[INFO] " << heightDesc << " 图片成功提取 " << corners.size() << " 个角点";

        // 可视化（可选）
        if (visualize) {
            cv::Mat displayImg = image.clone();
            drawChessboardCorners(displayImg, boardSize, corners, found);
            // 标记选中的角点
            for (int idx : selectedCorners) {
                if (idx >= 0 && idx < (int)corners.size()) {
                    circle(displayImg, corners[idx], 6, cv::Scalar(0, 255, 0), -1);
                    putText(displayImg, std::to_string(idx), corners[idx] + cv::Point2f(10, -10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                }
            }
            cv::imshow(heightDesc + " 标定板", displayImg);
            cv::waitKey(1);
        }

        return corners;
    };

    std::vector<cv::Point2f> cornersLow = extractCorners(imagePathLow, "低位");
    std::vector<cv::Point2f> cornersHigh = extractCorners(imagePathHigh, "高位");

    if (cornersLow.empty() || cornersHigh.empty()) {
        qDebug() << "[ERROR] 角点提取失败";
        return false;
    }

    // ============================================================
    // 第三步：构建点对（合并两张图片的数据）
    // ============================================================
    std::vector<cv::Point3f> objectPoints;  // 世界坐标
    std::vector<cv::Point2f> imagePoints;   // 像素坐标

    // 添加低位图片的点
    for (size_t i = 0; i < selectedCorners.size(); i++) {
        int idx = selectedCorners[i];
        if (idx < 0 || idx >= (int)cornersLow.size()) {
            qDebug() << "[ERROR] 低位图片角点索引 " << idx << " 超出范围";
            return false;
        }
        imagePoints.push_back(cornersLow[idx]);
        objectPoints.push_back(robotPointsLow[i]);
    }

    // 添加高位图片的点
    for (size_t i = 0; i < selectedCorners.size(); i++) {
        int idx = selectedCorners[i];
        if (idx < 0 || idx >= (int)cornersHigh.size()) {
            qDebug() << "[ERROR] 高位图片角点索引 " << idx << " 超出范围";
            return false;
        }
        imagePoints.push_back(cornersHigh[idx]);
        objectPoints.push_back(robotPointsHigh[i]);
    }

    qDebug() << "[INFO] 成功构建 " << imagePoints.size() << " 组点对（低位 "
         << robotPointsLow.size() << " 组 + 高位 " << robotPointsHigh.size() << " 组）";

    // 打印Z方向变化范围，验证深度标定效果
    float zMin = FLT_MAX, zMax = -FLT_MAX;
    for (const auto& p : objectPoints) {
        if (p.z < zMin) zMin = p.z;
        if (p.z > zMax) zMax = p.z;
    }
    qDebug() << "[INFO] Z方向范围: " << zMin << " ~ " << zMax << " mm，差值 = " << (zMax - zMin) << " mm";

    // ============================================================
    // 第四步：PnP 求解
    // ============================================================
    cv::Mat rvec, tvec;
    bool success = solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs,
                            rvec, tvec, false, cv::SOLVEPNP_EPNP);

    if (!success) {
        qDebug() << "[ERROR] PnP 求解失败";
        return false;
    }

    // 将旋转向量转换为旋转矩阵
    cv::Mat R;
    Rodrigues(rvec, R);

    // 计算重投影误差
    std::vector<cv::Point2f> projectedPoints;
    projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    double totalError = 0, maxError = 0;
    for (size_t i = 0; i < imagePoints.size(); i++) {
        double error = norm(imagePoints[i] - projectedPoints[i]);
        totalError += error;
        if (error > maxError) maxError = error;
    }
    double meanError = totalError / imagePoints.size();

    qDebug() << "[INFO] 重投影平均误差: " << meanError << " 像素";
    qDebug() << "[INFO] 重投影最大误差: " << maxError << " 像素";

    cv::Mat R_inv = R.t();  // 旋转矩阵的逆 = 转置
    cv::Mat t_inv = -R_inv * tvec;

    // ============================================================
    // 第五步：保存标定结果
    // ============================================================
    cv::FileStorage fsSave(savePath, cv::FileStorage::WRITE);
    if (!fsSave.isOpened()) {
        qDebug() << "[ERROR] 无法创建保存文件: " << savePath ;
        return false;
    }

    // 写入标定结果
    fsSave << "camera_matrix" << cameraMatrix;
    fsSave << "distortion_coefficients" << distCoeffs;
    fsSave << "rotation_matrix" << R_inv;
    fsSave << "translation_vector" << t_inv;
    fsSave << "rotation_vector" << rvec;
    fsSave << "reprojection_error_mean" << meanError;
    fsSave << "reprojection_error_max" << maxError;
    fsSave << "num_calibration_points" << (int)objectPoints.size();
    fsSave << "height_difference" << (zMax - zMin);  // 记录高度差，便于评估

    // 记录原始数据
    fsSave << "selected_corner_indices" << selectedCorners;
    fsSave << "points_pixel" << imagePoints;
    fsSave << "points_world" << objectPoints;

    fsSave.release();


    if (visualize) {
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return true;
}


bool Cloud_selve::loadHandToEyeCalibration(const std::string& calibFile,
                              cv::Mat& R,
                              cv::Mat& t)
{

    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        qDebug()<<"[ERROR] 无法打开标定文件: " << calibFile;
        return false;
    }

    // 读取旋转矩阵和平移向量
    fs["rotation_matrix"] >> R;
    fs["translation_vector"] >> t;


    fs.release();

    if (R.empty() || t.empty()) {
        qDebug()<<"[ERROR] 标定文件格式错误，缺少 rotation_matrix 或 translation_vector";
        return false;
    }
    qDebug()<<"[INFO] 标定文件加载成功";
    return true;
}

bool Cloud_selve::transformCameraToBase(const std::string& calibFile,
                           const std::vector<cv::Point3f>& pointsCamera,
                           std::vector<cv::Point3f>& pointsBase)
{

    if (pointsCamera.empty()) {
        qDebug()<<"[WARNING] 输入点为空";
        return false;
    }

    // 加载标定文件
    cv::Mat R, t;
    if (!loadHandToEyeCalibration(calibFile, R, t)) {
        return false;
    }

    // 转换每个点
    pointsBase.clear();
    pointsBase.reserve(pointsCamera.size());

    for (const auto& pt : pointsCamera) {
        // 构造齐次坐标
        cv::Mat ptCamera = (cv::Mat_<double>(3, 1) << pt.x, pt.y, pt.z);

        // 执行转换：P_base = R * P_camera + t
        cv::Mat ptBase = R * ptCamera + t;

        // 转换为 Point3f
        cv::Point3f result(
            static_cast<float>(ptBase.at<double>(0)),
            static_cast<float>(ptBase.at<double>(1)),
            static_cast<float>(ptBase.at<double>(2))
            );
        pointsBase.push_back(result);
    }

    return true;
}


bool Cloud_selve::transformCameraToBase(const std::string& calibFile,
                           const cv::Point3f& pointCamera,
                           cv::Point3f& pointBase)
{
    std::vector<cv::Point3f> pointsCamera = {pointCamera};
    std::vector<cv::Point3f> pointsBase;

    if (!transformCameraToBase(calibFile, pointsCamera, pointsBase)) {
        return false;
    }

    pointBase = pointsBase[0];
    return true;
}