#ifndef WELD_SEARCH_H
#define WELD_SEARCH_H

#include "weld_search_global.h"
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <tuple>
#include <fstream>
#include <string>
#include <sstream>
#include <QString>
#include <QDebug>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <nlohmann/json.hpp>


// 基础结构定义
struct Point2D {
    double x, y, angle;
    Point2D() : x(0), y(0), angle(0) {}
    Point2D(double _x, double _y) : x(_x), y(_y), angle(0) {}
    double distance(const Point2D& p) const {
        return std::sqrt((x-p.x)*(x-p.x)+(y-p.y)*(y-p.y));
    }
};

struct Line2D {
    double a, b, c;
    double angle;
    Line2D() : a(0), b(0), c(0), angle(0) {}
    Line2D(double _a, double _b, double _c) : a(_a), b(_b), c(_c) {
        angle = atan2(-a, b);
    }
    double pointDistance(const Point2D& p) const {
        return std::abs(a * p.x + b * p.y + c) / std::sqrt(a * a + b * b);
    }
};

struct WeldSeam2D {
    Line2D line;
    Point2D start;
    Point2D end;
    double length;
    double angle;
    std::vector<int> inlier_indices;

    WeldSeam2D() : length(0), angle(0) {}

    Point2D center() const {
        return Point2D((start.x + end.x) / 2, (start.y + end.y) / 2);
    }

    Point2D direction() const {
        return Point2D(end.x - start.x, end.y - start.y);
    }
};

struct WeldSeam3D {
    pcl::PointXYZ start;
    pcl::PointXYZ end;
    int start_stye=0;
    int end_stye=0;
    double length;
    int original_index = -1;

    double pointDistance_start(const pcl::PointXYZ& p) const {
        return std::sqrt((p.x-start.x)*(p.x-start.x)+(p.y-start.y)*(p.y-start.y)+(p.z-start.z)*(p.z-start.z));
    }
    double pointDistance_end(const pcl::PointXYZ& p) const {
        return std::sqrt((p.x-end.x)*(p.x-end.x)+(p.y-end.y)*(p.y-end.y)+(p.z-end.z)*(p.z-end.z));
    }
};

struct WeldLSeam3D {
    pcl::PointXYZ start;
    pcl::PointXYZ end;
    double angle;
};

struct Arc2D {
    double cx, cy;
    double radius;
    double start_angle;
    double end_angle;
    bool is_clockwise;

    Arc2D() : cx(0), cy(0), radius(0), start_angle(0), end_angle(0), is_clockwise(true) {}

    double pointDistance(const Point2D& p) const {
        double dist_to_center = std::hypot(p.x - cx, p.y - cy);
        double dist_to_circle = std::abs(dist_to_center - radius);

        if (start_angle == 0 && end_angle == 0) {
            return dist_to_circle;
        }

        double angle = std::atan2(p.y - cy, p.x - cx);
        if (!isAngleInRange(angle)) {
            double start_dist = std::hypot(p.x - getPointAtAngle(start_angle).x,
                                           p.y - getPointAtAngle(start_angle).y);
            double end_dist = std::hypot(p.x - getPointAtAngle(end_angle).x,
                                         p.y - getPointAtAngle(end_angle).y);
            return std::min(start_dist, end_dist);
        }
        return dist_to_circle;
    }

    bool isAngleInRange(double angle) const {
        double start = start_angle;
        double end = end_angle;
        angle = std::fmod(angle, 2 * M_PI);
        if (angle < 0) angle += 2 * M_PI;
        start = std::fmod(start, 2 * M_PI);
        if (start < 0) start += 2 * M_PI;
        end = std::fmod(end, 2 * M_PI);
        if (end < 0) end += 2 * M_PI;

        if (is_clockwise) {
            if (start > end) {
                return angle <= start && angle >= end;
            } else {
                return angle <= start || angle >= end;
            }
        } else {
            if (start < end) {
                return angle >= start && angle <= end;
            } else {
                return angle >= start || angle <= end;
            }
        }
    }

    Point2D getPointAtAngle(double angle) const {
        return Point2D(cx + radius * std::cos(angle), cy + radius * std::sin(angle));
    }

    double getArcLength() const {
        double delta = std::abs(end_angle - start_angle);
        if (delta > M_PI * 2) delta = M_PI * 2;
        if (!is_clockwise && end_angle < start_angle) delta = 2 * M_PI - delta;
        if (is_clockwise && end_angle > start_angle) delta = 2 * M_PI - delta;
        return radius * delta;
    }

    bool isMajorArc() const {
        double angle_range = std::abs(end_angle - start_angle);
        if (angle_range > 2 * M_PI) angle_range = 2 * M_PI - angle_range;
        return angle_range > M_PI;
    }
    bool isMinorArc() const {
        return !isMajorArc();
    }
};

struct WeldArc2D {
    Arc2D arc;
    Point2D start;
    Point2D middle;
    Point2D end;
    double length;
    std::vector<int> inlier_indices;

    WeldArc2D() : length(0) {}

    Point2D center() const {
        return Point2D(arc.cx, arc.cy);
    }

    double radius() const {
        return arc.radius;
    }
};

struct WeldArc3D {
    pcl::PointXYZ start;
    pcl::PointXYZ middle;
    pcl::PointXYZ end;
    pcl::PointXYZ center;
    double length;
};


enum class WeldType { FILLET, BUTT, GROOVE, UNKNOWN };// 焊接类型
enum class WeldPosition { FLAT, VERTICAL, OVERHEAD, UNKNOWN };// 焊接位置
enum class InterruptType { GY, SB, RW, WB, UNKNOWN };// 工艺中断/设备中断/人为中断/外部事件

// ============================================================================
// 3D向量工具
// ============================================================================


struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}// 从坐标构造
    Vec3(const std::vector<double>& v)
        : x(v.size() > 0 ? v[0] : 0),
        y(v.size() > 1 ? v[1] : 0),
        z(v.size() > 2 ? v[2] : 0) {}// 从向量构造（带边界检查）

    Vec3 operator+(const Vec3& other) const { return Vec3(x + other.x, y + other.y, z + other.z); }// 向量加法
    Vec3 operator-(const Vec3& other) const { return Vec3(x - other.x, y - other.y, z - other.z); }// 向量减法
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }// 点乘

    double dot(const Vec3& other) const { return x * other.x + y * other.y + z * other.z; }// 点积

    double norm() const { return std::sqrt(x * x + y * y + z * z); }// 向量模长

    Vec3 normalized() const {
        double n = norm();
        if (n < 1e-10) return Vec3(0, 0, 0);
        return Vec3(x / n, y / n, z / n);
    }// 归一化

    std::vector<double> to_vector() const { return {x, y, z}; }// 转换为向量

    std::string to_string() const {
        std::ostringstream ss;
        ss << "(" << x << ", " << y << ", " << z << ")";
        return ss.str();
    }// 转换为字符串
};

struct WeldSeamJSON {
    int id;
    int group_id;
    Vec3 start_point;
    Vec3 end_point;
    Vec3 direction;
    WeldType weld_type;
    WeldPosition weld_position;
    double plate_thickness;

    // 弧形焊缝参数
    bool is_arc;
    Vec3 center_point;
    double radius;
    double start_angle;
    double end_angle;

    // 角度表示
    double azimuth;
    double elevation;
    double angle_deg;
};


// 主要提取类
class IterativeRANSACLineFitting {
private:
    double distance_threshold_;
    int min_inliers_;
    int max_iterations_;
    int min_points_to_fit_;
    double max_gap_threshold_;
    double arc_distance_threshold_;
    int min_arc_inliers_;
    double arc_min_radius_;
    double arc_max_radius_;
    double arc_outlier_ratio_;
    double arc_angle_tolerance_;
    int arc_min_continuous_;

public:
    IterativeRANSACLineFitting();

    // 设置参数
    void setDistanceThreshold(double threshold);
    void setMinInliers(int min_inliers);
    void setMaxIterations(int max_iter);
    void setMinPointsToFit(int min_pts);
    void setMaxGapThreshold(double gap);
    void setArcDistanceThreshold(double threshold);
    void setMinArcInliers(int min_inliers);
    void setArcRadiusLimits(double min_radius, double max_radius);
    void setArcOutlierRatio(double ratio);
    void setArcAngleTolerance(double tolerance);
    void setArcMinContinuous(int min_pts);

    // 主要提取函数
    std::tuple<std::vector<WeldSeam2D>, std::vector<WeldArc2D>> extractAllWeldSeamsAndArcs(
        const std::vector<Point2D>& points);

private:
    void extractLines(const std::vector<Point2D>& points,
                      std::vector<int>& remaining_indices,
                      std::vector<bool>& used,
                      std::vector<WeldSeam2D>& seams);

    void extractArcs(const std::vector<Point2D>& points,
                     std::vector<int>& remaining_indices,
                     std::vector<bool>& used,
                     std::vector<WeldArc2D>& arcs);

    void updateRemainingIndices(std::vector<int>& remaining_indices,
                                const std::vector<bool>& used);

    std::vector<std::vector<int>> splitIntoContinuousSegments(
        const std::vector<Point2D>& all_points,
        const std::vector<int>& point_indices,
        const std::vector<int>& inliers,
        const Line2D& line);

    bool ransacFitLineWithIndices(const std::vector<Point2D>& all_points,
                                  const std::vector<int>& point_indices,
                                  Line2D& best_line,
                                  std::vector<int>& best_inliers);

    void computeEndpoints(const std::vector<Point2D>& all_points,
                          const std::vector<int>& point_indices,
                          const Line2D& line,
                          Point2D& start,
                          Point2D& end);

    void projectPointToLine(const Point2D& point,
                            const Line2D& line,
                            Point2D& projected);

    void refineLineWithInliers(const std::vector<Point2D>& all_points,
                               const std::vector<int>& point_indices,
                               const std::vector<int>& inliers,
                               Line2D& line);

    // 圆弧相关方法
    bool ransacFitArcWithIndices(const std::vector<Point2D>& all_points,
                                 const std::vector<int>& point_indices,
                                 Arc2D& best_arc,
                                 std::vector<int>& best_inliers);

    void determineArcAngles(const std::vector<Point2D>& all_points,
                            const std::vector<int>& point_indices,
                            const std::vector<int>& inliers,
                            const Arc2D& arc,
                            double& start_angle,
                            double& end_angle,
                            bool& is_clockwise);

    void refineArcParameters(const std::vector<Point2D>& points,
                             const std::vector<int>& indices,
                             const std::vector<int>& inliers,
                             Arc2D& arc);

    Point2D computeArcMiddlePoint(const Arc2D& arc);
};

// 焊缝处理类
class WELD_SEARCH_EXPORT WeldSeamExtractor  {
public:
    // 功能1: 提取直线和圆弧
    static std::tuple<std::vector<WeldSeam2D>, std::vector<WeldArc2D>>
    extractSeamsAndArcs(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                        double distance_threshold = 2.0,
                        int min_inliers = 150,
                        int max_iterations = 200,
                        int min_points_to_fit = 50,
                        double max_gap = 4.0,
                        double arc_dist_threshold = 1.0,
                        int min_arc_inliers = 100,
                        double arc_min_radius = 20.0,
                        double arc_max_radius = 60.0,
                        double arc_outlier_ratio = 0.35,
                        double arc_angle_tolerance = 0.01,
                        int arc_min_continuous = 3);

    // 功能2: 转换到3D并分组
    struct ExtractionResult3D {
        std::vector<std::vector<WeldSeam3D>> grouped_seams;
        std::vector<std::vector<WeldSeam3D>> grouped_seams_with_arc;
        std::vector<std::vector<WeldArc3D>> grouped_arcs;
        std::vector<WeldLSeam3D> vertical_seams;
    };

    static ExtractionResult3D convertAndGroupTo3D(
        std::vector<WeldSeam2D>& seams,
        const std::vector<WeldArc2D>& arcs,
        const std::vector<Point2D>& points_2d,
        double z0,
        const Eigen::Vector3f& axis,
        float angle,
        double vertical_length = 20.0,
        double connection_threshold = 5.0,
        double group_threshold = 20.0,
        double arc_proximity_threshold = 20.0,
        std::vector<std::vector<pcl::PointXYZ>>* arc_clouds_out = nullptr,
        double endpoint_proximity_threshold=6.0,
        bool do_clockwise_sort = true);

    // 功能3: 生成JSON文件
    static bool generateJSON(
        const std::vector<std::vector<WeldSeam3D>>& grouped_seams,
        const std::vector<std::vector<WeldSeam3D>>& grouped_seams_with_arc,
        const std::vector<std::vector<WeldArc3D>>& grouped_arcs,
        const std::vector<WeldLSeam3D>& vertical_seams,
        const std::string& filename,
        WeldType default_type = WeldType::FILLET,
        WeldPosition default_position = WeldPosition::FLAT,
        double default_thickness = 1.0);

private:
    static double pointToPoint(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);
    static std::string weldTypeToString(WeldType type);
    static std::string weldPositionToString(WeldPosition position);
};

class WELD_SEARCH_EXPORT Weld_search
{
public:
    Weld_search();
    ~Weld_search();

    /**
     * @brief 过滤点云在指定轴向上的范围
     * @param input_path 输入点云文件路径 (.pcd)
     * @param output_path 输出过滤后点云文件路径
     * @param axis 过滤轴向: "x", "y", 或 "z"
     * @param min_limit 范围下限
     * @param max_limit 范围上限
     * @param negative false:保留范围内点云; true:保留范围外点云
     */
    void filterPointCloud(const std::string& input_path,
                          const std::string& output_path,
                          const std::string& axis,
                          double min_limit,
                          double max_limit,
                          bool negative = false);

    /**
     * @brief 过滤底板并将点云投影到底板面上
     * @param input_path 输入点云文件路径 (.pcd)
     * @param output_path 输出过滤后点云文件路径
     * @param axis 旋转轴
     * @param angle 旋转角
     * @param z0 底板高度
     * @return true:成功; false:失败
     */
    bool filterGroundFace(const std::string& input_path,
                          const std::string& output_path,
                          Eigen::Vector3f& axis,
                          float& angle,
                          float& z0);

    /**
     * @brief 焊缝寻找并生成
     * @param input_path 输入点云文件路径 (.pcd)
     * @param output_path 输出过滤后点云文件路径
     * @return true:成功; false:失败
     */
    bool weldSearch();


    /**
     * @brief 焊缝路径规划
     * @param input_file 输入焊缝文件.josn
     * @param output_path 输出规划好的路径.josn
     * @param param_file 配置文件
     * @param show_demo 是否执行并输出中断恢复演示代码块
     */
    void process_welding_system(
        const std::string& input_file = "",
        const std::string& output_dir = ".",
        const std::string& param_file = "",
        bool show_demo = true);

};

#endif // WELD_SEARCH_H
