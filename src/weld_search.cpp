#include "weld_search.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <iostream>   // std::cout, std::cin

Weld_search::Weld_search() {}

Weld_search::~Weld_search()
{
}

// 在文件顶部定义全局静态点云对象
static pcl::PointCloud<pcl::PointXYZ> g_cloud;
static pcl::PointCloud<pcl::PointXYZ> g_cloud_out;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // M_PI

// ============================================================================
// 常量定义（需求4）
// ============================================================================

// 板厚参数
const double DEFAULT_HORIZONTAL_PLATE_THICKNESS = 50.0;   // 横板厚度
const double DEFAULT_VERTICAL_PLATE_THICKNESS = 50.0;     // 纵板厚度
const double DEFAULT_PLATE_THICKNESS = 50.0;              // 默认板厚

// 分组距离阈值
const double DEFAULT_GROUP_DISTANCE_THRESHOLD = 100.0;    // 平行焊缝拆分分组判定距离阈值
const double TRAVEL_SPEED = 5.0;                          // 移动速度 mm/s（时间计算）
const double DIRECTION_PARALLEL_THRESHOLD = 0.90;         // 焊缝平行判定阈值（余弦相似度）

// 遗传算法参数
const int GA_POP_SIZE = 50;       // 种群大小
const int GA_GENERATIONS = 100;   // 迭代轮数
const int GA_MAX_GENERATIONS = GA_GENERATIONS; // [修复] 最大代数别名
const int GA_ELITISM_COUNT = 5;                 // [修复] 精英保留数量
const double GA_MUTATION_RATE = 0.1;   // 变异率
const double GA_CROSSOVER_RATE = 0.8;  // 交叉率

// A*算法参数
// const double A_STAR_WEIGHT = 1.0;      // A*启发式权重
// const double A_STAR_EPSILON = 1e-6;    // 精度

// 权重（适应度函数）
const double WEIGHT_CONTINUITY = 0.40;  // 连续性优先
const double WEIGHT_SYMMETRY = 0.30;    // 对称性次之
const double WEIGHT_LENGTH = 0.20;      // 总长度最短
const double WEIGHT_TIME = 0.10;        // 焊接时间最短

// 归一化参数
const double MAX_EXPECTED_LENGTH = 5000.0;   // 预期最大焊接总长度 mm
const double MAX_EXPECTED_TIME = 1000.0;     // 预期最大焊接时间 s

// ============================================================================
// 枚举定义（需求4-5）
// ============================================================================


double dist_3d(const Vec3& a, const Vec3& b) {
    return (a - b).norm();
}// 计算3D空间中的距离

// 方向向量余弦相似度（需求17）
double direction_cosine(const Vec3& a, const Vec3& b) {
    Vec3 na = a.normalized();
    Vec3 nb = b.normalized();
    double dot = na.dot(nb);
    return std::fabs(dot);  // 取绝对值，方向相反也视为平行
}

// 安全的字符串转换辅助函数
inline int safe_stoi(const std::string& s, int default_val = 0) {
    try {
        return std::stoi(s);
    } catch (...) {
        return default_val;
    }
}

inline double safe_stod(const std::string& s, double default_val = 0.0) {
    try {
        return std::stod(s);
    } catch (...) {
        return default_val;
    }
}

// ============================================================================
// 焊缝数据模型（需求2, 5, 6）
// ============================================================================

struct WeldSeamInfo  {
    int id;              // 焊缝序号
    int group_id;        // 分组序号
    Vec3 start_point;    // 起点坐标（3D）
    Vec3 end_point;      // 终点坐标（3D）
    Vec3 direction;      // 方向向量
    WeldType weld_type;  // 焊缝类型：FILLET/BUTT/GROOVE
    WeldPosition weld_position;  // 焊接位置：FLAT/VERTICAL/OVERHEAD
    double plate_thickness;      // 板厚

    // 弧形焊缝参数
    bool is_arc;         // 是否为弧形焊缝
    Vec3 center_point;   // 圆心坐标
    double radius;       // 圆弧半径
    double start_angle;  // 起始角度（度）
    double end_angle;    // 终止角度（度）

    // 角度表示（用于输入兼容）
    double azimuth;      // 水平角：XY平面内与x轴正方向的夹角（度），逆时针为正
    double elevation;    // 垂直角：与XY平面的夹角（度），向上为正
    double angle_deg;    // 简化角度表示（仅XY平面内）

    double start_stye=0;//起点类型
    double end_stye=0;//终点类型

    WeldSeamInfo () : id(0), group_id(0), weld_type(WeldType::UNKNOWN),
        weld_position(WeldPosition::UNKNOWN), plate_thickness(DEFAULT_PLATE_THICKNESS),
        is_arc(false), radius(0), start_angle(0), end_angle(0),
        azimuth(0), elevation(0), angle_deg(0) {}

    WeldSeamInfo (int id_, int gid, const Vec3& start, const Vec3& end, const Vec3& dir,
             WeldType type, WeldPosition pos, double thick)
        : id(id_), group_id(gid), start_point(start), end_point(end), direction(dir),
        weld_type(type), weld_position(pos), plate_thickness(thick),
        is_arc(false), radius(0), start_angle(0), end_angle(0),
        azimuth(0), elevation(0), angle_deg(0) {
        // 从方向向量计算角度
        update_angles_from_direction();
        // 确保立焊缝从高度最小的点开始
        normalize_vertical_seam();
    }

    // 需求7：从JSON加载（完整实现）
    static std::optional<WeldSeamInfo > from_json(const std::string& json_str) {
        WeldSeamInfo  seam;

        // 提取id（焊缝序号）
        size_t id_pos = json_str.find("\"id\"");
        if (id_pos != std::string::npos) {
            size_t val_start = json_str.find(':', id_pos);
            if (val_start != std::string::npos) {
                size_t val_end = json_str.find_first_of(",\n}", val_start);
                if (val_end != std::string::npos) {
                    std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                    seam.id = safe_stoi(val);
                }
            }
        }

        // 提取group_id（分组序号）
        size_t gid_pos = json_str.find("\"group_id\"");
        if (gid_pos != std::string::npos) {
            size_t val_start = json_str.find(':', gid_pos);
            if (val_start != std::string::npos) {
                size_t val_end = json_str.find_first_of(",\n}", val_start);
                if (val_end != std::string::npos) {
                    std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                    seam.group_id = safe_stoi(val);
                }
            }
        }

        // 提取start_point [x, y, z]
        seam.start_point = parse_vec3(json_str, "\"start_point\"");

        // 提取end_point [x, y, z]
        seam.end_point = parse_vec3(json_str, "\"end_point\"");

        // 提取角度表示（兼容多种格式，优先级：angle > azimuth+elevation > direction）
        bool has_angle_input = false;

        // 格式1：简化角度 angle（仅XY平面内，与x轴正方向夹角，逆时针为正）
        size_t angle_pos = json_str.find("\"angle\"");
        if (angle_pos != std::string::npos) {
            size_t val_start = json_str.find(':', angle_pos);
            if (val_start != std::string::npos) {
                size_t val_end = json_str.find_first_of(",\n}", val_start);
                if (val_end != std::string::npos) {
                    std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                    seam.angle_deg = safe_stod(val);
                    seam.azimuth = seam.angle_deg;
                    seam.elevation = 0;
                    // 转换为方向向量（XY平面内）
                    double angle_rad = seam.angle_deg * M_PI / 180.0;
                    seam.direction = Vec3(std::cos(angle_rad), std::sin(angle_rad), 0);
                    has_angle_input = true;
                }
            }
        }

        // 格式2：方位角 azimuth（XY平面内）+ 垂直角 elevation
        size_t azimuth_pos = json_str.find("\"azimuth\"");
        size_t elevation_pos = json_str.find("\"elevation\"");

        if (!has_angle_input && (azimuth_pos != std::string::npos || elevation_pos != std::string::npos)) {
            if (azimuth_pos != std::string::npos) {
                size_t val_start = json_str.find(':', azimuth_pos);
                if (val_start != std::string::npos) {
                    size_t val_end = json_str.find_first_of(",\n}", val_start);
                    if (val_end != std::string::npos) {
                        std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                        seam.azimuth = safe_stod(val);
                    }
                }
            }

            if (elevation_pos != std::string::npos) {
                size_t val_start = json_str.find(':', elevation_pos);
                if (val_start != std::string::npos) {
                    size_t val_end = json_str.find_first_of(",\n}", val_start);
                    if (val_end != std::string::npos) {
                        std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                        seam.elevation = safe_stod(val);
                    }
                }
            }

            // 从角度计算方向向量
            seam.update_direction_from_angles();
            seam.angle_deg = seam.azimuth;
            has_angle_input = true;
        }

        // 格式3：方向向量 direction [x, y, z]（最低优先级）
        if (!has_angle_input) {
            seam.direction = parse_vec3(json_str, "\"direction\"");
            // 从方向向量计算角度
            seam.update_angles_from_direction();
        }

        // 提取plate_thickness
        size_t thick_pos = json_str.find("\"plate_thickness\"");
        if (thick_pos != std::string::npos) {
            size_t val_start = json_str.find(':', thick_pos);
            if (val_start != std::string::npos) {
                size_t val_end = json_str.find_first_of(",\n}", val_start);
                if (val_end != std::string::npos) {
                    std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                    seam.plate_thickness = safe_stod(val, DEFAULT_PLATE_THICKNESS);
                }
            }
        }

        // 提取weld_type
        size_t type_pos = json_str.find("\"weld_type\"");
        if (type_pos != std::string::npos) {
            size_t val_start = json_str.find('"', json_str.find(':', type_pos));
            size_t val_end = json_str.find('"', val_start + 1);
            if (val_start != std::string::npos && val_end != std::string::npos) {
                std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                if (val == "FILLET") seam.weld_type = WeldType::FILLET;
                else if (val == "BUTT") seam.weld_type = WeldType::BUTT;
                else if (val == "GROOVE") seam.weld_type = WeldType::GROOVE;
                else seam.weld_type = WeldType::UNKNOWN;
            }
        }

        // 提取weld_position（焊接位置）
        size_t pos_pos = json_str.find("\"weld_position\"");
        if (pos_pos != std::string::npos) {
            size_t val_start = json_str.find('"', json_str.find(':', pos_pos));
            size_t val_end = json_str.find('"', val_start + 1);
            if (val_start != std::string::npos && val_end != std::string::npos) {
                std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                if (val == "FLAT") seam.weld_position = WeldPosition::FLAT;
                else if (val == "VERTICAL") seam.weld_position = WeldPosition::VERTICAL;
                else if (val == "OVERHEAD") seam.weld_position = WeldPosition::OVERHEAD;
                else seam.weld_position = WeldPosition::UNKNOWN;
            }
        }
        // 提取start_stye（起点类型）
        size_t start_stye_pos = json_str.find("\"start_stye\"");
        if (start_stye_pos != std::string::npos) {
            size_t val_start = json_str.find(':', start_stye_pos);
            if (val_start != std::string::npos) {
                size_t val_end = json_str.find_first_of(",\n}", val_start);
                if (val_end != std::string::npos) {
                    std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                    seam.start_stye = safe_stod(val, 0.0);
                }
            }
        }

        // 提取end_stye（终点类型）
        size_t end_stye_pos = json_str.find("\"end_stye\"");
        if (end_stye_pos != std::string::npos) {
            size_t val_start = json_str.find(':', end_stye_pos);
            if (val_start != std::string::npos) {
                size_t val_end = json_str.find_first_of(",\n}", val_start);
                if (val_end != std::string::npos) {
                    std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                    seam.end_stye = safe_stod(val, 0.0);
                }
            }
        }
        // 提取弧形焊缝参数
        size_t arc_pos = json_str.find("\"is_arc\"");
        if (arc_pos != std::string::npos) {
            size_t val_start = json_str.find(':', arc_pos);
            if (val_start != std::string::npos) {
                std::string val = json_str.substr(val_start + 1);
                seam.is_arc = (val.find("true") != std::string::npos);
            }
        }

        // 提取圆心坐标
        seam.center_point = parse_vec3(json_str, "\"center_point\"");

        // 提取半径
        size_t rad_pos = json_str.find("\"radius\"");
        if (rad_pos != std::string::npos) {
            size_t val_start = json_str.find(':', rad_pos);
            if (val_start != std::string::npos) {
                size_t val_end = json_str.find_first_of(",\n}", val_start);
                if (val_end != std::string::npos) {
                    std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
                    seam.radius = safe_stod(val);
                }
            }
        }

        // 确保立焊缝从高度最小的点开始
        seam.normalize_vertical_seam();

        return seam;
    }

    // 辅助函数：解析Vec3
    static Vec3 parse_vec3(const std::string& json_str, const std::string& key) {
        size_t key_pos = json_str.find(key);
        if (key_pos == std::string::npos) return Vec3();

        size_t arr_start = json_str.find('[', key_pos);
        size_t arr_end = json_str.find(']', arr_start);
        if (arr_start == std::string::npos || arr_end == std::string::npos) return Vec3();

        std::string arr_content = json_str.substr(arr_start + 1, arr_end - arr_start - 1);

        double vals[3] = {0, 0, 0};
        size_t idx = 0;
        size_t pos = 0;
        while (idx < 3 && pos < arr_content.length()) {
            while (pos < arr_content.length() && (arr_content[pos] == ' ' || arr_content[pos] == ',' || arr_content[pos] == '\n')) {
                pos++;
            }
            if (pos >= arr_content.length()) break;

            size_t val_end = pos;
            while (val_end < arr_content.length() && arr_content[val_end] != ',' && arr_content[val_end] != ' ' && arr_content[val_end] != '\n') {
                val_end++;
            }

            std::string val_str = arr_content.substr(pos, val_end - pos);
            vals[idx] = safe_stod(val_str);
            idx++;
            pos = val_end;
        }

        return Vec3(vals[0], vals[1], vals[2]);
    }

    // 从方向向量计算角度表示
    void update_angles_from_direction() {
        Vec3 norm_dir = direction.normalized();

        // 计算水平角（XY平面内与x轴正方向的夹角，逆时针为正）
        double xy_length = std::sqrt(norm_dir.x * norm_dir.x + norm_dir.y * norm_dir.y);
        if (xy_length < 0.0001) {
            // 沿z轴方向
            azimuth = 0;
        } else {
            azimuth = std::atan2(norm_dir.y, norm_dir.x) * 180.0 / M_PI;
            if (azimuth < 0) azimuth += 360;  // 转换为0-360度
        }

        // 计算垂直角（与XY平面的夹角，向上为正）
        elevation = std::asin(norm_dir.z) * 180.0 / M_PI;

        // 简化角度（仅XY平面内）
        angle_deg = azimuth;
    }

    // 从角度计算方向向量
    void update_direction_from_angles() {
        double azimuth_rad = azimuth * M_PI / 180.0;
        double elevation_rad = elevation * M_PI / 180.0;

        double xy_length = std::cos(elevation_rad);
        direction.x = xy_length * std::cos(azimuth_rad);
        direction.y = xy_length * std::sin(azimuth_rad);
        direction.z = std::sin(elevation_rad);
    }

    // 需求5：自动检测焊接位置（基于新定义）
    // 3.2 FLAT：方向与重力方向垂直且z坐标为零的焊缝
    // 3.3 VERTICAL：方向与重力方向平行的焊缝
    // 3.4 OVERHEAD：方向与重力方向垂直且z坐标>=100mm的焊缝
    void auto_detect_position() {
        Vec3 down(0, 0, -1);  // 重力方向

        // 计算方向向量与重力方向的点积
        double dot_product = direction.normalized().dot(down);
        double abs_dot = std::fabs(dot_product);

        // 判断是否与重力方向平行（VERTICAL）
        if (abs_dot > 0.99) {
            weld_position = WeldPosition::VERTICAL;
            return;
        }

        // 判断是否与重力方向垂直
        if (abs_dot < 0.01) {
            // 获取焊缝的平均z坐标
            double avg_z = (start_point.z + end_point.z) / 2.0;

            // FLAT：方向与重力方向垂直且z坐标为零（或接近零）
            if (std::fabs(avg_z) < 1.0) {
                weld_position = WeldPosition::FLAT;
            }
            // OVERHEAD：方向与重力方向垂直且z坐标>=100mm
            else if (avg_z >= 100.0) {
                weld_position = WeldPosition::OVERHEAD;
            }
            // 方向垂直但z坐标介于(0, 100)之间：不满足任何条件，设为UNKNOWN
            else {
                weld_position = WeldPosition::UNKNOWN;
            }
            return;
        }

        // 方向既不平行也不垂直：不满足任何条件，设为UNKNOWN
        weld_position = WeldPosition::UNKNOWN;
    }

    // 判断方向向量是否沿x轴
    // 判断方向向量是否沿x轴
    bool is_x_direction() const {
        Vec3 norm_dir = direction.normalized();
        return std::fabs(norm_dir.x) > 0.99 && std::fabs(norm_dir.y) < 0.01 && std::fabs(norm_dir.z) < 0.01;
    }

    // 判断方向向量是否沿y轴
    bool is_y_direction() const {
        Vec3 norm_dir = direction.normalized();
        return std::fabs(norm_dir.y) > 0.99 && std::fabs(norm_dir.x) < 0.01 && std::fabs(norm_dir.z) < 0.01;
    }

    // 判断方向向量是否沿z轴
    bool is_z_direction() const {
        Vec3 norm_dir = direction.normalized();
        return std::fabs(norm_dir.z) > 0.99 && std::fabs(norm_dir.x) < 0.01 && std::fabs(norm_dir.y) < 0.01;
    }

    // 确保立焊缝从高度最小的点开始
    void normalize_vertical_seam() {
        if (weld_position == WeldPosition::VERTICAL) {
            if (start_point.z > end_point.z) {
                std::swap(start_point, end_point);
            }
        }
    }

    double length() const {
        if (is_arc) {
            // 弧形焊缝长度 = 弧长
            double angle_diff = std::fabs(end_angle - start_angle);
            if (angle_diff > 360) angle_diff -= 360;
            return radius * angle_diff * M_PI / 180.0;
        }
        return dist_3d(start_point, end_point);
    }

    Vec3 centroid() const {
        return Vec3(
            (start_point.x + end_point.x) / 2,
            (start_point.y + end_point.y) / 2,
            (start_point.z + end_point.z) / 2
            );
    }

    std::string to_json() const {
        std::ostringstream ss;
        ss << "{\n";
        ss << "  \"id\": " << id << ",\n";
        ss << "  \"group_id\": " << group_id << ",\n";
        ss << "  \"start_point\": [" << start_point.x << ", " << start_point.y << ", " << start_point.z << "],\n";
        ss << "  \"end_point\": [" << end_point.x << ", " << end_point.y << ", " << end_point.z << "],\n";
        ss << "  \"direction\": [" << direction.x << ", " << direction.y << ", " << direction.z << "],\n";
        ss << "  \"azimuth\": " << azimuth << ",\n";           // 水平角（度）
        ss << "  \"elevation\": " << elevation << ",\n";       // 垂直角（度）
        ss << "  \"angle\": " << angle_deg << ",\n";           // 简化角度（XY平面内）
        ss << "  \"weld_type\": \"";
        if (weld_type == WeldType::FILLET) ss << "FILLET";
        else if (weld_type == WeldType::BUTT) ss << "BUTT";
        else ss << "GROOVE";
        ss << "\",\n";
        ss << "  \"weld_position\": \"";
        if (weld_position == WeldPosition::FLAT) ss << "FLAT";
        else if (weld_position == WeldPosition::VERTICAL) ss << "VERTICAL";
        else ss << "OVERHEAD";
        ss << "\",\n";
        ss << "  \"plate_thickness\": " << plate_thickness << ",\n";
        ss << "  \"is_arc\": " << (is_arc ? "true" : "false");
        if (is_arc) {
            ss << ",\n";
            ss << "  \"center_point\": [" << center_point.x << ", " << center_point.y << ", " << center_point.z << "],\n";
            ss << "  \"radius\": " << radius << ",\n";
            ss << "  \"start_angle\": " << start_angle << ",\n";
            ss << "  \"end_angle\": " << end_angle;
        }
        ss << "\n}";
        return ss.str();
    }
};

// ============================================================================
// 焊缝分组（需求6）
// ============================================================================

struct SeamGroup {
    int group_id;
    std::vector<int> seam_ids;
    WeldType weld_type;
    WeldPosition weld_position;
    double plate_thickness;

    bool is_arc_group;  // 是否包含弧形焊缝

    Vec3 centroid(const std::vector<WeldSeamInfo >& all_seams) const {
        if (seam_ids.empty()) return Vec3();

        double cx = 0, cy = 0, cz = 0;
        for (int sid : seam_ids) {
            for (const auto& seam : all_seams) {
                if (seam.id == sid) {
                    Vec3 c = seam.centroid();
                    cx += c.x;
                    cy += c.y;
                    cz += c.z;
                    break;
                }
            }
        }

        int n = seam_ids.size();
        return Vec3(cx / n, cy / n, cz / n);
    }
};

// ============================================================================
// 路径点（用于优化算法）
// ============================================================================

struct PathNode {
    int seam_id;
    int index;        // 在焊缝序列中的位置
    Vec3 position;   // 焊接起点

    PathNode() : seam_id(0), index(0), position(Vec3())  {}
    PathNode(int sid, int idx, const Vec3& pos) : seam_id(sid), index(idx), position(pos) {}
};

// ============================================================================
// 路径评估（需求18-21）
// ============================================================================

struct PathEvaluation {
    double continuity = 0.0;   // 连续性：相邻焊缝端点连接占比
    double symmetry = 0.0;     // 对称性：对称焊接顺序惩罚
    double length = 0.0;       // 总路径长度空行程
    double time = 0.0;         // 焊接时间
    double total = 0.0;        // 加权总分

    bool operator<(const PathEvaluation& other) const {
        return total < other.total;
    }
};

// ============================================================================
// WeldingPathPlanner - 路径规划（需求10-17）
// ============================================================================

class WeldingPathPlanner {
public:
    double plate_thickness;
    double group_distance_threshold;

    WeldingPathPlanner(double pt = DEFAULT_PLATE_THICKNESS,
                       double gdt = DEFAULT_GROUP_DISTANCE_THRESHOLD)
        : plate_thickness(pt), group_distance_threshold(gdt) {}

    // 需求6：焊缝分组规则
    std::vector<SeamGroup> group_seams(const std::vector<WeldSeamInfo >& seams) {
        std::vector<SeamGroup> groups;

        // 6.1：首先按焊接位置分组
        std::unordered_map<WeldPosition, std::vector<int>> position_groups;
        for (const auto& seam : seams) {
            position_groups[seam.weld_position].push_back(seam.id);
        }

        // 处理每个位置组
        for (auto& [pos, seam_ids] : position_groups) {
            if (pos == WeldPosition::VERTICAL) {
                // 6.2：立焊缝不再参与以下再分组，直接作为一组
                SeamGroup group;
                group.group_id = static_cast<int>(groups.size() + 1);
                group.seam_ids = seam_ids;
                group.weld_position = WeldPosition::VERTICAL;

                if (!seam_ids.empty()) {
                    const WeldSeamInfo & first = find_seam_by_id(seams, seam_ids[0]);
                    group.weld_type = first.weld_type;
                    group.plate_thickness = first.plate_thickness;
                }
                groups.push_back(group);
            } else {
                // 6.3：在已分组内以焊缝类型进行分组
                std::unordered_map<WeldType, std::vector<int>> type_groups;
                for (int sid : seam_ids) {
                    const WeldSeamInfo & seam = find_seam_by_id(seams, sid);
                    type_groups[seam.weld_type].push_back(sid);
                }

                // 6.4：按焊缝平行性进行分组
                for (auto& [type, type_seam_ids] : type_groups) {
                    auto parallel_clusters = cluster_by_parallelism(seams, type_seam_ids);

                    // 6.5：平行焊缝再次按距离分组
                    for (auto& parallel_cluster : parallel_clusters) {
                        auto final_clusters = cluster_by_distance(seams, parallel_cluster);

                        for (auto& cluster : final_clusters) {
                            // 6.6：检查弧形焊缝并处理
                            auto arc_groups = process_arc_seams(seams, cluster);

                            for (auto& arc_cluster : arc_groups) {
                                SeamGroup group;
                                group.group_id = static_cast<int>(groups.size() + 1);
                                group.seam_ids = arc_cluster;
                                group.weld_type = type;
                                group.weld_position = pos;

                                if (!arc_cluster.empty()) {
                                    const WeldSeamInfo & first = find_seam_by_id(seams, arc_cluster[0]);
                                    group.plate_thickness = first.plate_thickness;
                                    group.is_arc_group = first.is_arc;
                                }
                                groups.push_back(group);
                            }
                        }
                    }
                }
            }
        }

        return groups;
    }

    // 6.4：按焊缝平行性分组
    std::vector<std::vector<int>> cluster_by_parallelism(const std::vector<WeldSeamInfo >& seams,
                                                         const std::vector<int>& seam_ids) {
        std::vector<std::vector<int>> clusters;
        std::unordered_set<int> assigned;

        for (int sid : seam_ids) {
            if (assigned.count(sid)) continue;

            std::vector<int> cluster;
            const WeldSeamInfo & ref_seam = find_seam_by_id(seams, sid);
            cluster.push_back(sid);
            assigned.insert(sid);

            for (int other_sid : seam_ids) {
                if (assigned.count(other_sid)) continue;

                const WeldSeamInfo & other_seam = find_seam_by_id(seams, other_sid);
                if (direction_cosine(ref_seam.direction, other_seam.direction) >= DIRECTION_PARALLEL_THRESHOLD) {
                    cluster.push_back(other_sid);
                    assigned.insert(other_sid);
                }
            }

            clusters.push_back(cluster);
        }

        return clusters;
    }

    // 6.5：按距离分组（板厚内为一组，超过阈值另分组）
    std::vector<std::vector<int>> cluster_by_distance(const std::vector<WeldSeamInfo >& seams,
                                                      const std::vector<int>& seam_ids) {
        if (seam_ids.size() <= 1) return {seam_ids};

        std::vector<std::vector<int>> clusters;
        std::unordered_set<int> assigned;

        for (int sid : seam_ids) {
            if (assigned.count(sid)) continue;

            std::vector<int> cluster;
            std::queue<int> bfs_queue;
            bfs_queue.push(sid);
            assigned.insert(sid);

            while (!bfs_queue.empty()) {
                int current = bfs_queue.front();
                bfs_queue.pop();
                cluster.push_back(current);

                const WeldSeamInfo & current_seam = find_seam_by_id(seams, current);

                for (int other_sid : seam_ids) {
                    if (assigned.count(other_sid)) continue;

                    const WeldSeamInfo & other_seam = find_seam_by_id(seams, other_sid);
                    double dist = dist_3d(current_seam.centroid(), other_seam.centroid());

                    // 距离在板厚内：同一组
                    if (dist <= plate_thickness) {
                        assigned.insert(other_sid);
                        bfs_queue.push(other_sid);
                    }
                    // 距离超过阈值：另一组（后续处理）
                }
            }

            clusters.push_back(cluster);
        }

        return clusters;
    }

    // 6.6：处理弧形焊缝
    std::vector<std::vector<int>> process_arc_seams(const std::vector<WeldSeamInfo >& seams,
                                                    const std::vector<int>& seam_ids) {
        std::vector<std::vector<int>> result;
        std::unordered_set<int> assigned;

        // 检查是否有弧形焊缝
        bool has_arc = false;
        for (int sid : seam_ids) {
            if (find_seam_by_id(seams, sid).is_arc) {
                has_arc = true;
                break;
            }
        }

        if (!has_arc) {
            result.push_back(seam_ids);
            return result;
        }

        // 处理弧形焊缝：按连接关系分组
        for (int sid : seam_ids) {
            if (assigned.count(sid)) continue;

            std::vector<int> cluster;
            std::queue<int> bfs_queue;
            bfs_queue.push(sid);
            assigned.insert(sid);

            while (!bfs_queue.empty()) {
                int current = bfs_queue.front();
                bfs_queue.pop();
                cluster.push_back(current);

                const WeldSeamInfo & current_seam = find_seam_by_id(seams, current);

                for (int other_sid : seam_ids) {
                    if (assigned.count(other_sid)) continue;

                    const WeldSeamInfo & other_seam = find_seam_by_id(seams, other_sid);

                    // 检查是否连接（端点距离很近）
                    double d1 = dist_3d(current_seam.end_point, other_seam.start_point);
                    double d2 = dist_3d(current_seam.end_point, other_seam.end_point);
                    double d3 = dist_3d(current_seam.start_point, other_seam.start_point);
                    double d4 = dist_3d(current_seam.start_point, other_seam.end_point);

                    if (d1 < 1.0 || d2 < 1.0 || d3 < 1.0 || d4 < 1.0) {
                        assigned.insert(other_sid);
                        bfs_queue.push(other_sid);
                    }
                }
            }

            result.push_back(cluster);
        }

        return result;
    }

    // 需求11：方向平行+板厚内为一组（改进的BFS聚类算法）
    std::vector<std::vector<int>> cluster_by_direction_and_distance(
        const std::vector<WeldSeamInfo >& seams) {

        if (seams.empty()) return {};

        std::vector<std::vector<int>> clusters;
        std::vector<bool> assigned(seams.size(), false);

        for (size_t i = 0; i < seams.size(); ++i) {
            if (assigned[i]) continue;

            std::vector<int> cluster;
            std::queue<size_t> bfs_queue;
            bfs_queue.push(i);
            assigned[i] = true;

            while (!bfs_queue.empty()) {
                size_t current = bfs_queue.front();
                bfs_queue.pop();
                cluster.push_back(seams[current].id);

                for (size_t j = 0; j < seams.size(); ++j) {
                    if (assigned[j]) continue;

                    if (direction_cosine(seams[current].direction, seams[j].direction) < DIRECTION_PARALLEL_THRESHOLD) {
                        continue;
                    }

                    double dist = dist_3d(seams[current].centroid(), seams[j].centroid());
                    if (dist > plate_thickness) {
                        continue;
                    }

                    assigned[j] = true;
                    bfs_queue.push(j);
                }
            }

            if (!cluster.empty()) {
                clusters.push_back(cluster);
            }
        }

        return clusters;
    }

    // 需求13-16：组内规划焊接顺序规划程序
    std::vector<int> plan_group(const SeamGroup& group,
                                const std::vector<WeldSeamInfo >& all_seams,
                                const std::optional<Vec3>& last_vertical_end = std::nullopt) {
        std::vector<int> result;

        // 分离立焊和水平焊
        std::vector<WeldSeamInfo > verticals;  // 立焊缝
        std::vector<WeldSeamInfo > horizontals;  // 水平焊缝

        for (int sid : group.seam_ids) {
            const WeldSeamInfo & seam = find_seam_by_id(all_seams, sid);
            if (seam.weld_position == WeldPosition::VERTICAL) {
                verticals.push_back(seam);
            } else {
                horizontals.push_back(seam);
            }
        }

        // 需求13：先焊立角焊缝，坐标最小起点
        auto vertical_order = sort_verticals(verticals);
        result.insert(result.end(), vertical_order.begin(), vertical_order.end());

        // 需求15：水平焊缝起点以最后立焊起点最近相邻为准
        auto horizontal_order = sort_horizontals(horizontals, last_vertical_end);
        result.insert(result.end(), horizontal_order.begin(), horizontal_order.end());

        return result;
    }

    // 需求13：先焊立角焊缝，坐标最小起点
    // 需求14：防变形对称约束（两端交替）
    std::vector<int> sort_verticals(const std::vector<WeldSeamInfo >& verticals) {
        if (verticals.empty()) return {};

        // 确保每条立焊缝从高度最小的点开始，到高度最大的点结束
        std::vector<WeldSeamInfo > normalized_verticals;
        for (const auto& seam : verticals) {
            WeldSeamInfo  normalized = seam;
            // 如果起点高度大于终点高度，交换起点和终点
            if (seam.start_point.z > seam.end_point.z) {
                normalized.start_point = seam.end_point;
                normalized.end_point = seam.start_point;
            }
            normalized_verticals.push_back(normalized);
        }

        // 按起点坐标排序（先x，再y，再z）
        std::vector<WeldSeamInfo > sorted = normalized_verticals;
        std::sort(sorted.begin(), sorted.end(), [](const WeldSeamInfo & a, const WeldSeamInfo & b) {
            if (std::fabs(a.start_point.x - b.start_point.x) > 0.01)
                return a.start_point.x < b.start_point.x;
            if (std::fabs(a.start_point.y - b.start_point.y) > 0.01)
                return a.start_point.y < b.start_point.y;
            return a.start_point.z < b.start_point.z;
        });

        // 需求14：对称约束 - 两端交替焊接
        std::vector<int> result;
        int left = 0;
        int right = sorted.size() - 1;
        bool from_right = false;

        while (left <= right) {
            if (from_right && left <= right) {
                result.push_back(sorted[right].id);
                right--;
            } else if (!from_right && left <= right) {
                result.push_back(sorted[left].id);
                left++;
            }
            from_right = !from_right;
        }

        return result;
    }

    // 需求15：水平焊缝起点确定（最后立焊起点最近邻+方向相同）
    // 需求16：完成同组水平角焊缝（连续排序，方向相同）
    std::vector<int> sort_horizontals(const std::vector<WeldSeamInfo >& horizontals,
                                      const std::optional<Vec3>& last_vertical_end) {
        if (horizontals.empty()) return {};

        std::vector<WeldSeamInfo > sorted = horizontals;

        if (last_vertical_end.has_value()) {
            // 需求15：以最后立焊起点最近邻为准
            std::sort(sorted.begin(), sorted.end(),
                      [&last_vertical_end](const WeldSeamInfo & a, const WeldSeamInfo & b) {
                          double da = dist_3d(a.start_point, last_vertical_end.value());
                          double db = dist_3d(b.start_point, last_vertical_end.value());
                          return da < db;
                      });
        } else {
            // 默认按起点坐标排序
            std::sort(sorted.begin(), sorted.end(), [](const WeldSeamInfo & a, const WeldSeamInfo & b) {
                if (std::fabs(a.start_point.x - b.start_point.x) > 0.01)
                    return a.start_point.x < b.start_point.x;
                if (std::fabs(a.start_point.y - b.start_point.y) > 0.01)
                    return a.start_point.y < b.start_point.y;
                return a.start_point.z < b.start_point.z;
            });
        }

        // 需求16：连续排序（方向相同的焊缝排在一起）
        std::vector<int> result;
        std::vector<bool> visited(sorted.size(), false);

        for (size_t i = 0; i < sorted.size(); ++i) {
            if (visited[i]) continue;

            result.push_back(sorted[i].id);
            visited[i] = true;

            // 查找方向相同的相邻焊缝
            Vec3 ref_dir = sorted[i].direction.normalized();
            for (size_t j = i + 1; j < sorted.size(); ++j) {
                if (visited[j]) continue;

                Vec3 other_dir = sorted[j].direction.normalized();
                if (direction_cosine(ref_dir, other_dir) >= DIRECTION_PARALLEL_THRESHOLD) {
                    result.push_back(sorted[j].id);
                    visited[j] = true;
                }
            }
        }

        return result;
    }

    // 需求17：识别组间平行关系
    std::unordered_map<int, std::vector<int>> build_parallel_groups(
        const std::vector<SeamGroup>& groups,
        const std::vector<WeldSeamInfo >& all_seams) {

        std::unordered_map<int, std::vector<int>> parallel_map;

        for (size_t i = 0; i < groups.size(); ++i) {
            const auto& g1 = groups[i];
            Vec3 c1 = g1.centroid(all_seams);
            Vec3 dir1 = group_direction(g1, all_seams);

            for (size_t j = i + 1; j < groups.size(); ++j) {
                const auto& g2 = groups[j];
                Vec3 c2 = g2.centroid(all_seams);
                Vec3 dir2 = group_direction(g2, all_seams);

                double dist = dist_3d(c1, c2);

                // 跳过距离<=板厚的（同组）
                if (dist <= plate_thickness) continue;
                // 跳过距离>group_distance_threshold的（太远）
                if (dist > group_distance_threshold) continue;

                // 需求17：方向余弦相似度判定
                if (direction_cosine(dir1, dir2) >= DIRECTION_PARALLEL_THRESHOLD) {
                    parallel_map[g1.group_id].push_back(g2.group_id);
                    parallel_map[g2.group_id].push_back(g1.group_id);
                }
            }
        }

        return parallel_map;
    }

    // 需求17：基于平行关系确定组间顺序
    // 修复：组间顺序按焊接位置优先级排序（VERTICAL > FLAT > OVERHEAD）
    std::vector<int> get_sorted_groups(
        const std::vector<SeamGroup>& groups,
        const std::unordered_map<int, std::vector<int>>& parallel_map) {

        // 焊接位置优先级：VERTICAL=0, FLAT=1, OVERHEAD=2
        auto position_priority = [](WeldPosition pos) -> int {
            if (pos == WeldPosition::VERTICAL) return 0;
            if (pos == WeldPosition::FLAT) return 1;
            return 2;
        };

        // 创建可排序的副本，按优先级排序
        std::vector<SeamGroup> sorted_groups = groups;
        std::sort(sorted_groups.begin(), sorted_groups.end(),
                  [&](const SeamGroup& a, const SeamGroup& b) {
                      int pa = position_priority(a.weld_position);
                      int pb = position_priority(b.weld_position);
                      if (pa != pb) return pa < pb;
                      // 同优先级按group_id排序
                      return a.group_id < b.group_id;
                  });

        std::vector<int> result;
        std::unordered_set<int> visited;

        // 按优先级贪心遍历：每次选择最近平行组
        for (size_t i = 0; i < sorted_groups.size(); ++i) {
            if (visited.count(sorted_groups[i].group_id)) continue;

            result.push_back(sorted_groups[i].group_id);
            visited.insert(sorted_groups[i].group_id);

            // 扩展到平行组
            std::queue<int> q;
            if (parallel_map.count(sorted_groups[i].group_id)) {
                for (int pg : parallel_map.at(sorted_groups[i].group_id)) {
                    q.push(pg);
                }
            }

            while (!q.empty()) {
                int gid = q.front();
                q.pop();

                if (visited.count(gid)) continue;

                visited.insert(gid);
                result.push_back(gid);

                if (parallel_map.count(gid)) {
                    for (int pg : parallel_map.at(gid)) {
                        if (!visited.count(pg)) {
                            q.push(pg);
                        }
                    }
                }
            }
        }

        return result;
    }

private:
    std::vector<WeldSeamInfo > filter_by_ids(const std::vector<WeldSeamInfo >& seams,
                                        const std::vector<int>& ids) {
        std::vector<WeldSeamInfo > result;
        for (int id : ids) {
            for (const auto& s : seams) {
                if (s.id == id) {
                    result.push_back(s);
                    break;
                }
            }
        }
        return result;
    }

    const WeldSeamInfo & find_seam_by_id(const std::vector<WeldSeamInfo >& seams, int id) const {
        for (const auto& s : seams) {
            if (s.id == id) return s;
        }
        throw std::out_of_range("焊缝ID " + std::to_string(id) + " 未找到");
    }

    Vec3 group_direction(const SeamGroup& group, const std::vector<WeldSeamInfo >& all_seams) const {
        if (group.seam_ids.empty()) return Vec3();

        // 取第一个焊缝的方向
        return find_seam_by_id(all_seams, group.seam_ids[0]).direction;
    }
};

// ============================================================================
// PathOptimizer - 路径优化（需求18-21）
// ============================================================================

class PathOptimizer {
public:
    PathOptimizer() : rng_(std::random_device{}()) {}

    // 需求18：遗传算法全局优化（遵循初步规划约束）
    std::vector<int> genetic_algorithm(const std::vector<int>& initial_path,
                                       const std::vector<WeldSeamInfo >& all_seams,
                                       int generations = GA_GENERATIONS,
                                       int pop_size = GA_POP_SIZE) {

        if (initial_path.size() <= 2) return initial_path;

        // 初始化种群：遵循初步规划的基本约束
        std::vector<std::vector<int>> population;
        for (int i = 0; i < pop_size; ++i) {
            std::vector<int> ind;
            if (i == 0) {
                // 第一个个体：保留初始路径
                ind = initial_path;
            } else {
                // 其他个体：在保持基本约束的前提下进行局部调整
                ind = constrained_shuffle(initial_path, all_seams);
            }
            population.push_back(ind);
        }

        std::vector<int> best = initial_path;


        // 迭代
        for (int gen = 0; gen < generations; ++gen) {
            // 选择
            std::sort(population.begin(), population.end(),
                      [this, &all_seams](const std::vector<int>& a, const std::vector<int>& b) {
                          return evaluate_path(a, all_seams) < evaluate_path(b, all_seams);
                      });

            // 保留精英
            best = population[0];


            // 交叉
            std::vector<std::vector<int>> new_pop;
            new_pop.push_back(best);  // 精英保留

            std::uniform_real_distribution<double> dist(0.0, 1.0);

            while (static_cast<int>(new_pop.size()) < pop_size) {
                // 使用轮盘赌选择父代
                int p1_idx = tournament_selection(population, all_seams);
                int p2_idx = tournament_selection(population, all_seams);
                const auto& p1 = population[p1_idx];
                const auto& p2 = population[p2_idx];

                std::vector<int> child;
                if (dist(rng_) < GA_CROSSOVER_RATE) {
                    child = order_crossover(p1, p2, all_seams);
                } else {
                    child = p1;
                }

                if (dist(rng_) < GA_MUTATION_RATE) {
                    mutate(child);
                }

                new_pop.push_back(child);
            }

            population = new_pop;
        }

        return best;
    }

    // 需求18：评估函数
    PathEvaluation evaluate_path(const std::vector<int>& path,
                                 const std::vector<WeldSeamInfo >& all_seams) const {

        PathEvaluation eval;
        if (path.empty()) {
            eval.total = std::numeric_limits<double>::infinity();
            return eval;
        }

        // 硬约束检查：立焊缝必须在水平焊缝之前（需求13）
        // 修复：不满足硬约束的路径给予极高惩罚，确保遗传算法不选择
        bool seen_horizontal = false;
        for (int sid : path) {
            const auto& seam = find_seam_by_id(all_seams, sid);
            if (seam.weld_position != WeldPosition::VERTICAL) {
                seen_horizontal = true;
            } else {
                if (seen_horizontal) {
                    // 硬约束违反：水平焊缝之后出现了立焊缝
                    eval.total = 1e9;
                    return eval;
                }
            }
        }

        // 计算连续性
        double continuity_score = 0;
        for (size_t i = 1; i < path.size(); ++i) {
            const auto& prev = find_seam_by_id(all_seams, path[i-1]);
            const auto& curr = find_seam_by_id(all_seams, path[i]);

            double d = dist_3d(prev.end_point, curr.start_point);
            // 如果端点相连，计为连续
            if (d < 1.0) {
                continuity_score += 1.0;
            }
        }
        eval.continuity = continuity_score / (path.size() - 1);

        // 计算对称性（需求21）
        eval.symmetry = calculate_symmetry(path, all_seams);

        // 计算总长度
        eval.length = 0;
        for (int sid : path) {
            const auto& seam = find_seam_by_id(all_seams, sid);
            eval.length += seam.length();
        }

        // 计算焊接时间
        eval.time = eval.length / TRAVEL_SPEED;

        // 加权总分（需求18：连续性40%+对称性30%+长度20%+时间10%）
        // 所有指标归一化到[0,1]范围，确保权重正确生效
        double normalized_length = std::min(eval.length / MAX_EXPECTED_LENGTH, 1.0);
        double normalized_time = std::min(eval.time / MAX_EXPECTED_TIME, 1.0);

        eval.total = WEIGHT_CONTINUITY * (1 - eval.continuity) +
                     WEIGHT_SYMMETRY * (1 - eval.symmetry) +
                     WEIGHT_LENGTH * normalized_length +
                     WEIGHT_TIME * normalized_time;

        return eval;
    }

    // 需求21：对称焊接顺序防变形
    // 修复：放宽对称对计分条件，所有对称对都计分
    double calculate_symmetry(const std::vector<int>& path,
                              const std::vector<WeldSeamInfo >& all_seams) const {
        if (path.size() < 2) return 1.0;

        // 查找对称配对（修复：放宽计分条件，所有对称对都计分）
        double symmetric_pairs = 0;
        for (size_t i = 0; i < path.size(); ++i) {
            for (size_t j = i + 1; j < path.size(); ++j) {
                const auto& s1 = find_seam_by_id(all_seams, path[i]);
                const auto& s2 = find_seam_by_id(all_seams, path[j]);

                // 检查是否对称（方向相反，位置关于某点对称）
                if (is_symmetric_pair(s1, s2)) {
                    symmetric_pairs += 1;
                }
            }
        }

        // 归一化：对称对数量 / 所有可能对的数量
        double total_pairs = path.size() * (path.size() - 1) / 2;
        return total_pairs > 0 ? symmetric_pairs / total_pairs : 1.0;
    }

private:
    mutable std::mt19937 rng_;

    // 约束打乱：保持初步规划的基本约束
    // 1. 保持立焊缝在前，水平焊缝在后
    // 2. 保持对称焊缝的交替顺序
    // 3. 只在同类型焊缝内部进行局部调整
    std::vector<int> constrained_shuffle(const std::vector<int>& initial_path,
                                         const std::vector<WeldSeamInfo >& all_seams) {
        std::vector<int> result = initial_path;

        // 分离立焊和水平焊
        std::vector<int> verticals;
        std::vector<int> horizontals;

        for (int sid : result) {
            const WeldSeamInfo & seam = find_seam_by_id(all_seams, sid);
            if (seam.weld_position == WeldPosition::VERTICAL) {
                verticals.push_back(sid);
            } else {
                horizontals.push_back(sid);
            }
        }

        // 在立焊缝内部进行局部调整（保持对称交替）
        if (verticals.size() > 2) {
            // 只在相邻位置交换，保持对称交替模式
            std::uniform_int_distribution<int> dist(0, verticals.size() - 2);
            int swaps = std::max(1, static_cast<int>(verticals.size() / 4));

            for (int k = 0; k < swaps; ++k) {
                int i = dist(rng_);
                // 交换相邻立焊缝
                std::swap(verticals[i], verticals[i + 1]);
            }
        }

        // 在水平焊缝内部进行局部调整
        if (horizontals.size() > 2) {
            std::uniform_int_distribution<int> dist(0, horizontals.size() - 2);
            int swaps = std::max(1, static_cast<int>(horizontals.size() / 4));

            for (int k = 0; k < swaps; ++k) {
                int i = dist(rng_);
                // 交换相邻水平焊缝
                std::swap(horizontals[i], horizontals[i + 1]);
            }
        }

        // 重新组合：立焊在前，水平焊在后
        result.clear();
        result.insert(result.end(), verticals.begin(), verticals.end());
        result.insert(result.end(), horizontals.begin(), horizontals.end());

        return result;
    }

    int tournament_selection(const std::vector<std::vector<int>>& population,
                             const std::vector<WeldSeamInfo >& all_seams) const {
        int k = 3;  // 锦标赛规模
        std::uniform_int_distribution<int> dist(0, population.size() - 1);

        int best_idx = dist(rng_);
        double best_score = evaluate_path(population[best_idx], all_seams).total;

        for (int i = 1; i < k; ++i) {
            int idx = dist(rng_);
            double score = evaluate_path(population[idx], all_seams).total;
            if (score < best_score) {
                best_score = score;
                best_idx = idx;
            }
        }
        return best_idx;
    }

    const WeldSeamInfo & find_seam_by_id(const std::vector<WeldSeamInfo >& seams, int id) const {
        for (const auto& s : seams) {
            if (s.id == id) return s;
        }
        throw std::out_of_range("焊缝ID " + std::to_string(id) + " 未找到");
    }

    bool is_symmetric_pair(const WeldSeamInfo & s1, const WeldSeamInfo & s2) const {
        // 方向相反
        double cos_sim = direction_cosine(s1.direction, s2.direction);
        if (cos_sim > 0.99) return false;  // 方向相同，不是对称

        // 位置对称（关于某点）
        Vec3 c1 = s1.centroid();
        Vec3 c2 = s2.centroid();
        double dist = dist_3d(c1, c2);

        return dist < 50.0;  // 距离小于50mm视为可配对
    }

    std::vector<int> order_crossover(const std::vector<int>& p1, const std::vector<int>& p2,
                                     const std::vector<WeldSeamInfo >& all_seams) {
        if (p1.empty() || p2.empty()) return p1;

        // 约束交叉：保持基本约束
        // 1. 保持立焊缝和水平焊缝的相对顺序
        // 2. 使用部分映射交叉，避免破坏约束

        std::vector<int> child(p1.size(), -1);

        // 选择一个交叉段
        std::uniform_int_distribution<int> dist(0, p1.size() - 1);
        int start = dist(rng_);
        int end = start + dist(rng_) % (p1.size() - start);

        // 从p1复制交叉段
        for (int i = start; i < end; ++i) {
            child[i] = p1[i];
        }

        // 从p2填充剩余位置（保持顺序）
        int idx = 0;
        for (int gene : p2) {
            if (std::find(child.begin(), child.end(), gene) == child.end()) {
                while (idx < static_cast<int>(child.size()) && child[idx] != -1) {
                    idx++;
                }
                if (idx < static_cast<int>(child.size())) {
                    child[idx] = gene;
                }
            }
        }

        // 验证并修复约束：确保立焊缝在前，水平焊缝在后
        child = repair_constraints(child, all_seams);

        return child;
    }

    // 修复约束：确保立焊缝在前，水平焊缝在后
    // 修复：reference 参数改为 all_seams，正确查找焊缝
    std::vector<int> repair_constraints(const std::vector<int>& path,
                                        const std::vector<WeldSeamInfo >& all_seams) const {
        std::vector<int> result = path;

        // 分离立焊和水平焊
        std::vector<int> verticals;
        std::vector<int> horizontals;

        for (int sid : result) {
            const WeldSeamInfo & seam = find_seam_by_id(all_seams, sid);
            if (seam.weld_position == WeldPosition::VERTICAL) {
                verticals.push_back(sid);
            } else {
                horizontals.push_back(sid);
            }
        }

        // 重新组合：立焊在前，水平焊在后
        result.clear();
        result.insert(result.end(), verticals.begin(), verticals.end());
        result.insert(result.end(), horizontals.begin(), horizontals.end());

        return result;
    }

    void mutate(std::vector<int>& path) {
        if (path.size() < 2) return;

        // 约束变异：只交换相邻的焊缝，保持基本约束
        std::uniform_int_distribution<int> dist(0, path.size() - 2);
        int i = dist(rng_);

        // 交换相邻元素，保持局部顺序约束
        std::swap(path[i], path[i + 1]);
    }
};

// ============================================================================
// WeldingState - 焊接状态（需求22-23）
// ============================================================================

struct WeldingState {
    std::string task_id;
    std::string task_name;
    int current_group_id;
    int current_seam_index;
    Vec3 current_position;
    std::vector<int> completed_seam_ids;
    std::vector<int> remaining_seam_ids;
    std::unordered_map<int, std::vector<int>> planned_paths;
    double start_time;
    double pause_time;
    double total_welding_time;
    bool is_paused;
    bool is_completed;
    InterruptType interrupt_type;
    std::string error_message;
    std::string created_at;
    std::string updated_at;

    WeldingState() : current_group_id(0), current_seam_index(0),
        start_time(0), pause_time(0), total_welding_time(0),
        is_paused(false), is_completed(false),
        interrupt_type(InterruptType::UNKNOWN) {}

    // 需求22：保存状态
    std::string to_json() const {
        std::ostringstream ss;
        ss << "{\n";
        ss << "  \"task_id\": \"" << task_id << "\",\n";
        ss << "  \"task_name\": \"" << task_name << "\",\n";
        ss << "  \"current_group_id\": " << current_group_id << ",\n";
        ss << "  \"current_seam_index\": " << current_seam_index << ",\n";
        ss << "  \"current_position\": [" << current_position.x << ", " << current_position.y << ", " << current_position.z << "],\n";

        ss << "  \"completed_seam_ids\": [";
        for (size_t i = 0; i < completed_seam_ids.size(); ++i) {
            ss << completed_seam_ids[i];
            if (i < completed_seam_ids.size() - 1) ss << ", ";
        }
        ss << "],\n";

        ss << "  \"remaining_seam_ids\": [";
        for (size_t i = 0; i < remaining_seam_ids.size(); ++i) {
            ss << remaining_seam_ids[i];
            if (i < remaining_seam_ids.size() - 1) ss << ", ";
        }
        ss << "],\n";

        ss << "  \"is_paused\": " << (is_paused ? "true" : "false") << ",\n";
        ss << "  \"is_completed\": " << (is_completed ? "true" : "false") << ",\n";
        ss << "  \"interrupt_type\": \"" << (interrupt_type == InterruptType::GY ? "GY" :
                                                 interrupt_type == InterruptType::SB ? "SB" :
                                                 interrupt_type == InterruptType::RW ? "RW" :
                                                 interrupt_type == InterruptType::WB ? "WB" : "UNKNOWN") << "\"\n";
        ss << "}";
        return ss.str();
    }

    // 需求22：加载状态
    static WeldingState from_json(const std::string& /*json_str*/) {
        // 简化解析
        return WeldingState();
    }
};

// ============================================================================
// WeldingStateManager - 状态管理（需求22-23）
// ============================================================================

class WeldingStateManager {
public:
    WeldingStateManager() {}

    // 需求22：初始化任务
    void initialize_task(const std::string& task_id,
                         const std::vector<WeldSeamInfo >& seams) {
        WeldingState state;
        state.task_id = task_id;
        state.task_name = "焊接任务_" + task_id;
        state.completed_seam_ids.clear();

        for (const auto& s : seams) {
            state.remaining_seam_ids.push_back(s.id);
        }

        states_[task_id] = state;
    }

    // 需求22：暂停焊接
    void pause_welding(const std::string& task_id, InterruptType itype) {
        if (states_.count(task_id)) {
            states_[task_id].is_paused = true;
            states_[task_id].interrupt_type = itype;
        }
    }

    // 需求22：恢复焊接
    void resume_welding(const std::string& task_id) {
        if (states_.count(task_id)) {
            states_[task_id].is_paused = false;
        }
    }

    // 需求22：实时重规划
    std::vector<int> replan_from_current(const std::string& task_id,
                                         const std::vector<WeldSeamInfo >& all_seams,
                                         WeldingPathPlanner& planner) {
        if (!states_.count(task_id)) return {};

        auto& state = states_[task_id];
        std::vector<WeldSeamInfo > remaining;

        for (int sid : state.remaining_seam_ids) {
            for (const auto& s : all_seams) {
                if (s.id == sid) {
                    remaining.push_back(s);
                    break;
                }
            }
        }

        auto groups = planner.group_seams(remaining);

        std::vector<int> result;
        for (const auto& group : groups) {
            auto path = planner.plan_group(group, all_seams);
            result.insert(result.end(), path.begin(), path.end());
        }

        return result;
    }

    // 需求22：保存状态
    void save_state(const std::string& task_id, const std::string& filename) const {
        if (!states_.count(task_id)) return;

        std::ofstream file(filename);
        file << states_.at(task_id).to_json();
    }

    // 需求22：加载状态
    void load_state(const std::string& task_id, const std::string& filename) {
        std::ifstream file(filename);
        std::stringstream buffer;
        buffer << file.rdbuf();

        states_[task_id] = WeldingState::from_json(buffer.str());
    }

private:
    std::unordered_map<std::string, WeldingState> states_;
};

// ============================================================================
// WeldingSystem - 系统集成（需求1, 9, 17, 24-25）
// ============================================================================

class WeldingSystem {
public:
    WeldingPathPlanner planner;
    PathOptimizer optimizer;
    WeldingStateManager state_manager;

    WeldingSystem(double plate_thickness = DEFAULT_PLATE_THICKNESS,
                  double group_distance_threshold = DEFAULT_GROUP_DISTANCE_THRESHOLD)
        : planner(plate_thickness, group_distance_threshold) {}

    // 需求1：主规划入口
    std::vector<WeldSeamInfo > plan_and_optimize(const std::vector<WeldSeamInfo >& input_seams) {
        // 分组
        auto groups = planner.group_seams(input_seams);

        // 需求17：识别平行关系
        auto parallel_map = planner.build_parallel_groups(groups, input_seams);

        // 需求17：组间顺序
        auto group_order = planner.get_sorted_groups(groups, parallel_map);

        // 按组规划+优化
        std::vector<WeldSeamInfo > result;
        std::optional<Vec3> last_vertical_end;

        for (int gid : group_order) {
            SeamGroup* group = nullptr;
            for (auto& g : groups) {
                if (g.group_id == gid) {
                    group = &g;
                    break;
                }
            }

            if (!group) continue;

            // 组内规划
            auto path = planner.plan_group(*group, input_seams, last_vertical_end);

            // 优化
            auto optimized = optimizer.genetic_algorithm(path, input_seams);

            // 添加结果
            for (int sid : optimized) {
                auto seam = find_seam_by_id(input_seams, sid);

                // 确保立焊缝从高度最小的点开始
                if (seam.weld_position == WeldPosition::VERTICAL) {
                    if (seam.start_point.z > seam.end_point.z) {
                        std::swap(seam.start_point, seam.end_point);
                    }
                }

                result.push_back(seam);

                // 记录最后立焊终点
                if (seam.weld_position == WeldPosition::VERTICAL) {
                    last_vertical_end = seam.end_point;
                }
            }
        }

        return result;
    }

    // 需求9：JSON导入（完整实现）
    std::vector<WeldSeamInfo > load_from_json(const std::string& filename) {
        std::vector<WeldSeamInfo > seams;

        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法打开文件: " + filename);
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string content = buffer.str();

        // 解析JSON数组
        size_t arr_start = content.find('[');
        size_t arr_end = content.rfind(']');

        if (arr_start == std::string::npos || arr_end == std::string::npos) {
            throw std::runtime_error("无效的JSON格式");
        }

        // 提取数组内容
        std::string arr_content = content.substr(arr_start + 1, arr_end - arr_start - 1);

        // 分割对象
        size_t pos = 0;
        while (pos < arr_content.length()) {
            // 找到对象开始
            size_t obj_start = arr_content.find('{', pos);
            if (obj_start == std::string::npos) break;

            // 找到对应的结束花括号（处理嵌套）
            size_t obj_end = obj_start;
            int depth = 1;
            while (obj_end < arr_content.length() && depth > 0) {
                obj_end++;
                if (arr_content[obj_end] == '{') depth++;
                if (arr_content[obj_end] == '}') depth--;
            }

            if (depth != 0) break;

            // 提取对象字符串
            std::string obj_str = arr_content.substr(obj_start, obj_end - obj_start + 1);

            // 解析对象
            auto seam_opt = WeldSeamInfo ::from_json(obj_str);
            if (seam_opt.has_value()) {
                seams.push_back(seam_opt.value());
            }

            pos = obj_end + 1;
        }

        return seams;
    }

    // 需求3：导出优化后重新分组排列的焊缝数据
    void export_grouped_seams(const std::vector<WeldSeamInfo >& seams,
                              const std::string& output_dir) const {
        // 按组ID排序
        std::vector<WeldSeamInfo > sorted = seams;
        std::sort(sorted.begin(), sorted.end(), [](const WeldSeamInfo & a, const WeldSeamInfo & b) {
            return a.group_id < b.group_id;
        });

        std::string filepath = output_dir + "/grouped_seams.json";
        std::ofstream file(filepath);

        file << "[\n";
        for (size_t i = 0; i < sorted.size(); ++i) {
            file << "  " << sorted[i].to_json();
            if (i < sorted.size() - 1) file << ",";
            file << "\n";
        }
        file << "]";

        std::cout << "分组焊缝数据已导出到: " << filepath << std::endl;
    }

    // 需求3：导出焊接路径排序
    void export_welding_path(const std::vector<WeldSeamInfo >& seams,
                             const std::string& output_dir) const {
        std::string filepath = output_dir + "/welding_path.json";
        std::ofstream file(filepath);

        file << "{\n";
        file << "  \"path\": [\n";

        for (size_t i = 0; i < seams.size(); ++i) {
            const WeldSeamInfo & seam = seams[i];
            int a=0;
            if(seam.weld_position==WeldPosition::VERTICAL)
            {
                a=1;
            }
            file << "    {\n";
            file << "      \"seam_id\": " << seam.id << ",\n";
            file << "      \"start_point\": [" << seam.start_point.x << ", " << seam.start_point.y << ", " << seam.start_point.z << "],\n";
            file << "      \"end_point\": [" << seam.end_point.x << ", " << seam.end_point.y << ", " << seam.end_point.z << "],\n";
            file << "      \"direction\": [" << seam.direction.x << ", " << seam.direction.y << ", " << seam.direction.z << "],\n";
            file << "      \"weld_position\": [" << a <<"],\n";
            file << "      \"angle_deg\": [" << seam.azimuth-180 <<"],\n";
            file << "      \"start_angle\": [" << seam.start_stye <<"],\n";
            file << "      \"end_angle\": [" << seam.end_stye <<"]\n";

            file << "    }";
            if (i < seams.size() - 1) file << ",";
            file << "\n";
        }

        file << "  ]\n";
        file << "}";

        std::cout << "焊接路径已导出到: " << filepath << std::endl;
    }

    // 需求8：综合优化（遗传算法 + A*）
    std::vector<WeldSeamInfo > optimize_with_hybrid(const std::vector<WeldSeamInfo >& input_seams) {
        // 先使用遗传算法优化顺序
        auto ga_result = plan_and_optimize(input_seams);

        // [修复] A*简化实现：直接返回GA结果
        // 完整A*实现需要障碍物地图等额外参数，此处暂用GA结果
        return ga_result;
    }

    // 需求24：JSON导出（通用）
    void export_json(const std::vector<WeldSeamInfo >& seams, const std::string& filename) const {
        std::ofstream file(filename);
        file << "[\n";

        for (size_t i = 0; i < seams.size(); ++i) {
            file << "  " << seams[i].to_json();
            if (i < seams.size() - 1) file << ",";
            file << "\n";
        }

        file << "]";
    }

private:
    const WeldSeamInfo & find_seam_by_id(const std::vector<WeldSeamInfo >& seams, int id) const {
        for (const auto& s : seams) {
            if (s.id == id) return s;
        }
        throw std::out_of_range("焊缝ID " + std::to_string(id) + " 未找到");
    }
};

// ============================================================================
// 示例数据创建（需求7）
// ============================================================================

std::vector<WeldSeamInfo > create_sample_data() {
    /**
     * 示例数据：模拟船体结构的多组平行角焊缝
     *
     * 板厚=10mm，分组阈值=10mm
     *
     * 组A（左侧纵骨）：焊缝1,2 — 两条FILLET/VERTICAL，方向(0,0,1)，间隔5mm < 板厚 → 同组
     * 组B（右侧纵骨）：焊缝3,4 — 与组A平行，距离80mm < 200mm → 平行组
     * 组C（左侧水平连接）：焊缝5 — FILLET/FLAT
     * 组D（右侧水平连接）：焊缝6 — FILLET/FLAT，与组C平行
     * 组E（远处纵骨）：焊缝7,8 — 与A/B平行但距离300mm > 200mm → 独立组
     */

    return {
        // === 组A：左侧立角焊缝 (FILLET/VERTICAL, direction=(0,0,1) 沿z轴向上) ===
        WeldSeamInfo (1, 0, Vec3(0, 0, 0), Vec3(0, 0, 200), Vec3(0, 0, 1),
                 WeldType::FILLET, WeldPosition::VERTICAL, 10.0),
        WeldSeamInfo (2, 0, Vec3(5, 0, 0), Vec3(5, 0, 200), Vec3(0, 0, 1),
                 WeldType::FILLET, WeldPosition::VERTICAL, 10.0),

        // === 组B：右侧立角焊缝 (FILLET/VERTICAL) — 与组A平行，距离80mm ===
        WeldSeamInfo (3, 0, Vec3(80, 0, 0), Vec3(80, 0, 200), Vec3(0, 0, 1),
                 WeldType::FILLET, WeldPosition::VERTICAL, 10.0),
        WeldSeamInfo (4, 0, Vec3(85, 0, 0), Vec3(85, 0, 200), Vec3(0, 0, 1),
                 WeldType::FILLET, WeldPosition::VERTICAL, 10.0),

        // === 组C：左侧水平连接 (FILLET/FLAT, direction=(1,0,0) 沿x轴) ===
        WeldSeamInfo (5, 0, Vec3(0, 0, 200), Vec3(5, 0, 200), Vec3(1, 0, 0),
                 WeldType::FILLET, WeldPosition::FLAT, 10.0),

        // === 组D：右侧水平连接 (FILLET/FLAT) — 与组C平行，距离80mm < 200mm ===
        WeldSeamInfo (6, 0, Vec3(80, 0, 200), Vec3(85, 0, 200), Vec3(1, 0, 0),
                 WeldType::FILLET, WeldPosition::FLAT, 10.0),

        // === 组E：远处纵骨 — 与A/B平行但距离300mm > 200mm阈值 ===
        WeldSeamInfo (7, 0, Vec3(300, 0, 0), Vec3(300, 0, 200), Vec3(0, 0, 1),
                 WeldType::FILLET, WeldPosition::VERTICAL, 10.0),
        WeldSeamInfo (8, 0, Vec3(305, 0, 0), Vec3(305, 0, 200), Vec3(0, 0, 1),
                 WeldType::FILLET, WeldPosition::VERTICAL, 10.0),
    };
}

// 全局参数变量（可从外部配置文件读取）
double g_horizontal_plate_thickness = DEFAULT_HORIZONTAL_PLATE_THICKNESS;
double g_vertical_plate_thickness = DEFAULT_VERTICAL_PLATE_THICKNESS;
double g_default_plate_thickness = DEFAULT_PLATE_THICKNESS;
double g_group_distance_threshold = DEFAULT_GROUP_DISTANCE_THRESHOLD;
double g_travel_speed = TRAVEL_SPEED;
double g_direction_parallel_threshold = DIRECTION_PARALLEL_THRESHOLD;

// 遗传算法参数
int g_ga_pop_size = GA_POP_SIZE;
int g_ga_max_generations = GA_MAX_GENERATIONS;
double g_ga_crossover_rate = GA_CROSSOVER_RATE;
double g_ga_mutation_rate = GA_MUTATION_RATE;
int g_ga_elitism_count = GA_ELITISM_COUNT;

// 从JSON配置文件加载参数
void load_parameters_from_file(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "警告: 无法打开参数文件: " << filepath << ", 使用默认参数\n";
        return;
    }

    std::string json_str((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
    file.close();

    auto get_value = [&](const std::string& key, double default_val) -> double {
        size_t key_pos = json_str.find("\"" + key + "\"");
        if (key_pos == std::string::npos) return default_val;
        size_t val_start = json_str.find(':', key_pos);
        if (val_start == std::string::npos) return default_val;
        size_t val_end = json_str.find_first_of(",\n}", val_start);
        if (val_end == std::string::npos) return default_val;
        std::string val = json_str.substr(val_start + 1, val_end - val_start - 1);
        return safe_stod(val, default_val);
    };

    auto get_int_value = [&](const std::string& key, int default_val) -> int {
        return static_cast<int>(get_value(key, static_cast<double>(default_val)));
    };

    g_horizontal_plate_thickness = get_value("horizontal_plate_thickness", DEFAULT_HORIZONTAL_PLATE_THICKNESS);
    g_vertical_plate_thickness = get_value("vertical_plate_thickness", DEFAULT_VERTICAL_PLATE_THICKNESS);
    g_default_plate_thickness = get_value("default_plate_thickness", DEFAULT_PLATE_THICKNESS);
    g_group_distance_threshold = get_value("group_distance_threshold", DEFAULT_GROUP_DISTANCE_THRESHOLD);
    g_travel_speed = get_value("travel_speed", TRAVEL_SPEED);
    g_direction_parallel_threshold = get_value("parallel_threshold", DIRECTION_PARALLEL_THRESHOLD);

    g_ga_pop_size = get_int_value("ga_pop_size", GA_POP_SIZE);
    g_ga_max_generations = get_int_value("ga_max_generations", GA_MAX_GENERATIONS);
    g_ga_crossover_rate = get_value("ga_crossover_rate", GA_CROSSOVER_RATE);
    g_ga_mutation_rate = get_value("ga_mutation_rate", GA_MUTATION_RATE);
    g_ga_elitism_count = get_int_value("ga_elitism_count", GA_ELITISM_COUNT);

    std::cout << "已加载参数配置文件: " << filepath << "\n";
}


// 辅助函数：计算总焊缝数量
static int getTotalCount(
    const std::vector<std::vector<WeldSeam3D>>& grouped_seams,
    const std::vector<std::vector<WeldSeam3D>>& grouped_seams_with_arc,
    const std::vector<std::vector<WeldArc3D>>& grouped_arcs,
    const std::vector<WeldLSeam3D>& vertical_seams) {

    int count = 0;
    for (const auto& g : grouped_seams) count += g.size();
    for (const auto& g : grouped_seams_with_arc) count += g.size();
    for (const auto& g : grouped_arcs) count += g.size();
    count += vertical_seams.size();
    return count;
}


// IterativeRANSACLineFitting 实现
IterativeRANSACLineFitting::IterativeRANSACLineFitting() {
    distance_threshold_ = 5;
    min_inliers_ = 20;
    max_iterations_ = 100;
    min_points_to_fit_ = 10;
    max_gap_threshold_ = 10.0;
    arc_distance_threshold_ = 5;
    min_arc_inliers_ = 20;
    arc_min_radius_ = 2.0;
    arc_max_radius_ = 1000.0;
    arc_outlier_ratio_ = 0.3;
    arc_angle_tolerance_ = 0.1;
    arc_min_continuous_ = 5;
}

// 设置方法实现
void IterativeRANSACLineFitting::setDistanceThreshold(double threshold) {
    distance_threshold_ = threshold;
}
void IterativeRANSACLineFitting::setMinInliers(int min_inliers) {
    min_inliers_ = min_inliers;
}
void IterativeRANSACLineFitting::setMaxIterations(int max_iter) {
    max_iterations_ = max_iter;
}
void IterativeRANSACLineFitting::setMinPointsToFit(int min_pts) {
    min_points_to_fit_ = min_pts;
}
void IterativeRANSACLineFitting::setMaxGapThreshold(double gap) {
    max_gap_threshold_ = gap;
}
void IterativeRANSACLineFitting::setArcDistanceThreshold(double threshold) {
    arc_distance_threshold_ = threshold;
}
void IterativeRANSACLineFitting::setMinArcInliers(int min_inliers) {
    min_arc_inliers_ = min_inliers;
}
void IterativeRANSACLineFitting::setArcRadiusLimits(double min_radius, double max_radius) {
    arc_min_radius_ = min_radius;
    arc_max_radius_ = max_radius;
}
void IterativeRANSACLineFitting::setArcOutlierRatio(double ratio) {
    arc_outlier_ratio_ = ratio;
}
void IterativeRANSACLineFitting::setArcAngleTolerance(double tolerance) {
    arc_angle_tolerance_ = tolerance;
}
void IterativeRANSACLineFitting::setArcMinContinuous(int min_pts) {
    arc_min_continuous_ = min_pts;
}

namespace {

constexpr double kGeometryEpsilon = 1e-9;
constexpr double kMaxTIntersectionDistance = 4.0;
constexpr double kMinTIntersectionAngleDeg = 45.0;
constexpr double kMinSplitSegmentLength = 20.0;
constexpr double kCutPointMergeDistance = 4.0;

Point2D interpolatePoint(const WeldSeam2D& seam, double t)
{
    return Point2D(
        seam.start.x + (seam.end.x - seam.start.x) * t,
        seam.start.y + (seam.end.y - seam.start.y) * t);
}

bool findTIntersectionCut(
    const WeldSeam2D& branch,
    const Point2D& branchEndpoint,
    const WeldSeam2D& throughSeam,
    double& cutParameter)
{
    const double branchDx = branch.end.x - branch.start.x;
    const double branchDy = branch.end.y - branch.start.y;
    const double throughDx = throughSeam.end.x - throughSeam.start.x;
    const double throughDy = throughSeam.end.y - throughSeam.start.y;

    const double branchLength = std::hypot(branchDx, branchDy);
    const double throughLength = std::hypot(throughDx, throughDy);
    if (branchLength < kGeometryEpsilon || throughLength < kGeometryEpsilon) {
        return false;
    }

    // 使用锐角判断两条焊缝是否明显相交，排除平行或近似平行焊缝。
    double cosAngle = std::abs(branchDx * throughDx + branchDy * throughDy) /
                      (branchLength * throughLength);
    cosAngle = std::max(0.0, std::min(1.0, cosAngle));
    const double acuteAngleDeg = std::acos(cosAngle) * 180.0 / M_PI;
    if (acuteAngleDeg < kMinTIntersectionAngleDeg) {
        return false;
    }

    const double throughLengthSquared = throughDx * throughDx + throughDy * throughDy;
    const double endpointDx = branchEndpoint.x - throughSeam.start.x;
    const double endpointDy = branchEndpoint.y - throughSeam.start.y;
    const double t = (endpointDx * throughDx + endpointDy * throughDy) /
                     throughLengthSquared;

    // 交点必须位于主焊缝内部，并保证切开后的两段都有可焊接长度。
    const double endpointGuard = kMinSplitSegmentLength / throughLength;
    if (t <= endpointGuard || t >= 1.0 - endpointGuard) {
        return false;
    }

    const Point2D foot = interpolatePoint(throughSeam, t);
    if (branchEndpoint.distance(foot) > kMaxTIntersectionDistance) {
        return false;
    }

    cutParameter = t;
    return true;
}

void appendSegmentInliers(
    const WeldSeam2D& source,
    const std::vector<Point2D>& points,
    double startParameter,
    double endParameter,
    bool includeEnd,
    WeldSeam2D& segment)
{
    const double dx = source.end.x - source.start.x;
    const double dy = source.end.y - source.start.y;
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared < kGeometryEpsilon) {
        return;
    }

    for (int index : source.inlier_indices) {
        if (index < 0 || static_cast<size_t>(index) >= points.size()) {
            continue;
        }

        const Point2D& point = points[index];
        const double pointParameter =
            ((point.x - source.start.x) * dx + (point.y - source.start.y) * dy) /
            lengthSquared;

        const bool afterStart = pointParameter >= startParameter - kGeometryEpsilon;
        const bool beforeEnd = includeEnd
                                   ? pointParameter <= endParameter + kGeometryEpsilon
                                   : pointParameter < endParameter;
        if (afterStart && beforeEnd) {
            segment.inlier_indices.push_back(index);
        }
    }
}

void splitLinesAtTIntersections(
    std::vector<WeldSeam2D>& seams,
    const std::vector<Point2D>& points)
{
    if (seams.size() < 2) {
        return;
    }

    // 先收集每条主焊缝上的所有切割位置，避免遍历过程中修改 vector
    // 导致漏检、重复分割或编号不稳定。
    std::vector<std::vector<double>> cutParameters(seams.size());

    for (size_t branchIndex = 0; branchIndex < seams.size(); ++branchIndex) {
        const WeldSeam2D& branch = seams[branchIndex];

        for (size_t throughIndex = 0; throughIndex < seams.size(); ++throughIndex) {
            if (branchIndex == throughIndex) {
                continue;
            }

            const WeldSeam2D& throughSeam = seams[throughIndex];
            double cutParameter = 0.0;

            if (findTIntersectionCut(branch, branch.start, throughSeam, cutParameter)) {
                cutParameters[throughIndex].push_back(cutParameter);
            }
            if (findTIntersectionCut(branch, branch.end, throughSeam, cutParameter)) {
                cutParameters[throughIndex].push_back(cutParameter);
            }
        }
    }

    std::vector<WeldSeam2D> splitSeams;
    splitSeams.reserve(seams.size() * 2);

    for (size_t seamIndex = 0; seamIndex < seams.size(); ++seamIndex) {
        const WeldSeam2D& source = seams[seamIndex];
        const double sourceLength = source.start.distance(source.end);
        if (sourceLength < kGeometryEpsilon || cutParameters[seamIndex].empty()) {
            splitSeams.push_back(source);
            continue;
        }

        auto& cuts = cutParameters[seamIndex];
        std::sort(cuts.begin(), cuts.end());

        // 合并因拟合误差产生的相近切割点，并确保相邻分段不小于最短长度。
        std::vector<double> acceptedCuts;
        acceptedCuts.push_back(0.0);
        for (double cut : cuts) {
            const double distanceFromPrevious =
                (cut - acceptedCuts.back()) * sourceLength;
            const double distanceToEnd = (1.0 - cut) * sourceLength;

            if (distanceFromPrevious < kCutPointMergeDistance ||
                distanceFromPrevious < kMinSplitSegmentLength ||
                distanceToEnd < kMinSplitSegmentLength) {
                continue;
            }
            acceptedCuts.push_back(cut);
        }
        acceptedCuts.push_back(1.0);

        if (acceptedCuts.size() == 2) {
            splitSeams.push_back(source);
            continue;
        }

        for (size_t cutIndex = 0; cutIndex + 1 < acceptedCuts.size(); ++cutIndex) {
            const double startParameter = acceptedCuts[cutIndex];
            const double endParameter = acceptedCuts[cutIndex + 1];
            const double segmentLength =
                (endParameter - startParameter) * sourceLength;
            if (segmentLength < kMinSplitSegmentLength) {
                continue;
            }

            WeldSeam2D segment;
            segment.line = source.line;
            segment.start = interpolatePoint(source, startParameter);
            segment.end = interpolatePoint(source, endParameter);
            segment.length = segmentLength;
            segment.angle = source.angle;

            appendSegmentInliers(
                source,
                points,
                startParameter,
                endParameter,
                cutIndex + 2 == acceptedCuts.size(),
                segment);
            splitSeams.push_back(std::move(segment));
        }
    }

    seams = std::move(splitSeams);
}

} // namespace

// 主要提取函数实现
std::tuple<std::vector<WeldSeam2D>, std::vector<WeldArc2D>>
IterativeRANSACLineFitting::extractAllWeldSeamsAndArcs(const std::vector<Point2D>& points) {
    std::vector<WeldSeam2D> seams;
    std::vector<WeldArc2D> arcs;
    std::vector<bool> used(points.size(), false);
    std::vector<int> remaining_indices;

    for (size_t i = 0; i < points.size(); ++i) {
        remaining_indices.push_back(i);
    }

    extractLines(points, remaining_indices, used, seams);
    // 只处理 T 形相交：支线端点靠近主焊缝内部时，将主焊缝断开。
    splitLinesAtTIntersections(seams, points);
    updateRemainingIndices(remaining_indices, used);
    extractArcs(points, remaining_indices, used, arcs);
    return {seams, arcs};
}


void IterativeRANSACLineFitting::extractLines(const std::vector<Point2D>& points,
                                              std::vector<int>& remaining_indices,
                                              std::vector<bool>& used,
                                              std::vector<WeldSeam2D>& seams) {
    while (remaining_indices.size() >= static_cast<size_t>(min_points_to_fit_)) {
        WeldSeam2D seam;
        std::vector<int> best_inliers;

        bool found = ransacFitLineWithIndices(points, remaining_indices,
                                              seam.line, best_inliers);

        if (!found || best_inliers.size() < static_cast<size_t>(min_inliers_)) {
            break;
        }

        std::vector<std::vector<int>> continuous_segments =
            splitIntoContinuousSegments(points, remaining_indices, best_inliers, seam.line);

        for (const auto& segment : continuous_segments) {
            if (segment.size() < static_cast<size_t>(min_inliers_ / 2)) continue;

            WeldSeam2D continuous_seam;
            continuous_seam.line = seam.line;

            for (int idx : segment) {
                continuous_seam.inlier_indices.push_back(remaining_indices[idx]);
                used[remaining_indices[idx]] = true;
            }
            computeEndpoints(points, continuous_seam.inlier_indices,
                             continuous_seam.line, continuous_seam.start, continuous_seam.end);

            continuous_seam.length = continuous_seam.start.distance(continuous_seam.end);
            continuous_seam.angle = continuous_seam.line.angle * 180 / M_PI;

            seams.push_back(continuous_seam);
        }

        for (const auto& segment : continuous_segments) {
            for (int idx : segment) {
                used[remaining_indices[idx]] = true;
            }
        }
        updateRemainingIndices(remaining_indices, used);

        if (remaining_indices.size() < static_cast<size_t>(min_points_to_fit_)) {
            break;
        }
    }
}


void IterativeRANSACLineFitting::extractArcs(const std::vector<Point2D>& points,
                                             std::vector<int>& remaining_indices,
                                             std::vector<bool>& used,
                                             std::vector<WeldArc2D>& arcs) {
    int arc_count = 0;
    int max_attempts = 10;

    while (remaining_indices.size() >= static_cast<size_t>(min_points_to_fit_) &&
           arc_count < max_attempts) {

        WeldArc2D weld_arc;
        std::vector<int> best_inliers;

        bool found = ransacFitArcWithIndices(points, remaining_indices,
                                             weld_arc.arc, best_inliers);

        if (!found || best_inliers.size() < static_cast<size_t>(min_arc_inliers_)) {
            break;
        }

        for (int idx : best_inliers) {
            weld_arc.inlier_indices.push_back(remaining_indices[idx]);
            used[remaining_indices[idx]] = true;
        }

        refineArcParameters(points, remaining_indices, best_inliers, weld_arc.arc);

        determineArcAngles(points, remaining_indices, best_inliers,
                           weld_arc.arc,
                           weld_arc.arc.start_angle,
                           weld_arc.arc.end_angle,
                           weld_arc.arc.is_clockwise);

        weld_arc.length = weld_arc.arc.getArcLength();
        weld_arc.start = weld_arc.arc.getPointAtAngle(weld_arc.arc.start_angle);
        weld_arc.end = weld_arc.arc.getPointAtAngle(weld_arc.arc.end_angle);
        weld_arc.middle = computeArcMiddlePoint(weld_arc.arc);

        arcs.push_back(weld_arc);
        arc_count++;

        updateRemainingIndices(remaining_indices, used);

        if (remaining_indices.size() < static_cast<size_t>(min_points_to_fit_)) {
            break;
        }
    }
}

void IterativeRANSACLineFitting::updateRemainingIndices(std::vector<int>& remaining_indices,
                                                        const std::vector<bool>& used) {
    std::vector<int> new_remaining;
    for (int idx : remaining_indices) {
        if (!used[idx]) {
            new_remaining.push_back(idx);
        }
    }
    remaining_indices = new_remaining;
}

std::vector<std::vector<int>> IterativeRANSACLineFitting::splitIntoContinuousSegments(
    const std::vector<Point2D>& all_points,
    const std::vector<int>& point_indices,
    const std::vector<int>& inliers,
    const Line2D& line) {

    if (inliers.empty()) return {};

    std::vector<std::pair<double, int>> projected_points;

    Point2D direction(-line.b, line.a);
    double norm = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (norm > 0) {
        direction.x /= norm;
        direction.y /= norm;
    }

    double ref_x, ref_y;
    if (std::abs(line.b) > 1e-6) {
        ref_x = 0;
        ref_y = -line.c / line.b;
    } else {
        ref_y = 0;
        ref_x = -line.c / line.a;
    }
    Point2D ref_point(ref_x, ref_y);

    for (int idx : inliers) {
        const Point2D& p = all_points[point_indices[idx]];
        Point2D vec(p.x - ref_point.x, p.y - ref_point.y);
        double proj = vec.x * direction.x + vec.y * direction.y;
        projected_points.push_back({proj, idx});
    }

    std::sort(projected_points.begin(), projected_points.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<std::vector<int>> segments;
    std::vector<int> current_segment;
    double last_proj = projected_points[0].first;
    current_segment.push_back(projected_points[0].second);

    for (size_t i = 1; i < projected_points.size(); ++i) {
        Point2D p1(all_points[point_indices[projected_points[i-1].second]].x,
                   all_points[point_indices[projected_points[i-1].second]].y);
        Point2D p2(all_points[point_indices[projected_points[i].second]].x,
                   all_points[point_indices[projected_points[i].second]].y);
        double actual_gap = p1.distance(p2);

        if (actual_gap > max_gap_threshold_) {
            if (current_segment.size() >= static_cast<size_t>(min_points_to_fit_ / 2)) {
                segments.push_back(current_segment);
            }
            current_segment.clear();
        }
        current_segment.push_back(projected_points[i].second);
        last_proj = projected_points[i].first;
    }

    if (!current_segment.empty() &&
        current_segment.size() >= static_cast<size_t>(min_points_to_fit_ / 2)) {
        segments.push_back(current_segment);
    }

    return segments;
}

bool IterativeRANSACLineFitting::ransacFitLineWithIndices(const std::vector<Point2D>& all_points,
                                                          const std::vector<int>& point_indices,
                                                          Line2D& best_line,
                                                          std::vector<int>& best_inliers) {
    if (point_indices.size() < static_cast<size_t>(min_points_to_fit_)) {
        return false;
    }

    best_inliers.clear();
    int best_inlier_count = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, point_indices.size() - 1);

    for (int iter = 0; iter < max_iterations_; ++iter) {
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        while (idx1 == idx2 && point_indices.size() > 1) {
            idx2 = dis(gen);
        }

        const Point2D& p1 = all_points[point_indices[idx1]];
        const Point2D& p2 = all_points[point_indices[idx2]];

        Line2D line;
        line.a = p2.y - p1.y;
        line.b = p1.x - p2.x;
        line.c = p2.x * p1.y - p1.x * p2.y;

        double norm = std::sqrt(line.a * line.a + line.b * line.b);
        if (norm < 1e-6) continue;
        line.a /= norm;
        line.b /= norm;
        line.c /= norm;

        std::vector<int> inliers;
        for (size_t i = 0; i < point_indices.size(); ++i) {
            double dist = line.pointDistance(all_points[point_indices[i]]);
            if (dist < distance_threshold_) {
                inliers.push_back(i);
            }
        }

        if (inliers.size() > static_cast<size_t>(best_inlier_count)) {
            best_inlier_count = inliers.size();
            best_inliers = inliers;
            best_line = line;
        }

        if (best_inlier_count > point_indices.size() * 0.8) {
            break;
        }
    }

    if (best_inliers.size() >= static_cast<size_t>(min_points_to_fit_)) {
        refineLineWithInliers(all_points, point_indices, best_inliers, best_line);
    }

    return best_inliers.size() >= static_cast<size_t>(min_inliers_);
}

void IterativeRANSACLineFitting::computeEndpoints(const std::vector<Point2D>& all_points,
                                                  const std::vector<int>& point_indices,
                                                  const Line2D& line,
                                                  Point2D& start,
                                                  Point2D& end) {
    if (point_indices.empty()) return;

    Point2D direction(-line.b, line.a);
    double norm = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (norm > 0) {
        direction.x /= norm;
        direction.y /= norm;
    }

    double ref_x, ref_y;
    if (std::abs(line.b) > 1e-6) {
        ref_x = 0;
        ref_y = -line.c / line.b;
    } else {
        ref_y = 0;
        ref_x = -line.c / line.a;
    }
    Point2D ref_point(ref_x, ref_y);

    std::vector<double> projections;
    std::vector<int> indices;

    for (int idx : point_indices) {
        const Point2D& p = all_points[idx];
        Point2D vec(p.x - ref_point.x, p.y - ref_point.y);
        double proj = vec.x * direction.x + vec.y * direction.y;
        projections.push_back(proj);
        indices.push_back(idx);
    }

    auto min_it = std::min_element(projections.begin(), projections.end());
    auto max_it = std::max_element(projections.begin(), projections.end());

    int min_idx = indices[std::distance(projections.begin(), min_it)];
    int max_idx = indices[std::distance(projections.begin(), max_it)];

    start = Point2D(all_points[min_idx].x, all_points[min_idx].y);
    end = Point2D(all_points[max_idx].x, all_points[max_idx].y);

    projectPointToLine(start, line, start);
    projectPointToLine(end, line, end);
}

void IterativeRANSACLineFitting::projectPointToLine(const Point2D& point,
                                                    const Line2D& line,
                                                    Point2D& projected) {
    double denom = line.a * line.a + line.b * line.b;
    if (denom < 1e-6) return;

    projected.x = point.x - line.a * (line.a * point.x + line.b * point.y + line.c) / denom;
    projected.y = point.y - line.b * (line.a * point.x + line.b * point.y + line.c) / denom;
}

void IterativeRANSACLineFitting::refineLineWithInliers(const std::vector<Point2D>& all_points,
                                                       const std::vector<int>& point_indices,
                                                       const std::vector<int>& inliers,
                                                       Line2D& line) {
    std::vector<Point2D> points;
    for (int idx : inliers) {
        points.push_back(all_points[point_indices[idx]]);
    }

    double sum_x = 0, sum_y = 0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
    }
    double mean_x = sum_x / points.size();
    double mean_y = sum_y / points.size();

    double cov_xx = 0, cov_xy = 0, cov_yy = 0;
    for (const auto& p : points) {
        double dx = p.x - mean_x;
        double dy = p.y - mean_y;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }

    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    double discriminant = std::sqrt(std::max(0.0, trace * trace / 4 - det));
    double eig_val1 = trace / 2 + discriminant;
    double eig_vec1_x, eig_vec1_y;

    if (std::abs(cov_xy) > 1e-6) {
        eig_vec1_x = eig_val1 - cov_yy;
        eig_vec1_y = cov_xy;
    } else {
        eig_vec1_x = 1;
        eig_vec1_y = 0;
    }

    double norm = std::sqrt(eig_vec1_x * eig_vec1_x + eig_vec1_y * eig_vec1_y);
    if (norm > 1e-6) {
        eig_vec1_x /= norm;
        eig_vec1_y /= norm;
    }

    line.a = -eig_vec1_y;
    line.b = eig_vec1_x;
    line.c = -(line.a * mean_x + line.b * mean_y);

    norm = std::sqrt(line.a * line.a + line.b * line.b);
    if (norm > 1e-6) {
        line.a /= norm;
        line.b /= norm;
        line.c /= norm;
    }

    line.angle = atan2(-line.a, line.b);
}

bool IterativeRANSACLineFitting::ransacFitArcWithIndices(const std::vector<Point2D>& all_points,
                                                         const std::vector<int>& point_indices,
                                                         Arc2D& best_arc,
                                                         std::vector<int>& best_inliers) {
    if (point_indices.size() < static_cast<size_t>(min_points_to_fit_)) {
        return false;
    }

    best_inliers.clear();
    int best_inlier_count = 0;
    Arc2D best_candidate;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, point_indices.size() - 1);

    for (int iter = 0; iter < max_iterations_; ++iter) {
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        int idx3 = dis(gen);

        while (idx1 == idx2 && point_indices.size() > 1) {
            idx2 = dis(gen);
        }
        while (idx3 == idx1 || idx3 == idx2) {
            idx3 = dis(gen);
        }

        const Point2D& p1 = all_points[point_indices[idx1]];
        const Point2D& p2 = all_points[point_indices[idx2]];
        const Point2D& p3 = all_points[point_indices[idx3]];

        Arc2D arc;
        double A = 2 * (p2.x - p1.x);
        double B = 2 * (p2.y - p1.y);
        double C = p2.x * p2.x - p1.x * p1.x + p2.y * p2.y - p1.y * p1.y;

        double D = 2 * (p3.x - p2.x);
        double E = 2 * (p3.y - p2.y);
        double F = p3.x * p3.x - p2.x * p2.x + p3.y * p3.y - p2.y * p2.y;

        double den = A * E - B * D;
        if (std::abs(den) < 1e-8) {
            continue;
        }

        arc.cx = (C * E - B * F) / den;
        arc.cy = (A * F - C * D) / den;
        arc.radius = std::hypot(p1.x - arc.cx, p1.y - arc.cy);

        if (arc.radius < arc_min_radius_ || arc.radius > arc_max_radius_) {
            continue;
        }

        std::vector<int> inliers;
        for (size_t i = 0; i < point_indices.size(); ++i) {
            double dist = std::abs(std::hypot(all_points[point_indices[i]].x - arc.cx,
                                              all_points[point_indices[i]].y - arc.cy) - arc.radius);
            if (dist < arc_distance_threshold_) {
                inliers.push_back(i);
            }
        }

        if (inliers.size() > static_cast<size_t>(best_inlier_count)) {
            best_inlier_count = inliers.size();
            best_inliers = inliers;
            best_candidate = arc;
        }

        if (best_inlier_count > point_indices.size() * 0.8) {
            break;
        }
    }

    if (best_inlier_count >= static_cast<size_t>(min_arc_inliers_)) {
        best_arc = best_candidate;
        determineArcAngles(all_points, point_indices, best_inliers, best_arc,
                           best_arc.start_angle, best_arc.end_angle, best_arc.is_clockwise);
        return true;
    }

    return false;
}

void IterativeRANSACLineFitting::determineArcAngles(const std::vector<Point2D>& all_points,
                                                    const std::vector<int>& point_indices,
                                                    const std::vector<int>& inliers,
                                                    const Arc2D& arc,
                                                    double& start_angle,
                                                    double& end_angle,
                                                    bool& is_clockwise) {
    if (inliers.size() < 3) {
        start_angle = 0;
        end_angle = 2 * M_PI;
        is_clockwise = false;
        return;
    }

    std::vector<double> angles;
    for (int idx : inliers) {
        const Point2D& p = all_points[point_indices[idx]];
        double angle = std::atan2(p.y - arc.cy, p.x - arc.cx);
        angles.push_back(angle);
    }

    std::sort(angles.begin(), angles.end());

    double max_gap = 0;
    int max_gap_idx = -1;

    for (size_t i = 0; i < angles.size(); ++i) {
        double gap;
        if (i == angles.size() - 1) {
            gap = 2 * M_PI - angles[i] + angles[0];
        } else {
            gap = angles[i + 1] - angles[i];
        }

        if (gap > max_gap) {
            max_gap = gap;
            max_gap_idx = i;
        }
    }

    if (max_gap_idx == static_cast<int>(angles.size()) - 1) {
        start_angle = angles[0];
        end_angle = angles[max_gap_idx];
    } else {
        start_angle = angles[max_gap_idx + 1];
        end_angle = angles[max_gap_idx];
    }

    if (end_angle < start_angle) {
        end_angle += 2 * M_PI;
    }

    is_clockwise = false;

    if (end_angle - start_angle > M_PI) {
        std::swap(start_angle, end_angle);
        if (end_angle < start_angle) end_angle += 2 * M_PI;
        is_clockwise = true;
    }
}

Point2D IterativeRANSACLineFitting::computeArcMiddlePoint(const Arc2D& arc) {
    double mid_angle;
    if (!arc.is_clockwise) {
        mid_angle = arc.start_angle + (arc.end_angle - arc.start_angle) / 2;
    } else {
        if (arc.start_angle > arc.end_angle) {
            mid_angle = arc.start_angle - (arc.start_angle - arc.end_angle) / 2;
        } else {
            mid_angle = arc.start_angle + (2 * M_PI - arc.end_angle + arc.start_angle) / 2;
            if (mid_angle > 2 * M_PI) mid_angle -= 2 * M_PI;
        }
    }

    return Point2D(arc.cx + arc.radius * std::cos(mid_angle),
                   arc.cy + arc.radius * std::sin(mid_angle));
}

void IterativeRANSACLineFitting::refineArcParameters(const std::vector<Point2D>& points,
                                                     const std::vector<int>& indices,
                                                     const std::vector<int>& inliers,
                                                     Arc2D& arc) {
    if (inliers.size() < 5) return;

    double sum_x = 0, sum_y = 0;
    for (int idx : inliers) {
        const Point2D& p = points[indices[idx]];
        sum_x += p.x;
        sum_y += p.y;
    }
    double mean_x = sum_x / inliers.size();
    double mean_y = sum_y / inliers.size();

    std::vector<double> x_rel, y_rel;
    for (int idx : inliers) {
        const Point2D& p = points[indices[idx]];
        x_rel.push_back(p.x - mean_x);
        y_rel.push_back(p.y - mean_y);
    }

    double A = 0, B = 0, C = 0, D = 0, E = 0;
    for (size_t i = 0; i < x_rel.size(); ++i) {
        double xi = x_rel[i];
        double yi = y_rel[i];
        A += xi * xi;
        B += xi * yi;
        C += yi * yi;
        D += xi * (xi * xi + yi * yi);
        E += yi * (xi * xi + yi * yi);
    }

    double det = A * C - B * B;
    if (std::abs(det) > 1e-8) {
        double uc = (D * C - B * E) / (2 * det);
        double vc = (A * E - B * D) / (2 * det);

        arc.cx = mean_x + uc;
        arc.cy = mean_y + vc;

        double sum_r = 0;
        for (int idx : inliers) {
            const Point2D& p = points[indices[idx]];
            double r = std::hypot(p.x - arc.cx, p.y - arc.cy);
            sum_r += r;
        }
        arc.radius = sum_r / inliers.size();
    }
}

// ============ WeldSeamExtractor 实现 ============

double WeldSeamExtractor::pointToPoint(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
}

std::tuple<std::vector<WeldSeam2D>, std::vector<WeldArc2D>>
WeldSeamExtractor::extractSeamsAndArcs(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                       double distance_threshold,
                                       int min_inliers,
                                       int max_iterations,
                                       int min_points_to_fit,
                                       double max_gap,
                                       double arc_dist_threshold,
                                       int min_arc_inliers,
                                       double arc_min_radius,
                                       double arc_max_radius,
                                       double arc_outlier_ratio,
                                       double arc_angle_tolerance,
                                       int arc_min_continuous) {
    std::vector<Point2D> points_2d;
    for (const auto& point : cloud.points) {
        points_2d.emplace_back(point.x, point.y);
    }

    IterativeRANSACLineFitting ransac;
    ransac.setDistanceThreshold(distance_threshold);
    ransac.setMinInliers(min_inliers);
    ransac.setMaxIterations(max_iterations);
    ransac.setMinPointsToFit(min_points_to_fit);
    ransac.setMaxGapThreshold(max_gap);
    ransac.setArcDistanceThreshold(arc_dist_threshold);
    ransac.setMinArcInliers(min_arc_inliers);
    ransac.setArcRadiusLimits(arc_min_radius, arc_max_radius);
    ransac.setArcOutlierRatio(arc_outlier_ratio);
    ransac.setArcAngleTolerance(arc_angle_tolerance);
    ransac.setArcMinContinuous(arc_min_continuous);

    return ransac.extractAllWeldSeamsAndArcs(points_2d);
}




WeldSeamExtractor::ExtractionResult3D WeldSeamExtractor::convertAndGroupTo3D(
    std::vector<WeldSeam2D>& seams,
    const std::vector<WeldArc2D>& arcs,
    const std::vector<Point2D>& points_2d,
    double z0,
    const Eigen::Vector3f& axis,
    float angle,
    double vertical_length,
    double connection_threshold,
    double group_threshold,
    double arc_proximity_threshold,
    std::vector<std::vector<pcl::PointXYZ>>* arc_clouds_out,
    double endpoint_proximity_threshold,
    bool do_clockwise_sort) {

    qDebug() << "1. 函数开始";

    ExtractionResult3D result;
    const double PI = 3.14159265358979323846;

    // 创建逆旋转矩阵
    Eigen::Matrix4f transform_back = Eigen::Matrix4f::Identity();
    if (axis.norm() > 1e-6) {
        Eigen::Vector3f rot_axis = axis.normalized();
        Eigen::AngleAxisf rotation_back(-angle, rot_axis);
        transform_back.block<3,3>(0,0) = rotation_back.matrix();
    }
    qDebug() << "2. 旋转矩阵创建完成";

    // // ================================================================
    // // 1. 查找立焊缝 - 端点距离法 + 位置+方向双重去重
    // // ================================================================
    // qDebug() << "3. 开始查找立焊缝, seams.size()=" << seams.size();

    // // 候选点结构
    // struct CandidatePoint {
    //     double x;
    //     double y;
    //     double angle;
    // };

    // std::vector<CandidatePoint> candidates;

    // for (size_t i = 0; i < seams.size(); i++) {
    //     for (size_t j = i + 1; j < seams.size(); j++) {
    //         double A1 = seams[i].line.a, B1 = seams[i].line.b;
    //         double A2 = seams[j].line.a, B2 = seams[j].line.b;

    //         double numerator = fabs(A1 * A2 + B1 * B2);
    //         double denominator = sqrt(A1*A1 + B1*B1) * sqrt(A2*A2 + B2*B2);
    //         double cos_theta = numerator / denominator;
    //         if (cos_theta > 1.0) cos_theta = 1.0;
    //         if (cos_theta < -1.0) cos_theta = -1.0;

    //         // 只处理近似垂直的焊缝对 (夹角 45°~135°)
    //         if (acos(cos_theta) > 45 * PI / 180.0 && acos(cos_theta) < 135 * PI / 180.0) {
    //             Point2D p1(seams[i].start.x, seams[i].start.y);
    //             Point2D p2(seams[i].end.x, seams[i].end.y);
    //             Point2D p3(seams[j].start.x, seams[j].start.y);
    //             Point2D p4(seams[j].end.x, seams[j].end.y);

    //             // 4种端点组合
    //             struct EndpointPair {
    //                 Point2D* ep_i;
    //                 Point2D* ep_j;
    //                 Point2D* other_i;
    //                 Point2D* other_j;
    //             };

    //             EndpointPair pairs[4] = {
    //                 {&p1, &p3, &p2, &p4},
    //                 {&p1, &p4, &p2, &p3},
    //                 {&p2, &p3, &p1, &p4},
    //                 {&p2, &p4, &p1, &p3}
    //             };

    //             for (int k = 0; k < 4; k++) {
    //                 Point2D& ep_i = *pairs[k].ep_i;
    //                 Point2D& ep_j = *pairs[k].ep_j;
    //                 Point2D& other_i = *pairs[k].other_i;
    //                 Point2D& other_j = *pairs[k].other_j;

    //                 double dist = ep_i.distance(ep_j);
    //                 if (dist < connection_threshold) {
    //                     CandidatePoint cand;
    //                     cand.x = (ep_i.x + ep_j.x) / 2.0;
    //                     cand.y = (ep_i.y + ep_j.y) / 2.0;

    //                     // 方向角：两条焊缝方向向量的夹角平分线
    //                     double dx1 = other_i.x - ep_i.x;
    //                     double dy1 = other_i.y - ep_i.y;
    //                     double len1 = sqrt(dx1*dx1 + dy1*dy1);
    //                     if (len1 > 1e-10) { dx1 /= len1; dy1 /= len1; }

    //                     double dx2 = other_j.x - ep_j.x;
    //                     double dy2 = other_j.y - ep_j.y;
    //                     double len2 = sqrt(dx2*dx2 + dy2*dy2);
    //                     if (len2 > 1e-10) { dx2 /= len2; dy2 /= len2; }

    //                     double bisect_x = dx1 + dx2;
    //                     double bisect_y = dy1 + dy2;
    //                     double bisect_len = sqrt(bisect_x*bisect_x + bisect_y*bisect_y);
    //                     if (bisect_len > 1e-10) {
    //                         cand.angle = atan2(bisect_y / bisect_len, bisect_x / bisect_len) * 180.0 / PI;
    //                     } else {
    //                         cand.angle = 0.0;
    //                     }
    //                     if (cand.angle < 0) cand.angle += 360.0;

    //                     candidates.push_back(cand);
    //                 }
    //             }
    //         }
    //     }
    // }

    // qDebug() << "原始候选点数量:" << candidates.size();

    // // ========== 位置+方向双重去重合并 ==========
    // const double MERGE_THRESHOLD = 5.0;      // 位置合并阈值 (mm)
    // const double ANGLE_THRESHOLD = 15.0;     // 角度合并阈值 (度)

    // std::vector<Point2D> v_p;

    // for (const auto& cand : candidates) {
    //     bool found = false;
    //     for (auto& existing : v_p) {
    //         double dist = sqrt(pow(cand.x - existing.x, 2) + pow(cand.y - existing.y, 2));
    //         if (dist < MERGE_THRESHOLD) {
    //             // 位置相近，检查角度是否也相近
    //             double angle_diff = fabs(cand.angle - existing.angle);
    //             if (angle_diff > 180.0) angle_diff = 360.0 - angle_diff;
    //             if (angle_diff < ANGLE_THRESHOLD) {
    //                 // 位置和方向都相近，合并（位置取平均，角度用sin/cos平均）
    //                 existing.x = (existing.x + cand.x) / 2.0;
    //                 existing.y = (existing.y + cand.y) / 2.0;
    //                 // 角度用sin/cos平均
    //                 double rad1 = existing.angle * PI / 180.0;
    //                 double rad2 = cand.angle * PI / 180.0;
    //                 double avg_rad = atan2(sin(rad1) + sin(rad2), cos(rad1) + cos(rad2));
    //                 existing.angle = avg_rad * 180.0 / PI;
    //                 if (existing.angle < 0) existing.angle += 360.0;
    //                 found = true;
    //                 break;
    //             }
    //             // 位置相近但方向不同，属于不同的立焊缝，不合并
    //         }
    //     }
    //     if (!found) {
    //         Point2D p;
    //         p.x = cand.x;
    //         p.y = cand.y;
    //         p.angle = cand.angle;
    //         v_p.push_back(p);
    //     }
    // }

    // qDebug() << "去重后立焊缝数量:" << v_p.size();



    // 2. 转换直线到3D
    std::vector<WeldSeam3D> seams3d;
    seams3d.resize(seams.size());

    qDebug() << "5. seams3d 分配完成, size=" << seams3d.size();

    for (size_t i = 0; i < seams.size(); i++) {
        seams3d[i].start.x = seams[i].start.x;
        seams3d[i].start.y = seams[i].start.y;
        seams3d[i].start.z = z0;
        seams3d[i].end.x = seams[i].end.x;
        seams3d[i].end.y = seams[i].end.y;
        seams3d[i].end.z = z0;
        seams3d[i].length = seams[i].length;

        seams3d[i].original_index = i;   // 新增

        Eigen::Vector4f start_homo(seams3d[i].start.x, seams3d[i].start.y, seams3d[i].start.z, 1.0f);
        Eigen::Vector4f end_homo(seams3d[i].end.x, seams3d[i].end.y, seams3d[i].end.z, 1.0f);

        Eigen::Vector4f start_back = transform_back * start_homo;
        Eigen::Vector4f end_back = transform_back * end_homo;

        seams3d[i].start.x = start_back.x();
        seams3d[i].start.y = start_back.y();
        seams3d[i].start.z = start_back.z();
        seams3d[i].end.x = end_back.x();
        seams3d[i].end.y = end_back.y();
        seams3d[i].end.z = end_back.z();


    }

    qDebug() << "6. 直线转换完成";

    // // 3. 转换立焊缝到3D
    // result.vertical_seams.resize(v_p.size());
    // qDebug() << "7. vertical_seams 分配完成";
    // for (size_t i = 0; i < v_p.size(); i++) {
    //     result.vertical_seams[i].start.x = v_p[i].x;
    //     result.vertical_seams[i].start.y = v_p[i].y;
    //     result.vertical_seams[i].start.z = z0;
    //     result.vertical_seams[i].end.x = v_p[i].x;
    //     result.vertical_seams[i].end.y = v_p[i].y;
    //     result.vertical_seams[i].end.z = z0 - vertical_length;
    //     result.vertical_seams[i].angle = v_p[i].angle;

    //     Eigen::Vector4f start_homo(result.vertical_seams[i].start.x, result.vertical_seams[i].start.y,
    //                                result.vertical_seams[i].start.z, 1.0f);
    //     Eigen::Vector4f end_homo(result.vertical_seams[i].end.x, result.vertical_seams[i].end.y,
    //                              result.vertical_seams[i].end.z, 1.0f);

    //     Eigen::Vector4f start_back = transform_back * start_homo;
    //     Eigen::Vector4f end_back = transform_back * end_homo;

    //     result.vertical_seams[i].start.x = start_back.x();
    //     result.vertical_seams[i].start.y = start_back.y();
    //     result.vertical_seams[i].start.z = start_back.z();
    //     result.vertical_seams[i].end.x = end_back.x();
    //     result.vertical_seams[i].end.y = end_back.y();
    //     result.vertical_seams[i].end.z = end_back.z();
    // }
    // qDebug() << "8. 立焊缝转换完成";

    // 4. 转换圆弧到3D
    std::vector<WeldArc3D> arc3d;
    arc3d.resize(arcs.size());
    qDebug() << "9. arc3d 分配完成, arcs.size()=" << arcs.size();

    for (size_t j = 0; j < arcs.size(); j++) {
        arc3d[j].start.x = arcs[j].start.x;
        arc3d[j].start.y = arcs[j].start.y;
        arc3d[j].start.z = z0;
        arc3d[j].middle.x = arcs[j].middle.x;
        arc3d[j].middle.y = arcs[j].middle.y;
        arc3d[j].middle.z = z0;
        arc3d[j].end.x = arcs[j].end.x;
        arc3d[j].end.y = arcs[j].end.y;
        arc3d[j].end.z = z0;
        arc3d[j].center.x = arcs[j].center().x;
        arc3d[j].center.y = arcs[j].center().y;
        arc3d[j].center.z = z0;
        arc3d[j].length = arcs[j].length;

        Eigen::Vector4f start_homo(arc3d[j].start.x, arc3d[j].start.y, arc3d[j].start.z, 1.0f);
        Eigen::Vector4f middle_homo(arc3d[j].middle.x, arc3d[j].middle.y, arc3d[j].middle.z, 1.0f);
        Eigen::Vector4f end_homo(arc3d[j].end.x, arc3d[j].end.y, arc3d[j].end.z, 1.0f);
        Eigen::Vector4f center_homo(arc3d[j].center.x, arc3d[j].center.y, arc3d[j].center.z, 1.0f);

        Eigen::Vector4f start_back = transform_back * start_homo;
        Eigen::Vector4f middle_back = transform_back * middle_homo;
        Eigen::Vector4f end_back = transform_back * end_homo;
        Eigen::Vector4f center_back = transform_back * center_homo;

        arc3d[j].start = pcl::PointXYZ(start_back.x(), start_back.y(), start_back.z());
        arc3d[j].middle = pcl::PointXYZ(middle_back.x(), middle_back.y(), middle_back.z());
        arc3d[j].end = pcl::PointXYZ(end_back.x(), end_back.y(), end_back.z());
        arc3d[j].center = pcl::PointXYZ(center_back.x(), center_back.y(), center_back.z());
    }

    qDebug() << "10. 圆弧转换完成";

    for (size_t i = 0; i < arcs.size(); i++) {
        for (int idx : arcs[i].inlier_indices) {
            if (idx >= points_2d.size() || idx < 0) {
                qDebug() << "错误：索引越界！idx=" << idx << ", points_2d.size()=" << points_2d.size();
                // 这里会崩溃
            }
        }
    }

    // 5. 生成圆弧点云并输出
    if (arc_clouds_out) {
        arc_clouds_out->resize(arcs.size());
        for (size_t i = 0; i < arcs.size(); i++) {
            for (int idx : arcs[i].inlier_indices) {
                if (idx < points_2d.size()) {
                    pcl::PointXYZ c(points_2d[idx].x, points_2d[idx].y, z0);
                    (*arc_clouds_out)[i].push_back(c);
                }
            }
        }

        // 转换圆弧点云到3D
        for (size_t i = 0; i < arcs.size(); i++) {
            for (size_t j = 0; j < (*arc_clouds_out)[i].size(); j++) {
                Eigen::Vector4f start_homo((*arc_clouds_out)[i][j].x,
                                           (*arc_clouds_out)[i][j].y,
                                           (*arc_clouds_out)[i][j].z, 1.0f);
                Eigen::Vector4f start_back = transform_back * start_homo;
                (*arc_clouds_out)[i][j] = pcl::PointXYZ(start_back.x(), start_back.y(), start_back.z());
            }
        }
    }

    // 6. 直线分组
    std::vector<WeldSeam3D> seams3d_copy = seams3d;
    std::vector<WeldSeam2D> seams_copy = seams;
    std::vector<WeldSeam3D> v_seams3d_arc3d;

    // 移除靠近圆弧的直线
    for (int i = seams3d_copy.size() - 1; i >= 0; i--) {
        bool should_delete = false;
        for (size_t j = 0; j < arc3d.size(); j++) {
            if (seams3d_copy[i].pointDistance_start(arc3d[j].start) < arc_proximity_threshold ||
                seams3d_copy[i].pointDistance_start(arc3d[j].end) < arc_proximity_threshold ||
                seams3d_copy[i].pointDistance_end(arc3d[j].start) < arc_proximity_threshold ||
                seams3d_copy[i].pointDistance_end(arc3d[j].end) < arc_proximity_threshold) {
                should_delete = true;
                break;
            }
        }
        if (should_delete) {
            v_seams3d_arc3d.push_back(seams3d_copy[i]);
            seams3d_copy.erase(seams3d_copy.begin() + i);
            seams_copy.erase(seams_copy.begin() + i);
        }
    }

    // 分组剩余的直线
    std::vector<bool> visited(seams_copy.size(), false);
    for (size_t i = 0; i < seams_copy.size(); i++) {
        if (visited[i]) continue;

        std::vector<WeldSeam3D> group;
        group.push_back(seams3d_copy[i]);
        visited[i] = true;

        for (size_t j = i + 1; j < seams_copy.size(); j++) {
            if (visited[j]) continue;

            Point2D p_s(seams_copy[j].start.x, seams_copy[j].start.y);
            Point2D p_e(seams_copy[j].end.x, seams_copy[j].end.y);

            if (seams_copy[i].line.pointDistance(p_s) < group_threshold &&
                seams_copy[i].line.pointDistance(p_e) < group_threshold) {
                group.push_back(seams3d_copy[j]);
                visited[j] = true;
            }
        }
        result.grouped_seams.push_back(group);
    }
    if (do_clockwise_sort) {
        // 7. 顺时针排序直线
        for (auto& group : result.grouped_seams) {
            double sum_x = 0, sum_y = 0;
            for (const auto& weld : group) {
                sum_x += weld.start.x + weld.end.x;
                sum_y += weld.start.y + weld.end.y;
            }
            double x0 = sum_x / (2 * group.size());
            double y0 = sum_y / (2 * group.size());

            struct WeldWithAngle {
                WeldSeam3D weld;
                double angle;
            };
            std::vector<WeldWithAngle> welds_with_angle;

            for (auto& weld : group) {
                double mid_x = (weld.start.x + weld.end.x) / 2.0;
                double mid_y = (weld.start.y + weld.end.y) / 2.0;
                double angle = atan2(mid_y - y0, mid_x - x0);
                if (angle < 0) angle += 2 * M_PI;
                welds_with_angle.push_back({weld, angle});
            }

            sort(welds_with_angle.begin(), welds_with_angle.end(),
                 [](const WeldWithAngle& a, const WeldWithAngle& b) {
                     return a.angle > b.angle;
                 });

            for (auto& item : welds_with_angle) {
                WeldSeam3D& weld = item.weld;
                double start_angle = atan2(weld.start.y - y0, weld.start.x - x0);
                double end_angle = atan2(weld.end.y - y0, weld.end.x - x0);
                if (start_angle < 0) start_angle += 2 * M_PI;
                if (end_angle < 0) end_angle += 2 * M_PI;

                bool need_swap = false;
                if (fabs(start_angle - end_angle) > M_PI) {
                    if (start_angle < end_angle) start_angle += 2 * M_PI;
                    else end_angle += 2 * M_PI;
                    if (start_angle < end_angle) need_swap = true;
                } else {
                    if (start_angle < end_angle) need_swap = true;
                }

                if (need_swap) {
                    pcl::PointXYZ temp = weld.start;
                    weld.start = weld.end;
                    weld.end = temp;

                    if (weld.original_index >= 0 && weld.original_index < (int)seams.size()) {
                        std::swap(seams[weld.original_index].start, seams[weld.original_index].end);
                        std::swap(seams3d[weld.original_index].start, seams3d[weld.original_index].end);  // 同步 3D
                    }
                }
            }

            for (size_t i = 0; i < welds_with_angle.size(); i++) {
                group[i] = welds_with_angle[i].weld;
            }
        }

        // // 新增：把排序后交换过的起终点写回 2D seams
        //     for (auto& group : result.grouped_seams) {
        //     for (auto& weld : group) {
        //         if (weld.original_index >= 0 && weld.original_index < (int)seams.size()) {
        //             seams[weld.original_index].start.x = weld.start.x;
        //             seams[weld.original_index].start.y = weld.start.y;
        //             seams[weld.original_index].end.x   = weld.end.x;
        //             seams[weld.original_index].end.y   = weld.end.y;
        //         }
        //     }
        // }

    }

    // 收集所有参与判断的平焊缝（展平所有分组）
    std::vector<WeldSeam3D*> all_flat_seams;
    for (auto& group : result.grouped_seams) {
        for (auto& weld : group) {
            all_flat_seams.push_back(&weld);
        }
    }

    qDebug() << "7.5 开始平焊缝端点类型判断, 平焊缝总数=" << all_flat_seams.size();

    for (size_t i = 0; i < all_flat_seams.size(); i++) {
        WeldSeam3D* cur = all_flat_seams[i];

        // 统计当前焊缝起点
        int count_start = 0;
        for (size_t j = 0; j < all_flat_seams.size(); j++) {
            if (j == i) continue;  // 不含自己
            WeldSeam3D* other = all_flat_seams[j];

            // 当前起点 与 其他焊缝起点（2D距离）
            double ds_s = std::sqrt((cur->start.x - other->start.x) * (cur->start.x - other->start.x) +
                                    (cur->start.y - other->start.y) * (cur->start.y - other->start.y));
            if (ds_s < endpoint_proximity_threshold) count_start++;

            // 当前起点 与 其他焊缝终点（2D距离）
            double ds_e = std::sqrt((cur->start.x - other->end.x) * (cur->start.x - other->end.x) +
                                    (cur->start.y - other->end.y) * (cur->start.y - other->end.y));
            if (ds_e < endpoint_proximity_threshold) count_start++;
        }
        cur->start_stye = (count_start >= 2) ? 1 : 0;

        // 统计当前焊缝终点
        int count_end = 0;
        for (size_t j = 0; j < all_flat_seams.size(); j++) {
            if (j == i) continue;  // 不含自己
            WeldSeam3D* other = all_flat_seams[j];

            // 当前终点 与 其他焊缝起点（2D距离）
            double de_s = std::sqrt((cur->end.x - other->start.x) * (cur->end.x - other->start.x) +
                                    (cur->end.y - other->start.y) * (cur->end.y - other->start.y));
            if (de_s < endpoint_proximity_threshold) count_end++;

            // 当前终点 与 其他焊缝终点（2D距离）
            double de_e = std::sqrt((cur->end.x - other->end.x) * (cur->end.x - other->end.x) +
                                    (cur->end.y - other->end.y) * (cur->end.y - other->end.y));
            if (de_e < endpoint_proximity_threshold) count_end++;
        }
        cur->end_stye = (count_end >= 2) ? 1 : 0;
    }

    qDebug() << "7.5 平焊缝端点类型判断完成";


    // 8. 分组带圆弧的直线
    std::vector<bool> visited_arc(arc3d.size(), false);
    std::vector<bool> visited_seam(v_seams3d_arc3d.size(), false);

    for (size_t i = 0; i < v_seams3d_arc3d.size(); i++) {
        if (visited_seam[i]) continue;

        std::vector<WeldSeam3D> group_seams;
        std::vector<WeldArc3D> group_arcs;

        group_seams.push_back(v_seams3d_arc3d[i]);
        visited_seam[i] = true;

        pcl::PointXYZ current_end(v_seams3d_arc3d[i].end);

        bool is_seam_mode = false;
        bool found_new = true;

        while (found_new) {
            found_new = false;

            if (!is_seam_mode) {
                for (size_t j = 0; j < arc3d.size(); j++) {
                    if (visited_arc[j]) continue;

                    if (pointToPoint(arc3d[j].start, current_end) < connection_threshold) {
                        current_end = arc3d[j].end;
                        group_arcs.push_back(arc3d[j]);
                        visited_arc[j] = true;
                        found_new = true;
                        is_seam_mode = true;
                        break;
                    } else if (pointToPoint(arc3d[j].end, current_end) < connection_threshold) {
                        current_end = arc3d[j].start;
                        group_arcs.push_back(arc3d[j]);
                        visited_arc[j] = true;
                        found_new = true;
                        is_seam_mode = true;
                        break;
                    }
                }
            } else {
                for (size_t j = 0; j < v_seams3d_arc3d.size(); j++) {
                    if (visited_seam[j]) continue;

                    if (pointToPoint(v_seams3d_arc3d[j].start, current_end) < connection_threshold) {
                        current_end = v_seams3d_arc3d[j].end;
                        group_seams.push_back(v_seams3d_arc3d[j]);
                        visited_seam[j] = true;
                        found_new = true;
                        is_seam_mode = false;
                        break;
                    } else if (pointToPoint(v_seams3d_arc3d[j].end, current_end) < connection_threshold) {
                        current_end = v_seams3d_arc3d[j].start;
                        group_seams.push_back(v_seams3d_arc3d[j]);
                        visited_seam[j] = true;
                        found_new = true;
                        is_seam_mode = false;
                        break;
                    }
                }
            }
        }

        result.grouped_seams_with_arc.push_back(group_seams);
        result.grouped_arcs.push_back(group_arcs);
    }

    // ================================================================
    // 9. 查找立焊缝（仅在第一次调用时执行）
    // ================================================================
    if (do_clockwise_sort) {
        qDebug() << "9. 开始查找立焊缝, seams.size()=" << seams.size();

        const double VERTICAL_POS_THRESHOLD = 20.0;   // 位置合并阈值 (mm)
        const double VERTICAL_ANGLE_THRESHOLD = 10.0; // 角度合并阈值 (度)

        // 候选点结构：位置 + 角度
        struct CandidatePoint {
            double x, y, z;
            double angle;
        };

        std::vector<CandidatePoint> candidates;

        // 遍历所有平焊缝对 (i, j)，i < j
        for (size_t i = 0; i < seams.size(); i++) {
            for (size_t j = i + 1; j < seams.size(); j++) {
                double A1 = seams[i].line.a, B1 = seams[i].line.b;
                double A2 = seams[j].line.a, B2 = seams[j].line.b;

                double numerator = fabs(A1 * A2 + B1 * B2);
                double denominator = sqrt(A1*A1 + B1*B1) * sqrt(A2*A2 + B2*B2);
                if (denominator < 1e-10) continue;
                double cos_theta = numerator / denominator;
                if (cos_theta > 1.0) cos_theta = 1.0;
                if (cos_theta < -1.0) cos_theta = -1.0;

                // 只处理近似垂直的焊缝对 (夹角 45°~135°)
                if (acos(cos_theta) > 45 * PI / 180.0 && acos(cos_theta) < 135 * PI / 180.0) {
                    // 4 种端点组合
                    struct EndpointPair {
                        Point2D* ep_i;
                        Point2D* ep_j;
                    };
                    EndpointPair pairs[4] = {
                        {&seams[i].start, &seams[j].start},
                        {&seams[i].start, &seams[j].end},
                        {&seams[i].end,   &seams[j].start},
                        {&seams[i].end,   &seams[j].end}
                    };

                    for (int k = 0; k < 4; k++) {
                        double dist = pairs[k].ep_i->distance(*pairs[k].ep_j);
                        if (dist < connection_threshold) {
                            CandidatePoint cand;
                            // 交点位置：两个靠近端点的中点（2D）
                            cand.x = (pairs[k].ep_i->x + pairs[k].ep_j->x) / 2.0;
                            cand.y = (pairs[k].ep_i->y + pairs[k].ep_j->y) / 2.0;
                            cand.z = z0;

                            // 角度：用 3D seams3d 的 direction 算
                            // 焊缝 i 的角度
                            double dir_i_x = seams3d[i].end.x - seams3d[i].start.x;
                            double dir_i_y = seams3d[i].end.y - seams3d[i].start.y;
                            double dir_i_len = sqrt(dir_i_x*dir_i_x + dir_i_y*dir_i_y);
                            double rz1 = 0.0;
                            if (dir_i_len > 1e-10) {
                                rz1 = atan2(dir_i_y / dir_i_len, dir_i_x / dir_i_len) * 180.0 / PI;
                                if (rz1 < 0) rz1 += 360.0;
                            }

                            // 焊缝 j 的角度
                            double dir_j_x = seams3d[j].end.x - seams3d[j].start.x;
                            double dir_j_y = seams3d[j].end.y - seams3d[j].start.y;
                            double dir_j_len = sqrt(dir_j_x*dir_j_x + dir_j_y*dir_j_y);
                            double rz2 = 0.0;
                            if (dir_j_len > 1e-10) {
                                rz2 = atan2(dir_j_y / dir_j_len, dir_j_x / dir_j_len) * 180.0 / PI;
                                if (rz2 < 0) rz2 += 360.0;
                            }

                            // 方式 A：处理角度差后平均
                            double diff = rz1 - rz2;
                            if (diff > 180.0) rz2 += 360.0;
                            else if (diff < -180.0) rz1 += 360.0;
                            double rz = (rz1 + rz2) / 2.0;
                            if (rz < 0) rz += 360.0;
                            if (rz >= 360.0) rz -= 360.0;

                            cand.angle = rz;
                            candidates.push_back(cand);
                        }
                    }
                }
            }
        }

        qDebug() << "原始候选点数量:" << candidates.size();

        // ========== 方案 Z 去重 ==========
        // 1. 先按距离把候选点聚成大组（传递性，类似 DBSCAN 单链）
        std::vector<int> parent(candidates.size());
        for (size_t i = 0; i < parent.size(); i++) parent[i] = i;

        std::function<int(int)> find_root = [&](int x) -> int {
            while (parent[x] != x) {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        };
        auto union_set = [&](int a, int b) {
            int ra = find_root(a), rb = find_root(b);
            if (ra != rb) parent[ra] = rb;
        };

        for (size_t i = 0; i < candidates.size(); i++) {
            for (size_t j = i + 1; j < candidates.size(); j++) {
                double dx = candidates[i].x - candidates[j].x;
                double dy = candidates[i].y - candidates[j].y;
                double dz = candidates[i].z - candidates[j].z;
                double dist = sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < VERTICAL_POS_THRESHOLD) {
                    union_set(i, j);
                }
            }
        }

        // 2. 按大组收集候选点
        std::map<int, std::vector<int>> groups;
        for (size_t i = 0; i < candidates.size(); i++) {
            groups[find_root(i)].push_back(i);
        }

        qDebug() << "距离聚类后大组数量:" << groups.size();

        // 3. 每个大组内：位置取平均，再按角度分子组
        std::vector<Point2D> v_p;

        for (auto& kv : groups) {
            const std::vector<int>& group_indices = kv.second;

            // 大组位置平均
            double sum_x = 0, sum_y = 0, sum_z = 0;
            for (int idx : group_indices) {
                sum_x += candidates[idx].x;
                sum_y += candidates[idx].y;
                sum_z += candidates[idx].z;
            }
            double avg_x = sum_x / group_indices.size();
            double avg_y = sum_y / group_indices.size();
            double avg_z = sum_z / group_indices.size();

            // 按角度分子组（简单聚类：角度差 < 阈值归为一组）
            std::vector<bool> used(group_indices.size(), false);
            for (size_t a = 0; a < group_indices.size(); a++) {
                if (used[a]) continue;

                std::vector<double> sub_angles;
                sub_angles.push_back(candidates[group_indices[a]].angle);
                used[a] = true;

                for (size_t b = a + 1; b < group_indices.size(); b++) {
                    if (used[b]) continue;

                    double angle_diff = fabs(candidates[group_indices[a]].angle -
                                             candidates[group_indices[b]].angle);
                    if (angle_diff > 180.0) angle_diff = 360.0 - angle_diff;
                    if (angle_diff < VERTICAL_ANGLE_THRESHOLD) {
                        sub_angles.push_back(candidates[group_indices[b]].angle);
                        used[b] = true;
                    }
                }

                // 子组角度 sin/cos 平均
                double sum_cos = 0, sum_sin = 0;
                for (double ang : sub_angles) {
                    double rad = ang * PI / 180.0;
                    sum_cos += cos(rad);
                    sum_sin += sin(rad);
                }
                double avg_rad = atan2(sum_sin, sum_cos);
                double avg_angle = avg_rad * 180.0 / PI;
                if (avg_angle < 0) avg_angle += 360.0;

                // 子组位置用大组平均位置
                Point2D p;
                p.x = avg_x;
                p.y = avg_y;
                p.angle = avg_angle;
                v_p.push_back(p);
            }
        }

        qDebug() << "去重后立焊缝数量:" << v_p.size();

        // 4. 转换立焊缝到 3D（做逆旋转）
        result.vertical_seams.resize(v_p.size());
        for (size_t i = 0; i < v_p.size(); i++) {
            result.vertical_seams[i].start.x = v_p[i].x;
            result.vertical_seams[i].start.y = v_p[i].y;
            result.vertical_seams[i].start.z = z0;
            result.vertical_seams[i].end.x = v_p[i].x;
            result.vertical_seams[i].end.y = v_p[i].y;
            result.vertical_seams[i].end.z = z0 - vertical_length;
            result.vertical_seams[i].angle = v_p[i].angle;

            Eigen::Vector4f start_homo(result.vertical_seams[i].start.x,
                                       result.vertical_seams[i].start.y,
                                       result.vertical_seams[i].start.z, 1.0f);
            Eigen::Vector4f end_homo(result.vertical_seams[i].end.x,
                                     result.vertical_seams[i].end.y,
                                     result.vertical_seams[i].end.z, 1.0f);

            Eigen::Vector4f start_back = transform_back * start_homo;
            Eigen::Vector4f end_back = transform_back * end_homo;

            result.vertical_seams[i].start.x = start_back.x();
            result.vertical_seams[i].start.y = start_back.y();
            result.vertical_seams[i].start.z = start_back.z();
            result.vertical_seams[i].end.x = end_back.x();
            result.vertical_seams[i].end.y = end_back.y();
            result.vertical_seams[i].end.z = end_back.z();
        }

        qDebug() << "立焊缝转换完成, 数量=" << result.vertical_seams.size();
    }

    return result;
}



std::string WeldSeamExtractor::weldTypeToString(WeldType type) {
    switch (type) {
    case WeldType::FILLET: return "FILLET";
    case WeldType::BUTT: return "BUTT";
    case WeldType::GROOVE: return "GROOVE";
    default: return "FILLET";
    }
}

std::string WeldSeamExtractor::weldPositionToString(WeldPosition position) {
    switch (position) {
    case WeldPosition::FLAT: return "FLAT";
    case WeldPosition::VERTICAL: return "VERTICAL";
    case WeldPosition::OVERHEAD: return "OVERHEAD";
    default: return "FLAT";
    }
}

bool WeldSeamExtractor::generateJSON(
    const std::vector<std::vector<WeldSeam3D>>& grouped_seams,
    const std::vector<std::vector<WeldSeam3D>>& grouped_seams_with_arc,
    const std::vector<std::vector<WeldArc3D>>& grouped_arcs,
    const std::vector<WeldLSeam3D>& vertical_seams,
    const std::string& filename,
    WeldType default_type,
    WeldPosition default_position,
    double default_thickness) {

    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    file << "[\n";

    int id = 0;
    int group_id = 0;

    // 输出分组直线焊缝
    for (const auto& group : grouped_seams) {
        group_id++;
        for (const auto& seam : group) {
            id++;
            Vec3 start(seam.start.x, seam.start.y, seam.start.z);
            Vec3 end(seam.end.x, seam.end.y, seam.end.z);
            Vec3 direction(seam.end.x - seam.start.x, seam.end.y - seam.start.y, seam.end.z - seam.start.z);

            // 归一化方向向量
            double len = std::sqrt(direction.x*direction.x + direction.y*direction.y + direction.z*direction.z);
            if (len > 1e-6) {
                direction.x /= len;
                direction.y /= len;
                direction.z /= len;
            }

            double azimuth = std::atan2(direction.y, direction.x) * 180 / M_PI;
            double elevation = std::asin(direction.z) * 180 / M_PI;

            file << "    {\n";
            file << "      \"id\": " << id << ",\n";
            file << "      \"group_id\": " << group_id << ",\n";
            file << "      \"start_point\": [" << start.x << ", " << start.y << ", " << start.z << "],\n";
            file << "      \"end_point\": [" << end.x << ", " << end.y << ", " << end.z << "],\n";
            file << "      \"direction\": [" << direction.x << ", " << direction.y << ", " << direction.z << "],\n";
            file << "      \"weld_type\": \"" << weldTypeToString(default_type) << "\",\n";
            file << "      \"weld_position\": \"" << weldPositionToString(default_position) << "\",\n";
            file << "      \"plate_thickness\": " << default_thickness << ",\n";
            file << "      \"is_arc\": false,\n";
            file << "      \"center_point\": [0, 0, 0],\n";
            file << "      \"radius\": 0,\n";
            file << "      \"start_angle\": " <<seam.start_stye<<",\n";
            file << "      \"end_angle\": " <<seam.end_stye<<",\n";
            file << "      \"azimuth\": " << azimuth << ",\n";
            file << "      \"elevation\": " << elevation << ",\n";
            file << "      \"angle_deg\": 0\n";
            file << "      \"start_stye\": " <<seam.start_stye<<",\n";
            file << "      \"end_stye\": " <<seam.end_stye<<",\n";
            file << "    }";
            if (!(id == getTotalCount(grouped_seams, grouped_seams_with_arc, grouped_arcs, vertical_seams))) {
                file << ",";
            }
            file << "\n";
        }
    }

    // // 输出带圆弧的直线焊缝
    // for (const auto& group : grouped_seams_with_arc) {
    //     group_id++;
    //     for (const auto& seam : group) {
    //         id++;
    //         Vec3 start(seam.start.x, seam.start.y, seam.start.z);
    //         Vec3 end(seam.end.x, seam.end.y, seam.end.z);
    //         Vec3 direction(seam.end.x - seam.start.x, seam.end.y - seam.start.y, seam.end.z - seam.start.z);

    //         double len = std::sqrt(direction.x*direction.x + direction.y*direction.y + direction.z*direction.z);
    //         if (len > 1e-6) {
    //             direction.x /= len;
    //             direction.y /= len;
    //             direction.z /= len;
    //         }

    //         double azimuth = std::atan2(direction.y, direction.x) * 180 / M_PI;
    //         double elevation = std::asin(direction.z) * 180 / M_PI;

    //         file << "    {\n";
    //         file << "      \"id\": " << id << ",\n";
    //         file << "      \"group_id\": " << group_id << ",\n";
    //         file << "      \"start_point\": [" << start.x << ", " << start.y << ", " << start.z << "],\n";
    //         file << "      \"end_point\": [" << end.x << ", " << end.y << ", " << end.z << "],\n";
    //         file << "      \"direction\": [" << direction.x << ", " << direction.y << ", " << direction.z << "],\n";
    //         file << "      \"weld_type\": \"" << weldTypeToString(default_type) << "\",\n";
    //         file << "      \"weld_position\": \"" << weldPositionToString(default_position) << "\",\n";
    //         file << "      \"plate_thickness\": " << default_thickness << ",\n";
    //         file << "      \"is_arc\": false,\n";
    //         file << "      \"center_point\": [0, 0, 0],\n";
    //         file << "      \"radius\": 0,\n";
    //         file << "      \"start_angle\": 0,\n";
    //         file << "      \"end_angle\": 0,\n";
    //         file << "      \"azimuth\": " << azimuth << ",\n";
    //         file << "      \"elevation\": " << elevation << ",\n";
    //         file << "      \"angle_deg\": 0\n";
    //         file << "    }";
    //         if (!(id == getTotalCount(grouped_seams, grouped_seams_with_arc, grouped_arcs, vertical_seams))) {
    //             file << ",";
    //         }
    //         file << "\n";
    //     }
    // }

    // // 输出圆弧焊缝
    // for (const auto& group : grouped_arcs) {
    //     group_id++;
    //     for (const auto& arc : group) {
    //         id++;
    //         Vec3 start(arc.start.x, arc.start.y, arc.start.z);
    //         Vec3 end(arc.end.x, arc.end.y, arc.end.z);
    //         Vec3 center(arc.center.x, arc.center.y, arc.center.z);
    //         Vec3 direction(arc.end.x - arc.start.x, arc.end.y - arc.start.y, arc.end.z - arc.start.z);

    //         double len = std::sqrt(direction.x*direction.x + direction.y*direction.y + direction.z*direction.z);
    //         if (len > 1e-6) {
    //             direction.x /= len;
    //             direction.y /= len;
    //             direction.z /= len;
    //         }

    //         double azimuth = std::atan2(direction.y, direction.x) * 180 / M_PI;
    //         double elevation = std::asin(direction.z) * 180 / M_PI;

    //         file << "    {\n";
    //         file << "      \"id\": " << id << ",\n";
    //         file << "      \"group_id\": " << group_id << ",\n";
    //         file << "      \"start_point\": [" << start.x << ", " << start.y << ", " << start.z << "],\n";
    //         file << "      \"end_point\": [" << end.x << ", " << end.y << ", " << end.z << "],\n";
    //         file << "      \"direction\": [" << direction.x << ", " << direction.y << ", " << direction.z << "],\n";
    //         file << "      \"weld_type\": \"" << weldTypeToString(default_type) << "\",\n";
    //         file << "      \"weld_position\": \"" << weldPositionToString(default_position) << "\",\n";
    //         file << "      \"plate_thickness\": " << default_thickness << ",\n";
    //         file << "      \"is_arc\": true,\n";
    //         file << "      \"center_point\": [" << center.x << ", " << center.y << ", " << center.z << "],\n";
    //         file << "      \"radius\": " << arc.length / (2 * M_PI) << ",\n";
    //         file << "      \"start_angle\": 0,\n";
    //         file << "      \"end_angle\": 0,\n";
    //         file << "      \"azimuth\": " << azimuth << ",\n";
    //         file << "      \"elevation\": " << elevation << ",\n";
    //         file << "      \"angle_deg\": 0\n";
    //         file << "    }";
    //         if (!(id == getTotalCount(grouped_seams, grouped_seams_with_arc, grouped_arcs, vertical_seams))) {
    //             file << ",";
    //         }
    //         file << "\n";
    //     }
    // }

    // 输出立焊缝
    for (const auto& seam : vertical_seams) {
        id++;
        Vec3 start(seam.start.x, seam.start.y, seam.start.z);
        Vec3 end(seam.end.x, seam.end.y, seam.end.z);
        Vec3 direction(seam.end.x - seam.start.x, seam.end.y - seam.start.y, seam.end.z - seam.start.z);

        double len = std::sqrt(direction.x*direction.x + direction.y*direction.y + direction.z*direction.z);
        if (len > 1e-6) {
            direction.x /= len;
            direction.y /= len;
            direction.z /= len;
        }

        double azimuth = std::atan2(direction.y, direction.x) * 180 / M_PI;
        double elevation = std::asin(direction.z) * 180 / M_PI;

        file << "    {\n";
        file << "      \"id\": " << id << ",\n";
        file << "      \"group_id\": " << (grouped_seams.size() + grouped_seams_with_arc.size() + grouped_arcs.size() + 1) << ",\n";
        file << "      \"start_point\": [" << start.x << ", " << start.y << ", " << start.z << "],\n";
        file << "      \"end_point\": [" << end.x << ", " << end.y << ", " << end.z << "],\n";
        file << "      \"direction\": [" << direction.x << ", " << direction.y << ", " << direction.z << "],\n";
        file << "      \"weld_type\": \"" << weldTypeToString(default_type) << "\",\n";
        file << "      \"weld_position\": \"" << weldPositionToString(WeldPosition::VERTICAL) << "\",\n";
        file << "      \"plate_thickness\": " << default_thickness << ",\n";
        file << "      \"is_arc\": false,\n";
        file << "      \"center_point\": [0, 0, 0],\n";
        file << "      \"radius\": 0,\n";
        file << "      \"start_angle\": 0,\n";
        file << "      \"end_angle\": 0,\n";
        file << "      \"azimuth\": " << seam.angle << ",\n";
        file << "      \"elevation\": " << seam.angle << ",\n";
        file << "      \"angle_deg\": " << seam.angle << "\n";
        file << "    }";
        if (id != getTotalCount(grouped_seams, grouped_seams_with_arc, grouped_arcs, vertical_seams)) {
            file << ",";
        }
        file << "\n";
    }

    file << "  ]\n";

    file.close();
    return true;
}




void Weld_search::filterPointCloud(
    const std::string& input_path,
    const std::string& output_path,
    const std::string& axis,
    double min_limit,
    double max_limit,
    bool negative)
{
    qDebug() << "=== filterPointCloud 开始 ===";

    try {
        // 清空全局点云
        g_cloud.clear();
        g_cloud_out.clear();

        if (pcl::io::loadPCDFile(input_path, g_cloud) < 0) {
            qDebug() << "加载点云失败";
            return;
        }

        qDebug() << "加载点数:" << g_cloud.size();

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(g_cloud.makeShared());
        pass.setFilterFieldName(axis);
        pass.setFilterLimits(min_limit, max_limit);
        pass.setFilterLimitsNegative(negative);
        pass.filter(g_cloud_out);

        qDebug() << "过滤完成，剩余点数:" << g_cloud_out.size();

        if (g_cloud_out.empty()) {
            qDebug() << "过滤后点云为空";
            return;
        }

        if (pcl::io::savePCDFile(output_path, g_cloud_out) < 0) {
            qDebug() << "保存点云失败";
            return;
        }

        qDebug() << "=== filterPointCloud 完成 ===";
        return;

    } catch (const std::exception& e) {
        qDebug() << "异常:" << e.what();
        return;
    }
}

bool Weld_search::filterGroundFace(const std::string& input_path,
                      const std::string& output_path,
                      Eigen::Vector3f& axis,
                      float& angle,
                      float& z0)
{
    if (pcl::io::loadPCDFile(input_path, g_cloud) == -1)
    {
        qDebug()<<"点云加载失败!";
        return false;
    }

    if (g_cloud.empty())
    {
        qDebug()<<"点云为空!";
        return false;
    }
    //对点云进行旋转，使其与Z轴重合
    // 2. 创建分割对象并设置参数
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);          // 开启系数优化
    seg.setModelType(pcl::SACMODEL_PLANE);      // 平面模型
    seg.setMethodType(pcl::SAC_RANSAC);         // RANSAC算法
    seg.setDistanceThreshold(2);             // 距离阈值（根据点云精度调整）
    seg.setMaxIterations(1000);                 // 最大迭代次数
    seg.setInputCloud(g_cloud.makeShared());

    // 3. 执行分割
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        qDebug()<<"未能拟合出平面模型";
        return false;
    }
    // 5. 提取法向量
    Eigen::Vector3f normal(
        coefficients->values[0],
        coefficients->values[1],
        coefficients->values[2]
        );

    normal.normalize();  // 确保单位向量
    Eigen::Vector3f target(0.0f, 0.0f, 1.0f);  // 目标Z轴方向
    axis = normal.cross(target);
    angle = acos(normal.dot(target));
    // 处理特殊情况：法向量已经与Z轴对齐
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    if (axis.norm() < 1e-6) {
        // 法向量已经平行于Z轴
        if (normal.dot(target) > 0) {
            qDebug()<<"底板已经是水平状态，无需旋转";
        } else {
            // 法向量指向Z轴负方向，需要旋转180度
            qDebug()<<"法向量指向下方，旋转180度";
            Eigen::AngleAxisf rotation(M_PI, Eigen::Vector3f::UnitX());
            transform.block<3,3>(0,0) = rotation.matrix();
        }
    } else {
        // 正常情况：构造旋转矩阵
        axis.normalize();
        Eigen::AngleAxisf rotation(angle, axis);
        transform.block<3,3>(0,0) = rotation.matrix();
    }

    // ========== 3. 应用变换到点云 ==========
    pcl::transformPointCloud(g_cloud, g_cloud, transform);

    // ========== 4. 验证变换后的法向量 ==========
    // 重新拟合变换后的点云平面，验证法向量
    pcl::ModelCoefficients::Ptr new_coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr new_inliers(new pcl::PointIndices);
    seg.setInputCloud(g_cloud.makeShared());
    seg.segment(*new_inliers, *new_coeff);

    Eigen::Vector3f new_normal(
        new_coeff->values[0],
        new_coeff->values[1],
        new_coeff->values[2]
        );
    new_normal.normalize();

    int idx = new_inliers->indices[0];
    float plane_height = g_cloud.points[idx].z-10;
    z0=g_cloud.points[idx].z;

    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud (g_cloud.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, plane_height);
    pass.setFilterLimitsNegative (false);
    pass.filter (g_cloud_out);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    sor.setInputCloud(g_cloud_out.makeShared());
    sor.setMeanK(100);               // 100--50设置用于计算平均距离的邻域点数量(k值)，常见取值为20-100
    sor.setStddevMulThresh(1.0);    // 1--1.5设置标准差倍数阈值(multiplier)，标准差倍数通常取1.0或2.0
    sor.filter(g_cloud_out);    // 执行滤波，结果保存在cloud_filtered中


    for (auto& point : g_cloud_out.points) {
        point.z = g_cloud_out.points[idx].z;
    }

    // 计算法线 - 使用K近邻更稳定
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::Normal> normals;

    normal_est.setInputCloud(g_cloud_out.makeShared());
    normal_est.setSearchMethod(tree);
    normal_est.setKSearch(20);  // 使用K近邻而非半径20--10
    normal_est.compute(normals);

    if (normals.empty()) {
        qDebug()<<"法线计算失败，请检查点云数据!";
        return false;
    }

    // 边界估计
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_est;

    boundary_est.setInputCloud(g_cloud_out.makeShared());
    boundary_est.setInputNormals(normals.makeShared());
    boundary_est.setSearchMethod(tree);
    boundary_est.setKSearch(30);//30--15
    boundary_est.setAngleThreshold(M_PI / 2.0);//2--3
    boundary_est.compute(boundaries);

    g_cloud = g_cloud_out;

    g_cloud_out.clear();  // 清空旧数据
    // 提取边界点
    int boundary_count = 0;
    for (size_t i = 0; i < boundaries.size(); ++i) {
        if (boundaries[i].boundary_point > 0) {
            g_cloud_out.push_back(g_cloud.points[i]);
            boundary_count++;
        }
    }

    if (g_cloud_out.empty()) {
        qDebug()<<"未检测到边界点，请调整参数!";
        return false;
    }


    pcl::io::savePCDFile(output_path,g_cloud_out);
    qDebug()<<"焊缝边界已保存到cloud_fitle.pcd文件中";

    return true;
}



// bool Weld_search::filterGroundFace(const std::string& input_path,
//                                    const std::string& output_path,
//                                    Eigen::Vector3f& axis,
//                                    float& angle,
//                                    float& z0)
// {
//     // ========== 1. 加载点云 ==========
//     if (pcl::io::loadPCDFile(input_path, g_cloud) == -1)
//     {
//         qDebug() << "点云加载失败!";
//         return false;
//     }
//     if (g_cloud.empty())
//     {
//         qDebug() << "点云为空!";
//         return false;
//     }

//     // ========== 2. RANSAC 拟合底板平面 ==========
//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//     pcl::SACSegmentation<pcl::PointXYZ> seg;

//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setDistanceThreshold(1);   // 0.5mm 点云，阈值取 1 倍点间距
//     seg.setMaxIterations(1000);
//     seg.setInputCloud(g_cloud.makeShared());
//     seg.segment(*inliers, *coefficients);

//     if (inliers->indices.empty()) {
//         qDebug() << "未能拟合出平面模型";
//         return false;
//     }

//     // ========== 3. 计算旋转，使底板法线对齐 Z 轴 ==========
//     Eigen::Vector3f normal(
//         coefficients->values[0],
//         coefficients->values[1],
//         coefficients->values[2]);
//     normal.normalize();

//     Eigen::Vector3f target(0.0f, 0.0f, 1.0f);
//     axis = normal.cross(target);
//     angle = acos(normal.dot(target));

//     Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

//     if (axis.norm() < 1e-6) {
//         if (normal.dot(target) > 0) {
//             qDebug() << "底板已经是水平状态，无需旋转";
//         } else {
//             qDebug() << "法向量指向下方，旋转180度";
//             Eigen::AngleAxisf rotation(M_PI, Eigen::Vector3f::UnitX());
//             transform.block<3, 3>(0, 0) = rotation.matrix();
//         }
//     } else {
//         axis.normalize();
//         Eigen::AngleAxisf rotation(angle, axis);
//         transform.block<3, 3>(0, 0) = rotation.matrix();
//     }

//     // ========== 4. 应用变换 ==========
//     pcl::transformPointCloud(g_cloud, g_cloud, transform);

//     // ========== 5. 重新拟合，获取变换后平面高度 ==========
//     pcl::ModelCoefficients::Ptr new_coeff(new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr new_inliers(new pcl::PointIndices);
//     seg.setInputCloud(g_cloud.makeShared());
//     seg.segment(*new_inliers, *new_coeff);

//     if (new_inliers->indices.empty()) {
//         qDebug() << "变换后平面拟合失败";
//         return false;
//     }

//     // 用所有内点的 z 均值作为平面高度，比取单点更稳
//     double z_sum = 0.0;
//     for (int i : new_inliers->indices)
//         z_sum += g_cloud.points[i].z;
//     z0 = static_cast<float>(z_sum / new_inliers->indices.size());

//     // 保留底板上方一定高度的点（原 -10 太大，0.5mm 点云改小）
//     // 按焊缝实际高度调整，这里保守取 2.0mm
//     float plane_height = z0 - 10.0f;


//     // ========== 6. PassThrough 提取底板以上区域 ==========
//     pcl::PassThrough<pcl::PointXYZ> pass;
//     pass.setInputCloud(g_cloud.makeShared());
//     pass.setFilterFieldName("z");
//     pass.setFilterLimits(0,plane_height); // 取平面之上
//     pass.setFilterLimitsNegative(false);
//     pass.filter(g_cloud_out);

//     if (g_cloud_out.empty()) {
//         qDebug() << "PassThrough 后点云为空，检查 plane_height";
//         return false;
//     }

//     // ========== 7. 去噪（关键修改：不用 SOR，改用半径滤波，避免删角点）==========
//     // 原 SOR MeanK=100 / Stddev=1.0 在 0.5mm 点云上会把角点当离群点删掉
//     pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
//     ror.setInputCloud(g_cloud_out.makeShared());
//     ror.setRadiusSearch(2);        // 约 1.6 倍点间距
//     ror.setMinNeighborsInRadius(3);  // 至少 3 个邻居
//     ror.filter(g_cloud_out);

//     if (g_cloud_out.empty()) {
//         qDebug() << "去噪后点云为空";
//         return false;
//     }

//     // ========== 8. 法线估计（关键修改：K 从 20 降到 8）==========
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::Normal> normals;

//     normal_est.setInputCloud(g_cloud_out.makeShared());
//     normal_est.setSearchMethod(tree);
//     normal_est.setKSearch(8);   // 邻居半径约 0.8mm，角点处不跨边
//     normal_est.compute(normals);

//     if (normals.empty()) {
//         qDebug() << "法线计算失败，请检查点云数据!";
//         return false;
//     }

//     // ========== 9. 边界估计（关键修改：K=10, 角度=120°）==========
//     pcl::PointCloud<pcl::Boundary> boundaries;
//     pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_est;

//     boundary_est.setInputCloud(g_cloud_out.makeShared());
//     boundary_est.setInputNormals(normals.makeShared());
//     boundary_est.setSearchMethod(tree);
//     boundary_est.setKSearch(20);                       // 邻居半径约 0.9mm
//     boundary_est.setAngleThreshold(2.0 * M_PI / 3.0);  // 120°，只保留真边界
//     boundary_est.compute(boundaries);

//     if (boundaries.empty()) {
//         qDebug() << "边界估计失败";
//         return false;
//     }

//     // ========== 10. 提取边界点 ==========
//     // 保留去噪后的完整点云，供后续可能使用
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_denoised(new pcl::PointCloud<pcl::PointXYZ>(g_cloud_out));

//     g_cloud_out.clear();
//     int boundary_count = 0;
//     for (size_t i = 0; i < boundaries.size(); ++i) {
//         if (boundaries[i].boundary_point > 0) {
//             g_cloud_out.push_back(cloud_denoised->points[i]);
//             boundary_count++;
//         }
//     }

//     qDebug() << "边界点数:" << boundary_count
//              << "/" << cloud_denoised->size()
//              << " 占比:" << (100.0 * boundary_count / cloud_denoised->size()) << "%";

//     if (g_cloud_out.empty()) {
//         qDebug() << "未检测到边界点，请调整 angleThreshold 或 KSearch!";
//         return false;
//     }

//     // ========== 11. 保存 ==========
//     pcl::io::savePCDFile(output_path, g_cloud_out);
//     qDebug() << "焊缝边界已保存到:" << QString::fromStdString(output_path);

//     return true;
// }


void Weld_search::process_welding_system(
    const std::string& input_file,
    const std::string& output_dir,
    const std::string& param_file,
    bool show_demo)
{

    // 2. 加载参数配置
    if (!param_file.empty()) {
        load_parameters_from_file(param_file);
    }

    // 3. 创建系统
    WeldingSystem system(g_default_plate_thickness, g_group_distance_threshold);

    // 4. 加载数据
    std::vector<WeldSeamInfo > input_seams;
    if (!input_file.empty()) {
        // std::cout << "[1/6] 从文件加载焊缝数据: " << input_file << "...\n";
        try {
            input_seams = system.load_from_json(input_file);
        } catch (const std::exception& e) {
            std::cerr << "错误: " << e.what() << std::endl;
            std::cout << "使用示例数据...\n";
            input_seams = create_sample_data();
        }
    } else {
        input_seams = create_sample_data();
    }

    // 5. 分组
    // std::cout << "[2/6] 分组结果:\n";
    auto groups = system.planner.group_seams(input_seams);
    for (const auto& g : groups) {
        std::string type_str = g.weld_type == WeldType::FILLET ? "FILLET" :
                                   g.weld_type == WeldType::BUTT ? "BUTT" : "GROOVE";
        std::string pos_str = g.weld_position == WeldPosition::FLAT ? "FLAT" :
                                  g.weld_position == WeldPosition::VERTICAL ? "VERTICAL" : "OVERHEAD";
        // std::cout << "  组" << g.group_id << " (" << type_str << "/" << pos_str << "): "
        //           << g.seam_ids.size() << " 条焊缝 [";
        // for (size_t i = 0; i < g.seam_ids.size(); ++i) {
        //     std::cout << g.seam_ids[i];
        //     if (i < g.seam_ids.size() - 1) std::cout << ", ";
        // }
        // std::cout << "]\n";
    }
    // std::cout << "\n";

    // 6. 平行关系
    // std::cout << "[3/6] 组间平行关系:\n";
    auto parallel_map = system.planner.build_parallel_groups(groups, input_seams);
    // for (const auto& [gid, pgs] : parallel_map) {
    //     std::cout << "  组" << gid << " ← 平行 → 组[";
    //     for (size_t i = 0; i < pgs.size(); ++i) {
    //         std::cout << pgs[i];
    //         if (i < pgs.size() - 1) std::cout << ", ";
    //     }
    //     std::cout << "]\n";
    // }
    // std::cout << "\n";

    // 7. 规划+优化
    // std::cout << "[4/6] 路径规划与优化（遗传算法 + A*）...\n";
    auto result = system.optimize_with_hybrid(input_seams);
    // std::cout << "  优化完成\n\n";

    // 8. 输出报告
    // std::cout << "[5/6] 规划报告:\n";
    // std::cout << "------------------------------------------------------------\n";
    double total_length = 0;
    for (const auto& s : result) {
        total_length += s.length();
    }

    // 空行程计算
    double travel_length = 0;
    for (size_t i = 1; i < result.size(); ++i) {
        travel_length += dist_3d(result[i-1].end_point, result[i].start_point);
    }

    // std::cout << "焊缝总数: " << result.size() << "\n";
    // std::cout << "焊接总长度: " << total_length << " mm\n";
    // std::cout << "空行程距离: " << travel_length << " mm\n\n";

    // std::cout << "焊接顺序:\n";
    for (size_t i = 0; i < result.size(); ++i) {
        const auto& s = result[i];
        std::string type_str = s.weld_type == WeldType::FILLET ? "FILLET" :
                                   s.weld_type == WeldType::BUTT ? "BUTT" : "GROOVE";
        std::string pos_str = s.weld_position == WeldPosition::FLAT ? "FLAT" :
                                  s.weld_position == WeldPosition::VERTICAL ? "VERTICAL" : "OVERHEAD";
        // std::cout << "    " << (i+1) << ". [G" << s.group_id << "] 焊缝#" << s.id << "    "
        //           << type_str << "  " << pos_str << "\n";
        // std::cout << "        起点: " << s.start_point.to_string() << "\n";
        // std::cout << "        终点: " << s.end_point.to_string() << "\n";
    }
    // std::cout << "\n";

    // 9. 导出JSON
    // std::cout << "[6/6] 导出结果到目录: " << output_dir << "\n";
    system.export_grouped_seams(result, output_dir);
    system.export_welding_path(result, output_dir);

    // 10. 中断恢复演示（可选）
    if (show_demo) {
        // std::cout << "\n------------------------------------------------------------\n";
        // std::cout << "中断恢复演示:\n";
        system.state_manager.initialize_task("demo_task", result);
        // std::cout << "  任务初始化完成\n";

        system.state_manager.pause_welding("demo_task", InterruptType::GY);
        // std::cout << "  模拟工艺中断 (GY)\n";

        system.state_manager.resume_welding("demo_task");
        // std::cout << "  恢复焊接\n";

        auto replanned = system.state_manager.replan_from_current("demo_task", result, system.planner);
        // std::cout << "  实时重规划完成，剩余焊缝: " << replanned.size() << " 条\n";
    }

}
