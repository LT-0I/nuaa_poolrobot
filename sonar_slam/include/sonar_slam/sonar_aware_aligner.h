#ifndef SONAR_AWARE_ALIGNER_H
#define SONAR_AWARE_ALIGNER_H
 

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <memory>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <boost/filesystem.hpp>

namespace sonar_slam {

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;
 

// ======================== 强度分级结构 ========================

/**
 * @brief 强度分级统计
 */
struct IntensityStats {
    std::vector<PointType> strong_points;   // ≥160
    std::vector<PointType> medium_points;   // 140-159
    std::vector<PointType> weak_points;     // 100-139
    float max_intensity;
    float min_intensity;
    float avg_intensity;
    float sensor_x, sensor_y;   
    
    IntensityStats() : max_intensity(0), min_intensity(255), avg_intensity(0),
                       sensor_x(0), sensor_y(0) {}   
};

// ======================== 几何形状结构 ========================

/**
 * @brief 线段特征（基于坐标聚类）
 */
struct LineSegment {
    float x1, y1, x2, y2;     // 端点坐标
    float angle;               // 角度（弧度）
    float length;              // 长度
    std::string orientation;   // "horizontal", "vertical", "diagonal"
    float confidence;          // 置信度（基于点密度和一致性）
    std::vector<int> point_indices;  // 构成线段的点索引
    float a, b, c;  // 直线参数 ax + by + c = 0
 
    LineSegment() : x1(0), y1(0), x2(0), y2(0), angle(0), 
                    length(0), confidence(0), a(0), b(0), c(0) {}
};

/**
 * @brief 角点特征（线段交叉点）
 */
struct CornerPoint {
    float x, y;                 // 坐标
    float angle;                // 夹角（弧度）
    std::string type;           // "right", "acute", "obtuse"
    int line1_idx;              // 第一条线段索引
    int line2_idx;              // 第二条线段索引
    float confidence;           // 置信度
    
    CornerPoint() : x(0), y(0), angle(0), line1_idx(-1), 
                    line2_idx(-1), confidence(0) {}
};

/**
 * @brief 封闭轮廓
 */
struct ClosedContour {
    std::vector<LineSegment> lines;      // 组成轮廓的线段
    std::vector<CornerPoint> corners;    // 角点
    float area;                           // 面积
    float perimeter;                       // 周长
    float centroid_x, centroid_y;         // 质心
    float min_x, max_x, min_y, max_y;      // 边界框
    float width, height;                   // 宽度和高度   
    bool is_closed;                        // 是否闭合
    float closure_error;                    // 闭合误差
    
    ClosedContour() : area(0), perimeter(0), centroid_x(0), centroid_y(0),
                      min_x(FLT_MAX), max_x(-FLT_MAX), min_y(FLT_MAX), max_y(-FLT_MAX),
                      width(0), height(0),  // 初始化
                      is_closed(false), closure_error(0) {}
};

/**
 * @brief 形状特征集
 */
struct ShapeFeatures {
    std::vector<LineSegment> lines;        // 所有线段
    std::vector<CornerPoint> corners;      // 所有角点
    ClosedContour main_contour;             // 主轮廓
    float dominant_angle;                    // 主方向
    float width, height;                     // 宽度和高度
    std::string shape_type;                  // "rectangle", "L_shape", "circle", etc.
    
    // 传感器信息（用于参考）
    float sensor_x, sensor_y;
    std::string scan_hint;
    
    ShapeFeatures() : dominant_angle(0), width(0), height(0),
                      sensor_x(0), sensor_y(0), scan_hint("unknown") {}
    
    bool hasEnoughFeatures() const {
        return lines.size() >= 2 || corners.size() >= 2;
    }
};


/**
 * @brief 几何特征集（用于轮廓对齐）
 */
struct GeometryFeatureSet {
    std::vector<LineSegment> lines;        // 线段
    std::vector<CornerPoint> corners;      // 角点
    ClosedContour main_contour;             // 主轮廓
    float min_x, max_x, min_y, max_y;       // 边界框
    float width, height;                     // 宽度和高度
    
    // 传感器信息
    float sensor_x, sensor_y;
    std::string scan_hint;
    
    GeometryFeatureSet() : min_x(FLT_MAX), max_x(-FLT_MAX),
                           min_y(FLT_MAX), max_y(-FLT_MAX),
                           width(0), height(0),
                           sensor_x(0), sensor_y(0),
                           scan_hint("unknown") {}
};

// ======================== 变换参数 ========================
/**
 * @brief 变换参数结构
 * 表示两个点云之间的相对变换
 */
struct SonarTransformParams {
    float dx, dy;           // 平移（米）
    float dtheta;           // 旋转（弧度）
    float confidence;       // 置信度 [0,1]
    
    SonarTransformParams() : dx(0), dy(0), dtheta(0), confidence(0) {}
    
    // 应用变换到点
    void apply(float x, float y, float& tx, float& ty) const {
        float cos_theta = cos(dtheta);
        float sin_theta = sin(dtheta);
        tx = x * cos_theta - y * sin_theta + dx;
        ty = x * sin_theta + y * cos_theta + dy;
    }
    
    // 获取变换矩阵
    Eigen::Matrix4f getMatrix() const {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T(0,0) = cos(dtheta); T(0,1) = -sin(dtheta); T(0,3) = dx;
        T(1,0) = sin(dtheta); T(1,1) = cos(dtheta);  T(1,3) = dy;
        return T;
    }
};

 
/**
 * @brief 角点特征结构
 * 用于表示水池的角落点
 */
struct CornerFeature {
    float x, y;                 // 坐标
    float angle;                // 夹角（弧度）
    float confidence;           // 置信度 [0,1]
    
    // 声呐特有属性
    float intensity;             // 原始强度
    float relative_intensity;    // 相对强度（相对于本次扫描最大强度）
    float distance_from_sensor;  // 到传感器的估计距离
    
    // 邻居信息（用于拓扑匹配）
    std::vector<int> neighbor_ids;        // 邻居角点ID
    std::vector<float> neighbor_dists;    // 到邻居的距离
    std::vector<float> neighbor_angles;   // 到邻居的角度
    
    int id;  // 唯一标识
    
    CornerFeature() : x(0), y(0), angle(0), confidence(0),
                      intensity(0), relative_intensity(0),
                      distance_from_sensor(0), id(-1) {}
};

/**
 * @brief 墙壁线段特征结构
 * 用于表示水池的直线墙壁
 */
struct WallFeature {
    float x1, y1;               // 起点
    float x2, y2;               // 终点
    float length;                // 长度
    float angle;                 // 角度（弧度）
    
    // 声呐特有属性
    float max_intensity;          // 最大强度
    float avg_intensity;          // 平均强度
    float min_intensity;          // 最小强度
    float intensity_gradient;     // 强度梯度（沿墙壁方向）
    float viewing_angle;          // 观测角度
    float distance_to_sensor;     // 到传感器的平均距离
    
    float confidence;             // 置信度 [0,1]
    
    // 直线参数 ax + by + c = 0
    float a, b, c;
    
    int id;
    std::vector<int> corner_ids;  // 连接的角点ID
    
    WallFeature() : x1(0), y1(0), x2(0), y2(0), length(0), angle(0),
                    max_intensity(0), avg_intensity(0), min_intensity(0),
                    intensity_gradient(0), viewing_angle(0),
                    distance_to_sensor(0), confidence(0),
                    a(0), b(0), c(0), id(-1) {}
};

/**
 * @brief 特征集结构
 * 包含从点云中提取的所有特征
 */
struct FeatureSet {
    std::vector<CornerFeature> corners;  // 角点特征
    std::vector<WallFeature> walls;      // 墙壁特征
    
    // 传感器估计位置
    float sensor_x;
    float sensor_y;
    
    // 强度统计
    float max_intensity;
    float min_intensity;
    float avg_intensity;
    
    // 扫描位置提示
    std::string scan_hint;  // "right_side", "left_side", "unknown"
    
    FeatureSet() : sensor_x(0), sensor_y(0),
                   max_intensity(0), min_intensity(255),
                   avg_intensity(0), scan_hint("unknown") {}
    
    // 判断特征是否足够进行对齐
    bool hasEnoughFeatures() const {
        return corners.size() >= 2 || walls.size() >= 2;
    }
    
    // 获取总特征数
    size_t totalFeatures() const {
        return corners.size() + walls.size();
    }
};



/**
 * @brief 匹配结果结构
 * 存储特征匹配信息
 */
struct MatchResult {
    std::vector<std::pair<int,int>> corner_matches;  // 角点匹配对 (idx1, idx2)
    std::vector<std::pair<int,int>> wall_matches;    // 墙壁匹配对
    std::vector<float> confidences;                   // 匹配置信度
    
    size_t totalMatches() const {
        return corner_matches.size() + wall_matches.size();
    }
};

/**
 * @brief 对齐结果结构
 * 返回给调用者的最终结果
 */
struct AlignmentResult {
    bool success;                    // 是否成功
    SonarTransformParams transform;        // 最优变换
    float confidence;                 // 整体置信度
    std::vector<std::string> constraints_used;  // 使用的约束
    std::string method;               // 使用的方法
    std::string diagnostic;           // 诊断信息
    
    AlignmentResult() : success(false), confidence(0) {}
};


// ======================== 大型水池对齐器 ========================
// ======================== 特征类型定义 ========================

enum FeatureType {
    CORNER,           // 角点（两条线段交点）
    LINE_SEGMENT,     // 线段
    LINE_ENDPOINT,    // 线段端点
    ARC               // 弧线段（用于圆形水池）
};

/**
 * @brief 扩展的特征结构
 */
struct ExtendedFeature {
    FeatureType type;
    int id;
    
    // 几何信息
    float x1, y1, x2, y2;  // 对于线段
    float x, y;             // 对于端点和角点
    float angle;            // 方向
    float length;           // 长度
    float confidence;
    
    // 关系信息
    std::vector<int> connected_features;  // 连接的其他特征ID
    std::vector<float> connection_angles; // 连接角度
    
    // 所属扫描
    int scan_id;
};

/**
 * @brief 特征关系
 */
struct FeatureRelation {
    enum RelationType {
        PARALLEL,        // 平行
        PERPENDICULAR,   // 垂直
        COLLINEAR,       // 共线
        EXTENSION,       // 延伸
        INTERSECTION     // 相交
    };
    
    RelationType type;
    int feature1_id;
    int feature2_id;
    float confidence;
    float parameter;  // 关系参数（如距离、角度差）
};

/**
 * @brief 拓扑图
 */
struct TopologicalGraph {
    std::vector<ExtendedFeature> nodes;
    std::vector<FeatureRelation> edges;
    
    void clear() {
        nodes.clear();
        edges.clear();
    }
};

// ======================== 大型水池对齐器 ========================
 
/**
 * @brief 声呐感知点云对齐器
 * 
 * 核心功能：
 * 1. 从声呐点云中提取特征（角点、墙壁）
 * 2. 基于几何和强度特征进行匹配
 * 3. 求解最优变换参数
 * 4. 多级验证确保结果可靠性
 * 
 * 专门针对单波束声呐数据优化：
 * - 考虑强度衰减与距离的关系
 * - 利用强弱互补特性
 * - 估计传感器位置辅助匹配
 */
class SonarAwareAligner {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit SonarAwareAligner(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~SonarAwareAligner();

    /**
     * @brief 对齐两个声呐点云
     * @param cloud1 第一次扫描点云
     * @param cloud2 第二次扫描点云
     * @param hint1 第一次扫描位置提示（可选）
     * @param hint2 第二次扫描位置提示（可选）
     * @return 对齐结果
     */
    AlignmentResult align(const PointCloudPtr& cloud1,
                          const PointCloudPtr& cloud2,
                          const std::string& hint1 = "unknown",
                          const std::string& hint2 = "unknown");

    // ======================== 配置接口 ========================
    
    /**
     * @brief 设置调试模式
     */
    void setDebugMode(bool enable) { debug_enabled_ = enable; }
    
    /**
     * @brief 设置最小角点置信度
     */
    void setMinCornerConfidence(float conf) { min_corner_confidence_ = conf; }
    
    /**
     * @brief 设置最小墙壁置信度
     */
    void setMinWallConfidence(float conf) { min_wall_confidence_ = conf; }
    
    /**
     * @brief 设置最大平移误差
     */
    void setMaxTranslationError(float error) { max_translation_error_ = error; }
    
    /**
     * @brief 设置最大旋转误差（弧度）
     */
    void setMaxRotationError(float error) { max_rotation_error_ = error; }

    void setStrongThreshold(float th) { strong_threshold_ = th; }
    void setMediumThreshold(float th) { medium_threshold_ = th; }
    void setWeakThreshold(float th) { weak_threshold_ = th; }


    enum AlignMode {
        MODE_NORMAL,        // 中小型水池（有重叠）
        MODE_LARGE_SCALE    // 大型水池（无重叠，需几何约束）
    };
    
    void setAlignMode(AlignMode mode) { align_mode_ = mode; }
    

     /**
     * @brief 添加一次扫描的特征
     */
    void addScanFeatures(int scan_id, const ShapeFeatures& features);
    
    /**
     * @brief 分析两次扫描之间的关系
     */
    FeatureRelation analyzeRelation(const ExtendedFeature& f1,
                                    const ExtendedFeature& f2);
    
    /**
     * @brief 基于几何约束对齐
     */
    SonarTransformParams alignWithGeometricConstraints(
        int scan_id1, int scan_id2,
        const std::string& relation_hint = "unknown");
    
    /**
     * @brief 构建全局拓扑图
     */
    void buildGlobalTopology();
    
    /**
     * @brief 图优化（用于闭环）
     */
    bool optimizeWithLoopClosure();

private:
    ros::NodeHandle& nh_;
    bool debug_enabled_;
    
    // ======================== 参数配置 ========================
    float min_corner_confidence_;      // 最小角点置信度
    float min_wall_confidence_;        // 最小墙壁置信度
    float max_translation_error_;      // 最大平移误差（米）
    float max_rotation_error_;         // 最大旋转误差（弧度）
    float intensity_weight_factor_;    // 强度权重因子
    float min_feature_distance_;       // 最小特征间距
    int min_points_for_wall_;          // 拟合墙壁所需最少点数
    float corner_curvature_threshold_; // 角点曲率阈值


    // ======================== 成员变量 ========================
    std::ofstream log_file_;
    pcl::KdTreeFLANN<PointType> kdtree_;

       // 强度阈值
    float strong_threshold_;   // ≥160
    float medium_threshold_;   // 140-159
    float weak_threshold_;     // 100-139
    
    // 误差阈值
 
 
    float line_cluster_tolerance_;  // 线段聚类容差
    int min_points_for_line_;       // 线段最少点数

    AlignMode align_mode_;
  
    
    // ======================== 核心功能函数 ========================
    
    /**
     * @brief 从点云提取声呐感知特征
     * @param cloud 输入点云
     * @param hint 扫描位置提示
     * @return 提取的特征集
     */
    FeatureSet extractFeatures(const PointCloudPtr& cloud,
                               const std::string& hint);
    
    /**
     * @brief 估计传感器位置（基于强度梯度）
     * 
     * 核心原理：强度高的区域离传感器近
     */
    void estimateSensorPosition(const PointCloudPtr& cloud,
                                float& sensor_x, 
                                float& sensor_y,
                                float& max_intensity);
    
    /**
     * @brief 检测角点特征
     * @param cloud 输入点云
     * @param sensor_x 传感器X坐标
     * @param sensor_y 传感器Y坐标
     * @param max_intensity 最大强度
     * @return 角点特征列表
     */
    std::vector<CornerFeature> detectCorners(const PointCloudPtr& cloud,
                                            float sensor_x, 
                                            float sensor_y,
                                            float max_intensity);
    
    /**
     * @brief 提取墙壁特征
     * @param cloud 输入点云
     * @param sensor_x 传感器X坐标
     * @param sensor_y 传感器Y坐标
     * @param max_intensity 最大强度
     * @return 墙壁特征列表
     */
    std::vector<WallFeature> extractWalls(const PointCloudPtr& cloud,
                                         float sensor_x,
                                         float sensor_y,
                                         float max_intensity);
    
    /**
     * @brief 计算强度梯度（沿墙壁方向）
     * @param cloud 点云
     * @param wall 墙壁特征
     * @return 强度梯度值
     */
    float computeIntensityGradient(const PointCloudPtr& cloud,
                                   const WallFeature& wall);
    
    /**
     * @brief 为角点构建邻居关系
     * @param corners 角点列表
     */
    void buildCornerNeighborhood(std::vector<CornerFeature>& corners);
    
    /**
     * @brief 匹配两个特征集
     * @param f1 第一个特征集
     * @param f2 第二个特征集
     * @return 匹配结果
     */
    MatchResult matchFeatures(const FeatureSet& f1,
                             const FeatureSet& f2);
    
    /**
     * @brief 计算两个角点的相似度
     * @param c1 第一个角点
     * @param c2 第二个角点
     * @return 相似度 [0,1]
     */
    float computeCornerSimilarity(const CornerFeature& c1,
                                  const CornerFeature& c2);
    
    /**
     * @brief 计算两条墙壁的相似度（考虑强弱互补）
     * @param w1 第一条墙壁
     * @param w2 第二条墙壁
     * @param hint1 第一次扫描提示
     * @param hint2 第二次扫描提示
     * @return 相似度 [0,1]
     */
    float computeWallSimilarity(const WallFeature& w1,
                                const WallFeature& w2,
                                const std::string& hint1,
                                const std::string& hint2);
    
    /**
     * @brief 验证匹配的一致性
     * @param matches 匹配结果
     * @param f1 第一个特征集
     * @param f2 第二个特征集
     * @return 是否一致
     */
    bool validateMatches(const MatchResult& matches,
                        const FeatureSet& f1,
                        const FeatureSet& f2);
    
    /**
     * @brief 从匹配求解变换
     * @param matches 匹配结果
     * @param f1 第一个特征集
     * @param f2 第二个特征集
     * @return 变换参数
     */
    SonarTransformParams solveTransform(const MatchResult& matches,
                                   const FeatureSet& f1,
                                   const FeatureSet& f2);
    
    /**
     * @brief 使用加权SVD求解变换
     * @param points1 第一组点
     * @param points2 第二组点
     * @param weights 权重
     * @return 变换参数
     */
    SonarTransformParams solveWeightedSVD(
        const std::vector<Eigen::Vector2d>& points1,
        const std::vector<Eigen::Vector2d>& points2,
        const std::vector<float>& weights);
    
    /**
     * @brief 从墙壁匹配求解旋转
     * @param wall_matches 墙壁匹配对
     * @param f1 第一个特征集
     * @param f2 第二个特征集
     * @return 旋转角度（弧度）
     */
    float solveRotationFromWalls(
        const std::vector<std::pair<int,int>>& wall_matches,
        const FeatureSet& f1,
        const FeatureSet& f2);
    
    /**
     * @brief 从墙壁匹配求解平移
     * @param dtheta 旋转角度
     * @param wall_matches 墙壁匹配对
     * @param f1 第一个特征集
     * @param f2 第二个特征集
     * @return 平移 (dx, dy)
     */
    std::pair<float, float> solveTranslationFromWalls(
        float dtheta,
        const std::vector<std::pair<int,int>>& wall_matches,
        const FeatureSet& f1,
        const FeatureSet& f2);
    
    /**
     * @brief 验证变换的正确性
     * @param cloud1 第一个点云
     * @param cloud2 第二个点云
     * @param transform 变换参数
     * @param confidence 输出置信度
     * @param diagnostic 输出诊断信息
     * @return 是否有效
     */
    bool validateTransform(const PointCloudPtr& cloud1,
                          const PointCloudPtr& cloud2,
                          const SonarTransformParams& transform,
                          float& confidence,
                          std::string& diagnostic);
    
    /**
     * @brief 检查强度互补性
     * @param cloud1 第一个点云
     * @param cloud2 第二个点云
     * @param transform 变换参数
     * @return 互补性得分 [0,1]
     */
    float checkIntensityComplementarity(const PointCloudPtr& cloud1,
                                        const PointCloudPtr& cloud2,
                                        const SonarTransformParams& transform);
    
    /**
     * @brief 检查墙壁连续性
     * @param cloud1 第一个点云
     * @param cloud2 第二个点云
     * @param transform 变换参数
     * @return 连续性误差（米）
     */
    float checkWallContinuity(const PointCloudPtr& cloud1,
                             const PointCloudPtr& cloud2,
                             const SonarTransformParams& transform);
    
    /**
     * @brief 检查几何闭合性
     * @param cloud1 第一个点云
     * @param cloud2 第二个点云
     * @param transform 变换参数
     * @return 闭合误差（米）
     */
    float checkGeometricClosure(const PointCloudPtr& cloud1,
                                const PointCloudPtr& cloud2,
                                const SonarTransformParams& transform);
    
    /**
     * @brief 基于强度互补性微调变换
     * @param cloud1 第一个点云
     * @param cloud2 第二个点云
     * @param initial 初始变换
     * @return 微调后的变换
     */
    SonarTransformParams refineWithIntensity(const PointCloudPtr& cloud1,
                                        const PointCloudPtr& cloud2,
                                        const SonarTransformParams& initial);

    // ======================== 几何轮廓函数 ========================
     /**
     * @brief 提取几何特征（线段、角点、轮廓）
     */
    GeometryFeatureSet extractGeometryFeatures(const PointCloudPtr& cloud,
                                               const std::string& hint);
    
 
    
    /**
     * @brief 检测角点（通过线段交叉）
     */
    std::vector<CornerPoint> detectCornersFromLines(
        const std::vector<LineSegment>& lines);
    
    /**
     * @brief 计算两条线段的交点
     */
    bool computeLineIntersection(const LineSegment& l1,
                                 const LineSegment& l2,
                                 float& x, float& y);
    
    /**
     * @brief 判断点是否在线段上
     */
    bool isPointOnSegment(float x, float y, const LineSegment& seg);
    
    /**
     * @brief 跟踪封闭轮廓
     */
    ClosedContour traceClosedContour(const std::vector<LineSegment>& lines,
                                      const std::vector<CornerPoint>& corners);
    
    /**
     * @brief 计算轮廓质心
     */
    void computeContourCentroid(const ClosedContour& contour,
                                float& cx, float& cy);
    
    /**
     * @brief 计算多边形面积
     */
    float computePolygonArea(const std::vector<LineSegment>& lines);
    
    /**
     * @brief 基于轮廓对齐两个点云
     */
    SonarTransformParams alignByContour(const GeometryFeatureSet& g1,
                                         const GeometryFeatureSet& g2);
    
    /**
     * @brief 匹配两个轮廓的角点
     */
    std::vector<std::pair<int,int>> matchCornersByGeometry(
        const std::vector<CornerPoint>& corners1,
        const std::vector<CornerPoint>& corners2);
    
    /**
     * @brief 计算两个角点的相似度（基于角度和位置关系）
     */
    float computeCornerGeometrySimilarity(const CornerPoint& c1,
                                          const CornerPoint& c2);
    
    // ======================== 工具函数 ========================
    
    /**
     * @brief 角度归一化到 [-π, π]
     */
    float normalizeAngle(float angle);
    
    /**
     * @brief 计算两点距离
     */
    float distance(float x1, float y1, float x2, float y2);
    
    /**
     * @brief 计算点到直线的距离
     */
    float pointToLineDistance(float x, float y, float a, float b, float c);
    
    /**
     * @brief 计算点到线段的距离
     */
    float pointToSegmentDistance(float x, float y,
                                 float x1, float y1,
                                 float x2, float y2);
    
    /**
     * @brief 记录日志
     */
    void log(const std::string& msg);
    
    /**
     * @brief 保存调试信息
     */
    void saveDebugInfo(const std::string& stage,
                       const FeatureSet& f1,
                       const FeatureSet& f2,
                       const SonarTransformParams& transform);
    
 
    
    // ======================== 第1层：强度分级 ========================
    
    /**
     * @brief 提取强度分级特征
     */
    IntensityStats extractIntensityStats(const PointCloudPtr& cloud);
    
    // ======================== 第2层：几何形状提取 ========================
    
    /**
     * @brief 提取形状特征（线段、角点、轮廓）
     */
    ShapeFeatures extractShapeFeatures(const PointCloudPtr& cloud,
                                       const std::string& hint);
    
    /**
     * @brief 提取线段（基于坐标聚类）
     * 
     * 核心思路：
     * - 水平线：Y值相近的点聚类
     * - 垂直线：X值相近的点聚类
     */
    std::vector<LineSegment> extractLineSegments(const PointCloudPtr& cloud);
    
    /**
     * @brief 提取水平线段
     */
    std::vector<LineSegment> extractHorizontalLines(const PointCloudPtr& cloud);
    
    /**
     * @brief 提取垂直线段
     */
    std::vector<LineSegment> extractVerticalLines(const PointCloudPtr& cloud);
    
   
    /**
     * @brief 判断形状类型
     */
    std::string identifyShapeType(const ClosedContour& contour);
    
    // ======================== 第3层：基于形状的对齐 ========================
    
    /**
     * @brief 基于形状特征对齐两个点云
     */
    SonarTransformParams alignByShape(const ShapeFeatures& shape1,
                                      const ShapeFeatures& shape2);
    
    /**
     * @brief 对齐主方向（旋转对齐）
     */
    float alignDominantAngles(const ShapeFeatures& shape1,
                              const ShapeFeatures& shape2);
    
    /**
     * @brief 对齐轮廓质心（平移对齐）
     */
    void alignCentroids(const ClosedContour& c1,
                        const ClosedContour& c2,
                        float dtheta,
                        float& dx, float& dy);
    
    /**
     * @brief 用角点精调变换
     */
    void refineWithCorners(const ShapeFeatures& shape1,
                           const ShapeFeatures& shape2,
                           float& dx, float& dy, float& dtheta);
 
    //===================超大型水池增加函数和定义========================
    // 关系类型枚举
    enum RelationType {
        UNKNOWN_RELATION,
        CORNER_TO_EDGE,    // 角落到边延伸
        EDGE_TO_EDGE,      // 边到边延伸
        PARALLEL_EDGES,    // 平行边
        OVERLAP            // 有重叠（中小型水池）
    };

    // 关系结构
    struct ScanRelation {
        RelationType type;
        float confidence;
        int corner_idx;      // 如果是角落，记录索引
        int edge1_idx;       // 第一条边索引
        int edge2_idx;       // 第二条边索引
        float angle_diff;    // 角度差
    };

     
    ScanRelation analyzeScanRelation(const ShapeFeatures& shape1,
                                    const ShapeFeatures& shape2,
                                    const std::string& hint);

    SonarTransformParams alignCornerToEdge(const ShapeFeatures& shape1,
                                        const ShapeFeatures& shape2,
                                        const ScanRelation& relation);

    SonarTransformParams alignEdgeToEdge(const ShapeFeatures& shape1,
                                        const ShapeFeatures& shape2,
                                        const ScanRelation& relation);

    SonarTransformParams alignParallelEdges(const ShapeFeatures& shape1,
                                            const ShapeFeatures& shape2,
                                            const ScanRelation& relation);

    bool validateGeometricConstraints(const PointCloudPtr& cloud1,
                                    const PointCloudPtr& cloud2,
                                    const SonarTransformParams& transform,
                                    const ScanRelation& relation);
    
     // 几何约束验证相关函数
    float validateCornerToEdge(const ShapeFeatures& shape1, 
                            const ShapeFeatures& shape2,
                            const SonarTransformParams& transform,
                            const ScanRelation& relation);

    float validateEdgeToEdge(const ShapeFeatures& shape1,
                            const ShapeFeatures& shape2,
                            const SonarTransformParams& transform,
                            const ScanRelation& relation);

    float validateParallelEdges(const ShapeFeatures& shape1,
                                const ShapeFeatures& shape2,
                                const SonarTransformParams& transform);

    float validateGeneral(const ShapeFeatures& shape1,
                        const ShapeFeatures& shape2,
                        const SonarTransformParams& transform);


        // 存储所有扫描的特征
    std::map<int, TopologicalGraph> scans_;
    
    // 扫描间的关系
    std::vector<std::tuple<int, int, FeatureRelation>> inter_scan_relations_;
    
    // ======================== 核心算法 ========================
    
    /**
     * @brief 将ShapeFeatures转换为ExtendedFeature
     */
    std::vector<ExtendedFeature> convertToExtendedFeatures(
        const ShapeFeatures& features, int scan_id);
    
    /**
     * @brief 分析两条线段的关系
     */
    FeatureRelation analyzeLineRelation(
        const ExtendedFeature& line1,
        const ExtendedFeature& line2);
    
    /**
     * @brief 分析线段和角点的关系
     */
    FeatureRelation analyzeLineCornerRelation(
        const ExtendedFeature& line,
        const ExtendedFeature& corner);
    
    /**
     * @brief 基于共线关系对齐
     */
    SonarTransformParams alignByCollinearity(
        const ExtendedFeature& line1,
        const ExtendedFeature& line2,
        const ExtendedFeature& corner);
    
    /**
     * @brief 基于延伸关系对齐
     */
    SonarTransformParams alignByExtension(
        const ExtendedFeature& line,
        const ExtendedFeature& corner_edge,
        const ExtendedFeature& corner);
    
    /**
     * @brief 验证几何约束
     */
    bool validateGeometricConstraint(
        const SonarTransformParams& transform,
        const FeatureRelation& relation);


    /**
     * @brief 合并共线的短线段
     * @param segments 输入线段列表
     * @return 合并后的线段列表
     */
    std::vector<LineSegment> mergeCollinearSegments(
        const std::vector<LineSegment>& segments);
};

} // namespace sonar_slam

#endif // SONAR_AWARE_ALIGNER_H