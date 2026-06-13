/**
 * @file intelligent_submap_aligner.h
 * @brief 智能子图对齐优化器 - 集成几何判别、ICP优化和智能迭代
 * 
 * 功能概述：
 * 1. 多指标几何特征判别（直线/曲线墙壁）
 * 2. 矩阵优化方法（雅可比、海塞、协方差）
 * 3. 智能迭代优化（最多5次迭代）
 * 4. 不确定性感知的决策机制
 * 5. 完整的诊断和日志系统
 */

#ifndef INTELLIGENT_SUBMAP_ALIGNER_H
#define INTELLIGENT_SUBMAP_ALIGNER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ndt.h> 
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions> 

#include <vector>
#include <string>
#include <memory>
#include <map>

#include <fstream>            
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>  // 添加这一行
 
 
  
namespace sonar_slam {

// 类型定义
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
 
 
 
/**
 * @brief 子图信息结构
 */
struct SubMapInfo {
    int index;                              // 子图索引
    PointCloudPtr cloud;                    // 点云数据
    geometry_msgs::Pose2D robot_pose;       // 机器人位姿
    float center_x;                         // 质心X
    float center_y;                         // 质心Y
    ros::Time timestamp;                    // 时间戳
    
    SubMapInfo() : index(-1), center_x(0), center_y(0) {
        cloud.reset(new PointCloud);
    }
};

/**
 * @brief 几何特征
 */
struct GeometricFeature {
    enum Type {
        STRAIGHT_WALL = 0,
        CURVED_WALL,
        CORNER,
        UNKNOWN
    };
    
    Type type;
    std::vector<PointType> points;
    float orientation;          // 方向（弧度）
    float length;               // 长度
    bool is_reliable;           // 是否可靠
    
    // 直线参数
    float slope_k;              // 斜率
    float intercept_b;          // 截距
    
    // 曲线参数
    float curvature;            // 曲率
    float radius;               // 半径
    
    GeometricFeature() : 
        type(UNKNOWN), orientation(0), length(0), 
        is_reliable(false), slope_k(0), intercept_b(0),
        curvature(0), radius(0) {}
};

/**
 * @brief 变换参数
 */
struct TransformParams {
    float dx;                   // X位移（米）
    float dy;                   // Y位移（米）
    float dtheta;               // 旋转角度（弧度）
    float confidence;           // 置信度 0-1
    
    Eigen::Matrix3f covariance; // 协方差矩阵
    Eigen::Matrix3f information; // 信息矩阵
    
    TransformParams() : 
        dx(0), dy(0), dtheta(0), confidence(0) {
        covariance.setIdentity();
        information.setIdentity();
    }
};

/**
 * @brief 对齐质量评估
 */
struct AlignmentQuality {
    // 基础指标
    float feature_match_score;      // 特征匹配得分 0-1
    float geometric_consistency;    // 几何一致性 0-1
    float wall_continuity;          // 墙壁连续性 0-1
    
    // 矩阵指标
    float condition_number;         // 条件数（病态程度）
    float information_gain;         // 信息增益
    float uncertainty_volume;       // 不确定体积
    
    // 综合评分
    float total_score;              // 总分 0-1
    bool is_valid;                  // 是否有效
    
    // 质量等级
    enum Level {
        EXCELLENT = 0,  // >0.85
        GOOD,           // 0.7-0.85
        FAIR,           // 0.5-0.7
        POOR            // <0.5
    };
    Level level;
    
    // 诊断信息
    std::string dominant_issue;     // 主要问题
    std::string suggestion;         // 优化建议
    
    AlignmentQuality() : 
        feature_match_score(0), geometric_consistency(0), wall_continuity(0),
        condition_number(0), information_gain(0), uncertainty_volume(0),
        total_score(0), is_valid(false), level(POOR) {}
};

/**
 * @brief 优化迭代状态
 */
struct OptimizationState {
    int iteration;                  // 当前迭代
    TransformParams current_params; // 当前参数
    AlignmentQuality current_quality; // 当前质量
    float search_range;             // 当前搜索范围
    bool should_change_strategy = false;   
    int strategy_change_count = 0;
    bool allow_rotation;  // 是否允许旋转优化
    
    // 历史记录
    struct Step {
        TransformParams params;
        AlignmentQuality quality;
        std::string decision;
        float improvement;
    };
    std::vector<Step> history;
    
    // 收敛标志
    bool has_converged;
    std::string convergence_reason;
    
    OptimizationState() : 
        iteration(0), search_range(0.3f), 
        has_converged(false) {}
};

/**
 * @brief 优化配置
 */
struct OptimizerConfig {
    // 迭代设置
    int max_iterations = 10;                 // 最大迭代次数
    float min_score_improvement = 0.01f;    // 最小分数提升
    float target_score = 0.92f;             // 目标分数
    
    // 搜索策略
    float initial_search_range = 0.5f;      // 初始搜索范围（米）
    float range_decay_factor = 0.6f;        // 范围衰减因子
    float angle_search_range = 15.0f * M_PI / 180.0f; // 角度搜索范围
    
    // 几何判别权重
    float weight_feature_match = 0.5f;      // 特征匹配权重
    float weight_geometric = 0.3f;         // 几何一致性权重
    float weight_continuity = 0.2f;        // 连续性权重
    
    // ICP参数
    float icp_max_distance = 0.3f;          // ICP最大对应距离
    int icp_max_iterations = 30;            // ICP最大迭代
    float icp_fitness_epsilon = 0.001f;     // ICP适应度阈值
    
    // 矩阵优化参数
    float damping_factor = 0.1f;            // 阻尼因子
    float regularization = 0.01f;           // 正则化系数
    
    // 阈值
    float min_valid_score = 0.5f;           // 最小有效分数
    float min_matched_features = 8.0f;         // 最小匹配特征数
    float max_wall_gap = 75.0f;             // 最大墙壁缺口（度）
};

/**
 * @brief 优化结果
 */
struct OptimizationResult {
    bool success;                           // 是否成功
    TransformParams best_transform;         // 最佳变换
    AlignmentQuality best_quality;          // 最佳质量
    int iterations_used;                    // 使用迭代次数
    std::string final_status;               // 最终状态
    PointCloudPtr merged_cloud;             // 合并后的点云
    
    // 诊断信息
    std::vector<std::string> optimization_log;
    float initial_score;                    // 初始分数
    float final_score;                      // 最终分数
    float improvement_ratio;                // 提升比例
    
    OptimizationResult() : 
        success(false), iterations_used(0),
        initial_score(0), final_score(0), improvement_ratio(0) {
        merged_cloud.reset(new PointCloud);
    }
};


// ==================== 新增：水池几何特征检查 ====================
/**
 * @brief 线方程结构
 */
struct LineEquation {
    float a;    // ax + by + c = 0
    float b;
    float c;
    float angle;  // 角度（弧度）
    
    LineEquation() : a(0), b(0), c(0), angle(0) {}
};

/**
 * @brief 墙壁检测结果
 */
struct WallDetectionResult {
    bool wall_exists;                         // 该方向是否有墙
    float average_distance;                   // 平均距离
    float angle_variance;                     // 角度方差（平直度）
    std::vector<PointType> wall_points;       // 墙壁点
    LineEquation fitted_line;                 // 拟合的直线方程
    float alignment_confidence;               // 对齐置信度 0-1
    
    WallDetectionResult() : 
        wall_exists(false), average_distance(0), 
        angle_variance(0), alignment_confidence(0) {}
};


/**
 * @brief 墙壁平行性检查结果
 */
 
struct WallParallelism {
    bool x_parallel;                    // X方向墙壁是否平行
    bool y_parallel;                    // Y方向墙壁是否平行
    float x_angle_diff;                 // X方向墙壁角度差（度）
    float y_angle_diff;                 // Y方向墙壁角度差（度）
    bool rectangles_consistent;         // 是否构成矩形
    float x_distance_diff;              // X方向墙壁距离差（米）
    float y_distance_diff;              // Y方向墙壁距离差（米）
    
    WallParallelism() : 
        x_parallel(false), y_parallel(false),
        x_angle_diff(0), y_angle_diff(0),
        rectangles_consistent(false),
        x_distance_diff(0), y_distance_diff(0) {}
};

/**
 * @brief 角落对齐检查结果
 */
struct CornerAlignment {
    struct Corner {
        PointType position;
        float confidence;
        std::string description;  // "north_east", "north_west"等
    };
    
    std::vector<Corner> detected_corners;     // 检测到的角落
    std::vector<float> corner_errors;         // 角落误差（米）
    float max_corner_error;                   // 最大角落误差
    float average_corner_error;               // 平均角落误差
    bool all_corners_aligned;                 // 所有角落是否对齐
    
    CornerAlignment() : 
        max_corner_error(0), average_corner_error(0),
        all_corners_aligned(false) {}
};

/**
 * @brief 四方向墙壁对齐结果
 */
struct FourWallAlignment {
    WallDetectionResult x_pos_wall;            
    WallDetectionResult x_neg_wall;          
    WallDetectionResult y_pos_wall;          
    WallDetectionResult y_neg_wall;           
 
    
    WallParallelism parallelism;              // 平行性
    CornerAlignment corners;                  // 角落对齐
    
    float overall_score;                      // 综合得分
    std::string major_issue;                  // 主要问题
    std::vector<std::string> suggestions;     // 优化建议
    
    FourWallAlignment() : overall_score(0) {}
};


/**
 * @brief 多位置建图的边约束（相邻站位之间的约束）
 */
struct MultiPositionEdge {
    int from_id;                        // 源站位索引
    int to_id;                          // 目标站位索引
    TransformParams relative_transform; // 相对变换（通过智能对齐得到）
    float confidence;                   // 约束置信度
    bool is_odom_edge;                  // 是否为里程计约束
    
    MultiPositionEdge() : from_id(-1), to_id(-1), confidence(1.0f), is_odom_edge(false) {}
};

/**
 * @brief 多位置建图优化结果
 */
struct MultiPositionResult {
    bool success;                                    // 是否成功
    std::vector<TransformParams> global_poses;      // 每个站位在世界坐标系下的位姿
    std::vector<AlignmentQuality> qualities;        // 每个对齐的质量
    PointCloudPtr global_cloud;                     // 全局合并点云
    std::vector<std::string> optimization_log;      // 优化日志
    float total_optimization_score;                 // 总优化分数
    int iterations_used;                            // 使用的迭代次数
    
    MultiPositionResult() : success(false), total_optimization_score(0), iterations_used(0) {
        global_cloud.reset(new PointCloud);
    }
};

/**
 * @brief 多位置建图配置
 */
struct MultiPositionConfig {
    bool enable_global_optimization = true;    // 是否启用全局优化
    int max_global_iterations = 5;             // 全局优化最大迭代
    float global_convergence_threshold = 0.01f; // 全局收敛阈值
    bool enable_loop_closure = false;          // 是否启用回环检测（闭合水池）
    float loop_closure_distance_threshold = 1.0f; // 回环检测距离阈值（米）
    
    // 约束权重
    float odom_weight = 0.3f;                  // 里程计约束权重
    float ndt_weight = 0.7f;                   // NDT/智能对齐约束权重
};

struct AlignmentDiagnostics {
        bool is_aligned;                    // 是否对齐成功
        float overall_score;                // 综合分数 0-1
        std::string reason;                 // 主要原因
        
        // X轴方向（前后墙壁）
        bool x_walls_detected;              // 前后墙壁是否都检测到
        bool x_parallel;                    // 前后墙壁是否平行
        float x_angle_diff_deg;             // 前后墙壁角度差（度）
        float x_distance_m;                 // 前后墙壁距离（米）
        float x_distance_error_m;           // 距离误差（正=拉远，负=靠近）
        
        // Y轴方向（左右墙壁）
        bool y_walls_detected;              // 左右墙壁是否都检测到
        bool y_parallel;                    // 左右墙壁是否平行
        float y_angle_diff_deg;             // 左右墙壁角度差（度）
        float y_distance_m;                 // 左右墙壁距离（米）
        float y_distance_error_m;           // 距离误差（正=拉远，负=靠近）
        
        // 旋转偏差
        float rotation_error_deg;           // 整体旋转误差
        
        // 微调建议
        float suggested_dx_m;               // X方向建议微调（米）
        float suggested_dy_m;               // Y方向建议微调（米）
        float suggested_dtheta_deg;         // 旋转建议微调（度）
        
        AlignmentDiagnostics() {
            is_aligned = false;
            overall_score = 0;
            x_walls_detected = false;
            x_parallel = false;
            x_angle_diff_deg = 0;
            x_distance_m = 0;
            x_distance_error_m = 0;
            y_walls_detected = false;
            y_parallel = false;
            y_angle_diff_deg = 0;
            y_distance_m = 0;
            y_distance_error_m = 0;
            rotation_error_deg = 0;
            suggested_dx_m = 0;
            suggested_dy_m = 0;
            suggested_dtheta_deg = 0;
        }
    };

/**
 * @brief 智能子图对齐优化器
 */
class IntelligentSubmapAligner {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param debug_enable 是否启用调试输出
     */
    explicit IntelligentSubmapAligner(ros::NodeHandle& nh, bool debug_enable = false);
    
    /**
     * @brief 析构函数
     */
    virtual ~IntelligentSubmapAligner();
    
    /**
     * @brief 主优化函数 - 在您的mergePhase1Submaps中直接调用
     * @param submap1 第一个子图
     * @param submap2 第二个子图
     * @param initial_guess 初始变换猜测
     * @return 优化结果
     */
    OptimizationResult optimizeAlignment(
        const SubMapInfo& submap1,
        const SubMapInfo& submap2,
        const TransformParams& initial_guess);
    
    /**
     * @brief 快速对齐评估（不进行优化迭代）
     * @param submap1 第一个子图
     * @param submap2 第二个子图
     * @param transform 变换参数
     * @return 对齐质量
     */
    AlignmentQuality evaluateAlignmentQuality(
        const SubMapInfo& submap1,
        const SubMapInfo& submap2,
        const TransformParams& transform);
    
    /**
     * @brief 获取详细优化报告
     * @param result 优化结果
     * @return 格式化报告
     */
    std::string getOptimizationReport(const OptimizationResult& result);
    
    /**
     * @brief 重置优化器状态
     */
    void reset();
    
    /**
     * @brief 保存优化日志到文件
     * @param filename 文件名
     */
    void saveLogToFile(const std::string& filename);
    
    // 配置访问
    void setConfig(const OptimizerConfig& config) { config_ = config; }
    const OptimizerConfig& getConfig() const { return config_; }


    /**
     * @brief 检查四方向墙壁对齐（您提到的核心功能）
     * @param merged_cloud 合并后的点云
     * @return 四方向墙壁对齐结果
     */
    FourWallAlignment checkFourWallAlignment(
        const PointCloud& merged_cloud);
    
    /**
     * @brief 检查东西墙对齐（东墙和西墙）
     * @param merged_cloud 合并后的点云
     * @return 东西墙对齐结果
     */
    WallParallelism checkXDirectionWallAlignment(
        const PointCloud& merged_cloud);
    
    /**
     * @brief 检查南北墙对齐（北墙和南墙）
     * @param merged_cloud 合并后的点云
     * @return 南北墙对齐结果
     */
    WallParallelism checkYDirectionWallAlignment(
        const PointCloud& merged_cloud);
    
    /**
     * @brief 检查角落对齐
     * @param merged_cloud 合并后的点云
     * @return 角落对齐结果
     */
    CornerAlignment checkCornerAlignment(
        const PointCloud& merged_cloud);
    
    /**
     * @brief 生成水池几何诊断报告
     * @param alignment 四方向墙壁对齐结果
     * @return 格式化诊断报告
     */
    std::string generatePoolGeometryReport(
        const FourWallAlignment& alignment);
    
    /**
     * @brief 基于几何诊断结果生成优化建议
     * @param alignment 对齐结果
     * @return 变换参数调整建议
     */
    TransformParams generateOptimizationSuggestion(
        const FourWallAlignment& alignment,
        const TransformParams& current_params);


    /**
     * @brief 计算整体对齐分数
     */
    float computeOverallAlignmentScore(const FourWallAlignment& alignment);


     /**
     * @brief 多位置建图全局优化
     * @param submaps 所有子图信息列表
     * @param config 多位置建图配置
     * @return 优化结果（包含每个站位的全局位姿）
     */
    MultiPositionResult optimizeMultiPositionAlignment(
        const std::vector<SubMapInfo>& submaps,
        const MultiPositionConfig& config = MultiPositionConfig());
    
    /**
     * @brief 增量式多位置建图（边采集边优化）
     * @param new_submap 新采集的子图
     * @param current_result 当前的优化结果
     * @return 更新后的优化结果
     */
    MultiPositionResult incrementalMultiPositionAlignment(
        const SubMapInfo& new_submap,
        const MultiPositionResult& current_result);
    
    /**
     * @brief 检测并添加回环约束
     * @param submaps 所有子图
     * @param current_poses 当前估计的位姿
     * @return 检测到的回环边
     */
    std::vector<MultiPositionEdge> detectLoopClosures(
        const std::vector<SubMapInfo>& submaps,
        const std::vector<TransformParams>& current_poses);


    /**
     * @brief 使用NDT匹配两个点云
     * @param source 源点云（待变换）
     * @param target 目标点云（参考系）
     * @param initial_guess 初始变换估计
     * @return NDT匹配得到的变换参数
     */
    TransformParams matchWithNDT(
        const PointCloudPtr& source,
        const PointCloudPtr& target,
        const TransformParams& initial_guess);
    
    /**
     * @brief 使用Pose Graph + NDT进行精确对齐
     */
    PointCloudPtr alignWithPoseGraphNDT(
        const SubMapInfo& submap1,
        const SubMapInfo& submap2,
        const TransformParams& initial_guess);

     /**
     * @brief 构建相邻站位之间的约束边
     * @param submaps 所有子图
     * @param initial_guesses 初始猜测（可选）
     * @return 约束边列表
     */
    // 头文件声明（推荐）
    std::vector<MultiPositionEdge> buildAdjacentEdges(
        const std::vector<SubMapInfo>& submaps,
        const std::vector<TransformParams>& initial_guesses = std::vector<TransformParams>());

    
    /**
     * @brief 全局图优化（使用最小二乘法）
     * @param edges 所有约束边
     * @param initial_poses 初始位姿估计
     * @param config 配置
     * @return 优化后的位姿
     */
    std::vector<TransformParams> globalGraphOptimization(
        const std::vector<MultiPositionEdge>& edges,
        const std::vector<TransformParams>& initial_poses,
        const MultiPositionConfig& config);
    
     /**
     * @brief 判断对齐质量是否可接受
     * @param cloud 合并后的点云
     * @param reason 输出：失败原因
     * @return true=可接受，false=需要优化
     */
    bool isAlignmentAcceptable(const PointCloudPtr& cloud, std::string& reason);
    
    /**
     * @brief 获取对齐质量分数
     * @param cloud 合并后的点云
     * @return 分数 0-1
     */
    float getAlignmentScore(const PointCloudPtr& cloud);

    /**
     * @brief 计算全局残差
     */
    float computeGlobalResidual(
        const std::vector<MultiPositionEdge>& edges,
        const std::vector<TransformParams>& poses,
        const MultiPositionConfig& config);
    
    /**
     * @brief 使用Gauss-Newton进行全局优化
     */
    void gaussNewtonGlobalOptimization(
        std::vector<TransformParams>& poses,
        const std::vector<MultiPositionEdge>& edges,
        const MultiPositionConfig& config);
    
    /**
     * @brief 将变换参数转换为SE(2)矩阵
     */
    Eigen::Matrix3f toSE2(const TransformParams& params);
    
    /**
     * @brief 从SE(2)矩阵提取变换参数
     */
    TransformParams fromSE2(const Eigen::Matrix3f& T);
    
    // 多位置建图成员变量
    MultiPositionConfig multi_config_;
    std::vector<TransformParams> cached_global_poses_;


 
    
    /**
     * @brief 计算两个位姿之间的残差向量（SE(2)流形上的误差）
     * @param pose_i 位姿i
     * @param pose_j 位姿j
     * @param relative 期望的相对变换
     * @return 残差向量 (dx, dy, dtheta)
     */
    Eigen::Vector3f computeResidualVector(
        const TransformParams& pose_i,
        const TransformParams& pose_j,
        const TransformParams& relative);
    
    /**
     * @brief 计算残差对位姿的雅可比矩阵
     * @param pose_i 位姿i
     * @param pose_j 位姿j
     * @param relative 期望的相对变换（用于验证，实际未使用）
     * @return 3x6雅可比矩阵 [∂e/∂ξ_i, ∂e/∂ξ_j]
     */
    Eigen::Matrix<float, 3, 6> computeResidualJacobian(
        const TransformParams& pose_i,
        const TransformParams& pose_j,
        const TransformParams& relative);
    
 
    
    /**
     * @brief 合成两个变换：T_ab = T_a * T_b
     */
    TransformParams composeTransforms(
        const TransformParams& a,
        const TransformParams& b);
    
    /**
     * @brief 求逆变换：T_inv = T^{-1}
     */
    TransformParams inverseTransform(const TransformParams& params);
    
    /**
     * @brief 计算变换的对数映射（从SE(2)到se(2)）
     * @param T SE(2)矩阵
     * @return se(2)向量 (dx, dy, dtheta)
     */
    Eigen::Vector3f se2Log(const Eigen::Matrix3f& T);
    
    /**
     * @brief 计算变换的指数映射（从se(2)到SE(2)）
     * @param xi se(2)向量 (dx, dy, dtheta)
     * @return SE(2)矩阵
     */
    Eigen::Matrix3f se2Exp(const Eigen::Vector3f& xi);

    // NDT配置参数（可选）
    struct NDTConfig {
        float resolution = 0.5f;
        float step_size = 0.1f;
        float transformation_epsilon = 0.01f;
        int max_iterations = 35;
    } ndt_config_;



    /**
     * @brief 获取详细的对齐诊断信息
     * @param cloud 合并后的点云
     * @return 详细的诊断信息（包含微调建议）
     */
    AlignmentDiagnostics getDetailedAlignmentDiagnostics(const PointCloudPtr& cloud);
    
private:
    // ==================== 核心优化组件 ====================
  
     Eigen::VectorXf computeResiduals(
        const PointCloud& source,
        const PointCloud& target,
        const TransformParams& params);
    
    GeometricFeature fitLineToGeometricFeature(GeometricFeature& feature);
    
    /**
     * @brief 执行一次迭代优化
     */
    OptimizationState::Step performIteration(
        const SubMapInfo& submap1,
        const SubMapInfo& submap2,
        const OptimizationState& state);
    
    /**
     * @brief 分析当前问题并生成优化策略
     */
    std::string analyzeProblemAndGenerateStrategy(
        const AlignmentQuality& quality,
        const TransformParams& params);
    
    /**
     * @brief 生成候选变换参数
     */
    std::vector<TransformParams> generateCandidateTransforms(
        const OptimizationState& state,
        const std::string& strategy);
    
    /**
     * @brief 评估候选变换
     */
    AlignmentQuality evaluateCandidateTransform(
        const SubMapInfo& submap1,
        const SubMapInfo& submap2,
        const TransformParams& candidate);
    
    /**
     * @brief 选择最佳候选
     */
    TransformParams selectBestCandidate(
        const std::vector<TransformParams>& candidates,
        const std::vector<AlignmentQuality>& qualities);
    
    // ==================== 几何特征处理 ====================
    
    /**
     * @brief 提取几何特征
     */
    std::vector<GeometricFeature> extractGeometricFeatures(
        const PointCloud& cloud);
    
    /**
     * @brief 几何一致性评估
     */
    float evaluateGeometricConsistency(
        const std::vector<GeometricFeature>& features1,
        const std::vector<GeometricFeature>& features2,
        const TransformParams& transform);
    
    /**
     * @brief 墙壁连续性评估
     */
    float evaluateWallContinuity(
        const PointCloud& merged_cloud);
    
    // ==================== 矩阵优化方法 ====================
    
    /**
     * @brief 使用高斯-牛顿法优化
     */
    TransformParams optimizeWithGaussNewton(
        const SubMapInfo& submap1,
        const SubMapInfo& submap2,
        const TransformParams& initial);
    
    /**
     * @brief 计算雅可比矩阵
     */
    Eigen::MatrixXf computeJacobianMatrix(
        const PointCloud& source,
        const PointCloud& target,
        const TransformParams& params);
    
    /**
     * @brief 计算协方差矩阵
     */
    Eigen::Matrix3f computeCovarianceMatrix(
        const Eigen::MatrixXf& J,
        const Eigen::VectorXf& residuals);
    
    /**
     * @brief 计算信息矩阵
     */
    Eigen::Matrix3f computeInformationMatrix(
        const Eigen::Matrix3f& covariance);
    
    // ==================== ICP优化 ====================
    
    /**
     * @brief ICP细化
     */
    TransformParams refineWithICP(
        const PointCloudPtr& source,
        const PointCloudPtr& target,
        const TransformParams& initial);
    
    // ==================== 收敛判断 ====================
    
    /**
     * @brief 检查收敛条件
     */
    bool checkConvergence(
          OptimizationState& state,
        const OptimizationState::Step& new_step);
    
    /**
     * @brief 调整搜索范围
     */
    float adjustSearchRange(
        const OptimizationState& state,
        const AlignmentQuality& quality);
    
    // ==================== 工具函数 ====================
    
    /**
     * @brief 合并点云
     */
    PointCloudPtr mergePointClouds(
        const SubMapInfo& submap1,
        const SubMapInfo& submap2,
        const TransformParams& transform);
    
    /**
     * @brief 应用变换到点云
     */
    PointCloudPtr transformPointCloud(
        const PointCloudPtr& cloud,
        const TransformParams& transform);
    
    /**
     * @brief 计算特征匹配得分
     */
    float computeFeatureMatchScore(
        const PointCloud& cloud1,
        const PointCloud& cloud2,
        const TransformParams& transform);
    
    /**
     * @brief 计算综合评分
     */
    float computeTotalScore(const AlignmentQuality& quality);


    /**
     * @brief 在指定角度范围内检测墙壁
     * @param cloud 点云
     * @param min_angle 最小角度（弧度）
     * @param max_angle 最大角度（弧度）
     * @return 墙壁检测结果
     */
    WallDetectionResult detectWallInDirection(
        const PointCloud& cloud,
        float min_angle, float max_angle);

   

    WallDetectionResult  detectXPosWall(const PointCloud& cloud) {
        // 前方墙壁：机器人前方 X+，声纳角度 270°
        return detectWallInDirection(cloud, 270.0f * DEG_TO_RAD - 30.0f * DEG_TO_RAD,
                                            270.0f * DEG_TO_RAD + 30.0f * DEG_TO_RAD);
    }

    WallDetectionResult  detectXNegWall(const PointCloud& cloud) {
        // 后方墙壁：机器人后方 X-，声纳角度 90°
        return detectWallInDirection(cloud, 90.0f * DEG_TO_RAD - 30.0f * DEG_TO_RAD,
                                            90.0f * DEG_TO_RAD + 30.0f * DEG_TO_RAD);
    }

    WallDetectionResult  detectYPosWall(const PointCloud& cloud) {
        // 左侧墙壁：机器人左侧 Y+，声纳角度 0°
        return detectWallInDirection(cloud, -30.0f * DEG_TO_RAD, 30.0f * DEG_TO_RAD);
    }

    WallDetectionResult  detectYNegWall(const PointCloud& cloud) {
        // 右侧墙壁：机器人右侧 Y-，声纳角度 180°
        return detectWallInDirection(cloud, 150.0f * DEG_TO_RAD, 210.0f * DEG_TO_RAD);
    }
        
    /**
     * @brief 拟合直线到点集
     */
    LineEquation fitLineToPoints(const std::vector<PointType>& points);
    
    /**
     * @brief 检测点云中的角落
     */
    std::vector<PointType> detectCorners(const PointCloud& cloud);
    
    /**
     * @brief 计算两条直线的平行性
     */
    float computeLineParallelism(const LineEquation& line1, 
                                const LineEquation& line2);
    
    /**
     * @brief 计算两条直线的垂直性
     */
    float computeLinePerpendicularity(const LineEquation& line1, 
                                     const LineEquation& line2);
    
    /**
     * @brief 计算点集的质心
     */
    PointType computeCentroid(const std::vector<PointType>& points);
    
    /**
     * @brief 判断点是否构成角落
     */
    bool isCornerPoint(const PointType& point, 
                      const std::vector<PointType>& neighbors,
                      float corner_threshold = 0.3f);

    


    /**
     * @brief 检查墙壁平行性
     */
    WallParallelism checkWallParallelism(const FourWallAlignment& alignment);

    /**
     * @brief 生成对齐诊断信息
     */
    void generateAlignmentDiagnostics(FourWallAlignment& alignment);
    
    // ==================== 成员变量 ====================
    
    OptimizerConfig config_;                    // 配置参数
    ros::NodeHandle& nh_;                       // ROS节点句柄
    bool debug_enabled_;                        // 调试模式
    
    // 内部工具
    std::shared_ptr<pcl::KdTreeFLANN<PointType>> kdtree_;
    std::shared_ptr<pcl::IterativeClosestPoint<PointType, PointType>> icp_;
    
    // 日志记录
    std::vector<std::string> optimization_log_;
    std::ofstream log_file_;
    
    // 状态缓存
    std::map<std::string, GeometricFeature> feature_cache_;
    
    // 私有常量
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;

    
};

} // namespace sonar_slam

#endif // INTELLIGENT_SUBMAP_ALIGNER_H