/**
 * @file intelligent_submap_aligner.cpp
 * @brief 智能子图对齐优化器实现
 */

#include "sonar_slam/intelligent_submap_aligner.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <sstream>

using namespace sonar_slam;
using namespace std;
using namespace Eigen;
using namespace std;

// ======================== 构造函数和析构函数 ========================

IntelligentSubmapAligner::IntelligentSubmapAligner(ros::NodeHandle& nh, bool debug_enable) 
    : nh_(nh), debug_enabled_(debug_enable) {
    
    ROS_INFO("初始化 IntelligentSubmapAligner...");
    
    // 从参数服务器读取配置
    nh_.param("aligner/max_iterations", config_.max_iterations, 5);
    nh_.param("aligner/target_score", config_.target_score, 0.75f);
    nh_.param("aligner/initial_search_range", config_.initial_search_range, 0.3f);
    
    nh_.param("aligner/weight_feature_match", config_.weight_feature_match, 0.4f);
    nh_.param("aligner/weight_geometric", config_.weight_geometric, 0.35f);
    nh_.param("aligner/weight_continuity", config_.weight_continuity, 0.25f);
    
    nh_.param("icp/max_distance", config_.icp_max_distance, 0.3f);
    nh_.param("icp/max_iterations", config_.icp_max_iterations, 30);
    
    nh_.param("aligner/min_valid_score", config_.min_valid_score, 0.5f);
    nh_.param("aligner/min_matched_features", config_.min_matched_features, 8.0f);
    
    // 初始化工具
    kdtree_.reset(new pcl::KdTreeFLANN<PointType>());
    icp_.reset(new pcl::IterativeClosestPoint<PointType, PointType>());
    
    // 配置ICP
    icp_->setMaxCorrespondenceDistance(config_.icp_max_distance);
    icp_->setMaximumIterations(config_.icp_max_iterations);
    icp_->setTransformationEpsilon(1e-6);
    icp_->setEuclideanFitnessEpsilon(config_.icp_fitness_epsilon);
    
    // 创建日志目录
    boost::filesystem::path log_dir("/tmp/submap_aligner_logs");
    if (!boost::filesystem::exists(log_dir)) {
        boost::filesystem::create_directories(log_dir);
    }
    
    // 打开日志文件
    string log_filename = "/tmp/submap_aligner_logs/alignment_" + 
                         to_string(static_cast<long>(ros::Time::now().toSec())) + ".log";
    log_file_.open(log_filename, ios::out | ios::app);
    
    if (log_file_.is_open()) {
        log_file_ << "timestamp,iteration,dx,dy,dtheta,score,improvement,decision\n";
        log_file_.flush();
    }
    
    ROS_INFO("IntelligentSubmapAligner 初始化完成");
    ROS_INFO("配置: 最大迭代=%d, 目标分数=%.2f, 搜索范围=%.2fm",
             config_.max_iterations, config_.target_score, config_.initial_search_range);
}

IntelligentSubmapAligner::~IntelligentSubmapAligner() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
    ROS_INFO("IntelligentSubmapAligner 销毁");
}

// ======================== 主优化函数 ========================

OptimizationResult IntelligentSubmapAligner::optimizeAlignment(
    const SubMapInfo& submap1,
    const SubMapInfo& submap2,
    const TransformParams& initial_guess) {
    
    ROS_INFO("========================================");
    ROS_INFO("开始智能子图对齐优化 - 几何特征分析版");
    ROS_INFO("========================================");
    
    OptimizationResult result;
    OptimizationState state;
    
    // ========== 第一阶段：几何特征分析，决定旋转需求 ==========
    ROS_INFO("=== 第一阶段：几何特征分析 ===");
    
    // 1. 提取两个子图的几何特征
    vector<GeometricFeature> features1 = extractGeometricFeatures(*submap1.cloud);
    vector<GeometricFeature> features2 = extractGeometricFeatures(*submap2.cloud);
    
    ROS_INFO("特征提取: 子图1=%lu个，子图2=%lu个", features1.size(), features2.size());
    
    // 2. 过滤可靠特征
    vector<GeometricFeature> reliable_features1, reliable_features2;
    for (const auto& f : features1) {
        if (f.is_reliable && f.points.size() >= 5) {
            reliable_features1.push_back(f);
        }
    }
    for (const auto& f : features2) {
        if (f.is_reliable && f.points.size() >= 5) {
            reliable_features2.push_back(f);
        }
    }
    
    ROS_INFO("可靠特征: 子图1=%lu个，子图2=%lu个", 
             reliable_features1.size(), reliable_features2.size());
    
    // 3. 基于几何特征分析旋转需求
    bool allow_rotation_optimization = false;
    float suggested_rotation = 0.0f;
    TransformParams starting_guess = initial_guess;
    
    if (reliable_features1.size() >= 2 && reliable_features2.size() >= 2) {
        // 有足够特征进行分析
        
        // 方法1：计算特征方向直方图的匹配
        const int HIST_BINS = 36; // 每10度一个bin
        vector<float> hist1(HIST_BINS, 0);
        vector<float> hist2(HIST_BINS, 0);
        
        // 构建方向直方图（带权重）
        for (const auto& f : reliable_features1) {
            float angle_deg = f.orientation * RAD_TO_DEG;
            if (angle_deg < 0) angle_deg += 360.0f;
            int bin = static_cast<int>(angle_deg / 10.0f) % HIST_BINS;
            hist1[bin] += f.points.size(); // 用点数作为权重
        }
        
        for (const auto& f : reliable_features2) {
            float angle_deg = f.orientation * RAD_TO_DEG;
            if (angle_deg < 0) angle_deg += 360.0f;
            int bin = static_cast<int>(angle_deg / 10.0f) % HIST_BINS;
            hist2[bin] += f.points.size();
        }
        
        // 归一化直方图
        float sum1 = 0, sum2 = 0;
        for (int i = 0; i < HIST_BINS; i++) {
            sum1 += hist1[i];
            sum2 += hist2[i];
        }
        if (sum1 > 0) for (int i = 0; i < HIST_BINS; i++) hist1[i] /= sum1;
        if (sum2 > 0) for (int i = 0; i < HIST_BINS; i++) hist2[i] /= sum2;
        
        // 寻找最佳旋转匹配（直方图互相关）
        float best_correlation = -1.0f;
        int best_shift = 0;
        
        for (int shift = 0; shift < HIST_BINS; shift++) {
            float correlation = 0.0f;
            for (int i = 0; i < HIST_BINS; i++) {
                int j = (i + shift) % HIST_BINS;
                correlation += hist1[i] * hist2[j];
            }
            
            if (correlation > best_correlation) {
                best_correlation = correlation;
                best_shift = shift;
            }
        }
        
        // 将bin shift转换为角度
        suggested_rotation = static_cast<float>(best_shift) * 10.0f * DEG_TO_RAD;
        
        // 方法2：直接匹配特征方向（更精确但计算量大）
        vector<float> matched_rotations;
        
        for (const auto& f1 : reliable_features1) {
            for (const auto& f2 : reliable_features2) {
                // 只匹配类型相同的特征
                if (f1.type == f2.type) {
                    float angle_diff = f1.orientation - f2.orientation;
                    // 归一化到[-π, π]
                    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
                    
                    // 记录这个匹配的旋转
                    matched_rotations.push_back(-angle_diff); // 负号是因为要校正
                }
            }
        }
        
        // 如果有直接匹配的特征，使用它们的平均旋转
        if (!matched_rotations.empty()) {
            float sum_rotation = 0;
            for (float rot : matched_rotations) sum_rotation += rot;
            float average_rotation = sum_rotation / matched_rotations.size();
            
            // 计算旋转的一致性（标准差）
            float variance = 0;
            for (float rot : matched_rotations) {
                float diff = rot - average_rotation;
                while (diff > M_PI) diff -= 2 * M_PI;
                while (diff < -M_PI) diff += 2 * M_PI;
                variance += diff * diff;
            }
            variance /= matched_rotations.size();
            float std_dev = sqrt(variance);
            
            ROS_INFO("特征直接匹配: %lu对，平均旋转=%.1f°，标准差=%.1f°",
                    matched_rotations.size(), average_rotation * RAD_TO_DEG, std_dev * RAD_TO_DEG);
            
            // 如果旋转一致性好（标准差小），使用直接匹配的结果
            if (std_dev < 15.0f * DEG_TO_RAD && matched_rotations.size() >= 3) {
                suggested_rotation = average_rotation;
                best_correlation = 0.9f; // 高质量匹配
            }
        }
        
        ROS_INFO("直方图分析: 最佳匹配旋转=%.1f°，相关系数=%.3f",
                suggested_rotation * RAD_TO_DEG, best_correlation);
        
        // 决策：是否允许旋转优化
        if (best_correlation > 0.3f && fabs(suggested_rotation) > 2.0f * DEG_TO_RAD) {
            // 有明显的共同旋转模式
            allow_rotation_optimization = true;
            
            // 限制旋转幅度
            float max_allowed_rotation = 15.0f * DEG_TO_RAD;
            if (fabs(suggested_rotation) > max_allowed_rotation) {
                ROS_WARN("计算旋转%.1f°过大，限制为±15°", suggested_rotation * RAD_TO_DEG);
                suggested_rotation = (suggested_rotation > 0 ? max_allowed_rotation : -max_allowed_rotation);
            }
            
            // 与odom旋转比较
            float odom_rotation = submap2.robot_pose.theta - submap1.robot_pose.theta;
            float odom_diff = fabs(suggested_rotation - odom_rotation);
            if (odom_diff > 10.0f * DEG_TO_RAD) {
                ROS_WARN("计算旋转(%.1f°)与odom旋转(%.1f°)差异大(%.1f°)，谨慎处理",
                        suggested_rotation * RAD_TO_DEG, odom_rotation * RAD_TO_DEG, odom_diff * RAD_TO_DEG);
            }
            
            starting_guess.dtheta = suggested_rotation;
            ROS_INFO("✅ 检测到共同旋转模式: %.1f°，允许旋转优化", suggested_rotation * RAD_TO_DEG);
        } else {
            // 没有明显的共同旋转模式
            allow_rotation_optimization = false;
            starting_guess.dtheta = 0.0f;
            ROS_INFO("❌ 没有检测到明显的共同旋转模式，禁用旋转优化");
        }
    } else {
        // 特征不足，保守处理
        ROS_WARN("几何特征不足，禁用旋转优化");
        allow_rotation_optimization = false;
        starting_guess.dtheta = 0.0f;
    }
    
    // ========== 第二阶段：四方向墙壁对齐验证 ==========
    ROS_INFO("=== 第二阶段：四方向墙壁对齐验证 ===");
    
    // 检查初始猜测的几何对齐情况
    PointCloudPtr initial_merged = mergePointClouds(submap1, submap2, starting_guess);
    FourWallAlignment initial_alignment = checkFourWallAlignment(*initial_merged);
    
    // 生成对齐报告
    string alignment_report = generatePoolGeometryReport(initial_alignment);
    ROS_INFO("%s", alignment_report.c_str());
    
    // 如果四方向对齐很好，进一步限制旋转
    if (initial_alignment.overall_score > 0.8f) {
        ROS_INFO("✅ 初始几何对齐优秀(%.2f)，强制固定旋转为0", initial_alignment.overall_score);
        allow_rotation_optimization = false;
        starting_guess.dtheta = 0.0f;
    }
    
    // ========== 第三阶段：初始化优化状态 ==========
    state.iteration = 0;
    state.current_params = starting_guess;
    state.search_range = config_.initial_search_range;
    state.current_quality = evaluateAlignmentQuality(submap1, submap2, starting_guess);
    state.has_converged = false;
    state.allow_rotation = allow_rotation_optimization;
    
    // 记录日志
    result.initial_score = state.current_quality.total_score;
    result.optimization_log.push_back("初始分数: " + to_string(result.initial_score));
    result.optimization_log.push_back("几何特征分析: " + string(allow_rotation_optimization ? "允许旋转" : "禁用旋转"));
    result.optimization_log.push_back("建议旋转: " + to_string(starting_guess.dtheta * RAD_TO_DEG) + "°");
    result.optimization_log.push_back("初始几何对齐: " + to_string(initial_alignment.overall_score));
    
    // ========== 第四阶段：迭代优化（智能控制） ==========
    
    // 如果初始质量已经很好，直接返回（降低阈值）
    const float GOOD_ENOUGH_SCORE = 0.75f;
    if (state.current_quality.total_score >= GOOD_ENOUGH_SCORE && 
        initial_alignment.overall_score >= 0.8f) {
        ROS_INFO("✅ 初始对齐优秀(分数:%.3f, 几何:%.2f)，跳过优化", 
                state.current_quality.total_score, initial_alignment.overall_score);
        result.success = true;
        result.best_transform = starting_guess;
        result.best_quality = state.current_quality;
        result.iterations_used = 0;
        result.final_score = state.current_quality.total_score;
        result.improvement_ratio = 0;
        result.final_status = "初始质量已达标";
        result.merged_cloud = mergePointClouds(submap1, submap2, starting_guess);
        return result;
    }
    
    ROS_INFO("初始分数 %.3f，开始迭代优化（最大%d次）", 
            state.current_quality.total_score, config_.max_iterations);
    
    // 迭代优化循环
    while (state.iteration < config_.max_iterations && !state.has_converged) {
        state.iteration++;
        
        ROS_INFO("=== 迭代 %d/%d (旋转优化:%s) ===", 
                state.iteration, config_.max_iterations,
                state.allow_rotation ? "允许" : "禁用");
        
        // 执行一次迭代优化
        OptimizationState::Step new_step = performIteration(submap1, submap2, state);
        
        // 强制实施旋转限制
        if (!state.allow_rotation) {
            new_step.params.dtheta = 0.0f;
            // 重新评估质量（因为旋转被强制为0）
            new_step.quality = evaluateAlignmentQuality(submap1, submap2, new_step.params);
        }
        
        // 记录迭代结果
        state.history.push_back(new_step);
        
        // 判断是否接受新参数（考虑几何对齐改进）
        bool accept_new_params = false;
        float score_improvement = new_step.quality.total_score - state.current_quality.total_score;
        
        // 条件1：分数显著提升
        if (score_improvement > config_.min_score_improvement) {
            accept_new_params = true;
        }
        // 条件2：从不有效变为有效
        else if (new_step.quality.is_valid && !state.current_quality.is_valid) {
            accept_new_params = true;
        }
        // 条件3：分数几乎没变，但几何对齐可能改善了
        else if (fabs(score_improvement) < 0.005f) {
            PointCloudPtr test_merged = mergePointClouds(submap1, submap2, new_step.params);
            FourWallAlignment test_alignment = checkFourWallAlignment(*test_merged);
            
            if (test_alignment.overall_score > initial_alignment.overall_score + 0.05f) {
                ROS_INFO("迭代%d：几何对齐从%.2f提升到%.2f，接受参数",
                        state.iteration, initial_alignment.overall_score,
                        test_alignment.overall_score);
                accept_new_params = true;
                // 给予分数奖励
                new_step.quality.total_score += 0.05f;
                new_step.improvement = 0.05f;
            }
        }
        
        if (accept_new_params) {
            state.current_params = new_step.params;
            state.current_quality = new_step.quality;
            
            ROS_INFO("接受新参数: 分数 %.3f → %.3f (提升: %.3f)",
                    state.current_quality.total_score - new_step.improvement,
                    state.current_quality.total_score, new_step.improvement);
        }
        
        // 检查收敛条件
        state.has_converged = checkConvergence(state, new_step);
        if (state.has_converged) {
            state.convergence_reason = new_step.decision;
            ROS_INFO("达到收敛条件: %s", state.convergence_reason.c_str());
        }
        
        // 调整搜索范围
        if (!state.should_change_strategy) {
            state.search_range = adjustSearchRange(state, new_step.quality);
        }
        
        // 记录日志
        if (log_file_.is_open()) {
            log_file_ << fixed << setprecision(6)
                     << ros::Time::now().toSec() << ","
                     << state.iteration << ","
                     << new_step.params.dx << ","
                     << new_step.params.dy << ","
                     << new_step.params.dtheta * RAD_TO_DEG << ","
                     << new_step.quality.total_score << ","
                     << new_step.improvement << ","
                     << new_step.decision << "\n";
            log_file_.flush();
        }
        
        result.optimization_log.push_back(
            "迭代" + to_string(state.iteration) + ": " + 
            new_step.decision + " (分数: " + 
            to_string(new_step.quality.total_score) + ")");
        
        // 提前停止条件：达到足够好的分数和几何对齐
        if (state.current_quality.total_score >= 0.7f) {
            PointCloudPtr current_merged = mergePointClouds(submap1, submap2, state.current_params);
            FourWallAlignment current_align = checkFourWallAlignment(*current_merged);
            
            if (current_align.overall_score >= 0.8f) {
                ROS_INFO("✅ 达到足够好的条件: 分数=%.3f, 几何对齐=%.2f，提前停止",
                        state.current_quality.total_score, current_align.overall_score);
                state.has_converged = true;
                state.convergence_reason = "达到足够好的分数和几何对齐";
                break;
            }
        }
    }
    
    // ========== 第五阶段：最终几何优化 ==========
    TransformParams final_transform = state.current_params;
    AlignmentQuality final_quality = state.current_quality;
    
    // 最终的四方向检查
    PointCloudPtr final_merged = mergePointClouds(submap1, submap2, final_transform);
    FourWallAlignment final_alignment = checkFourWallAlignment(*final_merged);
    
    // 只在几何对齐明显不好时才尝试微调
    if (final_alignment.overall_score < 0.7f && final_quality.total_score > 0.5f) {
        ROS_INFO("最终几何对齐分数较低(%.2f)，尝试微调...", final_alignment.overall_score);
        
        TransformParams suggestion = generateOptimizationSuggestion(final_alignment, final_transform);
        
        // 检查建议是否合理
        if (!state.allow_rotation && fabs(suggestion.dtheta) > 1.0f * DEG_TO_RAD) {
            ROS_WARN("几何建议有旋转%.1f°，但不允许旋转，跳过", suggestion.dtheta * RAD_TO_DEG);
        } else {
            AlignmentQuality suggestion_quality = evaluateAlignmentQuality(submap1, submap2, suggestion);
            
            if (suggestion_quality.total_score > final_quality.total_score + 0.02f) {
                final_transform = suggestion;
                final_quality = suggestion_quality;
                ROS_INFO("几何微调成功: 分数 %.3f → %.3f",
                        final_quality.total_score - 0.02f, suggestion_quality.total_score);
            }
        }
    }
    
    // ========== 第六阶段：构建最终结果 ==========
    result.success = final_quality.is_valid;
    result.best_transform = final_transform;
    result.best_quality = final_quality;
    result.iterations_used = state.iteration;
    result.final_score = final_quality.total_score;
    result.merged_cloud = mergePointClouds(submap1, submap2, final_transform);
    
    // 计算最终几何对齐分数
    PointCloudPtr best_merged = mergePointClouds(submap1, submap2, result.best_transform);
    FourWallAlignment best_alignment = checkFourWallAlignment(*best_merged);
    result.optimization_log.push_back("最终几何对齐分数: " + to_string(best_alignment.overall_score));
    
    // 计算改进比率
    result.improvement_ratio = (result.final_score - result.initial_score) / 
                              max(0.01f, 1.0f - result.initial_score);
    
    // 确定最终状态
    result.final_status = state.has_converged ? 
                         ("收敛: " + state.convergence_reason) : 
                         ("达到最大迭代: " + to_string(config_.max_iterations));
    
    if (best_alignment.overall_score > 0.8f) {
        result.final_status += " + 几何对齐良好";
    }
    
    // 输出最终报告
    ROS_INFO("%s", getOptimizationReport(result).c_str());
    
    return result;
}
// ======================== 单次迭代优化 ========================

OptimizationState::Step IntelligentSubmapAligner::performIteration(
    const SubMapInfo& submap1,
    const SubMapInfo& submap2,
    const OptimizationState& state) {
    
    OptimizationState::Step step;
    
    // 1. 分析当前问题并生成策略
    string strategy = analyzeProblemAndGenerateStrategy(
        state.current_quality, state.current_params);
    
    ROS_INFO("优化策略: %s", strategy.c_str());
    
    // 2. 生成候选变换
    vector<TransformParams> candidates = generateCandidateTransforms(state, strategy);
    
    // 3. 评估每个候选
    vector<AlignmentQuality> qualities;
    for (const auto& cand : candidates) {
        AlignmentQuality quality = evaluateCandidateTransform(submap1, submap2, cand);
        qualities.push_back(quality);
    }
    
    // 4. 选择最佳候选
    TransformParams best_candidate = selectBestCandidate(candidates, qualities);
    AlignmentQuality best_quality = evaluateCandidateTransform(submap1, submap2, best_candidate);
    
    // 5. 尝试ICP细化（如果质量一般）
    if (best_quality.total_score > 0.4f && best_quality.total_score < 0.7f) {
        ROS_INFO("尝试ICP细化...");
        TransformParams refined = refineWithICP(
            submap2.cloud, submap1.cloud, best_candidate);
        
        AlignmentQuality refined_quality = evaluateCandidateTransform(
            submap1, submap2, refined);
        
        if (refined_quality.total_score > best_quality.total_score + 0.05f) {
            best_candidate = refined;
            best_quality = refined_quality;
            strategy += " + ICP细化";
        }
    }
    
    // 6. 尝试矩阵优化（如果特征匹配较好）
    if (best_quality.feature_match_score > 0.6f) {
        ROS_INFO("尝试矩阵优化...");
        TransformParams optimized = optimizeWithGaussNewton(
            submap1, submap2, best_candidate);
        
        AlignmentQuality optimized_quality = evaluateCandidateTransform(
            submap1, submap2, optimized);
        
        if (optimized_quality.total_score > best_quality.total_score) {
            best_candidate = optimized;
            best_quality = optimized_quality;
            strategy += " + 矩阵优化";
        }
    }
    
    // 7. 填充步骤信息
    step.params = best_candidate;
    step.quality = best_quality;
    step.decision = strategy;
    step.improvement = best_quality.total_score - state.current_quality.total_score;
    
    return step;
}

// ======================== 问题分析和策略生成 ========================

string IntelligentSubmapAligner::analyzeProblemAndGenerateStrategy(
    const AlignmentQuality& quality,
    const TransformParams& params) {
    
    stringstream strategy;
    
    // 1. 分析当前状态
    bool severe_feature_mismatch = (quality.feature_match_score < 0.1f);
    bool good_geometry = (quality.geometric_consistency > 0.8f && quality.wall_continuity > 0.7f);
    bool has_significant_rotation = (fabs(params.dtheta) > 5.0f * DEG_TO_RAD);
    bool close_to_success = (quality.total_score > 0.7f && quality.total_score < 0.85f);
    
    // 2. 基于具体问题选择策略
    if (severe_feature_mismatch && good_geometry) {
        // 情况1：特征匹配极差但几何好（您的失败案例）
        strategy << "几何优先策略 → ";
        if (has_significant_rotation) {
            strategy << "优化旋转角度为主";
        } else {
            strategy << "优化平移为主";
        }
        
        // 根据分数决定激进程度
        if (quality.total_score < 0.6f) {
            strategy << " (大范围搜索)";
        } else if (close_to_success) {
            strategy << " (精细调整)";
        } else {
            strategy << " (中等范围搜索)";
        }
    }
    else if (quality.feature_match_score < 0.3f) {
        // 情况2：特征匹配一般
        strategy << "特征匹配差 → ";
        if (params.confidence < 0.3f || quality.total_score < 0.6f) {
            strategy << "大范围搜索寻找匹配";
        } else {
            // 即使置信度高，如果分数不够好，也扩大搜索
            if (close_to_success) {
                strategy << "多方向精细搜索";
            } else {
                strategy << "扩大范围搜索";
            }
        }
    } 
    else if (quality.geometric_consistency < 0.4f) {
        // 情况3：几何不一致
        strategy << "几何不一致 → ";
        strategy << "检查墙壁平行性和距离";
        if (has_significant_rotation) {
            strategy << "，优先调整旋转";
        }
    }
    else if (quality.wall_continuity < 0.5f) {
        // 情况4：墙壁不连续
        strategy << "墙壁不连续 → ";
        strategy << "优化旋转角度和位置";
    }
    else if (close_to_success) {
        // 情况5：接近成功
        strategy << "接近成功 → ";
        strategy << "多目标平衡精细调整";
    }
    else {
        // 默认情况
        strategy << "综合优化 → ";
        strategy << "多目标平衡搜索";
    }
    
    return strategy.str();
}

// ======================== 候选变换生成 ========================
vector<TransformParams> IntelligentSubmapAligner::generateCandidateTransforms(
    const OptimizationState& state,
    const string& strategy) {
    
    vector<TransformParams> candidates;
    const TransformParams& current = state.current_params;
    
    // ========== 限制旋转搜索范围 ==========
    float MAX_ALLOWED_ROTATION = 5.0f * DEG_TO_RAD; // 最大±5°
    
    // 如果不允许旋转，进一步限制
    if (!state.allow_rotation) {
        MAX_ALLOWED_ROTATION = 0.1f * DEG_TO_RAD; // 几乎不允许旋转
    }
    
    // 根据策略动态调整搜索参数
    float range_factor = 1.0f;
    float angle_factor = 1.0f;
    int grid_size = 2;  // 默认网格大小
    
    // 解析策略关键词
    if (strategy.find("大范围") != string::npos) {
        range_factor = 2.0f;
        angle_factor = 2.0f;
        grid_size = 3;
    } else if (strategy.find("精细") != string::npos) {
        range_factor = 0.5f;
        angle_factor = 0.5f;
        grid_size = 2;
    } else if (strategy.find("扩大范围") != string::npos) {
        range_factor = 1.5f;
        angle_factor = 1.5f;
        grid_size = 3;
    }
    
    float adjusted_range = state.search_range * range_factor;
    float adjusted_angle_range = min(config_.angle_search_range * angle_factor, MAX_ALLOWED_ROTATION);
    
    // 基础候选：当前参数
    candidates.push_back(current);
    
    // 生成网格候选点
    for (int dx_idx = -grid_size; dx_idx <= grid_size; dx_idx++) {
        for (int dy_idx = -grid_size; dy_idx <= grid_size; dy_idx++) {
            // 跳过(0,0)，即当前位置
            if (dx_idx == 0 && dy_idx == 0) continue;
            
            for (int dtheta_idx = -grid_size; dtheta_idx <= grid_size; dtheta_idx++) {
                TransformParams cand;
                cand.dx = current.dx + (dx_idx * adjusted_range / grid_size);
                cand.dy = current.dy + (dy_idx * adjusted_range / grid_size);
                cand.dtheta = current.dtheta + (dtheta_idx * adjusted_angle_range / grid_size);
                
                // 角度归一化
                cand.dtheta = fmod(cand.dtheta + M_PI, 2 * M_PI) - M_PI;
                
                // 【关键修复】绝对限制在允许范围内
                if (fabs(cand.dtheta) > MAX_ALLOWED_ROTATION) {
                    cand.dtheta = (cand.dtheta > 0 ? MAX_ALLOWED_ROTATION : -MAX_ALLOWED_ROTATION);
                }
                
                // 设置置信度
                float dist_factor = sqrt(dx_idx*dx_idx + dy_idx*dy_idx + dtheta_idx*dtheta_idx) / (grid_size*sqrt(3));
                cand.confidence = max(0.1f, 1.0f - dist_factor * 0.5f);
                
                candidates.push_back(cand);
            }
        }
    }
    
    ROS_DEBUG("生成 %lu 个候选变换 (范围因子:%.1f, 角度因子:%.1f, 最大旋转:%.1f°)", 
              candidates.size(), range_factor, angle_factor, MAX_ALLOWED_ROTATION * RAD_TO_DEG);
    
    return candidates;
}
// ======================== 对齐质量评估 ========================
 
AlignmentQuality IntelligentSubmapAligner::evaluateAlignmentQuality(
    const SubMapInfo& submap1,
    const SubMapInfo& submap2,
    const TransformParams& transform) {
    
    AlignmentQuality quality;
    
    // 1. 特征匹配得分
    quality.feature_match_score = computeFeatureMatchScore(
        *submap1.cloud, *submap2.cloud, transform);
    
    // 2. 提取几何特征并评估一致性
    auto features1 = extractGeometricFeatures(*submap1.cloud);
    auto features2 = extractGeometricFeatures(*submap2.cloud);
    quality.geometric_consistency = evaluateGeometricConsistency(
        features1, features2, transform);
    
    // 3. 评估墙壁连续性
    PointCloudPtr merged = mergePointClouds(submap1, submap2, transform);
    quality.wall_continuity = evaluateWallContinuity(*merged);
    
    // ==========【1】交叉验证 ==========
    // 如果特征匹配差，几何一致性和墙壁连续性不可能高
    if (quality.feature_match_score < 0.3f) {
        // 特征匹配差，限制其他分数
        float feature_factor = quality.feature_match_score / 0.3f;
        
        quality.geometric_consistency = min(
            quality.geometric_consistency,
            feature_factor * 1.2f);  // 最高比特征匹配高20%
        
        quality.wall_continuity = min(
            quality.wall_continuity,
            feature_factor * 1.1f);  // 最高比特征匹配高10%
        
        ROS_DEBUG("特征匹配差(%.1f%%)，限制几何一致性<%.1f%%，墙壁连续性<%.1f%%",
                 quality.feature_match_score * 100,
                 quality.geometric_consistency * 100,
                 quality.wall_continuity * 100);
    }
    
    // ==========【2】ODOM旋转约束 ==========
    float odom_rotation = submap2.robot_pose.theta - submap1.robot_pose.theta;
    float rotation_error = fabs(transform.dtheta - odom_rotation);
    
    if (rotation_error > 10.0f * DEG_TO_RAD) {
        // 旋转偏离ODOM太远，严重惩罚
        float rotation_penalty = 1.0f / (1.0f + rotation_error / (10.0f * DEG_TO_RAD));
        quality.geometric_consistency *= rotation_penalty;
        quality.wall_continuity *= rotation_penalty;
        
        ROS_WARN("旋转偏离ODOM过大(%.1f°)，应用惩罚因子%.2f",
                rotation_error * RAD_TO_DEG, rotation_penalty);
    }
    
    // 4. 矩阵指标（保留原有计算，但在交叉验证后）
    MatrixXf J = computeJacobianMatrix(*submap1.cloud, *submap2.cloud, transform);
    VectorXf residuals = computeResiduals(*submap1.cloud, *submap2.cloud, transform);
    
    Matrix3f covariance = computeCovarianceMatrix(J, residuals);
    quality.condition_number = covariance.norm() * covariance.inverse().norm();
    quality.uncertainty_volume = sqrt(covariance.determinant());
    
    Matrix3f information = computeInformationMatrix(covariance);
    quality.information_gain = information.norm();
    
    // ==========【3】计算总分（调整权重） ==========
    // 增加特征匹配的权重，降低不可靠分数的权重
    quality.total_score = 
        0.5f * quality.feature_match_score +      // 特征匹配50%
        0.3f * quality.geometric_consistency +    // 几何一致性30%
        0.2f * quality.wall_continuity;           // 墙壁连续性20%
    
    // ==========【4】有效性判定（更加合理） ==========
    // 必须同时满足多个条件
    bool is_geometrically_valid = (quality.geometric_consistency > 0.4f && 
                                 quality.wall_continuity > 0.4f);  // 降低阈值
    
    bool is_feature_valid = (quality.feature_match_score > 0.2f);  // 提高阈值
    
    bool is_overall_valid = (quality.total_score >= config_.min_valid_score);
    
    bool is_rotation_valid = (rotation_error < 15.0f * DEG_TO_RAD);  // 旋转不能偏离ODOM太多
    
    // 必须满足所有条件
    quality.is_valid = is_overall_valid && 
                      (is_geometrically_valid || is_feature_valid) &&  // 几何或特征有效
                      is_rotation_valid;                                // 旋转有效
    
    // ==========【5】质量等级（基于交叉验证后的分数） ==========
    if (quality.total_score >= 0.85f && quality.feature_match_score >= 0.4f) {
        quality.level = AlignmentQuality::EXCELLENT;
    } else if (quality.total_score >= 0.7f && quality.feature_match_score >= 0.3f) {
        quality.level = AlignmentQuality::GOOD;
    } else if (quality.total_score >= 0.5f && quality.feature_match_score >= 0.2f) {
        quality.level = AlignmentQuality::FAIR;
    } else {
        quality.level = AlignmentQuality::POOR;
    }
    
    // ==========【6】诊断信息（更加准确） ==========
    if (!quality.is_valid || quality.level == AlignmentQuality::POOR) {
        // 检查主要问题
        if (quality.feature_match_score < 0.2f) {
            quality.dominant_issue = "特征匹配严重不足";
            quality.suggestion = "检查重叠区域，可能需要重新扫描";
        } else if (rotation_error > 10.0f * DEG_TO_RAD) {
            quality.dominant_issue = "旋转偏离ODOM过大";
            quality.suggestion = "检查odom数据，或强制使用odom旋转";
        } else if (quality.geometric_consistency < 0.4f) {
            quality.dominant_issue = "几何不一致";
            quality.suggestion = "墙壁可能未对齐，检查旋转角度";
        } else if (quality.wall_continuity < 0.4f) {
            quality.dominant_issue = "墙壁断裂严重";
            quality.suggestion = "可能有多径反射，尝试调整滤波参数";
        } else if (quality.total_score < config_.min_valid_score) {
            quality.dominant_issue = "综合评分过低";
            quality.suggestion = "可能需要手动调整参数";
        } else {
            quality.dominant_issue = "未知问题";
            quality.suggestion = "检查各项指标详细日志";
        }
        
        // 添加详细诊断信息
        ROS_WARN("对齐质量诊断：特征=%.1f%%，几何=%.1f%%，墙壁=%.1f%%，总分=%.1f%%，旋转误差=%.1f°",
                quality.feature_match_score * 100,
                quality.geometric_consistency * 100,
                quality.wall_continuity * 100,
                quality.total_score * 100,
                rotation_error * RAD_TO_DEG);
    }
    
    return quality;
}
// ======================== 高斯-牛顿法优化 ======================== 

TransformParams IntelligentSubmapAligner::optimizeWithGaussNewton(
    const SubMapInfo& submap1,
    const SubMapInfo& submap2,
    const TransformParams& initial) {
    
    TransformParams optimized = initial;
    
    // 获取ODOM旋转信息
    float odom_rotation = submap2.robot_pose.theta - submap1.robot_pose.theta;
    
    // ========== 基于ODOM决定是否优化旋转 ==========
    bool optimize_rotation = true;
    
    // 如果ODOM旋转很小，考虑不优化旋转
    if (fabs(odom_rotation) < 2.0f * DEG_TO_RAD) {
        // ODOM说基本没旋转
        if (fabs(initial.dtheta) < 3.0f * DEG_TO_RAD) {
            // 初始猜测也认为旋转小，固定旋转为0
            optimize_rotation = false;
            ROS_INFO("ODOM(%.1f°)和初始猜测(%.1f°)都认为旋转小，固定旋转为0",
                    odom_rotation * RAD_TO_DEG, initial.dtheta * RAD_TO_DEG);
        }
    }
    
    // 高斯-牛顿迭代
    for (int gn_iter = 0; gn_iter < 10; gn_iter++) {
        // 计算雅可比和残差
        MatrixXf J = computeJacobianMatrix(*submap1.cloud, *submap2.cloud, optimized);
        VectorXf residuals = computeResiduals(*submap1.cloud, *submap2.cloud, optimized);
        
        // 如果不优化旋转，修改雅可比矩阵
        if (!optimize_rotation) {
            // 创建一个只包含x,y梯度的雅可比
            MatrixXf J_translation_only(J.rows(), 2);
            for (int i = 0; i < J.rows(); i++) {
                J_translation_only(i, 0) = J(i, 0);  // ∂e/∂x
                J_translation_only(i, 1) = J(i, 1);  // ∂e/∂y
                // 忽略 ∂e/∂θ
            }
            J = J_translation_only;
        }
        
        // 计算海塞矩阵
        MatrixXf H;
        if (optimize_rotation) {
            H = J.transpose() * J;
        } else {
            // 2x2海塞矩阵（只包含x,y）
            if (J.cols() == 2) {
                Matrix2f H_2x2 = J.transpose() * J;
                H = Matrix3f::Zero();
                H.block<2,2>(0,0) = H_2x2;
                H(2,2) = 1.0f;  // 旋转分量设为固定值
            } else {
                // 安全回退
                H = J.transpose() * J;
                if (H.rows() < 3) {
                    MatrixXf H_temp = MatrixXf::Zero(3, 3);
                    H_temp.block(0, 0, H.rows(), H.cols()) = H;
                    H_temp(2, 2) = 1.0f;
                    H = H_temp;
                }
            }
        }
        
        // 添加阻尼防止奇异
        float damping = config_.damping_factor * (gn_iter + 1);
        H += damping * MatrixXf::Identity(H.rows(), H.cols());
        
        // 计算梯度
        VectorXf g = J.transpose() * residuals;
        
        // 扩展g的维度以匹配H（如果不优化旋转）
        if (!optimize_rotation && g.size() == 2) {
            VectorXf g_temp = VectorXf::Zero(3);
            g_temp.segment(0, 2) = g;
            g = g_temp;
        }
        
        // 计算更新量
        VectorXf delta = -H.inverse() * g;
        
        // 记录更新前的参数用于收敛检查
        TransformParams before_update = optimized;
        
        // 应用更新
        optimized.dx += delta(0);
        optimized.dy += delta(1);
        
        // 检查delta维度
        if (optimize_rotation && delta.size() > 2) {
            float rotation_update = delta(2);
            optimized.dtheta += rotation_update;
            
            // ========== 旋转限制 ==========
            // 1. 限制单次迭代旋转变化
            const float MAX_SINGLE_ROTATION = 5.0f * DEG_TO_RAD;
            if (fabs(rotation_update) > MAX_SINGLE_ROTATION) {
                ROS_WARN("迭代%d：旋转变化过大(%.1f°)，限制为5°", 
                        gn_iter, rotation_update * RAD_TO_DEG);
                optimized.dtheta = before_update.dtheta + 
                                 (rotation_update > 0 ? MAX_SINGLE_ROTATION : -MAX_SINGLE_ROTATION);
            }
            
            // 2. 限制总旋转量（相对odom）
            const float MAX_TOTAL_ROTATION = 10.0f * DEG_TO_RAD;
            float relative_to_odom = fabs(optimized.dtheta - odom_rotation);
            if (relative_to_odom > MAX_TOTAL_ROTATION) {
                ROS_WARN("迭代%d：相对ODOM旋转过大(%.1f°)，限制为10°", 
                        gn_iter, relative_to_odom * RAD_TO_DEG);
                // 保持在odom_rotation ±10°内
                if (optimized.dtheta > odom_rotation) {
                    optimized.dtheta = odom_rotation + MAX_TOTAL_ROTATION;
                } else {
                    optimized.dtheta = odom_rotation - MAX_TOTAL_ROTATION;
                }
            }
            
            // 3. 如果ODOM旋转很小，进一步限制
            if (fabs(odom_rotation) < 3.0f * DEG_TO_RAD) {
                const float MAX_SMALL_ODOM_ROTATION = 3.0f * DEG_TO_RAD;
                if (fabs(optimized.dtheta) > MAX_SMALL_ODOM_ROTATION) {
                    ROS_WARN("迭代%d：ODOM旋转小但优化旋转大(%.1f°)，限制为3°", 
                            gn_iter, optimized.dtheta * RAD_TO_DEG);
                    optimized.dtheta = (optimized.dtheta > 0 ? 
                                       MAX_SMALL_ODOM_ROTATION : -MAX_SMALL_ODOM_ROTATION);
                }
            }
        }
        
        // ========== 收敛检查 ==========
        float dx_change = fabs(optimized.dx - before_update.dx);
        float dy_change = fabs(optimized.dy - before_update.dy);
        float dtheta_change = optimize_rotation ? 
                            fabs(optimized.dtheta - before_update.dtheta) : 0.0f;
        
        // 检查参数变化是否很小
        if (dx_change < 0.001f && dy_change < 0.001f && dtheta_change < 0.1f * DEG_TO_RAD) {
            ROS_DEBUG("高斯-牛顿迭代%d收敛：变化很小", gn_iter);
            break;
        }
        
        // 检查残差是否减小
        float new_residual_norm = residuals.norm();
        static float prev_residual_norm = new_residual_norm;
        if (gn_iter > 0 && new_residual_norm > prev_residual_norm * 0.95f) {
            // 残差减少不明显
            ROS_DEBUG("高斯-牛顿迭代%d：残差减少不明显，停止", gn_iter);
            break;
        }
        prev_residual_norm = new_residual_norm;
    }
    
    // 最终旋转处理
    if (!optimize_rotation) {
        optimized.dtheta = 0.0f;
    }
    
    // 角度归一化
    optimized.dtheta = fmod(optimized.dtheta + M_PI, 2 * M_PI) - M_PI;
    
    return optimized;
}

// ======================== 矩阵计算函数 ========================

MatrixXf IntelligentSubmapAligner::computeJacobianMatrix(
    const PointCloud& source,
    const PointCloud& target,
    const TransformParams& params) {
    
    // 构建变换矩阵
    Matrix4f T = Matrix4f::Identity();
    T(0, 0) = cos(params.dtheta);
    T(0, 1) = -sin(params.dtheta);
    T(1, 0) = sin(params.dtheta);
    T(1, 1) = cos(params.dtheta);
    T(0, 3) = params.dx;
    T(1, 3) = params.dy;
    
    // 寻找对应点并计算雅可比
    vector<Vector3f> residuals;
    vector<Matrix<float, 3, 3>> jacobians;
    
    kdtree_->setInputCloud(target.makeShared());
    
    int valid_correspondences = 0;
    const float MAX_DIST = 0.5f;  // 最大对应距离
    
    for (const auto& p_src : source.points) {
        if (p_src.intensity < 120.0f) continue;  // 只使用高质量点
        
        // 变换源点
        Vector4f p_src_homo(p_src.x, p_src.y, p_src.z, 1.0f);
        Vector4f p_transformed_homo = T * p_src_homo;
        Vector3f p_transformed = p_transformed_homo.head<3>();
        
        // 寻找最近的目标点
        vector<int> indices(1);
        vector<float> distances(1);
        
        PointType search_point;
        search_point.x = p_transformed(0);
        search_point.y = p_transformed(1);
        search_point.z = p_transformed(2);
        
        if (kdtree_->nearestKSearch(search_point, 1, indices, distances) > 0) {
            if (distances[0] < MAX_DIST * MAX_DIST) {
                const auto& p_tgt = target.points[indices[0]];
                Vector3f p_target(p_tgt.x, p_tgt.y, p_tgt.z);
                
                // ************** ：残差检查 **************
                Vector3f e = p_transformed - p_target;
                float residual_norm = e.norm();
                
                // 跳过过大的残差（可能是错误匹配）
                if (residual_norm > 2.0f * MAX_DIST) {
                    continue;
                }
                
                // 跳过数值异常的残差
                if (!isfinite(e(0)) || !isfinite(e(1))) {
                    continue;
                }
                // ************** 残差检查结束 **************
                
                residuals.push_back(e);
                
                // 雅可比矩阵
                Matrix<float, 3, 3> J_i = Matrix<float, 3, 3>::Zero();
                J_i(0, 0) = 1.0f;  // ∂e_x/∂x
                J_i(1, 1) = 1.0f;  // ∂e_y/∂y
                J_i(0, 2) = -p_transformed.y();  // ∂e_x/∂θ
                J_i(1, 2) = p_transformed.x();   // ∂e_y/∂θ
                
                // ************** ：雅可比数值检查 **************
                // 检查雅可比是否过大
                bool jacobian_valid = true;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        if (fabs(J_i(i, j)) > 100.0f) {
                            jacobian_valid = false;
                            break;
                        }
                    }
                    if (!jacobian_valid) break;
                }
                
                if (jacobian_valid) {
                    jacobians.push_back(J_i);
                    valid_correspondences++;
                }
                // ************** 雅可比检查结束 **************
            }
        }
    }
    
    // ************** ：充分性检查 **************
    if (valid_correspondences < 10) {
        ROS_WARN("有效对应点不足: %d个 (至少需要10个)", valid_correspondences);
        
        // 返回一个安全的雅可比矩阵（单位矩阵的缩放版）
        MatrixXf J_safe(6, 3);
        J_safe.setZero();
        J_safe(0, 0) = 0.1f;  // 较小的梯度，防止大幅更新
        J_safe(1, 1) = 0.1f;
        J_safe(2, 2) = 0.1f;
        return J_safe;
    }
    // ************** 充分性检查结束 **************
    
    // 组装整体雅可比矩阵
    MatrixXf J(2 * valid_correspondences, 3);
    for (int i = 0; i < valid_correspondences; i++) {
        J.block<2, 3>(2*i, 0) = jacobians[i].block<2, 3>(0, 0);
    }
    
    return J;
}

/****
MatrixXf IntelligentSubmapAligner::computeJacobianMatrix(
    const PointCloud& source,
    const PointCloud& target,
    const TransformParams& params) {
    
    // 构建变换矩阵
    Matrix4f T = Matrix4f::Identity();
    T(0, 0) = cos(params.dtheta);
    T(0, 1) = -sin(params.dtheta);
    T(1, 0) = sin(params.dtheta);
    T(1, 1) = cos(params.dtheta);
    T(0, 3) = params.dx;
    T(1, 3) = params.dy;
    
    // 寻找对应点并计算雅可比
    vector<Vector3f> residuals;
    vector<Matrix<float, 3, 3>> jacobians;
    
    kdtree_->setInputCloud(target.makeShared());
    
    int valid_correspondences = 0;
    const float MAX_DIST = 0.5f;  // 最大对应距离
    
    for (const auto& p_src : source.points) {
        if (p_src.intensity < 120.0f) continue;  // 只使用高质量点
        
        // 变换源点
        Vector4f p_src_homo(p_src.x, p_src.y, p_src.z, 1.0f);
        Vector4f p_transformed_homo = T * p_src_homo;
        Vector3f p_transformed = p_transformed_homo.head<3>();
        
        // 寻找最近的目标点
        vector<int> indices(1);
        vector<float> distances(1);
        
        PointType search_point;
        search_point.x = p_transformed(0);
        search_point.y = p_transformed(1);
        search_point.z = p_transformed(2);
        
        if (kdtree_->nearestKSearch(search_point, 1, indices, distances) > 0) {
            if (distances[0] < MAX_DIST * MAX_DIST) {
                const auto& p_tgt = target.points[indices[0]];
                Vector3f p_target(p_tgt.x, p_tgt.y, p_tgt.z);
                
                // 残差
                Vector3f e = p_transformed - p_target;
                residuals.push_back(e);
                
                // 雅可比矩阵
                Matrix<float, 3, 3> J_i = Matrix<float, 3, 3>::Zero();
                J_i(0, 0) = 1.0f;  // ∂e_x/∂x
                J_i(1, 1) = 1.0f;  // ∂e_y/∂y
                J_i(0, 2) = -p_transformed.y();  // ∂e_x/∂θ
                J_i(1, 2) = p_transformed.x();   // ∂e_y/∂θ
                
                jacobians.push_back(J_i);
                valid_correspondences++;
            }
        }
    }
    
    if (valid_correspondences < 10) {
        // 返回一个安全的雅可比矩阵
        MatrixXf J_safe(6, 3);
        J_safe.setZero();
        J_safe(0, 0) = 1.0f;
        J_safe(1, 1) = 1.0f;
        J_safe(2, 2) = 1.0f;
        return J_safe;
    }
    
    // 组装整体雅可比矩阵
    MatrixXf J(2 * valid_correspondences, 3);
    for (int i = 0; i < valid_correspondences; i++) {
        J.block<2, 3>(2*i, 0) = jacobians[i].block<2, 3>(0, 0);
    }
    
    return J;
}

Matrix3f IntelligentSubmapAligner::computeCovarianceMatrix(
    const MatrixXf& J,
    const VectorXf& residuals) {
    
    // 计算残差方差
    float sigma2 = residuals.squaredNorm() / max(1.0f, static_cast<float>(residuals.size() - 3));
    sigma2 = max(sigma2, 0.001f);  // 防止除零
    
    // 海塞矩阵
    Matrix3f H = J.transpose() * J;
    
    // 添加正则化防止奇异
    H += config_.regularization * Matrix3f::Identity();
    
    // 协方差矩阵
    Matrix3f covariance = sigma2 * H.inverse();
    
    return covariance;
}
***/


Matrix3f IntelligentSubmapAligner::computeCovarianceMatrix(
    const MatrixXf& J,
    const VectorXf& residuals) {
    
    // 计算残差方差
    float sigma2 = residuals.squaredNorm() / max(1.0f, static_cast<float>(residuals.size() - 3));
    sigma2 = max(sigma2, 0.001f);  // 防止除零
    
    // 海塞矩阵
    Matrix3f H = J.transpose() * J;
    
    // ************** ：海塞矩阵安全检查 **************
    // 检查是否奇异
    float det = H.determinant();
    bool is_singular = fabs(det) < 1e-6f;
    
    // 检查条件数
    Eigen::JacobiSVD<Matrix3f> svd(H);
    float cond = svd.singularValues()(0) / svd.singularValues()(2);
    bool ill_conditioned = cond > 1e6f;
    
    if (is_singular || ill_conditioned) {
        ROS_WARN("Hessian矩阵奇异(行列式=%.2e)或病态(条件数=%.2e)，添加正则化", 
                 det, cond);
        
        // 自适应正则化强度
        float lambda = 0.0f;
        if (is_singular) lambda = 0.1f;
        else if (ill_conditioned) lambda = 0.01f * cond / 1e6f;
        
        H += lambda * Matrix3f::Identity();
        
        // 再次检查
        det = H.determinant();
        if (fabs(det) < 1e-6f) {
            ROS_ERROR("Hessian矩阵仍奇异，使用单位矩阵");
            return Matrix3f::Identity() * 0.5f;
        }
    }
    // ************** 安全检查结束 **************
    
    // 添加正则化防止奇异
    H += config_.regularization * Matrix3f::Identity();
    
    // 协方差矩阵
    Matrix3f covariance = sigma2 * H.inverse();
    
    // ************** ：协方差合理性检查 **************
    // 检查对角线元素是否为负或过大
    bool covariance_valid = true;
    for (int i = 0; i < 3; i++) {
        if (covariance(i, i) < 0) {
            ROS_WARN("协方差对角线元素[%d]为负: %.6f", i, covariance(i, i));
            covariance_valid = false;
        } else if (covariance(i, i) > 100.0f) {
            ROS_WARN("协方差对角线元素[%d]过大: %.6f", i, covariance(i, i));
            covariance_valid = false;
        }
    }
    
    if (!covariance_valid) {
        ROS_WARN("协方差矩阵不合理，进行缩放修正");
        // 限制对角线元素范围
        for (int i = 0; i < 3; i++) {
            if (covariance(i, i) < 0) covariance(i, i) = 0.01f;
            if (covariance(i, i) > 100.0f) covariance(i, i) = 1.0f;
        }
    }
    // ************** 合理性检查结束 **************
    
    return covariance;
}

Matrix3f IntelligentSubmapAligner::computeInformationMatrix(
    const Matrix3f& covariance) {
    
    // 信息矩阵是协方差矩阵的逆
    Matrix3f information = covariance.inverse();
    
    // 添加小量防止数值问题
    information += 0.001f * Matrix3f::Identity();
    
    return information;
}

VectorXf IntelligentSubmapAligner::computeResiduals(
    const PointCloud& source,
    const PointCloud& target,
    const TransformParams& params) {
    
    vector<float> residuals_list;
    
    // 构建变换矩阵
    Matrix4f T = Matrix4f::Identity();
    T(0, 0) = cos(params.dtheta);
    T(0, 1) = -sin(params.dtheta);
    T(1, 0) = sin(params.dtheta);
    T(1, 1) = cos(params.dtheta);
    T(0, 3) = params.dx;
    T(1, 3) = params.dy;
    
    kdtree_->setInputCloud(target.makeShared());
    
    for (const auto& p_src : source.points) {
        if (p_src.intensity < 120.0f) continue;
        
        // 变换源点
        Vector4f p_src_homo(p_src.x, p_src.y, p_src.z, 1.0f);
        Vector4f p_transformed_homo = T * p_src_homo;
        Vector3f p_transformed = p_transformed_homo.head<3>();
        
        // 寻找最近点
        vector<int> indices(1);
        vector<float> distances(1);
        
        PointType search_point;
        search_point.x = p_transformed(0);
        search_point.y = p_transformed(1);
        search_point.z = p_transformed(2);
        
        if (kdtree_->nearestKSearch(search_point, 1, indices, distances) > 0) {
            const auto& p_tgt = target.points[indices[0]];
            Vector3f p_target(p_tgt.x, p_tgt.y, p_tgt.z);
            
            // 残差（只取x,y）
            Vector2f e = (p_transformed - p_target).head<2>();
            residuals_list.push_back(e(0));
            residuals_list.push_back(e(1));
        }
    }
    
    // 转换为Eigen向量
    VectorXf residuals(residuals_list.size());
    for (size_t i = 0; i < residuals_list.size(); i++) {
        residuals(i) = residuals_list[i];
    }
    
    return residuals;
}

// ======================== ICP细化 ========================

TransformParams IntelligentSubmapAligner::refineWithICP(
    const PointCloudPtr& source,
    const PointCloudPtr& target,
    const TransformParams& initial) {
    
    TransformParams refined = initial;
    
    if (source->empty() || target->empty()) {
        return refined;
    }
    
    // 构建初始变换矩阵
    Matrix4f initial_guess = Matrix4f::Identity();
    initial_guess(0, 0) = cos(initial.dtheta);
    initial_guess(0, 1) = -sin(initial.dtheta);
    initial_guess(1, 0) = sin(initial.dtheta);
    initial_guess(1, 1) = cos(initial.dtheta);
    initial_guess(0, 3) = initial.dx;
    initial_guess(1, 3) = initial.dy;
    
    // 执行ICP
    icp_->setInputSource(source);
    icp_->setInputTarget(target);
    
    PointCloud aligned;
    icp_->align(aligned, initial_guess);
    
    if (icp_->hasConverged()) {
        Matrix4f final_transform = icp_->getFinalTransformation();
        
        // 提取变换参数
        refined.dx = final_transform(0, 3);
        refined.dy = final_transform(1, 3);
        refined.dtheta = atan2(final_transform(1, 0), final_transform(0, 0));
        refined.confidence = min(1.0f, refined.confidence + 0.2f);
    }
    
    return refined;
}

// ======================== 几何特征处理 ========================

vector<GeometricFeature> IntelligentSubmapAligner::extractGeometricFeatures(
    const PointCloud& cloud) {
    
    vector<GeometricFeature> features;
    
    // 这里实现几何特征提取
    // 简化的实现：按角度分区提取主方向
    
    const int ANGLE_BINS = 8;  // 8个方向
    vector<vector<PointType>> direction_points(ANGLE_BINS);
    
    for (const auto& p : cloud.points) {
        if (p.intensity >= 160.0f) {  // 高质量墙壁点
            float angle = atan2(p.y, p.x);
            if (angle < 0) angle += 2 * M_PI;
            
            int bin = static_cast<int>(angle / (2 * M_PI / ANGLE_BINS));
            bin = max(0, min(ANGLE_BINS - 1, bin));
            
            direction_points[bin].push_back(p);
        }
    }
    
    // 为每个有足够点的方向创建特征
    for (int i = 0; i < ANGLE_BINS; i++) {
        if (direction_points[i].size() >= 10) {
            GeometricFeature feature;
            feature.type = GeometricFeature::STRAIGHT_WALL;  // 简化假设
            feature.points = direction_points[i];
            feature.orientation = i * (2 * M_PI / ANGLE_BINS);
            feature.is_reliable = true;
            
            // 简单计算特征属性
            if (direction_points[i].size() >= 5) {
                // 计算近似直线
                feature = fitLineToGeometricFeature(feature);
            }
            
            features.push_back(feature);
        }
    }
    
    return features;
}

float IntelligentSubmapAligner::evaluateGeometricConsistency(
    const vector<GeometricFeature>& features1,
    const vector<GeometricFeature>& features2,
    const TransformParams& transform) {
    
    if (features1.empty() || features2.empty()) {
        return 0.3f;  // 数据不足，返回中等分数
    }
    
    // ==========【1】智能旋转检查 ==========
    float rotation_penalty = 1.0f;
    
    // 检查旋转是否合理
    if (fabs(transform.dtheta) > 10.0f * DEG_TO_RAD) {
        // 旋转大，需要检查是否有对应的几何依据
        bool has_matching_direction = false;
        
        for (const auto& f1 : features1) {
            for (const auto& f2 : features2) {
                float angle1 = f1.orientation;
                float angle2 = f2.orientation + transform.dtheta;
                
                // 角度归一化
                while (angle1 > M_PI) angle1 -= 2 * M_PI;
                while (angle1 < -M_PI) angle1 += 2 * M_PI;
                while (angle2 > M_PI) angle2 -= 2 * M_PI;
                while (angle2 < -M_PI) angle2 += 2 * M_PI;
                
                float angle_diff = fabs(angle1 - angle2);
                if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
                
                if (angle_diff < 10.0f * DEG_TO_RAD) {
                    has_matching_direction = true;
                    break;
                }
            }
            if (has_matching_direction) break;
        }
        
        // 如果没有匹配的方向，旋转可能不合理
        if (!has_matching_direction) {
            rotation_penalty = 0.5f;  // 严重惩罚
            ROS_WARN("旋转%.1f°没有匹配的几何特征，应用惩罚", 
                    transform.dtheta * RAD_TO_DEG);
        } else {
            // 有匹配方向，轻微惩罚（因为毕竟有较大旋转）
            rotation_penalty = 0.8f;
            ROS_DEBUG("旋转%.1f°有几何依据，轻微惩罚", transform.dtheta * RAD_TO_DEG);
        }
    } 
    else if (fabs(transform.dtheta) > 5.0f * DEG_TO_RAD) {
        // 中等旋转，轻微惩罚
        rotation_penalty = 0.9f;
    }
    // 小于5°不惩罚
    
    // ==========【2】计算方向一致性 ==========
    float direction_consistency = 0.0f;
    int matched_pairs = 0;
    
    // 水池墙壁应该平行，方向差异应该小
    for (const auto& f1 : features1) {
        for (const auto& f2 : features2) {
            float angle1 = f1.orientation;
            float angle2 = f2.orientation + transform.dtheta;
            
            // 角度归一化
            while (angle1 > M_PI) angle1 -= 2 * M_PI;
            while (angle1 < -M_PI) angle1 += 2 * M_PI;
            while (angle2 > M_PI) angle2 -= 2 * M_PI;
            while (angle2 < -M_PI) angle2 += 2 * M_PI;
            
            float angle_diff = fabs(angle1 - angle2);
            if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
            
            // 方向差异越小越好
            if (angle_diff < 20.0f * DEG_TO_RAD) {  // 20度以内
                float pair_consistency = 1.0f - (angle_diff / (20.0f * DEG_TO_RAD));
                direction_consistency += pair_consistency;
                matched_pairs++;
            }
        }
    }
    
    // 归一化
    float consistency = 0.3f;  // 基础分数
    if (matched_pairs > 0) {
        consistency = direction_consistency / matched_pairs;
    }
    
    // ==========【3】应用旋转惩罚 ==========
    consistency *= rotation_penalty;
    
    // ==========【4】添加安全上限和验证 ==========
    consistency = min(consistency, 0.95f);
    
    if (consistency > 0.8f && matched_pairs < 3) {
        ROS_WARN("几何一致性高(%.1f%%)但匹配对少(%d)，可能有问题", 
                consistency * 100, matched_pairs);
        consistency *= 0.7f;  // 降低分数
    }
    
    return consistency;
}

 
// 原函数位置：intelligent_submap_aligner.cpp
// 修改目标：防止返回100%这种不合理的值

float IntelligentSubmapAligner::evaluateWallContinuity(
    const PointCloud& merged_cloud) {
    
    // ==========【1】检查数据质量 ==========
    if (merged_cloud.empty()) {
        return 0.0f;
    }
    
    // 统计高质量墙壁点
    int high_quality_points = 0;
    for (const auto& p : merged_cloud.points) {
        if (p.intensity >= 160.0f) high_quality_points++;
    }
    
    // 点太少，连续性不可能高
    if (high_quality_points < 50) {
        return max(0.1f, high_quality_points / 500.0f);
    }
    
    // ==========【2】按角度分区 ==========
    const int ANGLE_BINS = 72;  // 5度一个bin
    vector<int> bin_counts(ANGLE_BINS, 0);
    
    for (const auto& p : merged_cloud.points) {
        if (p.intensity >= 160.0f) {  // 只考虑高质量墙壁点
            float angle = atan2(p.y, p.x);
            if (angle < 0) angle += 2 * M_PI;
            
            int bin = static_cast<int>(angle / (2 * M_PI / ANGLE_BINS));
            bin = max(0, min(ANGLE_BINS - 1, bin));
            bin_counts[bin]++;
        }
    }
    
    // ==========【3】计算连续区域 ==========
    int continuous_regions = 0;
    bool in_wall = false;
    int current_region_length = 0;
    int max_region_length = 0;
    
    for (int count : bin_counts) {
        if (count >= 2) {  // 至少有2个高质量点
            if (!in_wall) {
                in_wall = true;
                continuous_regions++;
            }
            current_region_length++;
            max_region_length = max(max_region_length, current_region_length);
        } else {
            in_wall = false;
            current_region_length = 0;
        }
    }
    
    // ==========【4】计算连续性分数 ==========
    // 1. 区域数量分数（理想4面墙）
    float region_score = 0.0f;
    if (continuous_regions >= 3 && continuous_regions <= 5) {
        region_score = 0.8f;  // 3-5个区域，不错
    } else if (continuous_regions >= 1 && continuous_regions <= 2) {
        region_score = 0.4f;  // 太少区域
    } else if (continuous_regions >= 6) {
        region_score = 0.3f;  // 太多区域（可能断裂）
    }
    
    // 2. 最长区域长度分数
    float length_score = min(1.0f, max_region_length / 10.0f);  // 最多10个bin长度得满分
    
    // 3. 点云密度分数
    float density_score = min(1.0f, high_quality_points / 1000.0f);
    
    // ==========【5】计算最终连续性 ==========
    float continuity = 
        0.4f * region_score + 
        0.3f * length_score + 
        0.3f * density_score;
    
    // ==========【6】添加安全上限 ==========
    // 墙壁连续性永远不会是100%！
    continuity = min(continuity, 0.95f);
    
    // ==========【7】验证检查 ==========
    if (continuity > 0.8f && high_quality_points < 200) {
        ROS_WARN("连续性高(%.1f%%)但高质量点少(%d)，可能高估", 
                continuity * 100, high_quality_points);
        continuity *= 0.8f;
    }
    
    return continuity;
}

// ======================== 工具函数 ========================

float IntelligentSubmapAligner::computeFeatureMatchScore(
    const PointCloud& cloud1,
    const PointCloud& cloud2,
    const TransformParams& transform) {
    
    // 构建变换矩阵
    Matrix4f T = Matrix4f::Identity();
    T(0, 0) = cos(transform.dtheta);
    T(0, 1) = -sin(transform.dtheta);
    T(1, 0) = sin(transform.dtheta);
    T(1, 1) = cos(transform.dtheta);
    T(0, 3) = transform.dx;
    T(1, 3) = transform.dy;
    
    // 统计匹配点对
    kdtree_->setInputCloud(cloud2.makeShared());
    
    int matches = 0;
    const float MATCH_THRESHOLD = 1.2f;  // 120cm
    
    for (const auto& p1 : cloud1.points) {
        if (p1.intensity < 120.0f) continue;
        
        // 变换点
        Vector4f p1_homo(p1.x, p1.y, p1.z, 1.0f);
        Vector4f p1_transformed_homo = T * p1_homo;
        
        PointType search_point;
        search_point.x = p1_transformed_homo(0);
        search_point.y = p1_transformed_homo(1);
        search_point.z = p1_transformed_homo(2);
        
        // 寻找最近点
        vector<int> indices(1);
        vector<float> distances(1);
        
        if (kdtree_->nearestKSearch(search_point, 1, indices, distances) > 0) {
            if (sqrt(distances[0]) < MATCH_THRESHOLD) {
                matches++;
            }
        }
    }
    
    // 计算匹配率
    float match_ratio = static_cast<float>(matches) / 
                       max(1.0f, static_cast<float>(cloud1.size()));
    
    return match_ratio;
}

float IntelligentSubmapAligner::computeTotalScore(const AlignmentQuality& quality) {
    float total_score = 
        config_.weight_feature_match * quality.feature_match_score +
        config_.weight_geometric * quality.geometric_consistency +
        config_.weight_continuity * quality.wall_continuity;
    
    // 矩阵指标作为调整因子
    if (quality.condition_number > 1000) {
        total_score *= 0.8f;  // 病态问题，降低分数
    }
    
    if (quality.uncertainty_volume < 0.01f) {
        total_score *= 1.1f;  // 不确定性小，提高分数
        total_score = min(total_score, 1.0f);
    }
    
    return total_score;
}

PointCloudPtr IntelligentSubmapAligner::mergePointClouds(
    const SubMapInfo& submap1,
    const SubMapInfo& submap2,
    const TransformParams& transform) {
    
    PointCloudPtr merged(new PointCloud);
    
    // 添加子图1
    *merged += *submap1.cloud;
    
    // 变换并添加子图2
    PointCloud transformed_cloud2;
    Matrix4f T = Matrix4f::Identity();
    T(0, 0) = cos(transform.dtheta);
    T(0, 1) = -sin(transform.dtheta);
    T(1, 0) = sin(transform.dtheta);
    T(1, 1) = cos(transform.dtheta);
    T(0, 3) = transform.dx;
    T(1, 3) = transform.dy;
    
    pcl::transformPointCloud(*submap2.cloud, transformed_cloud2, T);
    *merged += transformed_cloud2;
    
    return merged;
}

bool IntelligentSubmapAligner::checkConvergence(
    OptimizationState& state,
    const OptimizationState::Step& new_step) {
    
    // 条件1：达到目标分数 
    if (new_step.quality.total_score >= 0.85f) {  
        state.convergence_reason = "达到目标分数";
        return true;
    }
    
    // 条件2：连续改进很小（局部最优检查）
    if (state.history.size() >= 3) {
        bool all_small_improvements = true;
        for (size_t i = state.history.size() - 3; i < state.history.size(); i++) {
            if (fabs(state.history[i].improvement) > config_.min_score_improvement * 0.3f) {
                all_small_improvements = false;
                break;
            }
        }
        
        if (all_small_improvements) {
            // 检查是否尝试过不同策略
            if (state.strategy_change_count < 2) {
                // 不退出，而是标记需要换策略
                state.convergence_reason = "局部最优，建议切换策略";
                state.should_change_strategy = true;
                return false;  // 不退出，但要换策略
            } else {
                state.convergence_reason = "多种策略后仍无法改进";
                return true;  // 退出
            }
        }
    }
    
    // 条件3：参数变化很小
    if (state.history.size() >= 2) {
        const auto& prev = state.history[state.history.size() - 2];
        float dx_change = fabs(new_step.params.dx - prev.params.dx);
        float dy_change = fabs(new_step.params.dy - prev.params.dy);
        float dtheta_change = fabs(new_step.params.dtheta - prev.params.dtheta);
        
        // 放宽变化阈值
        if (dx_change < 0.02f && dy_change < 0.02f && dtheta_change < 1.0f * DEG_TO_RAD) {
            state.convergence_reason = "参数变化过小";
            return true;
        }
    }
    
    // 条件4：达到最大迭代
    if (state.iteration >= config_.max_iterations) {
        state.convergence_reason = "达到最大迭代";
        return true;
    }
    
    return false;
}

float IntelligentSubmapAligner::adjustSearchRange(
    const OptimizationState& state,
    const AlignmentQuality& quality) {
    
    float new_range = state.search_range;
    
    // 根据质量调整搜索范围
    if (quality.total_score < 0.3f) {
        // 质量很差，保持或增加搜索范围
        new_range = min(new_range * 1.2f, 0.5f);  // 最大0.5米
    } else if (quality.total_score < 0.6f) {
        // 质量一般，适当减小范围
        new_range *= 0.8f;
    } else {
        // 质量较好，大幅减小范围
        new_range *= 0.5f;
    }
    
    // 确保最小范围
    new_range = max(new_range, 0.05f);
    
    return new_range;
}

TransformParams IntelligentSubmapAligner::selectBestCandidate(
    const vector<TransformParams>& candidates,
    const vector<AlignmentQuality>& qualities) {
    
    if (candidates.empty()) {
        return TransformParams();  // 返回默认参数
    }
    
    // 选择分数最高的候选
    int best_index = 0;
    float best_score = -1.0f;
    
    for (size_t i = 0; i < qualities.size(); i++) {
        if (qualities[i].total_score > best_score) {
            best_score = qualities[i].total_score;
            best_index = i;
        }
    }
    
    return candidates[best_index];
}

 

GeometricFeature IntelligentSubmapAligner::fitLineToGeometricFeature(GeometricFeature& feature) {
    // 简化的直线拟合
    if (feature.points.size() < 2) {
        return feature;
    }
    
    // 计算质心
    float sum_x = 0, sum_y = 0;
    for (const auto& p : feature.points) {
        sum_x += p.x;
        sum_y += p.y;
    }
    float mean_x = sum_x / feature.points.size();
    float mean_y = sum_y / feature.points.size();
    
    // 计算直线参数（最小二乘）
    float sum_xy = 0, sum_xx = 0;
    for (const auto& p : feature.points) {
        float dx = p.x - mean_x;
        float dy = p.y - mean_y;
        sum_xy += dx * dy;
        sum_xx += dx * dx;
    }
    
    if (sum_xx > 0.001f) {
        feature.slope_k = sum_xy / sum_xx;
        feature.intercept_b = mean_y - feature.slope_k * mean_x;
        
        // 计算方向
        feature.orientation = atan2(1.0f, feature.slope_k);  // 垂直于斜率的向量
        
        // 计算长度（点云的范围）
        float min_x = FLT_MAX, max_x = -FLT_MAX;
        for (const auto& p : feature.points) {
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
        }
        feature.length = max_x - min_x;
    }
    
    feature.type = GeometricFeature::STRAIGHT_WALL;
    feature.is_reliable = (feature.points.size() >= 5);
    
    return feature;
}

// ======================== 报告生成 ========================

string IntelligentSubmapAligner::getOptimizationReport(const OptimizationResult& result) {
    stringstream ss;
    
    ss << "\n========================================\n";
    ss << "智能子图对齐优化报告\n";
    ss << "========================================\n";
    
    ss << "优化结果: " << (result.success ? "✅ 成功" : "❌ 失败") << "\n";
    ss << "使用迭代: " << result.iterations_used << " / " << config_.max_iterations << "\n";
    ss << "分数变化: " << fixed << setprecision(3) 
       << result.initial_score << " → " << result.final_score 
       << " (提升: " << result.improvement_ratio * 100 << "%)\n";
    
    ss << "\n最佳变换参数:\n";
    ss << "  ΔX: " << result.best_transform.dx << " 米\n";
    ss << "  ΔY: " << result.best_transform.dy << " 米\n";
    ss << "  Δθ: " << result.best_transform.dtheta * RAD_TO_DEG << " 度\n";
    ss << "  置信度: " << result.best_transform.confidence * 100 << "%\n";
    
    ss << "\n对齐质量:\n";
    ss << "  特征匹配: " << result.best_quality.feature_match_score * 100 << "%\n";
    ss << "  几何一致性: " << result.best_quality.geometric_consistency * 100 << "%\n";
    ss << "  墙壁连续性: " << result.best_quality.wall_continuity * 100 << "%\n";
    ss << "  综合评分: " << result.best_quality.total_score * 100 << "%\n";
    ss << "  质量等级: ";
    switch (result.best_quality.level) {
        case AlignmentQuality::EXCELLENT: ss << "优秀"; break;
        case AlignmentQuality::GOOD: ss << "良好"; break;
        case AlignmentQuality::FAIR: ss << "一般"; break;
        case AlignmentQuality::POOR: ss << "差"; break;
    }
    ss << "\n";
    
    ss << "\n矩阵指标:\n";
    ss << "  条件数: " << scientific << result.best_quality.condition_number << "\n";
    ss << "  不确定体积: " << result.best_quality.uncertainty_volume << "\n";
    ss << "  信息增益: " << result.best_quality.information_gain << "\n";
    
    ss << "\n优化日志:\n";
    for (const auto& log_entry : result.optimization_log) {
        ss << "  " << log_entry << "\n";
    }
    
    ss << "\n最终状态: " << result.final_status << "\n";
    ss << "========================================\n";
    
    return ss.str();
}

// ======================== 其他接口函数 ========================

void IntelligentSubmapAligner::reset() {
    optimization_log_.clear();
    feature_cache_.clear();
    ROS_INFO("优化器已重置");
}

void IntelligentSubmapAligner::saveLogToFile(const string& filename) {
    ofstream file(filename, ios::out | ios::app);
    if (file.is_open()) {
        for (const auto& log_entry : optimization_log_) {
            file << log_entry << "\n";
        }
        file.close();
    }
}

PointCloudPtr IntelligentSubmapAligner::transformPointCloud(
    const PointCloudPtr& cloud,
    const TransformParams& transform) {
    
    PointCloudPtr transformed(new PointCloud);
    if (!cloud || cloud->empty()) {
        return transformed;
    }
    
    // 构建变换矩阵
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 0) = cos(transform.dtheta);
    T(0, 1) = -sin(transform.dtheta);
    T(1, 0) = sin(transform.dtheta);
    T(1, 1) = cos(transform.dtheta);
    T(0, 3) = transform.dx;
    T(1, 3) = transform.dy;
    
    pcl::transformPointCloud(*cloud, *transformed, T);
    return transformed;
}

// ======================== 评估候选变换 ========================

AlignmentQuality IntelligentSubmapAligner::evaluateCandidateTransform(
    const SubMapInfo& submap1,
    const SubMapInfo& submap2,
    const TransformParams& candidate) {
    
    // 直接调用主评估函数
    AlignmentQuality quality = evaluateAlignmentQuality(submap1, submap2, candidate);
    
    // 可以添加额外的评估逻辑
    if (debug_enabled_) {
        ROS_DEBUG("候选变换评估: dx=%.3f, dy=%.3f, dtheta=%.1f°, 分数=%.3f",
                 candidate.dx, candidate.dy, candidate.dtheta * RAD_TO_DEG,
                 quality.total_score);
    }
    
    return quality;
}


// ======================== 完整水池几何特征检查实现 ========================

WallDetectionResult IntelligentSubmapAligner::detectWallInDirection(
    const PointCloud& cloud,
    float min_angle, float max_angle) {
    
    WallDetectionResult result;
    result.wall_exists = false;
    result.alignment_confidence = 0.0f;
    
    // 1. 收集角度范围内的高质量点
    std::vector<PointType> candidate_points;
    std::vector<float> distances;
    
    for (const auto& p : cloud.points) {
        if (p.intensity < 100.0f) continue;  // 基础质量阈值
        
        float angle = atan2(p.y, p.x);
        
        // 角度归一化处理
        if (min_angle < -M_PI && angle > 0) {
            angle -= 2 * M_PI;
        } else if (max_angle > M_PI && angle < 0) {
            angle += 2 * M_PI;
        }
        
        if (angle >= min_angle && angle <= max_angle) {
            candidate_points.push_back(p);
            distances.push_back(sqrt(p.x * p.x + p.y * p.y));
        }
    }
    
    if (candidate_points.size() < 8) {
        return result;  // 点太少，认为没有可靠墙壁
    }
    
    // 2. 统计分析和滤波
    // 计算距离统计
    float sum_dist = 0, sum_dist2 = 0;
    for (float d : distances) {
        sum_dist += d;
        sum_dist2 += d * d;
    }
    
    float mean_dist = sum_dist / distances.size();
    float std_dist = sqrt(sum_dist2 / distances.size() - mean_dist * mean_dist);
    
    // 3. 移除离群点（距离均值±2倍标准差之外）
    std::vector<PointType> filtered_points;
    std::vector<float> filtered_distances;
    
    for (size_t i = 0; i < candidate_points.size(); i++) {
        if (fabs(distances[i] - mean_dist) < 2.0f * std_dist) {
            filtered_points.push_back(candidate_points[i]);
            filtered_distances.push_back(distances[i]);
        }
    }
    
    if (filtered_points.size() < 5) {
        return result;
    }
    
    // 4. 更新统计量
    sum_dist = 0;
    for (float d : filtered_distances) sum_dist += d;
    result.average_distance = sum_dist / filtered_distances.size();
    result.wall_points = filtered_points;
    
    // 5. 拟合直线
    result.fitted_line = fitLineToPoints(filtered_points);
    
    // 6. 计算角度方差（平直度）
    float sum_angle_var = 0;
    for (const auto& p : filtered_points) {
        float point_angle = atan2(p.y, p.x);
        float line_angle = result.fitted_line.angle;
        float angle_diff = fabs(point_angle - line_angle);
        
        // 处理角度环绕
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        sum_angle_var += angle_diff * angle_diff;
    }
    
    result.angle_variance = sum_angle_var / filtered_points.size();
    
    // 7. 计算对齐置信度
    // 考虑因素：点数、角度方差、距离一致性
    float point_score = min(1.0f, filtered_points.size() / 20.0f);  // 最多20个点得满分
    float angle_score = exp(-result.angle_variance * 5.0f);  // 角度方差越小越好
    float dist_score = exp(-std_dist / mean_dist);  // 距离一致性
    
    result.alignment_confidence = 0.4f * point_score + 0.4f * angle_score + 0.2f * dist_score;
    result.alignment_confidence = max(0.0f, min(1.0f, result.alignment_confidence));
    
    result.wall_exists = (result.alignment_confidence > 0.3f);
    
    return result;
}

LineEquation IntelligentSubmapAligner::fitLineToPoints(const std::vector<PointType>& points) {
    
    LineEquation line;
    
    if (points.size() < 10) {  // 提高最小点数要求
        return line;
    }
    
    // 使用 RANSAC 或加权最小二乘
    // 只保留高质量点
    std::vector<PointType> high_quality_points;
    for (const auto& p : points) {
        if (p.intensity >= 160.0f) {  // 只使用高强度点
            high_quality_points.push_back(p);
        }
    }
    
    if (high_quality_points.size() < 8) {
        return line;
    }
    
    // 最小二乘拟合
    float sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    float sum_w = 0;
    
    for (const auto& p : high_quality_points) {
        float weight = p.intensity / 255.0f;
        sum_w += weight;
        sum_x += weight * p.x;
        sum_y += weight * p.y;
        sum_xx += weight * p.x * p.x;
        sum_xy += weight * p.x * p.y;
    }
    
    float mean_x = sum_x / sum_w;
    float mean_y = sum_y / sum_w;
    
    float cov_xx = sum_xx / sum_w - mean_x * mean_x;
    float cov_xy = sum_xy / sum_w - mean_x * mean_y;
    
    // 直线方向 = 最大特征值对应的特征向量
    if (fabs(cov_xx) > 1e-6) {
        float slope = cov_xy / cov_xx;
        line.a = slope;
        line.b = -1.0f;
        
        // 归一化
        float norm = sqrt(line.a * line.a + line.b * line.b);
        line.a /= norm;
        line.b /= norm;
        
        line.c = -(line.a * mean_x + line.b * mean_y);
        line.angle = atan2(line.b, line.a);
        
        // 归一化角度到 [0, π)
        if (line.angle < 0) line.angle += M_PI;
        if (line.angle >= M_PI) line.angle -= M_PI;
    }
    
    return line;
}

FourWallAlignment IntelligentSubmapAligner::checkFourWallAlignment(
    const PointCloud& merged_cloud) {
    
    FourWallAlignment alignment;
    
    // 1. 检测四个方向的墙壁
 
    alignment.x_pos_wall = detectXPosWall(merged_cloud);   
    alignment.x_neg_wall = detectXNegWall(merged_cloud);   
    alignment.y_pos_wall = detectYPosWall(merged_cloud);  
    alignment.y_neg_wall = detectYNegWall(merged_cloud);   
    
    // 2. 检查平行性
    alignment.parallelism = checkWallParallelism(alignment);
    
    // 3. 检查角落对齐
    alignment.corners = checkCornerAlignment(merged_cloud);
    
    // 4. 计算综合得分
    alignment.overall_score = computeOverallAlignmentScore(alignment);
    
    // 5. 生成诊断信息
    generateAlignmentDiagnostics(alignment);
    
    return alignment;
}

WallParallelism IntelligentSubmapAligner::checkWallParallelism(const FourWallAlignment& alignment) {
    WallParallelism parallelism;
    
    // X方向墙壁平行性 
    if (alignment.x_pos_wall.wall_exists && alignment.x_neg_wall.wall_exists) {
        float angle_diff = fabs(alignment.x_pos_wall.fitted_line.angle - 
                               alignment.x_neg_wall.fitted_line.angle);
        
        // 处理角度环绕
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        
        parallelism.x_angle_diff = angle_diff * RAD_TO_DEG;
        parallelism.x_parallel = (parallelism.x_angle_diff < 10.0f);
        
        // 计算距离一致性
        float dist_diff = fabs(alignment.x_pos_wall.average_distance - 
                              alignment.x_neg_wall.average_distance);
        parallelism.x_distance_diff = dist_diff;
    }
    
    // Y方向墙壁平行性 
    if (alignment.y_pos_wall.wall_exists && alignment.y_neg_wall.wall_exists) {
        float angle_diff = fabs(alignment.y_pos_wall.fitted_line.angle - 
                               alignment.y_neg_wall.fitted_line.angle);
        
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        
        parallelism.y_angle_diff = angle_diff * RAD_TO_DEG;
        parallelism.y_parallel = (parallelism.y_angle_diff < 10.0f);
        
        float dist_diff = fabs(alignment.y_pos_wall.average_distance - 
                              alignment.y_neg_wall.average_distance);
        parallelism.y_distance_diff = dist_diff;
    }
    
    // 矩形一致性检查
    parallelism.rectangles_consistent = 
        parallelism.x_parallel && 
        parallelism.y_parallel &&
        alignment.x_pos_wall.wall_exists &&
        alignment.x_neg_wall.wall_exists &&
        alignment.y_pos_wall.wall_exists &&
        alignment.y_neg_wall.wall_exists;
    
    return parallelism;
}

CornerAlignment IntelligentSubmapAligner::checkCornerAlignment(const PointCloud& cloud) {
    CornerAlignment corner_alignment;
    
    // 1. 检测角落点
    std::vector<PointType> corner_points = detectCorners(cloud);
    
    if (corner_points.size() < 2) {
        return corner_alignment;  // 角落太少
    }
    
    // 2. 将角落点分类到四个象限
    std::vector<CornerAlignment::Corner> classified_corners;
    
    for (const auto& point : corner_points) {
        CornerAlignment::Corner corner;
        corner.position = point;
        corner.confidence = point.intensity / 255.0f;
        
        // 根据位置判断属于哪个角落
        if (point.x > 0 && point.y > 0) {
            corner.description = "north_east";
        } else if (point.x < 0 && point.y > 0) {
            corner.description = "north_west";
        } else if (point.x > 0 && point.y < 0) {
            corner.description = "south_east";
        } else if (point.x < 0 && point.y < 0) {
            corner.description = "south_west";
        } else {
            corner.description = "center";
        }
        
        classified_corners.push_back(corner);
    }
    
    corner_alignment.detected_corners = classified_corners;
    
    // 3. 计算角落误差（理想情况下，相对角落应该对称）
    if (corner_alignment.detected_corners.size() >= 4) {
        // 寻找配对的角落
        std::map<std::string, PointType> corner_map;
        for (const auto& corner : corner_alignment.detected_corners) {
            if (corner.description != "center") {
                corner_map[corner.description] = corner.position;
            }
        }
        
        // 计算对称误差
        if (corner_map.count("north_east") && corner_map.count("north_west")) {
            float error = fabs(corner_map["north_east"].y - corner_map["north_west"].y);
            corner_alignment.corner_errors.push_back(error);
        }
        
        if (corner_map.count("south_east") && corner_map.count("south_west")) {
            float error = fabs(corner_map["south_east"].y - corner_map["south_west"].y);
            corner_alignment.corner_errors.push_back(error);
        }
        
        if (corner_map.count("north_east") && corner_map.count("south_east")) {
            float error = fabs(corner_map["north_east"].x - corner_map["south_east"].x);
            corner_alignment.corner_errors.push_back(error);
        }
        
        if (corner_map.count("north_west") && corner_map.count("south_west")) {
            float error = fabs(corner_map["north_west"].x - corner_map["south_west"].x);
            corner_alignment.corner_errors.push_back(error);
        }
        
        // 计算统计量
        if (!corner_alignment.corner_errors.empty()) {
            float sum_error = 0, max_error = 0;
            for (float error : corner_alignment.corner_errors) {
                sum_error += error;
                max_error = max(max_error, error);
            }
            
            corner_alignment.average_corner_error = sum_error / corner_alignment.corner_errors.size();
            corner_alignment.max_corner_error = max_error;
            
            // 所有角落误差小于10cm认为对齐
            corner_alignment.all_corners_aligned = (max_error < 0.1f);
        }
    }
    
    return corner_alignment;
}

std::vector<PointType> IntelligentSubmapAligner::detectCorners(const PointCloud& cloud) {
    std::vector<PointType> corners;
    
    if (cloud.empty()) {
        return corners;
    }
    
    // 使用KD-tree进行近邻搜索
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud.makeShared());
    
    const float search_radius = 0.3f;  // 搜索半径30cm
    const int min_neighbors = 5;       // 最少邻居数
    
    for (size_t i = 0; i < cloud.size(); i++) {
        const auto& point = cloud.points[i];
        
        if (point.intensity < 160.0f) continue;  // 只考虑高质量点
        
        // 查找近邻点
        std::vector<int> indices;
        std::vector<float> distances;
        int neighbors_found = kdtree.radiusSearch(point, search_radius, indices, distances);
        
        if (neighbors_found < min_neighbors) {
            continue;
        }
        
        // 收集邻居点
        std::vector<PointType> neighbors;
        for (int idx : indices) {
            if (idx != (int)i) {  // 排除自身
                neighbors.push_back(cloud.points[idx]);
            }
        }
        
        // 检查是否是角落点
        if (isCornerPoint(point, neighbors, 0.3f)) {
            corners.push_back(point);
        }
    }
    
    return corners;
}

bool IntelligentSubmapAligner::isCornerPoint(
    const PointType& point, 
    const std::vector<PointType>& neighbors,
    float corner_threshold) {
    
    if (neighbors.size() < 4) {
        return false;
    }
    
    // 1. 计算邻居点相对于该点的角度分布
    std::vector<float> angles;
    for (const auto& neighbor : neighbors) {
        float dx = neighbor.x - point.x;
        float dy = neighbor.y - point.y;
        float angle = atan2(dy, dx);
        angles.push_back(angle);
    }
    
    // 2. 将角度排序
    std::sort(angles.begin(), angles.end());
    
    // 3. 计算相邻角度之间的最大间隔
    float max_gap = 0;
    for (size_t i = 0; i < angles.size(); i++) {
        size_t j = (i + 1) % angles.size();
        float gap = angles[j] - angles[i];
        if (gap < 0) gap += 2 * M_PI;
        
        max_gap = max(max_gap, gap);
    }
    
    // 4. 角落点通常有一个大角度间隔（> 90度）
    return (max_gap > corner_threshold * M_PI);
}

float IntelligentSubmapAligner::computeLineParallelism(
    const LineEquation& line1, 
    const LineEquation& line2) {
    
    float angle1 = line1.angle;
    float angle2 = line2.angle;
    
    // 归一化角度差
    float angle_diff = fabs(angle1 - angle2);
    if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
    }
    
    // 平行性得分：角度差越小，得分越高
    float parallelism = 1.0f - min(static_cast<float>(angle_diff / (M_PI / 2.0)), 1.0f);
    return parallelism;
}

float IntelligentSubmapAligner::computeLinePerpendicularity(
    const LineEquation& line1, 
    const LineEquation& line2) {
    
    float angle1 = line1.angle;
    float angle2 = line2.angle;
    
    // 计算垂直度：角度差接近90度
    float angle_diff = fabs(angle1 - angle2);
    angle_diff = min(angle_diff, static_cast<float>(M_PI) - angle_diff);// 取0-90度范围
    
    // 接近90度得高分
    float perpendicularity = 1.0f - fabs(angle_diff - M_PI / 2) / (M_PI / 2);
    
    return perpendicularity;
}

PointType IntelligentSubmapAligner::computeCentroid(const std::vector<PointType>& points) {
    PointType centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;
    centroid.intensity = 0;
    
    if (points.empty()) {
        return centroid;
    }
    
    float sum_x = 0, sum_y = 0, sum_z = 0, sum_intensity = 0;
    
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
        sum_intensity += p.intensity;
    }
    
    centroid.x = sum_x / points.size();
    centroid.y = sum_y / points.size();
    centroid.z = sum_z / points.size();
    centroid.intensity = sum_intensity / points.size();
    
    return centroid;
}

WallParallelism IntelligentSubmapAligner::checkXDirectionWallAlignment(
    const PointCloud& merged_cloud) {
    
    WallParallelism parallelism;
    
    WallDetectionResult x_pos_wall = detectXPosWall(merged_cloud);
    WallDetectionResult x_neg_wall = detectXNegWall(merged_cloud);
    
    if (x_pos_wall.wall_exists && x_neg_wall.wall_exists) {
        float parallelism_score = computeLineParallelism(
            x_pos_wall.fitted_line, x_neg_wall.fitted_line);
        
        parallelism.x_parallel = (parallelism_score > 0.8f);
        parallelism.x_angle_diff = fabs(
            x_pos_wall.fitted_line.angle - x_neg_wall.fitted_line.angle) * RAD_TO_DEG;
        
        // 计算距离一致性
        parallelism.x_distance_diff = fabs(
            x_pos_wall.average_distance - x_neg_wall.average_distance);
    }
    
    return parallelism;
}

WallParallelism IntelligentSubmapAligner::checkYDirectionWallAlignment(
    const PointCloud& merged_cloud) {
    
    WallParallelism parallelism;
    
    WallDetectionResult y_pos_wall = detectYPosWall(merged_cloud);
    WallDetectionResult y_neg_wall = detectYNegWall(merged_cloud);
    
    if (y_pos_wall.wall_exists && y_neg_wall.wall_exists) {
        float parallelism_score = computeLineParallelism(
            y_pos_wall.fitted_line, y_neg_wall.fitted_line);
        
        parallelism.y_parallel = (parallelism_score > 0.8f);
        parallelism.y_angle_diff = fabs(
            y_pos_wall.fitted_line.angle - y_neg_wall.fitted_line.angle) * RAD_TO_DEG;
        
        parallelism.y_distance_diff = fabs(
            y_pos_wall.average_distance - y_neg_wall.average_distance);
    }
    
    return parallelism;
}

float IntelligentSubmapAligner::computeOverallAlignmentScore(const FourWallAlignment& alignment) {
    float total_score = 0.0f;
    float max_possible = 0.0f;
    
    // 1. 墙壁存在性得分（25%）
    float wall_existence_score = 0.0f;
    if (alignment.x_pos_wall.wall_exists) wall_existence_score += 0.25f;
    if (alignment.x_neg_wall.wall_exists) wall_existence_score += 0.25f;
    if (alignment.y_pos_wall.wall_exists) wall_existence_score += 0.25f;
    if (alignment.y_neg_wall.wall_exists) wall_existence_score += 0.25f;
    total_score += wall_existence_score * 0.25f;
    max_possible += 0.25f;
    
    // 2. 墙壁平行性得分（30%）
    float parallelism_score = 0.0f;
    if (alignment.parallelism.x_parallel) parallelism_score += 0.5f;
    if (alignment.parallelism.y_parallel) parallelism_score += 0.5f;
    total_score += parallelism_score * 0.30f;
    max_possible += 0.30f;
    
    // 3. 角落对齐得分（25%）
    float corner_score = alignment.corners.all_corners_aligned ? 1.0f : 0.0f;
    total_score += corner_score * 0.25f;
    max_possible += 0.25f;
    
    // 4. 墙壁置信度得分（20%）
    float confidence_score = 0.0f;
    if (alignment.x_pos_wall.wall_exists) confidence_score += alignment.x_pos_wall.alignment_confidence * 0.25f;
    if (alignment.x_neg_wall.wall_exists) confidence_score += alignment.x_neg_wall.alignment_confidence * 0.25f;
    if (alignment.y_pos_wall.wall_exists) confidence_score += alignment.y_pos_wall.alignment_confidence * 0.25f;
    if (alignment.y_neg_wall.wall_exists) confidence_score += alignment.y_neg_wall.alignment_confidence * 0.25f;
    total_score += confidence_score * 0.20f;
    max_possible += 0.20f;
    
    // 归一化到[0, 1]
    if (max_possible > 0) {
        return total_score / max_possible;
    }
    
    return 0.0f;
}

void IntelligentSubmapAligner::generateAlignmentDiagnostics(FourWallAlignment& alignment) {
    std::vector<std::string> issues;
    std::vector<std::string> suggestions;
    
    // 检查墙壁缺失
    if (!alignment.x_pos_wall.wall_exists) {
        issues.push_back("东墙未检测到");
        suggestions.push_back("检查东侧数据质量或调整检测阈值");
    }
    if (!alignment.x_neg_wall.wall_exists) {
        issues.push_back("西墙未检测到");
        suggestions.push_back("检查西侧数据质量或调整检测阈值");
    }
    if (!alignment.y_pos_wall.wall_exists) {
        issues.push_back("北墙未检测到");
        suggestions.push_back("检查北侧数据质量或调整检测阈值");
    }
    if (!alignment.y_neg_wall.wall_exists) {
        issues.push_back("南墙未检测到");
        suggestions.push_back("检查南侧数据质量或调整检测阈值");
    }
    
    // 检查平行性问题
    if (alignment.x_pos_wall.wall_exists && alignment.x_neg_wall.wall_exists && 
        !alignment.parallelism.x_parallel) {
        issues.push_back("东西墙不平行");
        suggestions.push_back("调整旋转角度，当前角度差: " + 
                             std::to_string(alignment.parallelism.x_angle_diff) + "度");
    }
    
    if (alignment.y_pos_wall.wall_exists && alignment.y_neg_wall.wall_exists && 
        !alignment.parallelism.y_parallel) {
        issues.push_back("南北墙不平行");
        suggestions.push_back("调整旋转角度，当前角度差: " + 
                             std::to_string(alignment.parallelism.y_angle_diff) + "度");
    }
    
    // 检查角落对齐
    if (!alignment.corners.detected_corners.empty() && !alignment.corners.all_corners_aligned) {
        issues.push_back("角落未对齐");
        suggestions.push_back("微调位置，最大角落误差: " + 
                             std::to_string(alignment.corners.max_corner_error) + "米");
    }
    
    // 生成主要问题和建议
    if (!issues.empty()) {
        alignment.major_issue = issues[0];  // 第一个问题作为主要问题
        alignment.suggestions = suggestions;
    } else {
        alignment.major_issue = "几何对齐良好";
        alignment.suggestions.push_back("保持当前参数");
    }
}

std::string IntelligentSubmapAligner::generatePoolGeometryReport(
    const FourWallAlignment& alignment) {
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    
    ss << "========================================\n";
    ss << "水池几何特征检查报告\n";
    ss << "========================================\n\n";
    
    // 墙壁检测结果
    ss << "1. 墙壁检测结果:\n";
    ss << "   东墙: " << (alignment.x_pos_wall.wall_exists ? "✓ 检测到" : "✗ 未检测到");
    if (alignment.x_pos_wall.wall_exists) {
        ss << " | 距离: " << alignment.x_pos_wall.average_distance << "m";
        ss << " | 置信度: " << alignment.x_pos_wall.alignment_confidence * 100 << "%";
        ss << " | 角度: " << alignment.x_pos_wall.fitted_line.angle * RAD_TO_DEG << "°";
    }
    ss << "\n";
    
    ss << "   西墙: " << (alignment.x_neg_wall.wall_exists ? "✓ 检测到" : "✗ 未检测到");
    if (alignment.x_neg_wall.wall_exists) {
        ss << " | 距离: " << alignment.x_neg_wall.average_distance << "m";
        ss << " | 置信度: " << alignment.x_neg_wall.alignment_confidence * 100 << "%";
        ss << " | 角度: " << alignment.x_neg_wall.fitted_line.angle * RAD_TO_DEG << "°";
    }
    ss << "\n";
    
    ss << "   北墙: " << (alignment.y_pos_wall.wall_exists ? "✓ 检测到" : "✗ 未检测到");
    if (alignment.y_pos_wall.wall_exists) {
        ss << " | 距离: " << alignment.y_pos_wall.average_distance << "m";
        ss << " | 置信度: " << alignment.y_pos_wall.alignment_confidence * 100 << "%";
        ss << " | 角度: " << alignment.y_pos_wall.fitted_line.angle * RAD_TO_DEG << "°";
    }
    ss << "\n";
    
    ss << "   南墙: " << (alignment.y_neg_wall.wall_exists ? "✓ 检测到" : "✗ 未检测到");
    if (alignment.y_neg_wall.wall_exists) {
        ss << " | 距离: " << alignment.y_neg_wall.average_distance << "m";
        ss << " | 置信度: " << alignment.y_neg_wall.alignment_confidence * 100 << "%";
        ss << " | 角度: " << alignment.y_neg_wall.fitted_line.angle * RAD_TO_DEG << "°";
    }
    ss << "\n\n";
    
    // 平行性检查
    ss << "2. 墙壁平行性检查:\n";
    if (alignment.x_pos_wall.wall_exists && alignment.x_neg_wall.wall_exists) {
        ss << "   东西墙: " << (alignment.parallelism.x_parallel ? "✓ 平行" : "✗ 不平行");
        ss << " | 角度差: " << alignment.parallelism.x_angle_diff << "°";
        ss << " | 距离差: " << alignment.parallelism.x_distance_diff << "m\n";
    } else {
        ss << "   东西墙: 数据不足，无法检查平行性\n";
    }
    
    if (alignment.y_pos_wall.wall_exists && alignment.y_neg_wall.wall_exists) {
        ss << "   南北墙: " << (alignment.parallelism.y_parallel ? "✓ 平行" : "✗ 不平行");
        ss << " | 角度差: " << alignment.parallelism.y_angle_diff << "°";
        ss << " | 距离差: " << alignment.parallelism.y_distance_diff << "m\n";
    } else {
        ss << "   南北墙: 数据不足，无法检查平行性\n";
    }
    
    ss << "   矩形一致性: " << (alignment.parallelism.rectangles_consistent ? "✓ 良好" : "✗ 不满足");
    ss << "\n\n";
    
    // 角落对齐检查
    ss << "3. 角落对齐检查:\n";
    ss << "   检测到角落数: " << alignment.corners.detected_corners.size() << "\n";
    if (!alignment.corners.corner_errors.empty()) {
        ss << "   平均角落误差: " << alignment.corners.average_corner_error << "m\n";
        ss << "   最大角落误差: " << alignment.corners.max_corner_error << "m\n";
        ss << "   所有角落对齐: " << (alignment.corners.all_corners_aligned ? "✓ 是" : "✗ 否") << "\n";
    } else {
        ss << "   角落数据不足，无法计算误差\n";
    }
    ss << "\n";
    
    // 综合评估
    ss << "4. 综合评估:\n";
    ss << "   整体对齐分数: " << alignment.overall_score * 100 << "%\n";
    
    if (alignment.overall_score >= 0.85f) {
        ss << "   评价: 优秀 - 几何对齐良好\n";
    } else if (alignment.overall_score >= 0.7f) {
        ss << "   评价: 良好 - 基本对齐，有小问题\n";
    } else if (alignment.overall_score >= 0.5f) {
        ss << "   评价: 一般 - 存在明显对齐问题\n";
    } else {
        ss << "   评价: 差 - 需要显著改进\n";
    }
    ss << "\n";
    
    // 诊断信息
    ss << "5. 诊断与建议:\n";
    ss << "   主要问题: " << alignment.major_issue << "\n";
    ss << "   优化建议:\n";
    for (const auto& suggestion : alignment.suggestions) {
        ss << "     - " << suggestion << "\n";
    }
    
    ss << "========================================\n";
    
    return ss.str();
}

TransformParams IntelligentSubmapAligner::generateOptimizationSuggestion(
    const FourWallAlignment& alignment,
    const TransformParams& current_params) {
    
    TransformParams suggestion = current_params;
    
    // 如果几何对齐很好，不需要调整
    if (alignment.overall_score > 0.85f) {
        suggestion.confidence = min(1.0f, suggestion.confidence + 0.1f);
        return suggestion;
    }
    
    // 1. 处理墙壁平行性问题
    if (alignment.x_pos_wall.wall_exists && alignment.x_neg_wall.wall_exists && 
        !alignment.parallelism.x_parallel) {
        
        // 计算需要的旋转调整
        float angle_error = alignment.parallelism.x_angle_diff * DEG_TO_RAD;
        float rotation_correction = angle_error * 0.3f;  // 保守调整
        
        // 根据哪面墙更可靠来决定旋转方向
        if (alignment.x_pos_wall.alignment_confidence > alignment.x_neg_wall.alignment_confidence) {
            // 东墙更可靠，以西墙为基准调整
            suggestion.dtheta -= rotation_correction;
        } else {
            // 西墙更可靠，以东墙为基准调整
            suggestion.dtheta += rotation_correction;
        }
    }
    
    // 2. 处理位置偏移问题
    if (alignment.corners.all_corners_aligned == false && 
        !alignment.corners.corner_errors.empty()) {
        
        // 根据角落误差调整位置
        float avg_error = alignment.corners.average_corner_error;
        
        // 简单启发式：误差大时进行较大调整
        if (avg_error > 0.1f) {
            // 假设需要向误差减小的方向移动
            float adjustment_factor = 0.2f;  // 保守调整比例
            
            // 这里可以更智能地决定移动方向
            // 简化实现：根据墙壁检测结果决定
            if (alignment.x_pos_wall.wall_exists && alignment.x_neg_wall.wall_exists) {
                // 调整X方向
                float x_offset = (alignment.x_pos_wall.average_distance - 
                                 alignment.x_neg_wall.average_distance) * 0.1f;
                suggestion.dx += x_offset;
            }
            
            if (alignment.y_pos_wall.wall_exists && alignment.y_neg_wall.wall_exists) {
                // 调整Y方向
                float y_offset = (alignment.y_pos_wall.average_distance - 
                                 alignment.y_neg_wall.average_distance) * 0.1f;
                suggestion.dy += y_offset;
            }
        }
    }
    
    // 3. 处理墙壁缺失问题
    int missing_walls = 0;
    if (!alignment.x_pos_wall.wall_exists) missing_walls++;
    if (!alignment.x_neg_wall.wall_exists) missing_walls++;
    if (!alignment.y_pos_wall.wall_exists) missing_walls++;
    if (!alignment.y_neg_wall.wall_exists) missing_walls++;
    
    if (missing_walls >= 2) {
        // 太多墙壁缺失，可能需要重新搜索
        suggestion.confidence *= 0.7f;  // 降低置信度
    }
    
    // 4. 确保参数在合理范围内
    suggestion.dtheta = fmod(suggestion.dtheta + M_PI, 2 * M_PI) - M_PI;  // 限制在[-π, π]
    
    // 限制平移幅度
    const float max_adjustment = 0.2f;  // 最大调整20cm
    suggestion.dx = max(-max_adjustment, min(max_adjustment, suggestion.dx));
    suggestion.dy = max(-max_adjustment, min(max_adjustment, suggestion.dy));
    
    // 根据调整幅度更新置信度
    float adjustment_magnitude = sqrt(suggestion.dx * suggestion.dx + 
                                     suggestion.dy * suggestion.dy + 
                                     suggestion.dtheta * suggestion.dtheta);
    
    if (adjustment_magnitude > 0.05f) {
        // 进行了显著调整，稍微降低置信度
        suggestion.confidence *= 0.9f;
    } else if (adjustment_magnitude < 0.01f) {
        // 微小调整，增加置信度
        suggestion.confidence = min(1.0f, suggestion.confidence * 1.05f);
    }
    
    return suggestion;
}

 

TransformParams IntelligentSubmapAligner::matchWithNDT(
    const PointCloudPtr& source,
    const PointCloudPtr& target,
    const TransformParams& initial_guess) {
    
    TransformParams result = initial_guess;
    
    if (source->empty() || target->empty()) {
        ROS_WARN("NDT: 点云为空");
        return result;
    }
    
    ROS_INFO("NDT 配准: 源点云=%lu, 目标点云=%lu", source->size(), target->size());
    
    // 1. 下采样加速
    PointCloudPtr filtered_source(new PointCloud);
    PointCloudPtr filtered_target(new PointCloud);
    
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.2f, 0.2f, 0.2f);
    voxel.setInputCloud(source);
    voxel.filter(*filtered_source);
    voxel.setInputCloud(target);
    voxel.filter(*filtered_target);
    
    ROS_DEBUG("NDT 下采样: %lu → %lu 点", source->size(), filtered_source->size());
    
    // 2. 构建初始猜测矩阵
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    float cos_theta = cos(initial_guess.dtheta);
    float sin_theta = sin(initial_guess.dtheta);
    
    init_guess(0, 0) = cos_theta;
    init_guess(0, 1) = -sin_theta;
    init_guess(1, 0) = sin_theta;
    init_guess(1, 1) = cos_theta;
    init_guess(0, 3) = initial_guess.dx;
    init_guess(1, 3) = initial_guess.dy;
    
    ROS_INFO("NDT 初始猜测: dx=%.3f, dy=%.3f, dtheta=%.1f°",
             initial_guess.dx, initial_guess.dy, initial_guess.dtheta * RAD_TO_DEG);
    
    // 3. 创建 NDT 对象
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    
    // 4. 设置 NDT 参数（针对声纳稀疏数据优化）
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(0.8f);      // 增大分辨率适应稀疏数据
    ndt.setMaximumIterations(45); // 增加迭代次数
    
    // 5. 设置输入（源点云变换到目标点云）
    ndt.setInputSource(filtered_source);
    ndt.setInputTarget(filtered_target);
    
    // 6. 执行配准
    PointCloud output_cloud;
    ndt.align(output_cloud, init_guess);
    
    // 7. 获取结果
    if (ndt.hasConverged()) {
        Eigen::Matrix4f final_transform = ndt.getFinalTransformation();
        
        result.dx = final_transform(0, 3);
        result.dy = final_transform(1, 3);
        result.dtheta = atan2(final_transform(1, 0), final_transform(0, 0));
        
        float fitness = ndt.getFitnessScore();
        result.confidence = 1.0f - std::min(1.0f, fitness);
        
        ROS_INFO("✅ NDT 配准成功: 适应度=%.4f, 变换=(%.3f,%.3f,%.1f°)",
                 fitness, result.dx, result.dy, result.dtheta * RAD_TO_DEG);
    } else {
        ROS_WARN("❌ NDT 未收敛，使用初始猜测");
        result.confidence = 0.3f;
    }
    
    return result;
}
 
std::vector<MultiPositionEdge> IntelligentSubmapAligner::buildAdjacentEdges(
    const std::vector<SubMapInfo>& submaps,
    const std::vector<TransformParams>& initial_guesses) {
    
    std::vector<MultiPositionEdge> edges;
    
    if (submaps.size() < 2) return edges;
    
    ROS_INFO("构建相邻NDT约束边，共 %lu 个子图", submaps.size());
    
    for (size_t i = 0; i < submaps.size() - 1; i++) {
        const auto& submap1 = submaps[i];
        const auto& submap2 = submaps[i + 1];
        
        // 构建初始猜测
        TransformParams initial_guess;
        if (i < initial_guesses.size()) {
            initial_guess = initial_guesses[i];
        } else {
            // 使用质心法作为初始猜测
            initial_guess.dx = submap2.center_x - submap1.center_x;
            initial_guess.dy = submap2.center_y - submap1.center_y;
            initial_guess.dtheta = submap2.robot_pose.theta - submap1.robot_pose.theta;
            initial_guess.confidence = 0.5f;
        }
        
        // NDT 匹配：源=子地图2，目标=子地图1
        TransformParams ndt_result = matchWithNDT(
            submap2.cloud, submap1.cloud, initial_guess);
        
        // 【关键修复】如果 NDT 置信度低或结果明显错误，取逆变换
        // 判断依据：NDT 结果的 dx 符号与初始猜测相反
        if (ndt_result.confidence < 0.3f || 
            (ndt_result.dx * initial_guess.dx < 0 && fabs(initial_guess.dx) > 0.5f)) {
            
            ROS_WARN("  NDT 结果方向可能错误，应用逆变换");
            ROS_WARN("    原结果: dx=%.3f, dy=%.3f, dtheta=%.1f°",
                     ndt_result.dx, ndt_result.dy, ndt_result.dtheta * RAD_TO_DEG);
            
            // 取逆变换
            TransformParams inv_result;
            inv_result.dx = -ndt_result.dx;
            inv_result.dy = -ndt_result.dy;
            inv_result.dtheta = -ndt_result.dtheta;
            inv_result.confidence = ndt_result.confidence * 0.8f;
            ndt_result = inv_result;
            
            ROS_WARN("    逆变换: dx=%.3f, dy=%.3f, dtheta=%.1f°",
                     ndt_result.dx, ndt_result.dy, ndt_result.dtheta * RAD_TO_DEG);
        }
        
        MultiPositionEdge edge;
        edge.from_id = i;
        edge.to_id = i + 1;
        edge.relative_transform = ndt_result;
        edge.confidence = ndt_result.confidence;
        edge.is_odom_edge = false;
        
        edges.push_back(edge);
        
        ROS_INFO("  边 %d→%d: NDT约束 (%.3f,%.3f,%.1f°) 置信度=%.2f",
                (int)i, (int)i+1,
                ndt_result.dx, ndt_result.dy, ndt_result.dtheta * RAD_TO_DEG,
                ndt_result.confidence);
    }
    
    return edges;
}

std::vector<TransformParams> IntelligentSubmapAligner::globalGraphOptimization(
    const std::vector<MultiPositionEdge>& edges,
    const std::vector<TransformParams>& initial_poses,
    const MultiPositionConfig& config) {
    
    if (initial_poses.empty()) {
        ROS_ERROR("初始位姿为空");
        return std::vector<TransformParams>();
    }
    
    // 复制位姿用于优化
    std::vector<TransformParams> optimized_poses = initial_poses;
    
    // 固定第一个位姿（原点）
    optimized_poses[0].dx = 0;
    optimized_poses[0].dy = 0;
    optimized_poses[0].dtheta = 0;
    
    ROS_INFO("开始全局图优化，共 %lu 个位姿，%lu 条边",
             optimized_poses.size(), edges.size());
    
    // 使用Gauss-Newton迭代优化
    for (int iter = 0; iter < config.max_global_iterations; iter++) {
        float residual_before = computeGlobalResidual(edges, optimized_poses, config);
        
        computeGlobalResidual(edges, optimized_poses, config);
        // 执行一次Gauss-Newton迭代
        gaussNewtonGlobalOptimization(optimized_poses, edges, config);
        
        float residual_after = computeGlobalResidual(edges, optimized_poses, config);
        float improvement = (residual_before - residual_after) / residual_before;
        
        ROS_INFO("  全局迭代 %d: 残差 %.6f → %.6f (改善 %.2f%%)",
                iter, residual_before, residual_after, improvement * 100);
        
        if (improvement < config.global_convergence_threshold) {
            ROS_INFO("全局优化收敛");
            break;
        }
    }
    
    return optimized_poses;
}
 

float IntelligentSubmapAligner::computeGlobalResidual(
    const std::vector<MultiPositionEdge>& edges,
    const std::vector<TransformParams>& poses,
    const MultiPositionConfig& config) {  // 添加 config 参数
    
    if (edges.empty() || poses.empty()) {
        ROS_WARN("computeGlobalResidual: 边或位姿为空");
        return 0.0f;
    }
    
    float total_residual = 0.0f;
    int valid_edges = 0;
    
    for (size_t i = 0; i < edges.size(); i++) {
        const auto& edge = edges[i];
        
        if (edge.from_id < 0 || edge.from_id >= (int)poses.size() ||
            edge.to_id < 0 || edge.to_id >= (int)poses.size()) {
            continue;
        }
        
        const auto& pose_from = poses[edge.from_id];
        const auto& pose_to = poses[edge.to_id];
        const auto& relative = edge.relative_transform;
        
        Eigen::Vector3f residual = computeResidualVector(pose_from, pose_to, relative);
        
        float weight = edge.confidence;
        if (edge.is_odom_edge) {
            // 使用传入的 config 参数
            weight *= config.odom_weight / config.ndt_weight;
        }
        
        Eigen::Matrix3f info = weight * Eigen::Matrix3f::Identity();
        float edge_residual = residual.transpose() * info * residual;
        
        total_residual += edge_residual;
        valid_edges++;
    }
    
    if (valid_edges == 0) {
        return 0.0f;
    }
    
    return total_residual / valid_edges;
}


MultiPositionResult IntelligentSubmapAligner::optimizeMultiPositionAlignment(
    const std::vector<SubMapInfo>& submaps,
    const MultiPositionConfig& config) {
    
    MultiPositionResult result;
    
    if (submaps.size() < 2) {
        ROS_ERROR("至少需要2个子图");
        return result;
    }
    
    ROS_INFO("========================================");
    ROS_INFO("Pose Graph + NDT 多位置优化");
    ROS_INFO("子图数量: %lu", submaps.size());
    ROS_INFO("========================================");
    
    // 1. 构建相邻NDT约束边
    std::vector<MultiPositionEdge> edges = buildAdjacentEdges(submaps);
    
    // 2. 构建初始位姿估计
    std::vector<TransformParams> initial_poses;
    TransformParams cumulative;
    initial_poses.push_back(cumulative);
    
    for (size_t i = 0; i < edges.size(); i++) {
        cumulative = composeTransforms(cumulative, edges[i].relative_transform);
        initial_poses.push_back(cumulative);
        ROS_INFO("  位姿%lu: (%.3f,%.3f,%.1f°)",
                i+1, cumulative.dx, cumulative.dy, cumulative.dtheta * RAD_TO_DEG);
    }
    
    // 3. 回环检测
    if (config.enable_loop_closure) {
        std::vector<MultiPositionEdge> loop_edges = detectLoopClosures(submaps, initial_poses);
        edges.insert(edges.end(), loop_edges.begin(), loop_edges.end());
    }
    
    // 4. 全局图优化
    std::vector<TransformParams> optimized_poses;
    if (config.enable_global_optimization) {
        optimized_poses = globalGraphOptimization(edges, initial_poses, config);
    } else {
        optimized_poses = initial_poses;
    }
    
    // 5. 合并点云
    PointCloudPtr global_cloud(new PointCloud);
    for (size_t i = 0; i < submaps.size(); i++) {
        PointCloudPtr transformed = transformPointCloud(submaps[i].cloud, optimized_poses[i]);
        *global_cloud += *transformed;
    }
    
    result.success = true;
    result.global_poses = optimized_poses;
    result.global_cloud = global_cloud;
    result.total_optimization_score = computeGlobalResidual(edges, optimized_poses, config);
    
    ROS_INFO("========================================");
    ROS_INFO("优化完成: 残差=%.6f, 点云=%lu点",
            result.total_optimization_score, global_cloud->size());
    ROS_INFO("========================================");
    
    return result;
}

/*****
MultiPositionResult IntelligentSubmapAligner::optimizeMultiPositionAlignment(
    const std::vector<SubMapInfo>& submaps,
    const MultiPositionConfig& config) {
    
    MultiPositionResult result;
    
    if (submaps.size() < 2) {
        ROS_ERROR("至少需要2个子图才能进行多位置优化");
        return result;
    }
    
    ROS_INFO("========================================");
    ROS_INFO("开始多位置建图全局优化");
    ROS_INFO("  子图数量: %lu", submaps.size());
    ROS_INFO("  全局优化: %s", config.enable_global_optimization ? "启用" : "禁用");
    ROS_INFO("  回环检测: %s", config.enable_loop_closure ? "启用" : "禁用");
    ROS_INFO("========================================");
    
    // 第一步：构建相邻约束边（使用现有智能对齐）
    std::vector<MultiPositionEdge> edges = buildAdjacentEdges(submaps);
    
    // 第二步：构建初始位姿估计（基于累积变换）
    std::vector<TransformParams> initial_poses;
    TransformParams cumulative_pose;
    initial_poses.push_back(cumulative_pose);  // 第一个位姿在原点
    
    for (size_t i = 0; i < edges.size(); i++) {
        // 累积相对变换
        const auto& edge = edges[i];
        float cos_theta = cos(cumulative_pose.dtheta);
        float sin_theta = sin(cumulative_pose.dtheta);
        
        float new_x = cumulative_pose.dx + edge.relative_transform.dx * cos_theta - 
                     edge.relative_transform.dy * sin_theta;
        float new_y = cumulative_pose.dy + edge.relative_transform.dx * sin_theta + 
                     edge.relative_transform.dy * cos_theta;
        float new_theta = cumulative_pose.dtheta + edge.relative_transform.dtheta;
        
        cumulative_pose.dx = new_x;
        cumulative_pose.dy = new_y;
        cumulative_pose.dtheta = new_theta;
        
        initial_poses.push_back(cumulative_pose);
        
        ROS_INFO("  累积位姿 %lu: (%.3f, %.3f, %.1f°)",
                i+1, new_x, new_y, new_theta * RAD_TO_DEG);
    }
    
    // 第三步：回环检测（可选，适用于闭合水池）
    if (config.enable_loop_closure) {
        std::vector<MultiPositionEdge> loop_edges = detectLoopClosures(submaps, initial_poses);
        edges.insert(edges.end(), loop_edges.begin(), loop_edges.end());
        
        if (!loop_edges.empty()) {
            ROS_INFO("检测到 %lu 条回环边", loop_edges.size());
        }
    }
    
    // 第四步：全局图优化
    std::vector<TransformParams> optimized_poses;
    if (config.enable_global_optimization) {
        optimized_poses = globalGraphOptimization(edges, initial_poses, config);
    } else {
        optimized_poses = initial_poses;
    }
    
    // 第五步：记录每个对齐的质量
    for (size_t i = 0; i < submaps.size() - 1; i++) {
        // 重新评估每个相邻对齐的质量
        TransformParams guess;
        guess.dx = optimized_poses[i+1].dx - optimized_poses[i].dx;
        guess.dy = optimized_poses[i+1].dy - optimized_poses[i].dy;
        guess.dtheta = optimized_poses[i+1].dtheta - optimized_poses[i].dtheta;
        
        AlignmentQuality quality = evaluateAlignmentQuality(submaps[i], submaps[i+1], guess);
        result.qualities.push_back(quality);
    }
    
    // 第六步：使用优化后的位姿合并所有点云
    PointCloudPtr global_cloud(new PointCloud);
    
    for (size_t i = 0; i < submaps.size(); i++) {
        // 将子图变换到全局坐标系
        PointCloudPtr transformed = transformPointCloud(submaps[i].cloud, optimized_poses[i]);
        *global_cloud += *transformed;
    }
    
    result.success = true;
    result.global_poses = optimized_poses;
    result.global_cloud = global_cloud;
    result.optimization_log = optimization_log_;
    
    // 计算总评分
    float total_score = 0;
    for (const auto& q : result.qualities) {
        total_score += q.total_score;
    }
    result.total_optimization_score = total_score / result.qualities.size();
    result.iterations_used = config.max_global_iterations;
    
    ROS_INFO("========================================");
    ROS_INFO("多位置优化完成");
    ROS_INFO("  成功: %s", result.success ? "是" : "否");
    ROS_INFO("  总评分: %.2f%%", result.total_optimization_score * 100);
    ROS_INFO("  全局点云点数: %lu", global_cloud->size());
    ROS_INFO("========================================");
    
    return result;
}
***/

std::vector<MultiPositionEdge> IntelligentSubmapAligner::detectLoopClosures(
    const std::vector<SubMapInfo>& submaps,
    const std::vector<TransformParams>& current_poses) {
    
    std::vector<MultiPositionEdge> loop_edges;
    
    if (submaps.size() < 3) return loop_edges;
    
    ROS_INFO("检测回环闭合（NDT模式）...");
    
    for (size_t i = 0; i < submaps.size(); i++) {
        for (size_t j = i + 2; j < submaps.size(); j++) {
            float dx = current_poses[j].dx - current_poses[i].dx;
            float dy = current_poses[j].dy - current_poses[i].dy;
            float distance = sqrt(dx*dx + dy*dy);
            
            const float LOOP_THRESHOLD = 2.0f;
            if (distance < LOOP_THRESHOLD) {
                ROS_INFO("  潜在回环: %d → %d, 距离=%.2fm", (int)i, (int)j, distance);
                
                // 使用NDT匹配
                TransformParams initial_guess;
                initial_guess.dx = dx;
                initial_guess.dy = dy;
                initial_guess.dtheta = current_poses[j].dtheta - current_poses[i].dtheta;
                
                TransformParams ndt_result = matchWithNDT(submaps[j].cloud, submaps[i].cloud, initial_guess);
                
                if (ndt_result.confidence > 0.6f) {
                    MultiPositionEdge edge;
                    edge.from_id = i;
                    edge.to_id = j;
                    edge.relative_transform = ndt_result;
                    edge.confidence = ndt_result.confidence * 0.8f;
                    edge.is_odom_edge = false;
                    loop_edges.push_back(edge);
                    
                    ROS_INFO("  回环边 %d→%d 成功，置信度=%.2f",
                            (int)i, (int)j, edge.confidence);
                }
            }
        }
    }
    
    return loop_edges;
}


// ========== 判别机制实现 ==========

bool IntelligentSubmapAligner::isAlignmentAcceptable(
    const PointCloudPtr& cloud, std::string& reason) {
    
    FourWallAlignment alignment = checkFourWallAlignment(*cloud);
    
    const float SUCCESS_THRESHOLD = 0.70f;
    bool success = (alignment.overall_score >= SUCCESS_THRESHOLD);
    
    if (!success) {
        reason = alignment.major_issue;
        if (reason.empty()) reason = "对齐质量不足";
    } else {
        reason = "对齐良好";
    }
    
    return success;
}

float IntelligentSubmapAligner::getAlignmentScore(const PointCloudPtr& cloud) {
    FourWallAlignment alignment = checkFourWallAlignment(*cloud);
    return alignment.overall_score;
}

// ========== NDT精确对齐实现 ==========

PointCloudPtr IntelligentSubmapAligner::alignWithPoseGraphNDT(
    const SubMapInfo& submap1,
    const SubMapInfo& submap2,
    const TransformParams& initial_guess) {
    
    ROS_INFO("========================================");
    ROS_INFO("Pose Graph + NDT 精确对齐");
    ROS_INFO("========================================");
    
    std::vector<SubMapInfo> submaps = {submap1, submap2};
    
    MultiPositionConfig config;
    config.enable_global_optimization = true;
    config.max_global_iterations = 8;
    config.global_convergence_threshold = 0.005f;
    config.odom_weight = 0.2f;
    config.ndt_weight = 0.8f;
    config.enable_loop_closure = false;
    
    MultiPositionResult result = optimizeMultiPositionAlignment(submaps, config);
    
    if (result.success && result.global_cloud && !result.global_cloud->empty()) {
        ROS_INFO("NDT优化成功，点云=%lu点", result.global_cloud->size());
        return result.global_cloud;
    }
    
    ROS_WARN("NDT优化失败");
    return PointCloudPtr(new PointCloud);
}
 
Eigen::Matrix3f IntelligentSubmapAligner::toSE2(const TransformParams& params) {
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity();
    float cos_theta = cos(params.dtheta);
    float sin_theta = sin(params.dtheta);
    
    T(0, 0) = cos_theta;
    T(0, 1) = -sin_theta;
    T(0, 2) = params.dx;
    T(1, 0) = sin_theta;
    T(1, 1) = cos_theta;
    T(1, 2) = params.dy;
    
    return T;
}

TransformParams IntelligentSubmapAligner::fromSE2(const Eigen::Matrix3f& T) {
    TransformParams params;
    params.dx = T(0, 2);
    params.dy = T(1, 2);
    params.dtheta = atan2(T(1, 0), T(0, 0));
    
    // 角度归一化到 [-π, π]
    while (params.dtheta > M_PI) params.dtheta -= 2 * M_PI;
    while (params.dtheta < -M_PI) params.dtheta += 2 * M_PI;
    
    return params;
}

TransformParams IntelligentSubmapAligner::composeTransforms(
    const TransformParams& a,
    const TransformParams& b) {
    Eigen::Matrix3f T_a = toSE2(a);
    Eigen::Matrix3f T_b = toSE2(b);
    return fromSE2(T_a * T_b);
}

TransformParams IntelligentSubmapAligner::inverseTransform(const TransformParams& params) {
    TransformParams inv;
    float cos_theta = cos(params.dtheta);
    float sin_theta = sin(params.dtheta);
    
    inv.dtheta = -params.dtheta;
    inv.dx = -(params.dx * cos_theta + params.dy * sin_theta);
    inv.dy = -(-params.dx * sin_theta + params.dy * cos_theta);
    
    while (inv.dtheta > M_PI) inv.dtheta -= 2 * M_PI;
    while (inv.dtheta < -M_PI) inv.dtheta += 2 * M_PI;
    
    return inv;
}

Eigen::Vector3f IntelligentSubmapAligner::se2Log(const Eigen::Matrix3f& T) {
    Eigen::Vector3f xi;
    
    // 提取角度
    xi(2) = atan2(T(1, 0), T(0, 0));
    
    // 当旋转很小时，使用近似
    if (fabs(xi(2)) < 1e-6) {
        xi(0) = T(0, 2);
        xi(1) = T(1, 2);
    } else {
        // 精确计算平移部分
        float A = sin(xi(2)) / xi(2);
        float B = (1 - cos(xi(2))) / xi(2);
        
        Eigen::Matrix2f V_inv;
        V_inv(0, 0) = A;
        V_inv(0, 1) = B;
        V_inv(1, 0) = -B;
        V_inv(1, 1) = A;
        
        Eigen::Vector2f t(T(0, 2), T(1, 2));
        Eigen::Vector2f rho = V_inv * t;
        
        xi(0) = rho(0);
        xi(1) = rho(1);
    }
    
    return xi;
}

Eigen::Matrix3f IntelligentSubmapAligner::se2Exp(const Eigen::Vector3f& xi) {
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity();
    float theta = xi(2);
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    
    T(0, 0) = cos_theta;
    T(0, 1) = -sin_theta;
    T(1, 0) = sin_theta;
    T(1, 1) = cos_theta;
    
    if (fabs(theta) < 1e-6) {
        T(0, 2) = xi(0);
        T(1, 2) = xi(1);
    } else {
        float A = sin_theta / theta;
        float B = (1 - cos_theta) / theta;
        
        T(0, 2) = A * xi(0) - B * xi(1);
        T(1, 2) = B * xi(0) + A * xi(1);
    }
    
    return T;
}


// intelligent_submap_aligner.cpp

Eigen::Vector3f IntelligentSubmapAligner::computeResidualVector(
    const TransformParams& pose_i,
    const TransformParams& pose_j,
    const TransformParams& relative) {
    
    // 计算实际相对变换: T_ij_actual = T_i^{-1} * T_j
    TransformParams T_i_inv = inverseTransform(pose_i);
    TransformParams T_ij_actual = composeTransforms(T_i_inv, pose_j);
    
    // 计算误差: T_error = T_ij_actual * T_relative^{-1}
    TransformParams T_relative_inv = inverseTransform(relative);
    TransformParams T_error = composeTransforms(T_ij_actual, T_relative_inv);
    
    // 将误差映射到 se(2) 得到残差向量
    Eigen::Matrix3f T_error_mat = toSE2(T_error);
    Eigen::Vector3f residual = se2Log(T_error_mat);
    
    // 角度归一化
    while (residual(2) > M_PI) residual(2) -= 2 * M_PI;
    while (residual(2) < -M_PI) residual(2) += 2 * M_PI;
    
    return residual;
}

Eigen::Matrix<float, 3, 6> IntelligentSubmapAligner::computeResidualJacobian(
    const TransformParams& pose_i,
    const TransformParams& pose_j,
    const TransformParams& relative) {
    
    // 雅可比矩阵: ∂e/∂[ξ_i, ξ_j]
    // 其中 ξ_i = [dx_i, dy_i, dtheta_i], ξ_j = [dx_j, dy_j, dtheta_j]
    // 对于残差 e = log(T_i^{-1} * T_j * T_relative^{-1})
    
    Eigen::Matrix<float, 3, 6> J;
    J.setZero();
    
    float theta_i = pose_i.dtheta;
    float cos_theta = cos(theta_i);
    float sin_theta = sin(theta_i);
    
    // 对 ξ_i 的导数 (负的伴随矩阵)
    // ∂e/∂ξ_i = -Ad(T_i^{-1} * T_j * T_relative^{-1}) * Ad(T_relative) * (-Ad(T_i^{-1})?)
    // 简化版本：使用数值验证过的解析形式
    
    // 平移部分对 ξ_i 的导数
    J(0, 0) = -cos_theta;
    J(0, 1) = -sin_theta;
    J(1, 0) = sin_theta;
    J(1, 1) = -cos_theta;
    
    // 旋转部分对 ξ_i 的导数
    J(2, 2) = -1.0f;
    
    // 对 ξ_j 的导数 (单位矩阵)
    J(0, 3) = 1.0f;
    J(1, 4) = 1.0f;
    J(2, 5) = 1.0f;
    
    // 当旋转较大时，需要考虑耦合项
    if (fabs(theta_i) > 0.1f) {
        float dx_ij = pose_j.dx - pose_i.dx;
        float dy_ij = pose_j.dy - pose_i.dy;
        
        // 考虑旋转对平移的影响
        J(0, 2) = -dx_ij * sin_theta - dy_ij * cos_theta;
        J(1, 2) = dx_ij * cos_theta - dy_ij * sin_theta;
    }
    
    return J;
}


void IntelligentSubmapAligner::gaussNewtonGlobalOptimization(
    std::vector<TransformParams>& poses,
    const std::vector<MultiPositionEdge>& edges,
    const MultiPositionConfig& config) {
    
    if (poses.empty()) {
        ROS_WARN("gaussNewtonGlobalOptimization: 位姿为空");
        return;
    }
    
    if (edges.empty()) {
        ROS_WARN("gaussNewtonGlobalOptimization: 边为空");
        return;
    }
    
    int num_poses = poses.size();
    int num_variables = (num_poses - 1) * 3;  // 第一个位姿固定
    
    if (num_variables <= 0) {
        ROS_WARN("gaussNewtonGlobalOptimization: 没有可优化的变量");
        return;
    }
    
    ROS_DEBUG("GN优化: %d个位姿, %lu条边, %d个变量",
              num_poses, edges.size(), num_variables);
    
    // 固定第一个位姿为原点
    poses[0].dx = 0.0f;
    poses[0].dy = 0.0f;
    poses[0].dtheta = 0.0f;
    
    // 构建增量方程 H * Δx = b
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(num_variables, num_variables);
    Eigen::VectorXf b = Eigen::VectorXf::Zero(num_variables);
    
    int valid_edges = 0;
    
    for (const auto& edge : edges) {
        // 检查索引有效性
        if (edge.from_id < 0 || edge.from_id >= num_poses ||
            edge.to_id < 0 || edge.to_id >= num_poses) {
            continue;
        }
        
        // 跳过自环边
        if (edge.from_id == edge.to_id) {
            continue;
        }
        
        const auto& pose_from = poses[edge.from_id];
        const auto& pose_to = poses[edge.to_id];
        const auto& relative = edge.relative_transform;
        
        // 计算残差向量
        Eigen::Vector3f residual = computeResidualVector(pose_from, pose_to, relative);
        
        // 计算雅可比矩阵
        Eigen::Matrix<float, 3, 6> J_full = computeResidualJacobian(pose_from, pose_to, relative);
        
        // 计算权重和信息矩阵
        float weight = edge.confidence;
        if (edge.is_odom_edge) {
            weight *= config.odom_weight / config.ndt_weight;
        }
        
        // 限制权重范围
        weight = std::max(0.01f, std::min(1.0f, weight));
        
        Eigen::Matrix3f info = weight * Eigen::Matrix3f::Identity();
        
        // 将全雅可比映射到活跃变量索引
        // 活跃变量索引：跳过顶点0
        int from_idx = (edge.from_id - 1) * 3;
        int to_idx = (edge.to_id - 1) * 3;
        
        bool from_fixed = (edge.from_id == 0);
        bool to_fixed = (edge.to_id == 0);
        
        // 构建当前边的雅可比矩阵（只包含活跃变量）
        Eigen::Matrix<float, 3, Eigen::Dynamic> J(3, num_variables);
        J.setZero();
        
        if (!from_fixed && edge.from_id > 0) {
            J.block<3, 3>(0, from_idx) = J_full.block<3, 3>(0, 0);
        }
        if (!to_fixed && edge.to_id > 0) {
            J.block<3, 3>(0, to_idx) = J_full.block<3, 3>(0, 3);
        }
        
        // 更新 H 和 b
        H += J.transpose() * info * J;
        b += J.transpose() * info * residual;
        
        valid_edges++;
    }
    
    if (valid_edges == 0) {
        ROS_WARN("gaussNewtonGlobalOptimization: 没有有效的边");
        return;
    }
    
    // 添加正则化防止奇异（Levenberg-Marquardt 阻尼）
    float damping = config.global_convergence_threshold;
    for (int i = 0; i < num_variables; i++) {
        H(i, i) += damping;
    }
    
    // 检查 H 矩阵是否奇异
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H);
    float cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    
    if (cond > 1e6f) {
        ROS_WARN("Hessian矩阵病态，条件数=%.2e，增加阻尼", cond);
        damping *= 10.0f;
        for (int i = 0; i < num_variables; i++) {
            H(i, i) += damping;
        }
    }
    
    // 求解增量方程
    Eigen::VectorXf delta;
    
    // 使用 LDLT 分解（对称正定）
    Eigen::LDLT<Eigen::MatrixXf> ldlt(H);
    if (ldlt.info() == Eigen::Success) {
        delta = ldlt.solve(b);
    } else {
        // 回退到 QR 分解
        ROS_WARN("LDLT分解失败，使用QR分解");
        delta = H.colPivHouseholderQr().solve(b);
    }
    
    // 检查求解结果
    if (delta.hasNaN()) {
        ROS_ERROR("增量求解结果包含NaN，跳过本次迭代");
        return;
    }
    
    if (delta.array().abs().maxCoeff() > 10.0f) {
        ROS_WARN("增量过大 (max=%.3f)，限制更新幅度", delta.array().abs().maxCoeff());
        float scale = 10.0f / delta.array().abs().maxCoeff();
        delta *= scale;
    }
    
    // 更新位姿（使用 SE(2) 指数映射）
    for (int i = 1; i < num_poses; i++) {
        int idx = (i - 1) * 3;
        
        if (idx + 2 < delta.size()) {
            // 构建增量变换
            Eigen::Vector3f xi;
            xi(0) = delta(idx);
            xi(1) = delta(idx + 1);
            xi(2) = delta(idx + 2);
            
            // 角度归一化
            while (xi(2) > M_PI) xi(2) -= 2 * M_PI;
            while (xi(2) < -M_PI) xi(2) += 2 * M_PI;
            
            // 指数映射得到增量变换矩阵
            Eigen::Matrix3f T_delta = se2Exp(xi);
            
            // 应用更新: T_new = T_old * T_delta
            Eigen::Matrix3f T_old = toSE2(poses[i]);
            Eigen::Matrix3f T_new = T_old * T_delta;
            poses[i] = fromSE2(T_new);
        }
    }
    
    // 角度归一化
    for (int i = 1; i < num_poses; i++) {
        while (poses[i].dtheta > M_PI) poses[i].dtheta -= 2 * M_PI;
        while (poses[i].dtheta < -M_PI) poses[i].dtheta += 2 * M_PI;
    }
    
    ROS_DEBUG("GN优化完成: 处理了%d条边, H条件数=%.2e", valid_edges, cond);
}
 

AlignmentDiagnostics IntelligentSubmapAligner::getDetailedAlignmentDiagnostics(
    const PointCloudPtr& cloud) {
    
    AlignmentDiagnostics diag;
    
    if (!cloud || cloud->empty()) {
        diag.reason = "点云为空";
        return diag;
    }
    
    // 使用现有的墙壁检测（注意成员变量名）
    FourWallAlignment alignment = checkFourWallAlignment(*cloud);
    
    diag.overall_score = alignment.overall_score;
    diag.is_aligned = (diag.overall_score >= 0.70f);
    diag.reason = alignment.major_issue;
    if (diag.reason.empty()) diag.reason = "对齐质量不足";
    
    // ========== X轴方向（前后墙壁）使用 x_pos_wall 和 x_neg_wall ==========
    bool has_front = alignment.x_pos_wall.wall_exists;
    bool has_back = alignment.x_neg_wall.wall_exists;
    diag.x_walls_detected = (has_front && has_back);
    
    if (has_front && has_back) {
        // 前后墙壁的距离 = 两个墙壁距离的差值
        diag.x_distance_m = fabs(alignment.x_pos_wall.average_distance - 
                                  alignment.x_neg_wall.average_distance);
        
        const float EXPECTED_POOL_LENGTH = 6.12f;
        diag.x_distance_error_m = diag.x_distance_m - EXPECTED_POOL_LENGTH;
        
        diag.x_parallel = alignment.parallelism.x_parallel;
        diag.x_angle_diff_deg = alignment.parallelism.x_angle_diff;
        
        // X方向微调建议
        if (fabs(diag.x_distance_error_m) > 0.1f) {
            diag.suggested_dx_m = -diag.x_distance_error_m * 0.3f;
            diag.suggested_dx_m = std::max(-0.1f, std::min(0.1f, diag.suggested_dx_m));
        }
        
        // 旋转微调建议（基于X方向不平行）
        if (!diag.x_parallel && fabs(diag.x_angle_diff_deg) > 1.0f) {
            diag.suggested_dtheta_deg = -diag.x_angle_diff_deg * 0.3f;
            diag.rotation_error_deg = diag.x_angle_diff_deg;
        }
        
        ROS_INFO("X轴: 前墙距离=%.2fm, 后墙距离=%.2fm, 间距=%.2fm, 误差=%.2fm(%s)",
                 alignment.x_pos_wall.average_distance,
                 alignment.x_neg_wall.average_distance,
                 diag.x_distance_m,
                 fabs(diag.x_distance_error_m),
                 diag.x_distance_error_m > 0 ? "拉远" : "靠近");
    }
    
    // ========== Y轴方向（左右墙壁）使用 y_pos_wall 和 y_neg_wall ==========
    bool has_left = alignment.y_pos_wall.wall_exists;
    bool has_right = alignment.y_neg_wall.wall_exists;
    diag.y_walls_detected = (has_left && has_right);
    
    if (has_left && has_right) {
        // 左右墙壁的距离 = 两个墙壁距离的差值
        diag.y_distance_m = fabs(alignment.y_pos_wall.average_distance - 
                                  alignment.y_neg_wall.average_distance);
        
        const float EXPECTED_POOL_WIDTH = 4.12f;
        diag.y_distance_error_m = diag.y_distance_m - EXPECTED_POOL_WIDTH;
        
        diag.y_parallel = alignment.parallelism.y_parallel;
        diag.y_angle_diff_deg = alignment.parallelism.y_angle_diff;
        
        // Y方向微调建议
        if (fabs(diag.y_distance_error_m) > 0.1f) {
            diag.suggested_dy_m = -diag.y_distance_error_m * 0.3f;
            diag.suggested_dy_m = std::max(-0.1f, std::min(0.1f, diag.suggested_dy_m));
        }
        
        // 如果没有X方向旋转建议，用Y方向
        if (!diag.y_parallel && fabs(diag.y_angle_diff_deg) > 1.0f && diag.suggested_dtheta_deg == 0) {
            diag.suggested_dtheta_deg = -diag.y_angle_diff_deg * 0.3f;
            diag.rotation_error_deg = diag.y_angle_diff_deg;
        }
        
        ROS_INFO("Y轴: 左墙距离=%.2fm, 右墙距离=%.2fm, 间距=%.2fm, 误差=%.2fm(%s)",
                 alignment.y_pos_wall.average_distance,
                 alignment.y_neg_wall.average_distance,
                 diag.y_distance_m,
                 fabs(diag.y_distance_error_m),
                 diag.y_distance_error_m > 0 ? "拉远" : "靠近");
    }
    
    // 限制旋转建议幅度
    diag.suggested_dtheta_deg = std::max(-5.0f, std::min(5.0f, diag.suggested_dtheta_deg));
    
    ROS_INFO("微调建议: dx=%.3fm, dy=%.3fm, dtheta=%.2f°",
             diag.suggested_dx_m, diag.suggested_dy_m, diag.suggested_dtheta_deg);
    
    return diag;
}

 