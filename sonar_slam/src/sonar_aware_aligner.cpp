#include "sonar_slam/sonar_aware_aligner.h"
 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <algorithm>
#include <random>
#include <cmath>

 

namespace sonar_slam {

// ======================== 构造函数和析构函数 ========================
SonarAwareAligner::SonarAwareAligner(ros::NodeHandle& nh)
    : nh_(nh)
    , debug_enabled_(false) 
    {
    
    ROS_INFO("初始化 SonarAwareAligner（声呐感知对齐器）...");
    
    // 修复：使用 double 类型接收参数
    double temp_double;
     // 添加新变量的初始化
    double line_tolerance = 0.2;
    nh.param("sonar_aligner/line_cluster_tolerance", line_tolerance, 0.2);
    line_cluster_tolerance_ = static_cast<float>(line_tolerance);
    
    int min_line_points = 10;
    nh.param("sonar_aligner/min_points_for_line", min_line_points, 10);
    min_points_for_line_ = min_line_points;

    // 方式1：使用 double 临时变量
 
    double min_corner_conf = 0.1;
    nh.param("sonar_aligner/min_corner_confidence", min_corner_conf, 0.1);
    min_corner_confidence_ = static_cast<float>(min_corner_conf);
    
 
    double min_wall_conf = 0.1; 
    nh.param("sonar_aligner/min_wall_confidence", min_wall_conf, 0.1);
    min_wall_confidence_ = static_cast<float>(min_wall_conf);
    
    double max_trans_error = 0.15;
    nh.param("sonar_aligner/max_translation_error", max_trans_error, 0.15);
    max_translation_error_ = static_cast<float>(max_trans_error);
    
    // 处理带 M_PI 的参数
    temp_double = 3.0 * M_PI / 180.0;
    nh.param("sonar_aligner/max_rotation_error", temp_double, temp_double);
    max_rotation_error_ = static_cast<float>(temp_double);
    
    double intensity_weight = 0.3;
    nh.param("sonar_aligner/intensity_weight_factor", intensity_weight, 0.3);
    intensity_weight_factor_ = static_cast<float>(intensity_weight);
    
    double min_feat_dist = 0.2;
    nh.param("sonar_aligner/min_feature_distance", min_feat_dist, 0.2);
    min_feature_distance_ = static_cast<float>(min_feat_dist);
    
    int min_points = 5;
    nh.param("sonar_aligner/min_points_for_wall", min_points, 5);
    min_points_for_wall_ = min_points;
    
   
    double corner_curv = 0.01;
    nh.param("sonar_aligner/corner_curvature_threshold", corner_curv, 0.01);
    corner_curvature_threshold_ = static_cast<float>(corner_curv);
    
    // 创建日志目录
    boost::filesystem::path log_dir("/tmp/sonar_aligner_logs");
    if (!boost::filesystem::exists(log_dir)) {
        boost::filesystem::create_directories(log_dir);
    }
    
    std::string log_filename = "/tmp/sonar_aligner_logs/alignment_" + 
                               std::to_string(static_cast<long>(ros::Time::now().toSec())) + ".log";
    log_file_.open(log_filename, std::ios::out);
    
    ROS_INFO("SonarAwareAligner 初始化完成");
    ROS_INFO("  最小角点置信度: %.2f", min_corner_confidence_);
    ROS_INFO("  最小墙壁置信度: %.2f", min_wall_confidence_);
    ROS_INFO("  最大平移误差: %.3fm", max_translation_error_);
    ROS_INFO("  最大旋转误差: %.1f°", max_rotation_error_ * 180.0 / M_PI);
}




SonarAwareAligner::~SonarAwareAligner() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
    ROS_INFO("SonarAwareAligner 销毁");
}

// ======================== 主对齐接口 ========================
/**
AlignmentResult SonarAwareAligner::align(const PointCloudPtr& cloud1,
                                         const PointCloudPtr& cloud2,
                                         const std::string& hint1,
                                         const std::string& hint2) {
    
    ROS_INFO("========================================");
    ROS_INFO("开始声呐感知对齐（三层架构：强度分级+几何形状+三级对齐）");
    ROS_INFO("  点云1: %lu 点", cloud1->size());
    ROS_INFO("  点云2: %lu 点", cloud2->size());
    ROS_INFO("  提示1: %s", hint1.c_str());
    ROS_INFO("  提示2: %s", hint2.c_str());
    ROS_INFO("========================================");
    
    AlignmentResult result;
    SonarTransformParams transform;   
    
    // ========== 1. 输入检查 ==========
    if (cloud1->empty() || cloud2->empty()) {
        ROS_ERROR("输入点云为空");
        result.diagnostic = "输入点云为空";
        return result;
    }
    
    // ========== 2. 强度分级 ==========
    ROS_INFO("=== 第1阶段：强度分级 ===");
    
    IntensityStats stats1 = extractIntensityStats(cloud1);
    IntensityStats stats2 = extractIntensityStats(cloud2);
    
    ROS_INFO("点云1强度统计: 强=%lu, 中=%lu, 弱=%lu, 范围[%.0f,%.0f]",
             stats1.strong_points.size(), stats1.medium_points.size(), 
             stats1.weak_points.size(), stats1.min_intensity, stats1.max_intensity);
    ROS_INFO("点云2强度统计: 强=%lu, 中=%lu, 弱=%lu, 范围[%.0f,%.0f]",
             stats2.strong_points.size(), stats2.medium_points.size(), 
             stats2.weak_points.size(), stats2.min_intensity, stats2.max_intensity);
    
    // ========== 3. 几何形状提取 ==========
    ROS_INFO("=== 第2阶段：几何形状提取 ===");
    
    ShapeFeatures shape1 = extractShapeFeatures(cloud1, hint1);
    ShapeFeatures shape2 = extractShapeFeatures(cloud2, hint2);
    
    ROS_INFO("点云1形状: 线段=%lu, 角点=%lu, 封闭轮廓=%s, 主方向=%.1f°",
             shape1.lines.size(), shape1.corners.size(),
             shape1.main_contour.is_closed ? "是" : "否",
             shape1.dominant_angle * 180.0 / M_PI);
    ROS_INFO("点云2形状: 线段=%lu, 角点=%lu, 封闭轮廓=%s, 主方向=%.1f°",
             shape2.lines.size(), shape2.corners.size(),
             shape2.main_contour.is_closed ? "是" : "否",
             shape2.dominant_angle * 180.0 / M_PI);
    
    // 特征提取（用于传统匹配）
    FeatureSet features1 = extractFeatures(cloud1, hint1);
    FeatureSet features2 = extractFeatures(cloud2, hint2);
    
    // ========== 4. 基于形状的三级对齐 ==========
    bool use_shape_alignment = shape1.hasEnoughFeatures() && shape2.hasEnoughFeatures();
    
    if (use_shape_alignment) {
        ROS_INFO("=== 第3阶段：基于形状的三级对齐 ===");
        
        SonarTransformParams shape_transform;
        
        // 4.1 一级对齐：主方向对齐
        float dtheta = alignDominantAngles(shape1, shape2);
        shape_transform.dtheta = dtheta;
        ROS_INFO("一级对齐(主方向): dtheta=%.1f°", dtheta * 180.0 / M_PI);
        
        // 4.2 二级对齐：轮廓质心对齐
        if (shape1.main_contour.is_closed && shape2.main_contour.is_closed) {
            alignCentroids(shape1.main_contour, shape2.main_contour, dtheta,
                          shape_transform.dx, shape_transform.dy);
            ROS_INFO("二级对齐(轮廓质心): dx=%.3fm, dy=%.3fm", 
                     shape_transform.dx, shape_transform.dy);
        } else {
            // 无封闭轮廓时使用边界框
            float cx1 = (shape1.main_contour.min_x + shape1.main_contour.max_x) / 2;
            float cy1 = (shape1.main_contour.min_y + shape1.main_contour.max_y) / 2;
            float cx2 = (shape2.main_contour.min_x + shape2.main_contour.max_x) / 2;
            float cy2 = (shape2.main_contour.min_y + shape2.main_contour.max_y) / 2;
            
            float cos_t = cos(dtheta);
            float sin_t = sin(dtheta);
            float cx2_rot = cx2 * cos_t - cy2 * sin_t;
            float cy2_rot = cx2 * sin_t + cy2 * cos_t;
            
            shape_transform.dx = cx1 - cx2_rot;
            shape_transform.dy = cy1 - cy2_rot;
            ROS_INFO("二级对齐(边界框): dx=%.3fm, dy=%.3fm", 
                     shape_transform.dx, shape_transform.dy);
        }
        
        // 4.3 三级对齐：角点精调
        float dx_before = shape_transform.dx;
        float dy_before = shape_transform.dy;
        float dtheta_before = shape_transform.dtheta;
        
        refineWithCorners(shape1, shape2, 
                         shape_transform.dx, shape_transform.dy, 
                         shape_transform.dtheta);
        
        ROS_INFO("三级对齐(角点精调): dx变化=%.3fm, dy变化=%.3fm, dtheta变化=%.1f°",
                 shape_transform.dx - dx_before,
                 shape_transform.dy - dy_before,
                 (shape_transform.dtheta - dtheta_before) * 180.0 / M_PI);
        
        shape_transform.confidence = 0.8f;
        
        // ========== 5. 传统特征匹配（作为备选）==========
        MatchResult matches = matchFeatures(features1, features2);
        SonarTransformParams feature_transform;
        
        if (matches.totalMatches() >= 2) {
            feature_transform = solveTransform(matches, features1, features2);
        }
        
        // ========== 6. 结果融合 ==========
        if (shape_transform.confidence > 0.7) {
            transform = shape_transform;
            result.method = "基于形状的三级对齐";
            ROS_INFO("使用形状对齐结果");
        } else if (feature_transform.confidence > shape_transform.confidence) {
            transform = feature_transform;
            result.method = "传统特征匹配";
            ROS_INFO("使用传统特征匹配结果");
        } else {
            float alpha = shape_transform.confidence / 
                         (shape_transform.confidence + feature_transform.confidence + 1e-6);
            transform.dx = alpha * shape_transform.dx + (1-alpha) * feature_transform.dx;
            transform.dy = alpha * shape_transform.dy + (1-alpha) * feature_transform.dy;
            transform.dtheta = alpha * shape_transform.dtheta + (1-alpha) * feature_transform.dtheta;
            transform.confidence = (shape_transform.confidence + feature_transform.confidence) / 2;
            result.method = "融合对齐";
            ROS_INFO("使用融合对齐 (形状权重=%.2f)", alpha);
        }
    } else {
        // 形状特征不足，直接使用传统匹配
        ROS_WARN("形状特征不足，使用传统特征匹配");
        MatchResult matches = matchFeatures(features1, features2);
        if (matches.totalMatches() >= 2) {
            transform = solveTransform(matches, features1, features2);
            result.method = "传统特征匹配";
        } else {
            // 连传统匹配都不足，使用传感器位置
            ROS_WARN("特征匹配不足，使用传感器位置备选");
            transform.dx = stats2.sensor_x - stats1.sensor_x;
            transform.dy = stats2.sensor_y - stats1.sensor_y;
            transform.dtheta = 0;
            transform.confidence = 0.3;
            result.method = "传感器位置备选";
        }
    }
    
    // ========== 7. 验证与微调 ==========
    ROS_INFO("=== 第4阶段：验证与微调 ===");
    
    float confidence;
    std::string diagnostic;
    bool valid = validateTransform(cloud1, cloud2, transform, confidence, diagnostic);
    
    if (!valid) {
        ROS_WARN("初步求解验证失败: %s", diagnostic.c_str());
        ROS_INFO("尝试基于强度互补性微调...");
        
        SonarTransformParams refined = refineWithIntensity(cloud1, cloud2, transform);
        
        valid = validateTransform(cloud1, cloud2, refined, confidence, diagnostic);
        
        if (valid) {
            transform = refined;
            ROS_INFO("微调后验证通过");
        }
    }
    
    // ========== 8. 构建最终结果 ==========
    if (valid) {
        result.success = true;
        result.transform = transform;
        result.confidence = confidence;
        result.diagnostic = diagnostic;
        
        // 记录使用的约束
        if (shape1.corners.size() >= 2 && shape2.corners.size() >= 2) {
            result.constraints_used.push_back("形状角点约束");
        }
        if (shape1.main_contour.is_closed && shape2.main_contour.is_closed) {
            result.constraints_used.push_back("轮廓约束");
        }
        
        if (confidence > 0.8) {
            result.method = "高置信度" + result.method;
        } else if (confidence > 0.6) {
            result.method = "中等置信度" + result.method;
        } else {
            result.method = "低置信度" + result.method;
        }
        
        ROS_INFO("✅ 对齐成功！置信度: %.2f", confidence);
        ROS_INFO("最终变换: dx=%.3fm, dy=%.3fm, dtheta=%.1f°",
                 transform.dx, transform.dy, transform.dtheta * 180.0 / M_PI);
    } else {
        result.success = false;
        result.diagnostic = diagnostic;
        ROS_ERROR("❌ 对齐失败: %s", diagnostic.c_str());
    }
    
    // 保存调试信息
    if (debug_enabled_) {
        saveDebugInfo("final", features1, features2, transform);
    }
    
    ROS_INFO("========================================");
    return result;
}
***/

// ======================== 主对齐接口（增强版）=======================

AlignmentResult SonarAwareAligner::align(const PointCloudPtr& cloud1,
                                         const PointCloudPtr& cloud2,
                                         const std::string& hint1,
                                         const std::string& hint2) {
    
    ROS_INFO("========================================");
    ROS_INFO("开始声呐感知对齐（三层架构 + 大型水池支持）");
    ROS_INFO("  点云1: %lu 点", cloud1->size());
    ROS_INFO("  点云2: %lu 点", cloud2->size());
    ROS_INFO("  提示1: %s", hint1.c_str());
    ROS_INFO("  提示2: %s", hint2.c_str());
    ROS_INFO("========================================");
    
    AlignmentResult result;
    SonarTransformParams transform;   
    
    // ========== 1. 输入检查 ==========
    if (cloud1->empty() || cloud2->empty()) {
        ROS_ERROR("输入点云为空");
        result.diagnostic = "输入点云为空";
        return result;
    }
    
    // ========== 2. 强度分级 ==========
    ROS_INFO("=== 第1阶段：强度分级 ===");
    
    IntensityStats stats1 = extractIntensityStats(cloud1);
    IntensityStats stats2 = extractIntensityStats(cloud2);
    
    ROS_INFO("点云1强度统计: 强=%lu, 中=%lu, 弱=%lu, 范围[%.0f,%.0f]",
             stats1.strong_points.size(), stats1.medium_points.size(), 
             stats1.weak_points.size(), stats1.min_intensity, stats1.max_intensity);
    ROS_INFO("点云2强度统计: 强=%lu, 中=%lu, 弱=%lu, 范围[%.0f,%.0f]",
             stats2.strong_points.size(), stats2.medium_points.size(), 
             stats2.weak_points.size(), stats2.min_intensity, stats2.max_intensity);
    
    // ========== 3. 几何形状提取 ==========
    ROS_INFO("=== 第2阶段：几何形状提取 ===");
    
    ShapeFeatures shape1 = extractShapeFeatures(cloud1, hint1);
    ShapeFeatures shape2 = extractShapeFeatures(cloud2, hint2);
    
    ROS_INFO("点云1形状: 线段=%lu, 角点=%lu, 封闭轮廓=%s, 主方向=%.1f°",
             shape1.lines.size(), shape1.corners.size(),
             shape1.main_contour.is_closed ? "是" : "否",
             shape1.dominant_angle * 180.0 / M_PI);
    ROS_INFO("点云2形状: 线段=%lu, 角点=%lu, 封闭轮廓=%s, 主方向=%.1f°",
             shape2.lines.size(), shape2.corners.size(),
             shape2.main_contour.is_closed ? "是" : "否",
             shape2.dominant_angle * 180.0 / M_PI);
    
    // 特征提取（用于传统匹配）
    FeatureSet features1 = extractFeatures(cloud1, hint1);
    FeatureSet features2 = extractFeatures(cloud2, hint2);
    
    // ========== 4. 场景判断：中小型水池 vs 大型水池 ==========
    bool is_large_scale = false;
    std::string relation_type = "unknown";
    
    // 判断标准1：没有封闭轮廓（说明只扫描到局部）
    if (!shape1.main_contour.is_closed || !shape2.main_contour.is_closed) {
        is_large_scale = true;
        ROS_INFO("检测到局部扫描（无封闭轮廓），启用大型水池模式");
    }
    
    // 判断标准2：根据提示信息
    if (hint1.find("corner") != std::string::npos || 
        hint2.find("edge") != std::string::npos) {
        is_large_scale = true;
        relation_type = hint1 + "_to_" + hint2;
        ROS_INFO("根据提示信息启用大型水池模式: %s", relation_type.c_str());
    }
    
    // 判断标准3：点云范围很大
    float range1 = std::max(shape1.main_contour.max_x - shape1.main_contour.min_x,
                            shape1.main_contour.max_y - shape1.main_contour.min_y);
    float range2 = std::max(shape2.main_contour.max_x - shape2.main_contour.min_x,
                            shape2.main_contour.max_y - shape2.main_contour.min_y);
    if (range1 > 30.0f || range2 > 30.0f) {  // 提高到30米
        is_large_scale = true;
        ROS_INFO("检测到大范围扫描(>30m)，启用大型水池模式");
    }
    // 对于20-30米之间，让特征决定
    else if ((range1 > 20.0f || range2 > 20.0f) && 
            (!shape1.main_contour.is_closed || !shape2.main_contour.is_closed)) {
        is_large_scale = true;
        ROS_INFO("检测到中等范围但无封闭轮廓，启用大型水池模式");
    }
    
    // ========== 5. 根据场景选择对齐策略 ==========
    bool use_shape_alignment = shape1.hasEnoughFeatures() && shape2.hasEnoughFeatures();
    
    if (is_large_scale && use_shape_alignment) {
        // ========== 大型水池：基于几何约束的对齐 ==========
        ROS_INFO("=== 第3阶段：大型水池几何约束对齐 ===");
        
        SonarTransformParams geo_transform;
        
        // 5.1 分析两次扫描的几何关系
        ScanRelation relation;
        relation.type = UNKNOWN_RELATION;
        relation.confidence = 0.5f;

        // 根据之前判断的结果设置关系
        if (hint1.find("corner") != std::string::npos || relation_type.find("corner") != std::string::npos) {
            if (hint2.find("edge") != std::string::npos || relation_type.find("edge") != std::string::npos) {
                relation.type = CORNER_TO_EDGE;
                relation.confidence = 0.8f;
            }
        } else if (hint1.find("edge") != std::string::npos && hint2.find("edge") != std::string::npos) {
            relation.type = EDGE_TO_EDGE;
            relation.confidence = 0.8f;
        } else {
            // 让 analyzeScanRelation 做自动分析
            relation = analyzeScanRelation(shape1, shape2, relation_type);
        }
        
        // 5.2 根据关系类型选择对齐方法
        switch (relation.type) {
            case CORNER_TO_EDGE:
                ROS_INFO("检测到关系: 角落 → 边延伸");
                geo_transform = alignCornerToEdge(shape1, shape2, relation);
                break;
                
            case EDGE_TO_EDGE:
                ROS_INFO("检测到关系: 边 → 边延伸");
                geo_transform = alignEdgeToEdge(shape1, shape2, relation);
                break;
                
            case PARALLEL_EDGES:
                ROS_INFO("检测到关系: 平行边");
                geo_transform = alignParallelEdges(shape1, shape2, relation);
                break;
                
            default:
                ROS_WARN("未知关系类型，使用主方向对齐");
                geo_transform.dtheta = alignDominantAngles(shape1, shape2);
                break;
        }
        
        geo_transform.confidence = relation.confidence;
        transform = geo_transform;
        result.method = "大型水池几何约束对齐";
        
        // 5.3 验证几何约束
        bool geo_valid = validateGeometricConstraints(cloud1, cloud2, transform, relation);
        if (!geo_valid) {
            ROS_WARN("几何约束验证得分较低，降低置信度");
            transform.confidence *= 0.7f;  // 降低置信度但不放弃
            
            // 同时尝试普通模式作为备选
            SonarTransformParams normal_transform;
            float dtheta = alignDominantAngles(shape1, shape2);
            normal_transform.dtheta = dtheta;
            normal_transform.confidence = 0.5f;
            
            // 选择置信度高的
            if (normal_transform.confidence > transform.confidence) {
                transform = normal_transform;
                result.method = "降级到主方向对齐";
            }
        }
    }
    
    if (!is_large_scale && use_shape_alignment) {
        // ========== 中小型水池：基于形状的三级对齐 ==========
        ROS_INFO("=== 第3阶段：基于形状的三级对齐 ===");
        
        SonarTransformParams shape_transform;
        
        // 4.1 一级对齐：主方向对齐
        float dtheta = alignDominantAngles(shape1, shape2);
        shape_transform.dtheta = dtheta;
        ROS_INFO("一级对齐(主方向): dtheta=%.1f°", dtheta * 180.0 / M_PI);
        
        // 4.2 二级对齐：轮廓质心对齐
        if (shape1.main_contour.is_closed && shape2.main_contour.is_closed) {
            alignCentroids(shape1.main_contour, shape2.main_contour, dtheta,
                          shape_transform.dx, shape_transform.dy);
            ROS_INFO("二级对齐(轮廓质心): dx=%.3fm, dy=%.3fm", 
                     shape_transform.dx, shape_transform.dy);
        } else {
            // 无封闭轮廓时使用边界框
            float cx1 = (shape1.main_contour.min_x + shape1.main_contour.max_x) / 2;
            float cy1 = (shape1.main_contour.min_y + shape1.main_contour.max_y) / 2;
            float cx2 = (shape2.main_contour.min_x + shape2.main_contour.max_x) / 2;
            float cy2 = (shape2.main_contour.min_y + shape2.main_contour.max_y) / 2;
            
            float cos_t = cos(dtheta);
            float sin_t = sin(dtheta);
            float cx2_rot = cx2 * cos_t - cy2 * sin_t;
            float cy2_rot = cx2 * sin_t + cy2 * cos_t;
            
            shape_transform.dx = cx1 - cx2_rot;
            shape_transform.dy = cy1 - cy2_rot;
            ROS_INFO("二级对齐(边界框): dx=%.3fm, dy=%.3fm", 
                     shape_transform.dx, shape_transform.dy);
        }
        
        // 4.3 三级对齐：角点精调
        float dx_before = shape_transform.dx;
        float dy_before = shape_transform.dy;
        float dtheta_before = shape_transform.dtheta;
        
        refineWithCorners(shape1, shape2, 
                         shape_transform.dx, shape_transform.dy, 
                         shape_transform.dtheta);
        
        ROS_INFO("三级对齐(角点精调): dx变化=%.3fm, dy变化=%.3fm, dtheta变化=%.1f°",
                 shape_transform.dx - dx_before,
                 shape_transform.dy - dy_before,
                 (shape_transform.dtheta - dtheta_before) * 180.0 / M_PI);
        
        shape_transform.confidence = 0.8f;
        transform = shape_transform;
        result.method = "基于形状的三级对齐";
    }
    
    // ========== 6. 传统特征匹配（作为备选）==========
    if (!use_shape_alignment) {
        ROS_WARN("形状特征不足，使用传统特征匹配");
        MatchResult matches = matchFeatures(features1, features2);
        if (matches.totalMatches() >= 2) {
            transform = solveTransform(matches, features1, features2);
            result.method = "传统特征匹配";
        } else {
            // 连传统匹配都不足，使用传感器位置
            ROS_WARN("特征匹配不足，使用传感器位置备选");
            transform.dx = stats2.sensor_x - stats1.sensor_x;
            transform.dy = stats2.sensor_y - stats1.sensor_y;
            transform.dtheta = 0;
            transform.confidence = 0.3;
            result.method = "传感器位置备选";
        }
    }
    
    // ========== 7. 验证与微调 ==========
    ROS_INFO("=== 第4阶段：验证与微调 ===");
    
    float confidence;
    std::string diagnostic;
    bool valid = validateTransform(cloud1, cloud2, transform, confidence, diagnostic);
    
    if (!valid) {
        ROS_WARN("初步求解验证失败: %s", diagnostic.c_str());
        ROS_INFO("尝试基于强度互补性微调...");
        
        SonarTransformParams refined = refineWithIntensity(cloud1, cloud2, transform);
        
        valid = validateTransform(cloud1, cloud2, refined, confidence, diagnostic);
        
        if (valid) {
            transform = refined;
            ROS_INFO("微调后验证通过");
        }
    }
    
    // ========== 8. 构建最终结果 ==========
    if (valid) {
        result.success = true;
        result.transform = transform;
        result.confidence = confidence;
        result.diagnostic = diagnostic;
        
        // 记录使用的约束
        if (shape1.corners.size() >= 2 && shape2.corners.size() >= 2) {
            result.constraints_used.push_back("形状角点约束");
        }
        if (shape1.main_contour.is_closed && shape2.main_contour.is_closed) {
            result.constraints_used.push_back("轮廓约束");
        }
        
        if (confidence > 0.8) {
            result.method = "高置信度" + result.method;
        } else if (confidence > 0.6) {
            result.method = "中等置信度" + result.method;
        } else {
            result.method = "低置信度" + result.method;
        }
        
        ROS_INFO("✅ 对齐成功！置信度: %.2f", confidence);
        ROS_INFO("最终变换: dx=%.3fm, dy=%.3fm, dtheta=%.1f°",
                 transform.dx, transform.dy, transform.dtheta * 180.0 / M_PI);
    } else {
        result.success = false;
        result.diagnostic = diagnostic;
        ROS_ERROR("❌ 对齐失败: %s", diagnostic.c_str());
    }
    
    // 保存调试信息
    if (debug_enabled_) {
        saveDebugInfo("final", features1, features2, transform);
    }
    
    ROS_INFO("========================================");
    return result;
}
// ======================== 特征提取 ========================

FeatureSet SonarAwareAligner::extractFeatures(const PointCloudPtr& cloud,
                                              const std::string& hint) {
    
    FeatureSet features;
    features.scan_hint = hint;

    // ========== 全局诊断 ==========
    ROS_INFO("====== 特征提取诊断 (%s) ======", hint.c_str());
    ROS_INFO("点云总点数: %lu", cloud->size());
    
    // 1. 强度分布
    float sum_intensity = 0;
    float max_intensity_val = 0, min_intensity_val = 255;
    int high_quality = 0, medium_quality = 0, low_quality = 0;
    
    for (const auto& p : cloud->points) {
        sum_intensity += p.intensity;
        if (p.intensity > max_intensity_val) max_intensity_val = p.intensity;
        if (p.intensity < min_intensity_val) min_intensity_val = p.intensity;
        
        if (p.intensity >= 160) high_quality++;
        else if (p.intensity >= 140) medium_quality++;
        else if (p.intensity >= 100) low_quality++;
    }
    
    ROS_INFO("强度范围: [%.0f, %.0f], 平均: %.0f", 
             min_intensity_val, max_intensity_val, sum_intensity/cloud->size());
    ROS_INFO("点质量: 高≥160=%d, 中140-159=%d, 低100-139=%d", 
             high_quality, medium_quality, low_quality);
    
    // 2. 空间分布（简单检查）
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    
    for (const auto& p : cloud->points) {
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
    }
    
    ROS_INFO("X范围: [%.2f, %.2f], 宽度: %.2fm", min_x, max_x, max_x - min_x);
    ROS_INFO("Y范围: [%.2f, %.2f], 宽度: %.2fm", min_y, max_y, max_y - min_y);
    
    // 1. 估计传感器位置
    float max_intensity;
    estimateSensorPosition(cloud, features.sensor_x, features.sensor_y, max_intensity);
    features.max_intensity = max_intensity;
    
    // 2. 检测角点（使用宽松阈值）
    features.corners = detectCorners(cloud, features.sensor_x, features.sensor_y, max_intensity);
    
    // 3. 提取墙壁
    features.walls = extractWalls(cloud, features.sensor_x, features.sensor_y, max_intensity);
    
    // 4. 如果特征仍然不足，尝试使用更宽松的参数再提取一次
    if (features.corners.size() < 2 && features.walls.size() < 2) {
        ROS_WARN("特征不足，尝试使用宽松参数重新提取...");
        
        // 临时降低阈值
        float old_curvature = corner_curvature_threshold_;
        float old_wall_conf = min_wall_confidence_;
        
        corner_curvature_threshold_ = 0.03;  // 更低的曲率阈值
        min_wall_confidence_ = 0.2;          // 更低的墙壁置信度
        
        features.corners = detectCorners(cloud, features.sensor_x, features.sensor_y, max_intensity);
        features.walls = extractWalls(cloud, features.sensor_x, features.sensor_y, max_intensity);
        
        // 恢复原阈值
        corner_curvature_threshold_ = old_curvature;
        min_wall_confidence_ = old_wall_conf;
    }
    
    // 5. 建立墙壁和角点的连接关系
    for (auto& wall : features.walls) {
        for (size_t i = 0; i < features.corners.size(); i++) {
            const auto& corner = features.corners[i];
            
            float dist_to_start = distance(corner.x, corner.y, wall.x1, wall.y1);
            float dist_to_end = distance(corner.x, corner.y, wall.x2, wall.y2);
            
            if (dist_to_start < 0.3 || dist_to_end < 0.3) {
                wall.corner_ids.push_back(i);
            }
        }
    }

    // ========== 6. 添加多方向墙壁检测日志 ==========
    int walls_x_pos = 0, walls_x_neg = 0, walls_y_pos = 0, walls_y_neg = 0;
    int walls_other = 0;
    
    for (const auto& wall : features.walls) {
        // 计算墙壁中点
        float mid_x = (wall.x1 + wall.x2) / 2;
        float mid_y = (wall.y1 + wall.y2) / 2;
        
        // 墙壁角度（已归一化到0-π）
        float angle = wall.angle;
        
        // 判断方向（带10度容差）
        if (angle < 10.0 * M_PI / 180.0 || angle > 170.0 * M_PI / 180.0) {
            // 近似水平方向的墙壁（Y方向墙壁）
            if (mid_y > 0) walls_y_pos++;
            else walls_y_neg++;
        }
        else if (angle > 80.0 * M_PI / 180.0 && angle < 100.0 * M_PI / 180.0) {
            // 近似垂直方向的墙壁（X方向墙壁）
            if (mid_x > 0) walls_x_pos++;
            else walls_x_neg++;
        }
        else {
            walls_other++;
        }
    }
    
    ROS_INFO("墙壁方向统计:");
    ROS_INFO("  X方向 (垂直墙): 右侧=%d, 左侧=%d", walls_x_pos, walls_x_neg);
    ROS_INFO("  Y方向 (水平墙): 上方=%d, 下方=%d", walls_y_pos, walls_y_neg);
    ROS_INFO("  其他方向: %d", walls_other);
    
    return features;
}



void SonarAwareAligner::estimateSensorPosition(const PointCloudPtr& cloud,
                                               float& sensor_x, 
                                               float& sensor_y,
                                               float& max_intensity) {
    
    if (cloud->empty()) {
        sensor_x = sensor_y = 0;
        max_intensity = 0;
        return;
    }
    
    // 1. 找到最高强度点
    max_intensity = 0;
    PointType max_intensity_point;
    
    for (const auto& p : cloud->points) {
        if (p.intensity > max_intensity) {
            max_intensity = p.intensity;
            max_intensity_point = p;
        }
    }
    
    // 2. 收集高强度点（>80%最大强度）
    std::vector<PointType> high_intensity_points;
    for (const auto& p : cloud->points) {
        if (p.intensity > max_intensity * 0.8f) {
            high_intensity_points.push_back(p);
        }
    }
    
    // 3. 如果高强度点足够多，用它们的中心作为传感器位置
    if (high_intensity_points.size() >= 5) {
        float sum_x = 0, sum_y = 0;
        for (const auto& p : high_intensity_points) {
            sum_x += p.x;
            sum_y += p.y;
        }
        sensor_x = sum_x / high_intensity_points.size();
        sensor_y = sum_y / high_intensity_points.size();
    } else {
        // 否则用最高强度点
        sensor_x = max_intensity_point.x;
        sensor_y = max_intensity_point.y;
    }
}
std::vector<CornerFeature> SonarAwareAligner::detectCorners(const PointCloudPtr& cloud,
                                                            float sensor_x, 
                                                            float sensor_y,
                                                            float max_intensity) {
    
    std::vector<CornerFeature> corners;
    
    if (cloud->size() < 50) return corners;
    
    // 1. 计算法向量和曲率
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.15);
    ne.compute(*normals);
    
    // 2. 收集曲率大的点作为角点候选
    // 修复：使用结构体而不是pair来存储，避免排序问题
    struct CandidatePoint {
        float curvature;
        float x, y;
        float intensity;
    };
    std::vector<CandidatePoint> candidates;
    
    for (size_t i = 0; i < cloud->size(); i++) {
        if (normals->points[i].curvature > corner_curvature_threshold_ &&
            cloud->points[i].intensity > max_intensity * 0.1f) {
            
            CandidatePoint cand;
            cand.curvature = normals->points[i].curvature;
            cand.x = cloud->points[i].x;
            cand.y = cloud->points[i].y;
            cand.intensity = cloud->points[i].intensity;
            candidates.push_back(cand);
        }
    }
    
    // 3. 按曲率排序（使用自定义比较函数）
    std::sort(candidates.begin(), candidates.end(),
              [](const CandidatePoint& a, const CandidatePoint& b) {
                  return a.curvature > b.curvature;
              });
    
    // 4. 非极大值抑制
    std::vector<bool> used(candidates.size(), false);
    
    for (size_t i = 0; i < candidates.size(); i++) {
        if (used[i]) continue;
        
        CornerFeature corner;
        corner.x = candidates[i].x;
        corner.y = candidates[i].y;
        corner.intensity = candidates[i].intensity;
        
        // 计算相对强度
        corner.relative_intensity = corner.intensity / max_intensity;
        
        // 计算到传感器的距离
        corner.distance_from_sensor = distance(corner.x, corner.y, sensor_x, sensor_y);
        
        // 计算角度（使用周围点的方向）
        std::vector<float> directions;
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                directions.push_back(atan2(dy, dx));
            }
        }
        if (!directions.empty()) {
            std::sort(directions.begin(), directions.end());
            corner.angle = directions[directions.size() / 2];
        }
        
        // 计算置信度
        float curvature_score = candidates[i].curvature * 5.0f;
        float intensity_score = corner.relative_intensity;
        corner.confidence = (curvature_score + intensity_score) / 2.0f;
        corner.confidence = std::min(1.0f, corner.confidence);
        
        if (corner.confidence >= min_corner_confidence_) {
            corner.id = corners.size();
            corners.push_back(corner);
        }
        
        // 标记邻近候选点为已使用
        for (size_t j = i + 1; j < candidates.size(); j++) {
            float dist = distance(corner.x, corner.y,
                                  candidates[j].x, candidates[j].y);
            if (dist < min_feature_distance_) {
                used[j] = true;
            }
        }
    }
    
    return corners;
}

std::vector<WallFeature> SonarAwareAligner::extractWalls(const PointCloudPtr& cloud,
                                                         float sensor_x,
                                                         float sensor_y,
                                                         float max_intensity) {
    
    std::vector<WallFeature> walls;
    
    // 1. 欧几里得聚类，分离不同的墙壁
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(min_points_for_wall_);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

     ROS_INFO("  聚类数量: %lu", cluster_indices.size()); 
    
    // 2. 对每个聚类拟合直线
    for (const auto& indices : cluster_indices) {
        // 提取聚类点
        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>);
        for (int idx : indices.indices) {
            cluster->push_back(cloud->points[idx]);
        }
        
        if (cluster->size() < (size_t)min_points_for_wall_) continue;
        
        // 使用RANSAC拟合直线
        pcl::SampleConsensusModelLine<PointType>::Ptr
            model(new pcl::SampleConsensusModelLine<PointType>(cluster));
        
        pcl::RandomSampleConsensus<PointType> ransac(model);
        ransac.setDistanceThreshold(0.03);
        ransac.computeModel();
        
        std::vector<int> inliers;
        ransac.getInliers(inliers);
        
         ROS_INFO("    内点数量: %lu, 内点比例: %.2f", 
                 inliers.size(), (float)inliers.size()/cluster->size());  // 添加这行

        if (inliers.size() < cluster->size() * 0.2) continue;
        
        // 获取直线参数
        Eigen::VectorXf coeff;
        ransac.getModelCoefficients(coeff);
        
        WallFeature wall;
        
        // 计算直线端点
        Eigen::Vector3f point_on_line(coeff[0], coeff[1], coeff[2]);
        Eigen::Vector3f direction(coeff[3], coeff[4], coeff[5]);
        direction.normalize();
        
        float t_min = FLT_MAX, t_max = -FLT_MAX;
        for (int idx : inliers) {
            const auto& p = cluster->points[idx];
            Eigen::Vector3f vec(p.x - point_on_line.x(),
                               p.y - point_on_line.y(),
                               p.z - point_on_line.z());
            float t = vec.dot(direction);
            t_min = std::min(t_min, t);
            t_max = std::max(t_max, t);
        }
        
        wall.x1 = point_on_line.x() + t_min * direction.x();
        wall.y1 = point_on_line.y() + t_min * direction.y();
        wall.x2 = point_on_line.x() + t_max * direction.x();
        wall.y2 = point_on_line.y() + t_max * direction.y();
        
        wall.length = distance(wall.x1, wall.y1, wall.x2, wall.y2);
        wall.angle = atan2(direction.y(), direction.x());
        if (wall.angle < 0) wall.angle += M_PI;
        
        // 计算直线法线式参数
        wall.a = -direction.y();
        wall.b = direction.x();
        float norm = sqrt(wall.a * wall.a + wall.b * wall.b);
        if (norm > 1e-6) {
            wall.a /= norm;
            wall.b /= norm;
        }
        wall.c = -(wall.a * wall.x1 + wall.b * wall.y1);
        
        // 计算强度统计
        float sum_intensity = 0;
        wall.max_intensity = 0;
        wall.min_intensity = 255;
        
        for (int idx : inliers) {
            float intensity = cluster->points[idx].intensity;
            sum_intensity += intensity;
            wall.max_intensity = std::max(wall.max_intensity, intensity);
            wall.min_intensity = std::min(wall.min_intensity, intensity);
        }
        
        wall.avg_intensity = sum_intensity / inliers.size();
        
        // 计算强度梯度
        wall.intensity_gradient = computeIntensityGradient(cluster, wall);
        
        // 计算到传感器的平均距离
        float mid_x = (wall.x1 + wall.x2) / 2;
        float mid_y = (wall.y1 + wall.y2) / 2;
        wall.distance_to_sensor = distance(mid_x, mid_y, sensor_x, sensor_y);
        
        // 计算观测角度
        wall.viewing_angle = atan2(mid_y - sensor_y, mid_x - sensor_x);
        
        // 计算置信度
        float inlier_ratio = static_cast<float>(inliers.size()) / cluster->size();
        float intensity_score = wall.avg_intensity / max_intensity;
        wall.confidence = 0.6f * inlier_ratio + 0.4f * intensity_score;
        
        if (wall.confidence >= min_wall_confidence_ && wall.length > 0.3) {
            wall.id = walls.size();
            walls.push_back(wall);
        }
    }
    
    return walls;
}

float SonarAwareAligner::computeIntensityGradient(const PointCloudPtr& cloud,
                                                  const WallFeature& wall) {
    
    if (cloud->size() < 5) return 0;
    
    // 沿墙壁方向投影点
    std::vector<std::pair<float, float>> projected_points;
    
    float dx = wall.x2 - wall.x1;
    float dy = wall.y2 - wall.y1;
    float length = wall.length;
    
    if (length < 0.1) return 0;
    
    for (const auto& p : cloud->points) {
        // 计算点到直线的投影参数
        float t = ((p.x - wall.x1) * dx + (p.y - wall.y1) * dy) / (length * length);
        if (t >= 0 && t <= 1) {
            projected_points.push_back({t, p.intensity});
        }
    }
    
    if (projected_points.size() < 5) return 0;
    
    // 排序
    std::sort(projected_points.begin(), projected_points.end());
    
    // 计算梯度（简单线性拟合）
    float sum_t = 0, sum_i = 0, sum_tt = 0, sum_ti = 0;
    for (const auto& [t, i] : projected_points) {
        sum_t += t;
        sum_i += i;
        sum_tt += t * t;
        sum_ti += t * i;
    }
    
    float n = projected_points.size();
    if (n * sum_tt - sum_t * sum_t == 0) return 0;
    
    float gradient = (n * sum_ti - sum_t * sum_i) / (n * sum_tt - sum_t * sum_t);
    
    return gradient;
}

void SonarAwareAligner::buildCornerNeighborhood(std::vector<CornerFeature>& corners) {
    
    for (size_t i = 0; i < corners.size(); i++) {
        auto& corner = corners[i];
        
        // 找到最近的3个角点
        std::vector<std::pair<float, int>> neighbors;
        
        for (size_t j = 0; j < corners.size(); j++) {
            if (i == j) continue;
            
            float dist = distance(corner.x, corner.y,
                                  corners[j].x, corners[j].y);
            neighbors.push_back({dist, j});
        }
        
        std::sort(neighbors.begin(), neighbors.end());
        
        // 取前3个
        for (int k = 0; k < std::min(3, (int)neighbors.size()); k++) {
            int idx = neighbors[k].second;
            corner.neighbor_ids.push_back(idx);
            corner.neighbor_dists.push_back(neighbors[k].first);
            
            float angle = atan2(corners[idx].y - corner.y,
                                corners[idx].x - corner.x);
            corner.neighbor_angles.push_back(angle);
        }
    }
}

// ======================== 特征匹配 ========================

MatchResult SonarAwareAligner::matchFeatures(const FeatureSet& f1,
                                             const FeatureSet& f2) {
    
    MatchResult matches;
    
    // ========== 1. 角点匹配 ==========
    std::vector<std::tuple<float, int, int>> corner_similarities;
    
    for (size_t i = 0; i < f1.corners.size(); i++) {
        for (size_t j = 0; j < f2.corners.size(); j++) {
            float sim = computeCornerSimilarity(f1.corners[i], f2.corners[j]);
            if (sim > 0.6) {
                corner_similarities.push_back({sim, i, j});
            }
        }
    }
    
    // 按相似度排序
    std::sort(corner_similarities.begin(), corner_similarities.end(),
              std::greater<std::tuple<float, int, int>>());
    
    // 贪心匹配
    std::vector<bool> used1(f1.corners.size(), false);
    std::vector<bool> used2(f2.corners.size(), false);
    
    for (const auto& [sim, i, j] : corner_similarities) {
        if (!used1[i] && !used2[j]) {
            matches.corner_matches.push_back({i, j});
            matches.confidences.push_back(sim);
            used1[i] = true;
            used2[j] = true;
        }
    }
    
    // ========== 2. 墙壁匹配 ==========
    std::vector<std::tuple<float, int, int>> wall_similarities;
    
    for (size_t i = 0; i < f1.walls.size(); i++) {
        for (size_t j = 0; j < f2.walls.size(); j++) {
            float sim = computeWallSimilarity(f1.walls[i], f2.walls[j],
                                              f1.scan_hint, f2.scan_hint);
            if (sim > 0.5) {
                wall_similarities.push_back({sim, i, j});
            }
        }
    }
    
    std::sort(wall_similarities.begin(), wall_similarities.end(),
              std::greater<std::tuple<float, int, int>>());
    
    std::vector<bool> wall_used1(f1.walls.size(), false);
    std::vector<bool> wall_used2(f2.walls.size(), false);
    
    for (const auto& [sim, i, j] : wall_similarities) {
        if (!wall_used1[i] && !wall_used2[j]) {
            matches.wall_matches.push_back({i, j});
            matches.confidences.push_back(sim);
            wall_used1[i] = true;
            wall_used2[j] = true;
        }
    }
    
    return matches;
}
 
float SonarAwareAligner::computeCornerSimilarity(const CornerFeature& c1,
                                                 const CornerFeature& c2) {
    
    // 1. 几何角度相似度（旋转不变）
    float angle_diff = fabs(normalizeAngle(c1.angle - c2.angle));
    float max_angle_diff = static_cast<float>(M_PI/4);
    float angle_score = 1.0f - std::min(1.0f, angle_diff / max_angle_diff);
    
    // 2. 邻居拓扑相似度（旋转和平移不变）
    float topo_score = 0;
    int min_neighbors = std::min(c1.neighbor_ids.size(), c2.neighbor_ids.size());
    
    if (min_neighbors > 0) {
        float dist_diff_sum = 0;
        for (size_t k = 0; k < (size_t)min_neighbors; k++) {
            dist_diff_sum += fabs(c1.neighbor_dists[k] - c2.neighbor_dists[k]);
        }
        float dist_score = 1.0f - std::min(1.0f, dist_diff_sum / (min_neighbors * 0.5f));
        
        float angle_diff_sum = 0;
        for (size_t k = 0; k < (size_t)min_neighbors; k++) {
            float diff = fabs(normalizeAngle(c1.neighbor_angles[k] -
                                            c2.neighbor_angles[k]));
            angle_diff_sum += diff;
        }
        float max_neighbor_angle = static_cast<float>(M_PI/4);
        float neighbor_angle_score = 1.0f - std::min(1.0f, 
            angle_diff_sum / (min_neighbors * max_neighbor_angle));
        
        topo_score = (dist_score + neighbor_angle_score) / 2.0f;
    } else {
        topo_score = 0.5f;
    }
    
    // 3. 强度模式相似度（考虑相对强度）
    float intensity_diff = fabs(c1.relative_intensity - c2.relative_intensity);
    float intensity_score = 1.0f - intensity_diff;
    
    // 加权综合
    return 0.4f * angle_score + 0.4f * topo_score + 0.2f * intensity_score;
}
 
float SonarAwareAligner::computeWallSimilarity(const WallFeature& w1,
                                               const WallFeature& w2,
                                               const std::string& hint1,
                                               const std::string& hint2) {
    
    // 1. 几何角度相似度
    float angle_diff = fabs(normalizeAngle(w1.angle - w2.angle));
    float max_angle_diff = static_cast<float>(M_PI/8);
    float angle_score = 1.0f - std::min(1.0f, angle_diff / max_angle_diff);
    
    // 2. 长度相似度
    float length_ratio = std::min(w1.length, w2.length) / 
                        std::max(w1.length, w2.length);
    
    // 3. 强度互补性（关键特性）
    float intensity_complement = 1.0f;
    
    if (hint1 == "right_side" && hint2 == "left_side") {
        float expected_w2_intensity = 255.0f - w1.avg_intensity;
        intensity_complement = 1.0f - fabs(w2.avg_intensity - expected_w2_intensity) / 255.0f;
    } else {
        intensity_complement = 1.0f - fabs(w1.avg_intensity - w2.avg_intensity) / 255.0f;
    }
    
    // 4. 距离一致性
    float dist_diff = fabs(w1.distance_to_sensor - w2.distance_to_sensor);
    float dist_score = 1.0f - std::min(1.0f, dist_diff / 2.0f);
    
    return 0.3f * angle_score + 
           0.2f * length_ratio + 
           0.4f * intensity_complement + 
           0.1f * dist_score;
}

// ======================== 变换求解 ========================

SonarTransformParams SonarAwareAligner::solveTransform(const MatchResult& matches,
                                                  const FeatureSet& f1,
                                                  const FeatureSet& f2) {
    
    SonarTransformParams transform;
    
    // 1. 如果有角点匹配，优先使用角点
    if (matches.corner_matches.size() >= 2) {
        std::vector<Eigen::Vector2d> points1, points2;
        std::vector<float> weights;
        
        for (const auto& match : matches.corner_matches) {
            const auto& c1 = f1.corners[match.first];
            const auto& c2 = f2.corners[match.second];
            
            points1.push_back(Eigen::Vector2d(c1.x, c1.y));
            points2.push_back(Eigen::Vector2d(c2.x, c2.y));
            
            // 权重基于置信度
            weights.push_back((c1.confidence + c2.confidence) / 2.0f);
        }
        
        transform = solveWeightedSVD(points1, points2, weights);
        
        // 如果角点求解置信度高，直接返回
        if (transform.confidence > 0.8) {
            return transform;
        }
    }
    
    // 2. 如果有墙壁匹配，使用墙壁求解
    if (matches.wall_matches.size() >= 2) {
        // 求解旋转
        float dtheta = solveRotationFromWalls(matches.wall_matches, f1, f2);
        transform.dtheta = dtheta;
        
        // 求解平移
        auto [dx, dy] = solveTranslationFromWalls(dtheta, matches.wall_matches, f1, f2);
        transform.dx = dx;
        transform.dy = dy;
        
        // 计算置信度
        float conf_sum = 0;
        for (const auto& match : matches.wall_matches) {
            conf_sum += (f1.walls[match.first].confidence + 
                        f2.walls[match.second].confidence) / 2.0f;
        }
        transform.confidence = conf_sum / matches.wall_matches.size() * 0.8f;
    }
    
    return transform;
}

SonarTransformParams SonarAwareAligner::solveWeightedSVD(
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    const std::vector<float>& weights) {
    
    SonarTransformParams result;
    
    if (points1.size() < 2) {
        return result;
    }
    
    // 计算加权质心
    Eigen::Vector2d centroid1 = Eigen::Vector2d::Zero();
    Eigen::Vector2d centroid2 = Eigen::Vector2d::Zero();
    float weight_sum = 0;
    
    for (size_t i = 0; i < points1.size(); i++) {
        centroid1 += weights[i] * points1[i];
        centroid2 += weights[i] * points2[i];
        weight_sum += weights[i];
    }
    
    centroid1 /= weight_sum;
    centroid2 /= weight_sum;
    
    // 去中心化
    std::vector<Eigen::Vector2d> centered1, centered2;
    for (size_t i = 0; i < points1.size(); i++) {
        centered1.push_back(points1[i] - centroid1);
        centered2.push_back(points2[i] - centroid2);
    }
    
    // 构建加权协方差矩阵
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < points1.size(); i++) {
        H += weights[i] * centered2[i] * centered1[i].transpose();
    }
    
    // SVD分解
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
    
    // 确保是旋转矩阵
    if (R.determinant() < 0) {
        Eigen::Matrix2d V = svd.matrixV();
        V.col(1) *= -1;
        R = V * svd.matrixU().transpose();
    }
    
    // 提取参数
    result.dtheta = atan2(R(1,0), R(0,0));
    result.dx = centroid1.x() - (R * centroid2).x();
    result.dy = centroid1.y() - (R * centroid2).y();
    
    // 计算残差作为置信度
    float total_error = 0;
    for (size_t i = 0; i < points1.size(); i++) {
        Eigen::Vector2d transformed = R * centered2[i] + centroid1;
        total_error += weights[i] * (points1[i] - transformed).norm();
    }
    float avg_error = total_error / weight_sum;
    result.confidence = 1.0f - std::min(1.0f, avg_error / max_translation_error_);
    
    return result;
}

float SonarAwareAligner::solveRotationFromWalls(
    const std::vector<std::pair<int,int>>& wall_matches,
    const FeatureSet& f1,
    const FeatureSet& f2) {
    
    float sum_dtheta = 0;
    float sum_weights = 0;
    
    for (const auto& match : wall_matches) {
        const auto& w1 = f1.walls[match.first];
        const auto& w2 = f2.walls[match.second];
        
        float angle_diff = w2.angle - w1.angle;
        angle_diff = normalizeAngle(angle_diff);
        
        float weight = (w1.confidence + w2.confidence) / 2.0f;
        
        sum_dtheta += weight * angle_diff;
        sum_weights += weight;
    }
    
    if (sum_weights < 1e-6) return 0;
    
    return sum_dtheta / sum_weights;
}

std::pair<float, float> SonarAwareAligner::solveTranslationFromWalls(
    float dtheta,
    const std::vector<std::pair<int,int>>& wall_matches,
    const FeatureSet& f1,
    const FeatureSet& f2) {
    
    if (wall_matches.size() < 2) {
        return {0, 0};
    }
    
    // 构建方程组：对于每条匹配的墙，变换后应该重合
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    Eigen::Vector2d b = Eigen::Vector2d::Zero();
    
    float cos_theta = cos(dtheta);
    float sin_theta = sin(dtheta);
    
    for (const auto& match : wall_matches) {
        const auto& w1 = f1.walls[match.first];
        const auto& w2 = f2.walls[match.second];
        
        float weight = (w1.confidence + w2.confidence) / 2.0f;
        
        // 变换墙2的法向量
        float a2t = w2.a * cos_theta - w2.b * sin_theta;
        float b2t = w2.a * sin_theta + w2.b * cos_theta;
        
        // 取墙上一点（中点）
        float mx2 = (w2.x1 + w2.x2) / 2;
        float my2 = (w2.y1 + w2.y2) / 2;
        
        // 变换后的点
        float tx2 = mx2 * cos_theta - my2 * sin_theta;
        float ty2 = mx2 * sin_theta + my2 * cos_theta;
        
        // 计算变换后的c参数
        float c2t = -(a2t * tx2 + b2t * ty2);
        
        // 方程：a1*x + b1*y + c1 = a2t*x + b2t*y + c2t
        // 即：(a1 - a2t)*x + (b1 - b2t)*y = c2t - c1
        float da = w1.a - a2t;
        float db = w1.b - b2t;
        float dc = c2t - w1.c;
        
        // 添加到方程组（带权重）
        A(0,0) += weight * da * da;
        A(0,1) += weight * da * db;
        A(1,0) += weight * db * da;
        A(1,1) += weight * db * db;
        
        b(0) += weight * da * dc;
        b(1) += weight * db * dc;
    }
    
    // 求解最小二乘
    Eigen::Vector2d t = A.ldlt().solve(b);
    
    return {static_cast<float>(t.x()), static_cast<float>(t.y())};
}

// ======================== 验证函数 ========================

bool SonarAwareAligner::validateTransform(const PointCloudPtr& cloud1,
                                          const PointCloudPtr& cloud2,
                                          const SonarTransformParams& transform,
                                          float& confidence,
                                          std::string& diagnostic) {
    
    // 1. 检查几何闭合性
    float closure_error = checkGeometricClosure(cloud1, cloud2, transform);
    bool closure_ok = closure_error < max_translation_error_;
    
    // 2. 检查墙壁连续性
    float continuity_error = checkWallContinuity(cloud1, cloud2, transform);
    bool continuity_ok = continuity_error < max_translation_error_ * 1.5f;
    
    // 3. 检查强度互补性
    float complement_score = checkIntensityComplementarity(cloud1, cloud2, transform);
    bool complement_ok = complement_score > 0.5;
    
    // 4. 计算综合置信度
    confidence = 0.4f * (1.0f - closure_error / max_translation_error_) +
                 0.3f * (1.0f - continuity_error / (max_translation_error_ * 1.5f)) +
                 0.3f * complement_score;
    
    confidence = std::min(1.0f, std::max(0.0f, confidence));
    
    // 生成诊断信息
    std::stringstream ss;
    ss << "闭合=" << std::fixed << std::setprecision(3) << closure_error << "m"
       << ", 连续=" << continuity_error << "m"
       << ", 互补=" << std::setprecision(2) << complement_score;
    diagnostic = ss.str();
    
    // 所有检查必须通过
    return closure_ok && continuity_ok && complement_ok;
}

float SonarAwareAligner::checkGeometricClosure(const PointCloudPtr& cloud1,
                                               const PointCloudPtr& cloud2,
                                               const SonarTransformParams& transform) {
    
    // 变换cloud2
    PointCloudPtr transformed2(new PointCloud);
    Eigen::Matrix4f T = transform.getMatrix();
    pcl::transformPointCloud(*cloud2, *transformed2, T);
    
    // 构建KD树
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud1);
    
    // 计算平均最近距离
    float total_dist = 0;
    int valid_points = 0;
    
    for (const auto& p : transformed2->points) {
        if (p.intensity < 100) continue;  // 忽略低质量点
        
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        
        if (kdtree.nearestKSearch(p, 1, indices, distances) > 0) {
            total_dist += sqrt(distances[0]);
            valid_points++;
        }
    }
    
    if (valid_points == 0) return max_translation_error_;
    
    return total_dist / valid_points;
}

float SonarAwareAligner::checkWallContinuity(const PointCloudPtr& cloud1,
                                             const PointCloudPtr& cloud2,
                                             const SonarTransformParams& transform) {
    
    // 提取墙壁（简化版，仅用于验证）
    auto extractWallsSimple = [this](const PointCloudPtr& cloud) {
        std::vector<WallFeature> walls;
        float dummy_sensor_x = 0, dummy_sensor_y = 0, dummy_max_intensity = 255;
        walls = extractWalls(cloud, dummy_sensor_x, dummy_sensor_y, dummy_max_intensity);
        return walls;
    };
    
    auto walls1 = extractWallsSimple(cloud1);
    auto walls2 = extractWallsSimple(cloud2);
    
    if (walls1.empty() || walls2.empty()) {
        return 0.1f;
    }
    
    // 变换墙壁2
    Eigen::Matrix4f T = transform.getMatrix();
    
    float total_error = 0;
    int valid_pairs = 0;
    
    for (const auto& w2 : walls2) {
        // 变换墙壁2的端点
        Eigen::Vector4f p1(w2.x1, w2.y1, 0, 1);
        Eigen::Vector4f p2(w2.x2, w2.y2, 0, 1);
        Eigen::Vector4f tp1 = T * p1;
        Eigen::Vector4f tp2 = T * p2;
        
        // 找最近的墙壁1
        float min_error = 1e6;
        for (const auto& w1 : walls1) {
            // 计算点到直线的距离
            float d1 = pointToLineDistance(tp1.x(), tp1.y(), w1.a, w1.b, w1.c);
            float d2 = pointToLineDistance(tp2.x(), tp2.y(), w1.a, w1.b, w1.c);
            float error = (d1 + d2) / 2;
            
            min_error = std::min(min_error, error);
        }
        
        total_error += min_error;
        valid_pairs++;
    }
    
    if (valid_pairs == 0) return max_translation_error_;
    
    return total_error / valid_pairs;
}

float SonarAwareAligner::checkIntensityComplementarity(const PointCloudPtr& cloud1,
                                                       const PointCloudPtr& cloud2,
                                                       const SonarTransformParams& transform) {
    
    // 变换cloud2
    PointCloudPtr transformed2(new PointCloud);
    Eigen::Matrix4f T = transform.getMatrix();
    pcl::transformPointCloud(*cloud2, *transformed2, T);
    
    // 构建KD树
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud1);
    
    float complement_sum = 0;
    int valid_pairs = 0;
    
    for (const auto& p2 : transformed2->points) {
        if (p2.intensity < 100) continue;
        
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        
        if (kdtree.nearestKSearch(p2, 1, indices, distances) > 0) {
            if (distances[0] < 0.25) {  // 50cm内视为对应点
                const auto& p1 = cloud1->points[indices[0]];
                
                // 检查强度互补性
                // 理想情况下：p1.intensity + p2.intensity ≈ 255
                float sum_intensity = p1.intensity + p2.intensity;
                float complement_error = fabs(sum_intensity - 255.0f) / 255.0f;
                
                complement_sum += (1.0f - complement_error);
                valid_pairs++;
            }
        }
    }
    
    if (valid_pairs == 0) return 0.5f;
    
    return complement_sum / valid_pairs;
}

SonarTransformParams SonarAwareAligner::refineWithIntensity(const PointCloudPtr& cloud1,
                                                       const PointCloudPtr& cloud2,
                                                       const SonarTransformParams& initial) {
    
    SonarTransformParams best_params = initial;
    float best_complement = checkIntensityComplementarity(cloud1, cloud2, initial);
    
    // 在初始变换周围小范围搜索
    const float dx_range = 0.1;
    const float dy_range = 0.1;
    const float dtheta_range = 2.0 * M_PI / 180.0;
    
    for (float dx = -dx_range; dx <= dx_range; dx += 0.02) {
        for (float dy = -dy_range; dy <= dy_range; dy += 0.02) {
            for (float dtheta = -dtheta_range; dtheta <= dtheta_range; dtheta += 0.5 * M_PI / 180.0) {
                
                SonarTransformParams test = initial;
                test.dx += dx;
                test.dy += dy;
                test.dtheta += dtheta;
                
                float complement = checkIntensityComplementarity(cloud1, cloud2, test);
                
                if (complement > best_complement) {
                    best_complement = complement;
                    best_params = test;
                }
            }
        }
    }
    
    best_params.confidence = best_complement;
    return best_params;
}

//========================================================================**
// ======================== 几何轮廓函数实现 ========================

GeometryFeatureSet SonarAwareAligner::extractGeometryFeatures(
    const PointCloudPtr& cloud, const std::string& hint) {
    
    GeometryFeatureSet features;
    features.scan_hint = hint;
    
    ROS_INFO("====== 几何特征提取 (%s) ======", hint.c_str());
    
    // 1. 估计传感器位置（复用原有函数）
    float max_intensity;
    estimateSensorPosition(cloud, features.sensor_x, features.sensor_y, max_intensity);
    
    // 2. 提取线段
    features.lines = extractLineSegments(cloud);
    ROS_INFO("提取到 %lu 条线段", features.lines.size());
    
    // 3. 从线段检测角点
    features.corners = detectCornersFromLines(features.lines);
    ROS_INFO("检测到 %lu 个角点", features.corners.size());
    
    // 4. 跟踪封闭轮廓
    features.main_contour = traceClosedContour(features.lines, features.corners);
    if (features.main_contour.is_closed) {
        ROS_INFO("找到封闭轮廓，面积: %.2f m²", features.main_contour.area);
        computeContourCentroid(features.main_contour, 
                               features.main_contour.centroid_x,
                               features.main_contour.centroid_y);
    }
    
    // 5. 计算边界框
    for (const auto& line : features.lines) {
        features.min_x = std::min({features.min_x, line.x1, line.x2});
        features.max_x = std::max({features.max_x, line.x1, line.x2});
        features.min_y = std::min({features.min_y, line.y1, line.y2});
        features.max_y = std::max({features.max_y, line.y1, line.y2});
    }
    features.width = features.max_x - features.min_x;
    features.height = features.max_y - features.min_y;
    
    ROS_INFO("边界框: [%.2f, %.2f] x [%.2f, %.2f], 大小: %.2f x %.2f",
             features.min_x, features.max_x, features.min_y, features.max_y,
             features.width, features.height);
    
    return features;
}

std::vector<LineSegment> SonarAwareAligner::extractLineSegments(
    const PointCloudPtr& cloud) {
    
    std::vector<LineSegment> lines;
    
    // ========== 1. 提取水平线段（Y值相近的点） ==========
    std::map<int, std::vector<PointType>> y_bins;
    const float BIN_SIZE = 0.1;  // 10cm精度
    
    for (const auto& p : cloud->points) {
        if (p.intensity < 100) continue;  // 只考虑高质量点
        int y_bin = static_cast<int>(p.y / BIN_SIZE);
        y_bins[y_bin].push_back(p);
    }
    
    for (auto& [bin, points] : y_bins) {
        if (points.size() < 10) continue;
        
        // 按X排序
        std::sort(points.begin(), points.end(),
                 [](const PointType& a, const PointType& b) { return a.x < b.x; });
        
        // 找连续区间
        float x_start = points[0].x;
        float x_end = points[0].x;
        std::vector<int> indices;
        indices.push_back(0);
        
        for (size_t i = 1; i < points.size(); i++) {
            if (points[i].x - points[i-1].x < 0.2) {  // 20cm内连续
                x_end = points[i].x;
                indices.push_back(i);
            } else {
                // 保存当前线段
                if (x_end - x_start > 0.5) {  // 长度>50cm
                    LineSegment seg;
                    seg.x1 = x_start;
                    seg.y1 = bin * BIN_SIZE;
                    seg.x2 = x_end;
                    seg.y2 = bin * BIN_SIZE;
                    seg.angle = 0;
                    seg.orientation = "horizontal";
                    seg.length = x_end - x_start;
                    seg.confidence = static_cast<float>(indices.size()) / points.size();
                    seg.point_indices = indices;
                    lines.push_back(seg);
                }
                x_start = points[i].x;
                x_end = points[i].x;
                indices.clear();
                indices.push_back(i);
            }
        }
    }
    
    // ========== 2. 提取垂直线段（X值相近的点） ==========
    std::map<int, std::vector<PointType>> x_bins;
    
    for (const auto& p : cloud->points) {
        if (p.intensity < 100) continue;
        int x_bin = static_cast<int>(p.x / BIN_SIZE);
        x_bins[x_bin].push_back(p);
    }
    
    for (auto& [bin, points] : x_bins) {
        if (points.size() < 10) continue;
        
        std::sort(points.begin(), points.end(),
                 [](const PointType& a, const PointType& b) { return a.y < b.y; });
        
        float y_start = points[0].y;
        float y_end = points[0].y;
        std::vector<int> indices;
        indices.push_back(0);
        
        for (size_t i = 1; i < points.size(); i++) {
            if (points[i].y - points[i-1].y < 0.2) {
                y_end = points[i].y;
                indices.push_back(i);
            } else {
                if (y_end - y_start > 0.5) {
                    LineSegment seg;
                    seg.x1 = bin * BIN_SIZE;
                    seg.y1 = y_start;
                    seg.x2 = bin * BIN_SIZE;
                    seg.y2 = y_end;
                    seg.angle = M_PI/2;
                    seg.orientation = "vertical";
                    seg.length = y_end - y_start;
                    seg.confidence = static_cast<float>(indices.size()) / points.size();
                    seg.point_indices = indices;
                    lines.push_back(seg);
                }
                y_start = points[i].y;
                y_end = points[i].y;
                indices.clear();
                indices.push_back(i);
            }
        }
    }
    
    return lines;
}

std::vector<CornerPoint> SonarAwareAligner::detectCornersFromLines(
    const std::vector<LineSegment>& lines) {
    
    std::vector<CornerPoint> corners;
    
    for (size_t i = 0; i < lines.size(); i++) {
        for (size_t j = i + 1; j < lines.size(); j++) {
            float x, y;
            if (computeLineIntersection(lines[i], lines[j], x, y)) {
                // 检查交点是否在线段上
                if (isPointOnSegment(x, y, lines[i]) && 
                    isPointOnSegment(x, y, lines[j])) {
                    
                    CornerPoint corner;
                    corner.x = x;
                    corner.y = y;
                    corner.line1_idx = i;
                    corner.line2_idx = j;
                    
                    // 计算夹角
                    float angle_diff = fabs(normalizeAngle(lines[i].angle - lines[j].angle));
                    corner.angle = std::min(angle_diff, static_cast<float>(M_PI - angle_diff));
                    
                    // 判断角类型
                    if (fabs(corner.angle - M_PI/2) < 10.0 * M_PI / 180.0) {
                        corner.type = "right";
                    } else if (corner.angle < M_PI/2) {
                        corner.type = "acute";
                    } else {
                        corner.type = "obtuse";
                    }
                    
                    corner.confidence = lines[i].confidence * lines[j].confidence;
                    corners.push_back(corner);
                }
            }
        }
    }
    
    return corners;
}

bool SonarAwareAligner::computeLineIntersection(const LineSegment& l1,
                                                const LineSegment& l2,
                                                float& x, float& y) {
    // 将线段转换为直线方程
    float a1 = l1.y2 - l1.y1;
    float b1 = l1.x1 - l1.x2;
    float c1 = a1 * l1.x1 + b1 * l1.y1;
    
    float a2 = l2.y2 - l2.y1;
    float b2 = l2.x1 - l2.x2;
    float c2 = a2 * l2.x1 + b2 * l2.y1;
    
    float det = a1 * b2 - a2 * b1;
    
    if (fabs(det) < 1e-6) {
        return false;  // 平行
    }
    
    x = (b2 * c1 - b1 * c2) / det;
    y = (a1 * c2 - a2 * c1) / det;
    
    return true;
}

bool SonarAwareAligner::isPointOnSegment(float x, float y, const LineSegment& seg) {
    float min_x = std::min(seg.x1, seg.x2) - 0.1;
    float max_x = std::max(seg.x1, seg.x2) + 0.1;
    float min_y = std::min(seg.y1, seg.y2) - 0.1;
    float max_y = std::max(seg.y1, seg.y2) + 0.1;
    
    return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
}

ClosedContour SonarAwareAligner::traceClosedContour(
    const std::vector<LineSegment>& lines,
    const std::vector<CornerPoint>& corners) {
    
    ClosedContour contour;
    
    if (lines.empty()) return contour;
    
    // 构建线段连接图
    std::map<int, std::vector<int>> adjacency;
    for (size_t i = 0; i < corners.size(); i++) {
        adjacency[corners[i].line1_idx].push_back(corners[i].line2_idx);
        adjacency[corners[i].line2_idx].push_back(corners[i].line1_idx);
    }
    
    // 从任意线段开始跟踪
    std::vector<bool> used(lines.size(), false);
    
    for (size_t start = 0; start < lines.size(); start++) {
        if (used[start]) continue;
        
        std::vector<int> contour_indices;
        std::vector<int> corner_indices;
        
        int current = start;
        int prev = -1;
        
        while (!used[current]) {
            used[current] = true;
            contour_indices.push_back(current);
            
            // 找下一个连接的线段
            int next = -1;
            for (int neighbor : adjacency[current]) {
                if (neighbor != prev && !used[neighbor]) {
                    next = neighbor;
                    // 记录经过的角点
                    for (size_t i = 0; i < corners.size(); i++) {
                        if ((corners[i].line1_idx == current && corners[i].line2_idx == neighbor) ||
                            (corners[i].line1_idx == neighbor && corners[i].line2_idx == current)) {
                            corner_indices.push_back(i);
                            break;
                        }
                    }
                    break;
                }
            }
            
            if (next == -1) break;
            
            prev = current;
            current = next;
        }
        
        // 检查是否闭合（回到起点）
        if (contour_indices.size() >= 3) {
            bool closed = false;
            for (int neighbor : adjacency[contour_indices.back()]) {
                if (neighbor == contour_indices[0]) {
                    closed = true;
                    break;
                }
            }
            
            if (closed) {
                // 构建轮廓
                for (int idx : contour_indices) {
                    contour.lines.push_back(lines[idx]);
                }
                for (int idx : corner_indices) {
                    contour.corners.push_back(corners[idx]);
                }
                contour.is_closed = true;
                contour.area = computePolygonArea(contour.lines);
                
                // 计算周长
                float peri = 0;
                for (const auto& line : contour.lines) {
                    peri += line.length;
                }
                contour.perimeter = peri;
                
                break;
            }
        }
    }
    
    return contour;
}

void SonarAwareAligner::computeContourCentroid(const ClosedContour& contour,
                                               float& cx, float& cy) {
    if (contour.lines.empty()) {
        cx = cy = 0;
        return;
    }
    
    float area = 0;
    float cx_sum = 0, cy_sum = 0;
    
    // 使用多边形质心公式
    for (size_t i = 0; i < contour.lines.size(); i++) {
        const auto& l1 = contour.lines[i];
        const auto& l2 = contour.lines[(i + 1) % contour.lines.size()];
        
        float x1 = (l1.x1 + l1.x2) / 2;
        float y1 = (l1.y1 + l1.y2) / 2;
        float x2 = (l2.x1 + l2.x2) / 2;
        float y2 = (l2.y1 + l2.y2) / 2;
        
        float cross = x1 * y2 - x2 * y1;
        area += cross;
        cx_sum += (x1 + x2) * cross;
        cy_sum += (y1 + y2) * cross;
    }
    
    area /= 2;
    if (fabs(area) > 1e-6) {
        cx = cx_sum / (6 * area);
        cy = cy_sum / (6 * area);
    } else {
        cx = cy = 0;
    }
}

float SonarAwareAligner::computePolygonArea(const std::vector<LineSegment>& lines) {
    if (lines.size() < 3) return 0;
    
    float area = 0;
    for (size_t i = 0; i < lines.size(); i++) {
        const auto& l1 = lines[i];
        const auto& l2 = lines[(i + 1) % lines.size()];
        
        float x1 = (l1.x1 + l1.x2) / 2;
        float y1 = (l1.y1 + l1.y2) / 2;
        float x2 = (l2.x1 + l2.x2) / 2;
        float y2 = (l2.y1 + l2.y2) / 2;
        
        area += x1 * y2 - x2 * y1;
    }
    
    return fabs(area) / 2;
}

std::vector<std::pair<int,int>> SonarAwareAligner::matchCornersByGeometry(
    const std::vector<CornerPoint>& corners1,
    const std::vector<CornerPoint>& corners2) {
    
    std::vector<std::pair<int,int>> matches;
    std::vector<std::tuple<float, int, int>> similarities;
    
    for (size_t i = 0; i < corners1.size(); i++) {
        for (size_t j = 0; j < corners2.size(); j++) {
            float sim = computeCornerGeometrySimilarity(corners1[i], corners2[j]);
            if (sim > 0.6) {
                similarities.push_back({sim, i, j});
            }
        }
    }
    
    std::sort(similarities.begin(), similarities.end(),
              std::greater<std::tuple<float, int, int>>());
    
    std::vector<bool> used1(corners1.size(), false);
    std::vector<bool> used2(corners2.size(), false);
    
    for (const auto& [sim, i, j] : similarities) {
        if (!used1[i] && !used2[j]) {
            matches.push_back({i, j});
            used1[i] = true;
            used2[j] = true;
        }
    }
    
    return matches;
}

 

float SonarAwareAligner::computeCornerGeometrySimilarity(const CornerPoint& c1,
                                                          const CornerPoint& c2) {
    
    // 1. 角度相似度
    float angle_diff = fabs(normalizeAngle(c1.angle - c2.angle));
    float max_angle_diff = static_cast<float>(M_PI/4);  // 转换为float
    float angle_score = 1.0f - std::min(1.0f, angle_diff / max_angle_diff);
    
    // 2. 类型匹配
    float type_score = (c1.type == c2.type) ? 1.0f : 0.5f;
    
    return 0.7f * angle_score + 0.3f * type_score;
}

SonarTransformParams SonarAwareAligner::alignByContour(
    const GeometryFeatureSet& g1, const GeometryFeatureSet& g2) {
    
    SonarTransformParams transform;
    
    ROS_INFO("=== 基于轮廓对齐 ===");
    
    // 1. 如果有封闭轮廓，用轮廓质心粗对齐
    if (g1.main_contour.is_closed && g2.main_contour.is_closed) {
        transform.dx = g1.main_contour.centroid_x - g2.main_contour.centroid_x;
        transform.dy = g1.main_contour.centroid_y - g2.main_contour.centroid_y;
        ROS_INFO("轮廓质心对齐: dx=%.3f, dy=%.3f", transform.dx, transform.dy);
    }
    
    // 2. 用角点精调
    std::vector<std::pair<int,int>> corner_matches;   
    
    if (g1.corners.size() >= 2 && g2.corners.size() >= 2) {
        corner_matches = matchCornersByGeometry(g1.corners, g2.corners);
        
        if (corner_matches.size() >= 2) {
            std::vector<Eigen::Vector2d> points1, points2;
            std::vector<float> weights;
            
            for (const auto& match : corner_matches) {
                points1.push_back(Eigen::Vector2d(g1.corners[match.first].x,
                                                  g1.corners[match.first].y));
                points2.push_back(Eigen::Vector2d(g2.corners[match.second].x,
                                                  g2.corners[match.second].y));
                weights.push_back((g1.corners[match.first].confidence +
                                   g2.corners[match.second].confidence) / 2.0f);
            }
            
            transform = solveWeightedSVD(points1, points2, weights);
            ROS_INFO("角点精调: dx=%.3f, dy=%.3f, dtheta=%.1f°",
                     transform.dx, transform.dy, transform.dtheta*180/M_PI);
        }
    }
    
    // 3. 如果没有足够角点，用边界框对齐
    if (corner_matches.size() < 2) {
        transform.dx = (g1.min_x + g1.max_x)/2 - (g2.min_x + g2.max_x)/2;
        transform.dy = (g1.min_y + g1.max_y)/2 - (g2.min_y + g2.max_y)/2;
        ROS_INFO("边界框对齐: dx=%.3f, dy=%.3f", transform.dx, transform.dy);
    }
    
    transform.confidence = 0.7f;
    return transform;
}

// ======================== 形状特征函数 ========================



float SonarAwareAligner::alignDominantAngles(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2) {
    
    float dtheta = normalizeAngle(shape1.dominant_angle - shape2.dominant_angle);
    
    // 检查90度歧义
    float dtheta_alt = normalizeAngle(dtheta + M_PI/2);
    if (fabs(dtheta_alt) < fabs(dtheta)) {
        dtheta = dtheta_alt;
    }
    
    return dtheta;
}

void SonarAwareAligner::alignCentroids(
    const ClosedContour& c1, const ClosedContour& c2,
    float dtheta, float& dx, float& dy) {
    
    if (!c1.is_closed || !c2.is_closed) {
        dx = dy = 0;
        return;
    }
    
    float cos_t = cos(dtheta);
    float sin_t = sin(dtheta);
    
    float c2x_rot = c2.centroid_x * cos_t - c2.centroid_y * sin_t;
    float c2y_rot = c2.centroid_x * sin_t + c2.centroid_y * cos_t;
    
    dx = c1.centroid_x - c2x_rot;
    dy = c1.centroid_y - c2y_rot;
}

void SonarAwareAligner::refineWithCorners(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2,
    float& dx, float& dy, float& dtheta) {
    
    if (shape1.corners.size() < 2 || shape2.corners.size() < 2) return;
    
    auto matches = matchCornersByGeometry(shape1.corners, shape2.corners);
    if (matches.size() < 2) return;
    
    std::vector<Eigen::Vector2d> points1, points2;
    std::vector<float> weights;
    
    for (const auto& match : matches) {
        points1.push_back(Eigen::Vector2d(shape1.corners[match.first].x,
                                          shape1.corners[match.first].y));
        points2.push_back(Eigen::Vector2d(shape2.corners[match.second].x,
                                          shape2.corners[match.second].y));
        weights.push_back((shape1.corners[match.first].confidence +
                          shape2.corners[match.second].confidence) / 2.0f);
    }
    
    auto refined = solveWeightedSVD(points1, points2, weights);
    
    // 只接受小范围调整
    if (fabs(refined.dx - dx) < 0.3 && fabs(refined.dy - dy) < 0.3) {
        dx = refined.dx;
        dy = refined.dy;
        dtheta = refined.dtheta;
    }
}

 
std::string SonarAwareAligner::identifyShapeType(const ClosedContour& contour) {
    if (!contour.is_closed || contour.corners.size() < 3) {
        return "unknown";
    }
    
    // 计算宽度和高度
    float width = contour.max_x - contour.min_x;
    float height = contour.max_y - contour.min_y;
    
    // 统计角点类型
    int right_angles = 0;
    int acute_angles = 0;
    int obtuse_angles = 0;
    
    for (const auto& corner : contour.corners) {
        if (corner.type == "right") right_angles++;
        else if (corner.type == "acute") acute_angles++;
        else if (corner.type == "obtuse") obtuse_angles++;
    }
    
    // 根据角点特征判断形状
    if (right_angles == 4) {
        return "rectangle";
    } else if (right_angles == 3) {
        return "L_shape";
    } else if (right_angles == 0 && contour.corners.size() > 6) {
        // 很多角点且没有直角，可能是圆形/椭圆形
        float aspect_ratio = width / (height + 1e-6);
        if (fabs(aspect_ratio - 1.0) < 0.2) {
            return "circle";
        } else {
            return "ellipse";
        }
    } else {
        return "polygon";
    }
}
 

IntensityStats SonarAwareAligner::extractIntensityStats(const PointCloudPtr& cloud) {
    IntensityStats stats;
    
    if (cloud->empty()) {
        return stats;
    }
    
    float sum_intensity = 0;
    
    for (const auto& p : cloud->points) {
        // 强度分级
        if (p.intensity >= strong_threshold_) {
            stats.strong_points.push_back(p);
        } else if (p.intensity >= medium_threshold_) {
            stats.medium_points.push_back(p);
        } else if (p.intensity >= weak_threshold_) {
            stats.weak_points.push_back(p);
        }
        
        // 统计信息
        sum_intensity += p.intensity;
        stats.max_intensity = std::max(stats.max_intensity, p.intensity);
        stats.min_intensity = std::min(stats.min_intensity, p.intensity);
    }
    
    stats.avg_intensity = sum_intensity / cloud->size();
    
    ROS_DEBUG("强度分级: 强=%lu, 中=%lu, 弱=%lu, 范围[%.0f-%.0f], 平均=%.0f",
              stats.strong_points.size(), stats.medium_points.size(),
              stats.weak_points.size(), stats.min_intensity, 
              stats.max_intensity, stats.avg_intensity);
    
    return stats;
}

 
ShapeFeatures SonarAwareAligner::extractShapeFeatures(const PointCloudPtr& cloud,
                                                      const std::string& hint) {
    ShapeFeatures features;
    features.scan_hint = hint;
    
    ROS_INFO("====== 形状特征提取 (%s) ======", hint.c_str());
    
    // 1. 估计传感器位置
    float max_intensity;
    estimateSensorPosition(cloud, features.sensor_x, features.sensor_y, max_intensity);
    
    // 2. 提取线段
    features.lines = extractLineSegments(cloud);
    size_t original_count = features.lines.size();  // ✅ 正确位置
    ROS_INFO("提取到 %lu 条线段", original_count);

    // 2.1 合并共线的短线段 
    features.lines = mergeCollinearSegments(features.lines);
    if (original_count != features.lines.size()) {
        ROS_INFO("线段合并: %lu -> %lu", original_count, features.lines.size());
    }
    
    // 3. 检测角点
    features.corners = detectCornersFromLines(features.lines);
    ROS_INFO("检测到 %lu 个角点", features.corners.size());
    
    // 4. 跟踪封闭轮廓
    features.main_contour = traceClosedContour(features.lines, features.corners);
    if (features.main_contour.is_closed) {
        ROS_INFO("找到封闭轮廓，面积: %.2f m²", features.main_contour.area);
        // 更新边界框
        for (const auto& line : features.lines) {
            features.main_contour.min_x = std::min({features.main_contour.min_x, line.x1, line.x2});
            features.main_contour.max_x = std::max({features.main_contour.max_x, line.x1, line.x2});
            features.main_contour.min_y = std::min({features.main_contour.min_y, line.y1, line.y2});
            features.main_contour.max_y = std::max({features.main_contour.max_y, line.y1, line.y2});
        }
        features.width = features.main_contour.max_x - features.main_contour.min_x;
        features.height = features.main_contour.max_y - features.main_contour.min_y;
    }
    
    // 5. 计算主方向
    if (!features.lines.empty()) {
        std::map<int, int> angle_hist;
        for (const auto& line : features.lines) {
            int angle_deg = static_cast<int>(line.angle * 180 / M_PI) % 180;
            angle_hist[angle_deg]++;
        }
        
        int max_count = 0;
        for (const auto& [angle, count] : angle_hist) {
            if (count > max_count) {
                max_count = count;
                features.dominant_angle = angle * M_PI / 180;
            }
        }
        ROS_INFO("主方向: %.1f°", features.dominant_angle * 180 / M_PI);
    }
    
    // 6. 识别形状类型
    features.shape_type = identifyShapeType(features.main_contour);
    ROS_INFO("形状类型: %s", features.shape_type.c_str());
    
    return features;
}

// ======================== 工具函数 ========================

float SonarAwareAligner::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

float SonarAwareAligner::distance(float x1, float y1, float x2, float y2) {
    return hypot(x1 - x2, y1 - y2);
}

float SonarAwareAligner::pointToLineDistance(float x, float y,
                                             float a, float b, float c) {
    return fabs(a * x + b * y + c) / sqrt(a * a + b * b);
}

float SonarAwareAligner::pointToSegmentDistance(float x, float y,
                                                float x1, float y1,
                                                float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float length_sq = dx * dx + dy * dy;
    
    if (length_sq < 1e-6) {
        return distance(x, y, x1, y1);
    }
    
    float t = ((x - x1) * dx + (y - y1) * dy) / length_sq;
    t = std::max(0.0f, std::min(1.0f, t));
    
    float proj_x = x1 + t * dx;
    float proj_y = y1 + t * dy;
    
    return distance(x, y, proj_x, proj_y);
}

void SonarAwareAligner::log(const std::string& msg) {
    ROS_INFO("[SonarAligner] %s", msg.c_str());
    
    if (log_file_.is_open()) {
        log_file_ << ros::Time::now() << ": " << msg << std::endl;
    }
}

void SonarAwareAligner::saveDebugInfo(const std::string& stage,
                                      const FeatureSet& f1,
                                      const FeatureSet& f2,
                                      const SonarTransformParams& transform) {
    
    std::string debug_dir = "/tmp/sonar_aligner_debug/" + stage + "_" +
                            std::to_string(static_cast<long>(ros::Time::now().toSec()));
    boost::filesystem::create_directories(debug_dir);
    
    std::ofstream file(debug_dir + "/features.txt");
    if (file.is_open()) {
        file << "点云1特征:\n";
        file << "  传感器位置: (" << f1.sensor_x << ", " << f1.sensor_y << ")\n";
        file << "  角点: " << f1.corners.size() << "\n";
        for (const auto& c : f1.corners) {
            file << "    (" << c.x << ", " << c.y << ") conf=" << c.confidence
                 << " rel_int=" << c.relative_intensity << "\n";
        }
        
        file << "  墙壁: " << f1.walls.size() << "\n";
        for (const auto& w : f1.walls) {
            file << "    [" << w.x1 << "," << w.y1 << "]->[" << w.x2 << "," << w.y2
                 << "] len=" << w.length << " conf=" << w.confidence
                 << " avg_int=" << w.avg_intensity << "\n";
        }
        
        file << "\n点云2特征:\n";
        file << "  传感器位置: (" << f2.sensor_x << ", " << f2.sensor_y << ")\n";
        file << "  角点: " << f2.corners.size() << "\n";
        for (const auto& c : f2.corners) {
            file << "    (" << c.x << ", " << c.y << ") conf=" << c.confidence
                 << " rel_int=" << c.relative_intensity << "\n";
        }
        
        file << "  墙壁: " << f2.walls.size() << "\n";
        for (const auto& w : f2.walls) {
            file << "    [" << w.x1 << "," << w.y1 << "]->[" << w.x2 << "," << w.y2
                 << "] len=" << w.length << " conf=" << w.confidence
                 << " avg_int=" << w.avg_intensity << "\n";
        }
        
        file << "\n变换参数:\n";
        file << "  dx = " << transform.dx << "\n";
        file << "  dy = " << transform.dy << "\n";
        file << "  dtheta = " << transform.dtheta * 180.0 / M_PI << "°\n";
        file << "  confidence = " << transform.confidence << "\n";
        
        file.close();
    }
    
    ROS_DEBUG("调试信息保存到: %s", debug_dir.c_str());
}


// ======================== 超大型水池对齐相关函数 ========================
// ======================== 关系分析 ========================

SonarAwareAligner::ScanRelation SonarAwareAligner::analyzeScanRelation(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2,
    const std::string& hint) {
 
    ScanRelation relation;
    relation.type = UNKNOWN_RELATION;
    relation.confidence = 0;
    
    // 1. 根据提示信息判断
    if (hint == "corner_to_edge") {
        relation.type = CORNER_TO_EDGE;
        relation.confidence = 0.8;
        return relation;
    } else if (hint == "edge_to_edge") {
        relation.type = EDGE_TO_EDGE;
        relation.confidence = 0.8;
        return relation;
    }
    
    // 2. 自动分析：寻找可能的关系
    // 2.1 检查是否有角点
    if (!shape1.corners.empty() && !shape2.lines.empty()) {
        // 可能是角落到边
        relation.type = CORNER_TO_EDGE;
        relation.corner_idx = 0;  // 使用第一个角点
        relation.edge2_idx = 0;   // 使用第一条边
        
        // 计算角度一致性
        float corner_angle = shape1.corners[0].angle;
        float edge_angle = shape2.lines[0].angle;
        relation.angle_diff = fabs(normalizeAngle(corner_angle - edge_angle));
        
        // 如果是垂直关系（角落的两条边垂直，边应该平行于其中一条）
        if (fabs(relation.angle_diff) < 10.0*M_PI/180 || 
            fabs(relation.angle_diff - M_PI/2) < 10.0*M_PI/180) {
            relation.confidence = 0.7;
        }
    }
    
    return relation;
}

// ======================== 角落到边的对齐 ========================

SonarTransformParams SonarAwareAligner::alignCornerToEdge(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2,
    const ScanRelation& relation) {
    
    SonarTransformParams transform;
    
    // 获取角落和边
    if (shape1.corners.empty() || shape2.lines.empty()) {
        return transform;
    }
    
    const auto& corner = shape1.corners[0];
    const auto& edge = shape2.lines[0];
    
    ROS_INFO("角落到边对齐: 角落(%.2f,%.2f), 边角度=%.1f°",
             corner.x, corner.y, edge.angle * 180/M_PI);
    
    // 1. 方向对齐：边的方向应该与角落的某条边一致
    // 角落由两条线段构成，我们需要找出哪条线段与边平行
    float best_angle_diff = 1e6;
    float best_edge_angle = 0;
    
    for (const auto& line : shape1.lines) {
        // 检查这条线是否属于这个角落（简化判断）
        float dist_to_corner = distance(line.x1, line.y1, corner.x, corner.y);
        if (dist_to_corner < 0.3) {
            float angle_diff = fabs(normalizeAngle(line.angle - edge.angle));
            if (angle_diff < best_angle_diff) {
                best_angle_diff = angle_diff;
                best_edge_angle = line.angle;
            }
        }
    }
    
    // 使用最匹配的边进行方向对齐
    transform.dtheta = normalizeAngle(best_edge_angle - edge.angle);
    
    // 2. 位置对齐：让边的端点靠近角落
    float cos_t = cos(transform.dtheta);
    float sin_t = sin(transform.dtheta);
    
    // 计算边的两个端点在变换后的位置
    float ex1 = edge.x1 * cos_t - edge.y1 * sin_t;
    float ey1 = edge.x1 * sin_t + edge.y1 * cos_t;
    float ex2 = edge.x2 * cos_t - edge.y2 * sin_t;
    float ey2 = edge.x2 * sin_t + edge.y2 * cos_t;
    
    // 选择离角落近的端点
    float dist1 = distance(ex1, ey1, corner.x, corner.y);
    float dist2 = distance(ex2, ey2, corner.x, corner.y);
    
    if (dist1 < dist2) {
        transform.dx = corner.x - ex1;
        transform.dy = corner.y - ey1;
    } else {
        transform.dx = corner.x - ex2;
        transform.dy = corner.y - ey2;
    }
    
    // 3. 计算置信度
    float final_dist = std::min(dist1, dist2);
    transform.confidence = 1.0f - std::min(1.0f, final_dist / 0.2f);
    
    return transform;
}

// ======================== 边到边的对齐 ========================

SonarTransformParams SonarAwareAligner::alignEdgeToEdge(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2,
    const ScanRelation& relation) {
    
    SonarTransformParams transform;
    
    if (shape1.lines.empty() || shape2.lines.empty()) {
        return transform;
    }
    
    // 取最长的边
    const auto& edge1 = shape1.lines[0];
    const auto& edge2 = shape2.lines[0];
    
    ROS_INFO("边到边对齐: 边1角度=%.1f°, 边2角度=%.1f°",
             edge1.angle * 180/M_PI, edge2.angle * 180/M_PI);
    
    // 1. 方向对齐
    transform.dtheta = normalizeAngle(edge1.angle - edge2.angle);
    
    // 2. 共线对齐：让两条边共线
    float cos_t = cos(transform.dtheta);
    float sin_t = sin(transform.dtheta);
    
    // 计算边2变换后的端点
    float ex1 = edge2.x1 * cos_t - edge2.y1 * sin_t;
    float ey1 = edge2.x1 * sin_t + edge2.y1 * cos_t;
    float ex2 = edge2.x2 * cos_t - edge2.y2 * sin_t;
    float ey2 = edge2.x2 * sin_t + edge2.y2 * cos_t;
    
    // 计算边1的直线方程
    float a = edge1.y2 - edge1.y1;
    float b = edge1.x1 - edge1.x2;
    float c = -(a * edge1.x1 + b * edge1.y1);
    float norm = sqrt(a*a + b*b);
    
    // 找到离边1最近的端点
    float d1 = fabs(a * ex1 + b * ey1 + c) / norm;
    float d2 = fabs(a * ex2 + b * ey2 + c) / norm;
    
    if (d1 < d2) {
        // 平移使第一个端点落到边1上
        float t = -(a * ex1 + b * ey1 + c) / (a*a + b*b);
        transform.dx = a * t;
        transform.dy = b * t;
    } else {
        float t = -(a * ex2 + b * ey2 + c) / (a*a + b*b);
        transform.dx = a * t;
        transform.dy = b * t;
    }
    
    // 3. 计算置信度
    float final_error = std::min(d1, d2);
    transform.confidence = 1.0f - std::min(1.0f, final_error / 0.1f);
    
    return transform;
}

// ======================== 几何约束验证 ========================

// ======================== 几何约束验证（完整版）=======================

bool SonarAwareAligner::validateGeometricConstraints(
    const PointCloudPtr& cloud1, const PointCloudPtr& cloud2,
    const SonarTransformParams& transform,
    const ScanRelation& relation) {
    
    ROS_INFO("=== 几何约束验证 ===");
    
    // 提取形状特征（用于验证）
    ShapeFeatures shape1 = extractShapeFeatures(cloud1, "validation");
    ShapeFeatures shape2 = extractShapeFeatures(cloud2, "validation");
    
    if (shape1.lines.empty() || shape2.lines.empty()) {
        ROS_WARN("几何约束验证：特征不足");
        return false;
    }
    
    bool valid = false;
    float validation_score = 0.0f;
    
    // 根据关系类型进行特定验证
    switch (relation.type) {
        case CORNER_TO_EDGE:
            validation_score = validateCornerToEdge(shape1, shape2, transform, relation);
            valid = (validation_score > 0.7f);
            ROS_INFO("角落到边验证: 得分=%.2f, %s", 
                     validation_score, valid ? "✓" : "✗");
            break;
            
        case EDGE_TO_EDGE:
            validation_score = validateEdgeToEdge(shape1, shape2, transform, relation);
            valid = (validation_score > 0.7f);
            ROS_INFO("边到边验证: 得分=%.2f, %s", 
                     validation_score, valid ? "✓" : "✗");
            break;
            
        case PARALLEL_EDGES:
            validation_score = validateParallelEdges(shape1, shape2, transform);
            valid = (validation_score > 0.8f);
            ROS_INFO("平行边验证: 得分=%.2f, %s", 
                     validation_score, valid ? "✓" : "✗");
            break;
            
        default:
            // 默认使用通用验证
            validation_score = validateGeneral(shape1, shape2, transform);
            valid = (validation_score > 0.6f);
            ROS_INFO("通用验证: 得分=%.2f, %s", 
                     validation_score, valid ? "✓" : "✗");
            break;
    }
    
    return valid;
}

// ======================== 角落到边的验证 ========================

float SonarAwareAligner::validateCornerToEdge(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2,
    const SonarTransformParams& transform,
    const ScanRelation& relation) {
    
    if (shape1.corners.empty() || shape2.lines.empty()) {
        return 0.0f;
    }
    
    // 获取参考角落和边
    int corner_idx = (relation.corner_idx >= 0 && relation.corner_idx < (int)shape1.corners.size()) 
                    ? relation.corner_idx : 0;
    int edge_idx = (relation.edge2_idx >= 0 && relation.edge2_idx < (int)shape2.lines.size())
                  ? relation.edge2_idx : 0;
    
    const auto& corner = shape1.corners[corner_idx];
    const auto& edge = shape2.lines[edge_idx];
    
    // 变换边的端点
    float cos_t = cos(transform.dtheta);
    float sin_t = sin(transform.dtheta);
    
    float tex1 = edge.x1 * cos_t - edge.y1 * sin_t + transform.dx;
    float tey1 = edge.x1 * sin_t + edge.y1 * cos_t + transform.dy;
    float tex2 = edge.x2 * cos_t - edge.y2 * sin_t + transform.dx;
    float tey2 = edge.x2 * sin_t + edge.y2 * cos_t + transform.dy;
    
    // 验证1：边的某个端点应该靠近角落
    float dist_to_corner1 = distance(tex1, tey1, corner.x, corner.y);
    float dist_to_corner2 = distance(tex2, tey2, corner.x, corner.y);
    float min_dist = std::min(dist_to_corner1, dist_to_corner2);
    
    float endpoint_score = 1.0f - std::min(1.0f, min_dist / 0.3f);  // 30cm内得分高
    
    // 验证2：边的方向应该与角落的某条边一致
    float best_angle_score = 0.0f;
    for (const auto& line : shape1.lines) {
         // 计算直线的法线式参数
        float dx = line.x2 - line.x1;
        float dy = line.y2 - line.y1;
        float a_line = -dy;
        float b_line = dx;
        float norm = sqrt(a_line*a_line + b_line*b_line);
        if (norm > 1e-6) {
            a_line /= norm;
            b_line /= norm;
        }
        float c_line = -(a_line * line.x1 + b_line * line.y1);
        // 检查这条线是否属于这个角落（通过距离判断）
        float dist_to_line = pointToLineDistance(corner.x, corner.y, a_line, b_line, c_line);
        if (dist_to_line < 0.2f) {  // 20cm内认为是角落的边
            float angle_diff = fabs(normalizeAngle(line.angle - edge.angle));
 

            float max_angle_diff = static_cast<float>(10.0 * M_PI / 180.0);
            float angle_score = 1.0f - std::min(1.0f, angle_diff / max_angle_diff);
            best_angle_score = std::max(best_angle_score, angle_score);
        }
    }
    
    // 验证3：变换后的边应该与角落的边共线
    float collinearity_score = 0.0f;
    for (const auto& line : shape1.lines) {
        float dx_line = line.x2 - line.x1;
        float dy_line = line.y2 - line.y1;
        float a_line = -dy_line;
        float b_line = dx_line;
        float norm_line = sqrt(a_line*a_line + b_line*b_line);
        if (norm_line > 1e-6) {
            a_line /= norm_line;
            b_line /= norm_line;
        }
        float c_line = -(a_line * line.x1 + b_line * line.y1);
 
        float d1 = pointToLineDistance(tex1, tey1, a_line, b_line, c_line);
        float d2 = pointToLineDistance(tex2, tey2, a_line, b_line, c_line);
        
        float avg_dist = (d1 + d2) / 2.0f;
        float line_score = 1.0f - std::min(1.0f, avg_dist / 0.15f);
        
        

        collinearity_score = std::max(collinearity_score, line_score);
    }
    
    // 综合得分
    float total_score = 0.4f * endpoint_score + 
                        0.3f * best_angle_score + 
                        0.3f * collinearity_score;
    
    ROS_DEBUG("角落到边验证详情: 端点=%.2f, 角度=%.2f, 共线=%.2f, 总分=%.2f",
              endpoint_score, best_angle_score, collinearity_score, total_score);
    
    return total_score;
}

// ======================== 边到边的验证 ========================

float SonarAwareAligner::validateEdgeToEdge(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2,
    const SonarTransformParams& transform,
    const ScanRelation& relation) {
    
    if (shape1.lines.empty() || shape2.lines.empty()) {
        return 0.0f;
    }
    
    // 取最长的边进行验证
    const auto& edge1 = shape1.lines[0];
    const auto& edge2 = shape2.lines[0];
    
    // 变换边2
    float cos_t = cos(transform.dtheta);
    float sin_t = sin(transform.dtheta);
    
    float tex1 = edge2.x1 * cos_t - edge2.y1 * sin_t + transform.dx;
    float tey1 = edge2.x1 * sin_t + edge2.y1 * cos_t + transform.dy;
    float tex2 = edge2.x2 * cos_t - edge2.y2 * sin_t + transform.dx;
    float tey2 = edge2.x2 * sin_t + edge2.y2 * cos_t + transform.dy;
    
    // 验证1：方向一致性
    float angle_diff = fabs(normalizeAngle(edge1.angle - edge2.angle - transform.dtheta));
 
    float max_angle = static_cast<float>(5.0 * M_PI / 180.0);
    float angle_score = 1.0f - std::min(1.0f, angle_diff / max_angle);
    
    // 验证2：共线性（点到直线的距离）
    float d1 = pointToLineDistance(tex1, tey1, edge1.a, edge1.b, edge1.c);
    float d2 = pointToLineDistance(tex2, tey2, edge1.a, edge1.b, edge1.c);
    float avg_dist = (d1 + d2) / 2.0f;
    float collinearity_score = 1.0f - std::min(1.0f, avg_dist / 0.15f);
    
    // 验证3：端点连接性（判断是否首尾相接）
    float dist_to_start1 = distance(tex1, tey1, edge1.x1, edge1.y1);
    float dist_to_start2 = distance(tex2, tey2, edge1.x1, edge1.y1);
    float dist_to_end1 = distance(tex1, tey1, edge1.x2, edge1.y2);
    float dist_to_end2 = distance(tex2, tey2, edge1.x2, edge1.y2);
    
    float min_endpoint_dist = std::min({dist_to_start1, dist_to_start2, 
                                         dist_to_end1, dist_to_end2});
    float endpoint_score = 1.0f - std::min(1.0f, min_endpoint_dist / 0.3f);
    
    // 综合得分
    float total_score = 0.4f * angle_score + 
                        0.4f * collinearity_score + 
                        0.2f * endpoint_score;
    
    ROS_DEBUG("边到边验证详情: 角度=%.2f, 共线=%.2f, 端点=%.2f, 总分=%.2f",
              angle_score, collinearity_score, endpoint_score, total_score);
    
    return total_score;
}

// ======================== 平行边的验证 ========================

float SonarAwareAligner::validateParallelEdges(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2,
    const SonarTransformParams& transform) {
    
    if (shape1.lines.empty() || shape2.lines.empty()) {
        return 0.0f;
    }
    
    float best_score = 0.0f;
    int valid_pairs = 0;
    
    // 检查所有可能的边对
    for (const auto& line1 : shape1.lines) {
        for (const auto& line2 : shape2.lines) {
            // 变换边2
            float cos_t = cos(transform.dtheta);
            float sin_t = sin(transform.dtheta);
            
            float mx2 = (line2.x1 + line2.x2) / 2;
            float my2 = (line2.y1 + line2.y2) / 2;
            float tx2 = mx2 * cos_t - my2 * sin_t + transform.dx;
            float ty2 = mx2 * sin_t + my2 * cos_t + transform.dy;
            
            // 计算边1的中点
            float mx1 = (line1.x1 + line1.x2) / 2;
            float my1 = (line1.y1 + line1.y2) / 2;
            
            // 方向一致性
            float angle_diff = fabs(normalizeAngle(line1.angle - line2.angle - transform.dtheta));
            float max_angle = static_cast<float>(5.0 * M_PI / 180.0);
            float angle_score = 1.0f - std::min(1.0f, angle_diff / max_angle);
            
            // 距离合理性（平行线之间的距离应该合理）
            float dist = pointToLineDistance(tx2, ty2, line1.a, line1.b, line1.c);
            float dist_score = 1.0f - std::min(1.0f, dist / 2.0f);  // 距离太远不合理
            
            float pair_score = 0.6f * angle_score + 0.4f * dist_score;
            best_score += pair_score;
            valid_pairs++;
        }
    }
    
    if (valid_pairs == 0) return 0.0f;
    
    return best_score / valid_pairs;
}

// ======================== 通用验证 ========================

float SonarAwareAligner::validateGeneral(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2,
    const SonarTransformParams& transform) {
    
    // 使用原有的验证方法
    float confidence;
    std::string diagnostic;
    
    // 创建临时点云用于验证
    PointCloudPtr temp_cloud1(new PointCloud());
    PointCloudPtr temp_cloud2(new PointCloud());
    
    // 用形状特征重建简单点云（仅用于验证）
    for (const auto& line : shape1.lines) {
        PointType p1, p2;
        p1.x = line.x1; p1.y = line.y1; p1.intensity = 200;
        p2.x = line.x2; p2.y = line.y2; p2.intensity = 200;
        temp_cloud1->push_back(p1);
        temp_cloud1->push_back(p2);
    }
    
    for (const auto& line : shape2.lines) {
        PointType p1, p2;
        p1.x = line.x1; p1.y = line.y1; p1.intensity = 200;
        p2.x = line.x2; p2.y = line.y2; p2.intensity = 200;
        temp_cloud2->push_back(p1);
        temp_cloud2->push_back(p2);
    }
    
    // 使用原有的验证函数
    bool valid = validateTransform(temp_cloud1, temp_cloud2, transform, confidence, diagnostic);
    
    return valid ? confidence : 0.0f;
}


// ======================== 特征转换 ========================

std::vector<ExtendedFeature> SonarAwareAligner::convertToExtendedFeatures(
    const ShapeFeatures& features, int scan_id) {
    
    std::vector<ExtendedFeature> extended;
    int feature_id = 0;
    
    // 1. 转换角点
    for (const auto& corner : features.corners) {
        ExtendedFeature ef;
        ef.type = CORNER;
        ef.id = feature_id++;
        ef.scan_id = scan_id;
        ef.x = corner.x;
        ef.y = corner.y;
        ef.angle = corner.angle;
        ef.confidence = corner.confidence;
        
        // 找到构成这个角点的两条线段
        if (corner.line1_idx >= 0 && corner.line1_idx < (int)features.lines.size()) {
            ef.connected_features.push_back(corner.line1_idx);
        }
        if (corner.line2_idx >= 0 && corner.line2_idx < (int)features.lines.size()) {
            ef.connected_features.push_back(corner.line2_idx);
        }
        
        extended.push_back(ef);
    }
    
    // 2. 转换线段
    for (const auto& line : features.lines) {
        ExtendedFeature ef;
        ef.type = LINE_SEGMENT;
        ef.id = feature_id++;
        ef.scan_id = scan_id;
        ef.x1 = line.x1;
        ef.y1 = line.y1;
        ef.x2 = line.x2;
        ef.y2 = line.y2;
        ef.angle = line.angle;
        ef.length = line.length;
        ef.confidence = line.confidence;
        
        extended.push_back(ef);
        
        // 3. 添加线段端点作为独立特征
        ExtendedFeature ep1, ep2;
        ep1.type = LINE_ENDPOINT;
        ep1.id = feature_id++;
        ep1.scan_id = scan_id;
        ep1.x = line.x1;
        ep1.y = line.y1;
        ep1.connected_features.push_back(ef.id);
        extended.push_back(ep1);
        
        ep2.type = LINE_ENDPOINT;
        ep2.id = feature_id++;
        ep2.scan_id = scan_id;
        ep2.x = line.x2;
        ep2.y = line.y2;
        ep2.connected_features.push_back(ef.id);
        extended.push_back(ep2);
    }
    
    ROS_DEBUG("转换得到 %lu 个扩展特征", extended.size());
    return extended;
}

// ======================== 添加扫描 ========================

void SonarAwareAligner::addScanFeatures(int scan_id, const ShapeFeatures& features) {
    TopologicalGraph graph;
    graph.nodes = convertToExtendedFeatures(features, scan_id);
    
    // 分析节点间的关系
    for (size_t i = 0; i < graph.nodes.size(); i++) {
        for (size_t j = i + 1; j < graph.nodes.size(); j++) {
            auto relation = analyzeRelation(graph.nodes[i], graph.nodes[j]);
            if (relation.confidence > 0.5f) {
                graph.edges.push_back(relation);
            }
        }
    }
    
    scans_[scan_id] = graph;
    ROS_INFO("添加扫描 %d: %lu个节点, %lu个关系", 
             scan_id, graph.nodes.size(), graph.edges.size());
}

// ======================== 关系分析 ========================

FeatureRelation SonarAwareAligner::analyzeRelation(
    const ExtendedFeature& f1, const ExtendedFeature& f2) {
    
    FeatureRelation relation;
    relation.confidence = 0;
    
    // 线段-线段关系
    if (f1.type == LINE_SEGMENT && f2.type == LINE_SEGMENT) {
        return analyzeLineRelation(f1, f2);
    }
    
    // 线段-角点关系
    if (f1.type == LINE_SEGMENT && f2.type == CORNER) {
        return analyzeLineCornerRelation(f1, f2);
    }
    if (f1.type == CORNER && f2.type == LINE_SEGMENT) {
        return analyzeLineCornerRelation(f2, f1);
    }
    
    return relation;
}

FeatureRelation SonarAwareAligner::analyzeLineRelation(
    const ExtendedFeature& line1, const ExtendedFeature& line2) {
    
    FeatureRelation relation;
    relation.feature1_id = line1.id;
    relation.feature2_id = line2.id;
    
    // 计算角度差
    float angle_diff = fabs(normalizeAngle(line1.angle - line2.angle));
    float min_angle_diff = std::min(angle_diff, (float)(M_PI - angle_diff));
    
    // 检查平行
    if (min_angle_diff < 5.0 * M_PI / 180.0) {
        relation.type = FeatureRelation::PARALLEL;
        relation.confidence = 1.0f - min_angle_diff / (5.0 * M_PI / 180.0);
        relation.parameter = min_angle_diff;
        return relation;
    }
    
    // 检查垂直
    float perp_diff = fabs(min_angle_diff - M_PI/2);
    if (perp_diff < 5.0 * M_PI / 180.0) {
        relation.type = FeatureRelation::PERPENDICULAR;
        relation.confidence = 1.0f - perp_diff / (5.0 * M_PI / 180.0);
        relation.parameter = perp_diff;
        return relation;
    }
    
    // 检查共线（需要计算点到直线的距离）
    // 简化实现，实际需要更精确的计算
    
    return relation;
}

FeatureRelation SonarAwareAligner::analyzeLineCornerRelation(
    const ExtendedFeature& line, const ExtendedFeature& corner) {
    
    FeatureRelation relation;
    relation.feature1_id = line.id;
    relation.feature2_id = corner.id;
    
    // 计算角点到直线的距离
    float a = line.y2 - line.y1;
    float b = line.x1 - line.x2;
    float c = -(a * line.x1 + b * line.y1);
    float dist = fabs(a * corner.x + b * corner.y + c) / sqrt(a*a + b*b);
    
    // 如果角点在线段附近，可能是线段的端点
    if (dist < 0.1) {
        // 检查角点是否在线段端点附近
        float dist_to_start = distance(corner.x, corner.y, line.x1, line.y1);
        float dist_to_end = distance(corner.x, corner.y, line.x2, line.y2);
        
        if (dist_to_start < 0.2 || dist_to_end < 0.2) {
            relation.type = FeatureRelation::INTERSECTION;
            relation.confidence = 0.9;
            relation.parameter = std::min(dist_to_start, dist_to_end);
            return relation;
        }
        
        // 否则可能是共线但不在端点上
        relation.type = FeatureRelation::COLLINEAR;
        relation.confidence = 0.7;
        relation.parameter = dist;
        return relation;
    }
    
    return relation;
}

// ======================== 基于几何约束的对齐 ========================

SonarTransformParams SonarAwareAligner::alignWithGeometricConstraints(
    int scan_id1, int scan_id2, const std::string& relation_hint) {
    
    SonarTransformParams transform;
    
    auto it1 = scans_.find(scan_id1);
    auto it2 = scans_.find(scan_id2);
    
    if (it1 == scans_.end() || it2 == scans_.end()) {
        ROS_ERROR("找不到扫描 %d 或 %d", scan_id1, scan_id2);
        return transform;
    }
    
    const auto& graph1 = it1->second;
    const auto& graph2 = it2->second;
    
    ROS_INFO("对齐扫描 %d 和 %d, 提示: %s", scan_id1, scan_id2, relation_hint.c_str());
    
    // 根据提示选择对齐策略
    if (relation_hint == "corner_to_edge") {
        // 场景1：第一次是角落，第二次是延伸的边
        
        // 找到第一次扫描中的角点
        ExtendedFeature corner;
        bool found_corner = false;
        for (const auto& node : graph1.nodes) {
            if (node.type == CORNER && node.confidence > 0.7) {
                corner = node;
                found_corner = true;
                break;
            }
        }
        
        if (!found_corner) {
            ROS_WARN("未找到可靠的角点");
            return transform;
        }
        
        // 找到第二次扫描中的长线段
        ExtendedFeature edge;
        bool found_edge = false;
        float max_length = 0;
        for (const auto& node : graph2.nodes) {
            if (node.type == LINE_SEGMENT && node.length > max_length) {
                max_length = node.length;
                edge = node;
                found_edge = true;
            }
        }
        
        if (!found_edge) {
            ROS_WARN("未找到可靠的边");
            return transform;
        }
        
        // 使用共线关系对齐
        return alignByExtension(edge, corner, corner);
    }
    else if (relation_hint == "edge_to_edge") {
        // 场景2：两次都是边，需要首尾相接
        
        // 找到两条最长的边
        ExtendedFeature edge1, edge2;
        float max_length1 = 0, max_length2 = 0;
        
        // 从第一次扫描中找最长的边
        for (const auto& node : graph1.nodes) {
            if (node.type == LINE_SEGMENT && node.length > max_length1) {
                max_length1 = node.length;
                edge1 = node;
            }
        }
        
        // 从第二次扫描中找最长的边
        for (const auto& node : graph2.nodes) {
            if (node.type == LINE_SEGMENT && node.length > max_length2) {
                max_length2 = node.length;
                edge2 = node;
            }
        }
        
        if (max_length1 == 0 || max_length2 == 0) {
            ROS_WARN("未找到足够长的边");
            return transform;
        }
        
        ROS_INFO("边到边对齐: 边1长度=%.2f, 边2长度=%.2f", max_length1, max_length2);
        
        // 使用共线关系对齐
        return alignByCollinearity(edge1, edge2, ExtendedFeature());
    }
    
    return transform;
}

SonarTransformParams SonarAwareAligner::alignByCollinearity(
    const ExtendedFeature& line1,
    const ExtendedFeature& line2,
    const ExtendedFeature& corner) {
    
    SonarTransformParams transform;
    
    // 1. 方向对齐
    transform.dtheta = normalizeAngle(line1.angle - line2.angle);
    
    // 2. 计算line2变换后的端点
    float cos_t = cos(transform.dtheta);
    float sin_t = sin(transform.dtheta);
    
    float ex1 = line2.x1 * cos_t - line2.y1 * sin_t;
    float ey1 = line2.x1 * sin_t + line2.y1 * cos_t;
    float ex2 = line2.x2 * cos_t - line2.y2 * sin_t;
    float ey2 = line2.x2 * sin_t + line2.y2 * cos_t;
    
    // 3. 计算到line1的距离
    float a = line1.y2 - line1.y1;
    float b = line1.x1 - line1.x2;
    float c = -(a * line1.x1 + b * line1.y1);
    float norm = sqrt(a*a + b*b);
    
    float d1 = fabs(a * ex1 + b * ey1 + c) / norm;
    float d2 = fabs(a * ex2 + b * ey2 + c) / norm;
    
    // 4. 如果有点在角落附近，优先对齐那个点
    if (corner.type == CORNER) {
        float dist1 = distance(ex1, ey1, corner.x, corner.y);
        float dist2 = distance(ex2, ey2, corner.x, corner.y);
        
        if (dist1 < dist2) {
            transform.dx = corner.x - ex1;
            transform.dy = corner.y - ey1;
        } else {
            transform.dx = corner.x - ex2;
            transform.dy = corner.y - ey2;
        }
    } else {
        // 否则，平移使line2靠近line1
        float mx1 = (line1.x1 + line1.x2) / 2;
        float my1 = (line1.y1 + line1.y2) / 2;
        float mex = (ex1 + ex2) / 2;
        float mey = (ey1 + ey2) / 2;
        
        transform.dx = mx1 - mex;
        transform.dy = my1 - mey;
    }
    
    // 5. 计算置信度
    float alignment_error = (d1 + d2) / 2;
    transform.confidence = 1.0f - std::min(1.0f, alignment_error / 0.1f);
    
    return transform;
}

SonarTransformParams SonarAwareAligner::alignByExtension(
    const ExtendedFeature& line,
    const ExtendedFeature& corner_edge,
    const ExtendedFeature& corner) {
    
    SonarTransformParams transform;
    
    // 1. 方向对齐
    transform.dtheta = normalizeAngle(corner_edge.angle - line.angle);
    
    // 2. 确定延伸方向
    float cos_t = cos(transform.dtheta);
    float sin_t = sin(transform.dtheta);
    
    // 变换线的端点
    float ex1 = line.x1 * cos_t - line.y1 * sin_t;
    float ey1 = line.x1 * sin_t + line.y1 * cos_t;
    float ex2 = line.x2 * cos_t - line.y2 * sin_t;
    float ey2 = line.x2 * sin_t + line.y2 * cos_t;
    
    // 计算角落边的方向向量
    float corner_dx = corner_edge.x2 - corner_edge.x1;
    float corner_dy = corner_edge.y2 - corner_edge.y1;
    
    // 计算从角落到线端点的向量
    float to_line1_x = ex1 - corner_edge.x2;
    float to_line1_y = ey1 - corner_edge.y2;
    float to_line2_x = ex2 - corner_edge.x2;
    float to_line2_y = ey2 - corner_edge.y2;
    
    // 判断哪个端点在延伸方向上
    float dot1 = to_line1_x * corner_dx + to_line1_y * corner_dy;
    float dot2 = to_line2_x * corner_dx + to_line2_y * corner_dy;
    
    if (dot1 > dot2) {
        transform.dx = corner_edge.x2 - ex1;
        transform.dy = corner_edge.y2 - ey1;
    } else {
        transform.dx = corner_edge.x2 - ex2;
        transform.dy = corner_edge.y2 - ey2;
    }
    
    // 验证共线性
    float a = corner_edge.y2 - corner_edge.y1;
    float b = corner_edge.x1 - corner_edge.x2;
    float c = -(a * corner_edge.x1 + b * corner_edge.y1);
    float norm = sqrt(a*a + b*b);
    
    float tx = line.x1 * cos_t - line.y1 * sin_t + transform.dx;
    float ty = line.x1 * sin_t + line.y1 * cos_t + transform.dy;
    float dist = fabs(a * tx + b * ty + c) / norm;
    
    transform.confidence = 1.0f - std::min(1.0f, dist / 0.1f);
    
    return transform;
}

// ======================== 平行边的对齐 ========================
 
SonarTransformParams SonarAwareAligner::alignParallelEdges(
    const ShapeFeatures& shape1, const ShapeFeatures& shape2,
    const ScanRelation& relation) {
    
    SonarTransformParams transform;
    
    if (shape1.lines.empty() || shape2.lines.empty()) {
        ROS_WARN("平行边对齐: 特征不足");
        return transform;
    }
    
    ROS_INFO("平行边对齐");
    
    // 取最长的边进行对齐
    const auto& edge1 = shape1.lines[0];
    const auto& edge2 = shape2.lines[0];
    
    // 1. 方向对齐（平行边角度应该一致）
    transform.dtheta = normalizeAngle(edge1.angle - edge2.angle);
    
    // 2. 计算边1的中点
    float mx1 = (edge1.x1 + edge1.x2) / 2;
    float my1 = (edge1.y1 + edge1.y2) / 2;
    
    // 3. 计算边2变换后的中点
    float cos_t = cos(transform.dtheta);
    float sin_t = sin(transform.dtheta);
    
    float mx2 = (edge2.x1 + edge2.x2) / 2;
    float my2 = (edge2.y1 + edge2.y2) / 2;
    float tx2 = mx2 * cos_t - my2 * sin_t;
    float ty2 = mx2 * sin_t + my2 * cos_t;
    
    // 4. 平移使中点对齐
    transform.dx = mx1 - tx2;
    transform.dy = my1 - ty2;
    
    // 5. 计算置信度（基于两条边的距离）
    // 计算边2变换后的两个端点
    float tex1 = edge2.x1 * cos_t - edge2.y1 * sin_t + transform.dx;
    float tey1 = edge2.x1 * sin_t + edge2.y1 * cos_t + transform.dy;
    float tex2 = edge2.x2 * cos_t - edge2.y2 * sin_t + transform.dx;
    float tey2 = edge2.x2 * sin_t + edge2.y2 * cos_t + transform.dy;
    
    // 计算边1的直线方程参数
    float a1 = edge1.y2 - edge1.y1;
    float b1 = edge1.x1 - edge1.x2;
    float c1 = -(a1 * edge1.x1 + b1 * edge1.y1);
    float norm1 = sqrt(a1*a1 + b1*b1);
    
    // 修复：将 b2 改为 b1
    float d1 = fabs(a1 * tex1 + b1 * tey1 + c1) / norm1;
    float d2 = fabs(a1 * tex2 + b1 * tey2 + c1) / norm1;  // 这里 b2 改为 b1
    float avg_dist = (d1 + d2) / 2.0f;
    
    transform.confidence = 1.0f - std::min(1.0f, avg_dist / 0.2f);
    
    ROS_INFO("平行边对齐结果: dx=%.3f, dy=%.3f, dtheta=%.1f°, conf=%.2f",
             transform.dx, transform.dy, transform.dtheta * 180/M_PI, transform.confidence);
    
    return transform;
}

std::vector<LineSegment> SonarAwareAligner::mergeCollinearSegments(
    const std::vector<LineSegment>& segments) {
    
    if (segments.size() < 2) {
        return segments;
    }
    
    std::vector<LineSegment> merged;
    std::vector<bool> used(segments.size(), false);
    
    for (size_t i = 0; i < segments.size(); i++) {
        if (used[i]) continue;
        
        LineSegment merged_seg = segments[i];
        used[i] = true;
        
        // 找可以合并的线段
        for (size_t j = i + 1; j < segments.size(); j++) {
            if (used[j]) continue;
            
            // 1. 检查是否平行（角度差 < 10度）
            float angle_diff = fabs(normalizeAngle(segments[i].angle - segments[j].angle));
            if (angle_diff > 10.0 * M_PI / 180.0) continue;
            
            // 2. 检查是否在同一直线上（点到直线距离）
            float a = segments[i].y2 - segments[i].y1;
            float b = segments[i].x1 - segments[i].x2;
            float c = -(a * segments[i].x1 + b * segments[i].y1);
            float norm = sqrt(a*a + b*b);
            
            float d1 = fabs(a * segments[j].x1 + b * segments[j].y1 + c) / norm;
            float d2 = fabs(a * segments[j].x2 + b * segments[j].y2 + c) / norm;
            
            if (d1 > 0.3 || d2 > 0.3) continue;  // 距离直线超过30cm
            
            // 3. 检查是否首尾相接或重叠
            float dist1 = distance(merged_seg.x2, merged_seg.y2, 
                                   segments[j].x1, segments[j].y1);
            float dist2 = distance(merged_seg.x1, merged_seg.y1, 
                                   segments[j].x2, segments[j].y2);
            float dist3 = distance(merged_seg.x1, merged_seg.y1, 
                                   segments[j].x1, segments[j].y1);
            float dist4 = distance(merged_seg.x2, merged_seg.y2, 
                                   segments[j].x2, segments[j].y2);
            
            float min_dist = std::min({dist1, dist2, dist3, dist4});
            
            if (min_dist < 2.0) {  // 2米内可以合并（针对长墙）
                // 合并端点（取最小和最大的坐标）
                merged_seg.x1 = std::min({merged_seg.x1, segments[j].x1, segments[j].x2});
                merged_seg.x2 = std::max({merged_seg.x2, segments[j].x1, segments[j].x2});
                merged_seg.y1 = std::min({merged_seg.y1, segments[j].y1, segments[j].y2});
                merged_seg.y2 = std::max({merged_seg.y2, segments[j].y1, segments[j].y2});
                
                // 重新计算长度
                merged_seg.length = distance(merged_seg.x1, merged_seg.y1,
                                             merged_seg.x2, merged_seg.y2);
                
                // 更新置信度（取平均）
                merged_seg.confidence = (merged_seg.confidence + segments[j].confidence) / 2;
                
                // 更新角度（取平均）
                merged_seg.angle = (merged_seg.angle + segments[j].angle) / 2;
                
                used[j] = true;
            }
        }
        
        merged.push_back(merged_seg);
    }
    
    ROS_DEBUG("线段合并: %lu -> %lu", segments.size(), merged.size());
    return merged;
}

} // namespace sonar_slam