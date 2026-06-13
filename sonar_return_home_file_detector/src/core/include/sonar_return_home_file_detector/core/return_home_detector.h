#ifndef SONAR_RETURN_HOME_FILE_DETECTOR_CORE_RETURN_HOME_DETECTOR_H
#define SONAR_RETURN_HOME_FILE_DETECTOR_CORE_RETURN_HOME_DETECTOR_H

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>

#include "sonar_return_home_file_detector/core/fourier_soft2d_matcher.h"

namespace sonar_return_home_file_detector_core {

// Yaw values are radians unless a field name explicitly ends in _deg.
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

enum class ProcessedCloudSource {
  FilteredPointCloud,
  LocalPointCloud,
  ExternalProcessedCloud
};

struct SonarKeyframe {
  int id = -1;
  int position_number = -1;
  ros::Time stamp;
  ros::Time odom_stamp;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
  Pose2D odom_pose;
  bool odom_valid = false;
  std::string odom_frame_id = "odom";
  uint64_t odom_epoch = 0;
  uint64_t scan_id = 0;
  bool full_scan = false;
  double coverage_deg = 0.0;
  std::string frame_id = "local_sonar";
  ProcessedCloudSource cloud_source = ProcessedCloudSource::FilteredPointCloud;
  double quality = 0.0;
};

struct CurrentSonarFrame {
  ros::Time stamp;
  ros::Time odom_stamp;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
  Pose2D odom_pose;
  bool odom_valid = false;
  std::string odom_frame_id = "odom";
  uint64_t odom_epoch = 0;
  uint64_t scan_id = 0;
  bool full_scan = false;
  double coverage_deg = 0.0;
  std::string frame_id = "local_sonar";
  ProcessedCloudSource cloud_source = ProcessedCloudSource::FilteredPointCloud;
};

struct ReturnHomeSnapshot {
  SonarKeyframe home;
  std::vector<SonarKeyframe> history;
  CurrentSonarFrame current;
};

struct ReturnHomeParams {
  bool enabled = true;

  double odom_candidate_radius = 1.0;
  double odom_candidate_yaw_deg = 60.0;
  bool yaw_is_hard_constraint = false;
  bool use_adaptive_home_radius = true;
  bool sonar_before_odom_gate = true;
  double pool_length = 0.0;
  double pool_width = 0.0;
  double adaptive_home_radius_ratio = 0.08;
  double min_home_radius = 1.0;
  double max_home_radius = 3.0;
  bool require_leave_home_before_return = true;
  double leave_home_radius = 0.0;
  double leave_home_radius_multiplier = 1.5;
  double leave_home_radius_extra = 0.8;
  int min_scans_before_return_check = 2;

  double min_range = 0.45;
  double max_range = 8.0;
  double min_intensity = 120.0;
  double voxel_leaf_size = 0.05;
  int max_points_after_filter = 1200;
  int max_points_for_icp = 600;

  bool require_full_scan = true;
  double min_scan_coverage_deg = 300.0;
  double max_snapshot_odom_time_delta = 0.20;

  double search_translation = 0.8;
  double coarse_translation_step = 0.15;
  double fine_translation_step = 0.05;
  double search_yaw_deg = 35.0;
  double coarse_yaw_step_deg = 5.0;
  double fine_yaw_step_deg = 1.0;
  int fine_top_k = 5;
  double score_sigma = 0.10;
  int min_unique_matched_cells = 40;

  int fourier_dimension = 64;
  double fourier_extent = 8.0;
  bool fourier_draw_sonar_rays = true;
  bool fourier_use_initial_guess = false;

  double match_score_threshold = 0.60;
  double large_pool_diag_threshold = 20.0;
  double large_pool_match_score_threshold = 0.45;
  bool use_range_adaptive_match_threshold = true;
  double short_range_pool_ratio = 0.25;
  double medium_range_pool_ratio = 0.40;
  double short_range_threshold_relax = 0.08;
  double medium_range_threshold_relax = 0.04;
  double min_adaptive_match_score_threshold = 0.35;
  bool enable_polar_profile = true;
  bool enable_multi_scale_matching = true;
  bool enable_chamfer_score = true;
  bool weaken_margin_inside_home_zone = true;
  double near_home_margin_relax_radius_ratio = 1.0;
  double near_home_score_margin_threshold = -1.0;
  double polar_profile_weight = 0.35;
  double multi_scale_occupancy_weight = 0.25;
  double chamfer_shape_weight = 0.20;
  double fs2d_overlap_weight = 0.15;
  double fs2d_peak_weight = 0.05;
  double score_margin_threshold = 0.08;
  double correction_translation_threshold = 0.60;
  double correction_yaw_threshold_deg = 35.0;
  bool enable_icp_refine = false;

  int confirm_window_size = 1;
  int confirm_required_hits = 1;
  double check_interval = 1.0;

  int max_negative_keyframes = 16;
  double negative_keyframe_min_distance = 2.0;
  double negative_keyframe_min_leave_radius_multiplier = 2.0;
  int max_points_per_keyframe = 1200;
};

struct ReturnHomeResult {
  bool returned = false;
  bool odom_candidate = false;
  bool sonar_match_valid = false;
  int matched_keyframe_id = -1;

  double odom_distance = 0.0;
  double odom_yaw_delta = 0.0;
  double match_score = 0.0;
  double fs2d_overlap_score = 0.0;
  double fs2d_peak_score = 0.0;
  double polar_profile_score = 0.0;
  double multi_scale_occupancy_score = 0.0;
  double chamfer_shape_score = 0.0;
  double best_non_home_score = 0.0;
  double score_margin = 0.0;
  double correction_translation = 0.0;
  double correction_yaw = 0.0;
  double adaptive_home_radius = 0.0;
  double leave_home_radius = 0.0;
  double adaptive_sonar_threshold = 0.0;
  double effective_sonar_range = 0.0;
  double range_pool_ratio = 0.0;
  double estimated_pool_diag = 0.0;
  bool return_check_armed = false;
  int scans_since_home = 0;

  int confirm_hits = 0;
  int confirm_window = 0;
  uint64_t scan_id = 0;
  bool full_scan = false;
  double coverage_deg = 0.0;
  std::string reason;
};

class ReturnHomeDetector {
 public:
  explicit ReturnHomeDetector(const ReturnHomeParams& params = ReturnHomeParams());

  // Clears multi-frame confirmation state while keeping cached keyframes.
  void reset();

  void setHomeKeyframe(const SonarKeyframe& keyframe);

  // If no home keyframe has been set, the first valid keyframe becomes home.
  // Remaining valid keyframes are cached as non-home references for margin scoring.
  void setHistoryKeyframes(const std::vector<SonarKeyframe>& keyframes);

  // Appends one non-home keyframe without clearing confirmation/arming state.
  // This is used by the file-directory wrapper as new snapshots arrive online.
  bool addHistoryKeyframe(const SonarKeyframe& keyframe);

  ReturnHomeResult detectOnce(const CurrentSonarFrame& current) const;
  ReturnHomeResult detect(const CurrentSonarFrame& current);
  bool isReturnedToHome(const CurrentSonarFrame& current);

 private:
  struct Point2D {
    float x = 0.0F;
    float y = 0.0F;
    float intensity = 0.0F;
  };

  struct PreparedCloud {
    std::vector<Point2D> points;
    double quality = 0.0;
    double coverage_deg = 0.0;
    double effective_range = 0.0;
  };

  struct MatchCandidate {
    double dx = 0.0;
    double dy = 0.0;
    double yaw = 0.0;
    double score = 0.0;
    double fs2d_overlap_score = 0.0;
    double fs2d_peak_score = 0.0;
    double polar_profile_score = 0.0;
    double multi_scale_occupancy_score = 0.0;
    double chamfer_shape_score = 0.0;
    int unique_cells = 0;
    std::string reason;
  };

  struct CachedKeyframe {
    SonarKeyframe keyframe;
    PreparedCloud cloud;
    FourierSoft2DDescriptor descriptor;
  };

  PreparedCloud prepareCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const;
  FourierSoft2DDescriptor buildDescriptor(const PreparedCloud& cloud) const;
  bool buildCachedKeyframe(const SonarKeyframe& keyframe, CachedKeyframe* cached) const;

  MatchCandidate matchKeyframe(const CachedKeyframe& reference,
                               const PreparedCloud& current,
                               double initial_dx,
                               double initial_dy,
                               double initial_yaw) const;
  FourierSoft2DParams makeFourierParams(double initial_dx,
                                        double initial_dy,
                                        double initial_yaw) const;
  double estimatePoolDiagonal() const;
  double adaptiveHomeRadius(double pool_diag) const;
  double leaveHomeRadius(double home_radius) const;
  double adaptiveSonarThreshold(double pool_diag, double effective_range) const;

  bool sameKeyframeIdentity(const SonarKeyframe& lhs, const SonarKeyframe& rhs) const;
  int countConfirmationHits() const;
  void applyConfirmation(ReturnHomeResult* result, bool single_frame_hit);

  ReturnHomeParams params_;
  bool has_home_ = false;
  CachedKeyframe home_;
  std::vector<CachedKeyframe> negative_keyframes_;

  std::deque<std::pair<uint64_t, bool>> confirmation_window_;
  uint64_t last_counted_scan_id_ = 0;
  bool has_counted_scan_ = false;
  bool return_check_armed_ = false;
  int scans_since_home_ = 0;
  ReturnHomeResult previous_detect_result_;
};

}  // namespace sonar_return_home_file_detector_core

#endif  // SONAR_RETURN_HOME_FILE_DETECTOR_CORE_RETURN_HOME_DETECTOR_H
