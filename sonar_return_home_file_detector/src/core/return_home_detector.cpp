#include "sonar_return_home_file_detector/core/return_home_detector.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace sonar_return_home_file_detector_core {
namespace {

constexpr double kPi = 3.14159265358979323846;

double degToRad(double deg) { return deg * kPi / 180.0; }

double normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double clamp01(double value) {
  if (value <= 0.0) {
    return 0.0;
  }
  if (value >= 1.0) {
    return 1.0;
  }
  return value;
}

double clampValue(double value, double lower, double upper) {
  return std::min(upper, std::max(lower, value));
}

double effectiveRangeFromSamples(std::vector<double> ranges,
                                 double min_range,
                                 double max_range) {
  if (ranges.empty()) {
    return std::max(min_range, max_range);
  }
  const std::size_t index =
      std::min(ranges.size() - 1U,
               static_cast<std::size_t>(std::floor(0.90 * static_cast<double>(ranges.size() - 1U))));
  std::nth_element(ranges.begin(), ranges.begin() + static_cast<std::ptrdiff_t>(index), ranges.end());
  return clampValue(ranges[index], min_range, max_range);
}

double combinedEffectiveRange(double first, double second, double fallback) {
  const double combined = std::max(first, second);
  return combined > 0.0 ? combined : fallback;
}

int safePositiveInt(int value, int fallback) { return value > 0 ? value : fallback; }

}  // namespace

ReturnHomeDetector::ReturnHomeDetector(const ReturnHomeParams& params) : params_(params) {}

void ReturnHomeDetector::reset() {
  confirmation_window_.clear();
  last_counted_scan_id_ = 0;
  has_counted_scan_ = false;
  return_check_armed_ = !params_.require_leave_home_before_return;
  scans_since_home_ = 0;
  previous_detect_result_ = ReturnHomeResult();
}

void ReturnHomeDetector::setHomeKeyframe(const SonarKeyframe& keyframe) {
  CachedKeyframe cached;
  has_home_ = buildCachedKeyframe(keyframe, &cached);
  if (has_home_) {
    home_ = std::move(cached);
  } else {
    home_ = CachedKeyframe();
  }
  reset();
}

void ReturnHomeDetector::setHistoryKeyframes(const std::vector<SonarKeyframe>& keyframes) {
  negative_keyframes_.clear();

  std::size_t start_index = 0;
  if (!has_home_) {
    for (; start_index < keyframes.size(); ++start_index) {
      CachedKeyframe cached;
      if (buildCachedKeyframe(keyframes[start_index], &cached)) {
        home_ = std::move(cached);
        has_home_ = true;
        ++start_index;
        break;
      }
    }
  }

  const int max_negatives = std::max(0, params_.max_negative_keyframes);
  if (max_negatives == 0) {
    reset();
    return;
  }

  negative_keyframes_.reserve(static_cast<std::size_t>(max_negatives));
  for (std::size_t i = start_index; i < keyframes.size(); ++i) {
    if (has_home_ && sameKeyframeIdentity(keyframes[i], home_.keyframe)) {
      continue;
    }
    CachedKeyframe cached;
    if (!buildCachedKeyframe(keyframes[i], &cached)) {
      continue;
    }
    negative_keyframes_.push_back(std::move(cached));
    if (static_cast<int>(negative_keyframes_.size()) >= max_negatives) {
      break;
    }
  }

  reset();
}

bool ReturnHomeDetector::addHistoryKeyframe(const SonarKeyframe& keyframe) {
  if (!has_home_ || sameKeyframeIdentity(keyframe, home_.keyframe)) {
    return false;
  }
  for (const CachedKeyframe& negative : negative_keyframes_) {
    if (sameKeyframeIdentity(keyframe, negative.keyframe)) {
      return false;
    }
  }

  CachedKeyframe cached;
  if (!buildCachedKeyframe(keyframe, &cached)) {
    return false;
  }

  const int max_negatives = std::max(0, params_.max_negative_keyframes);
  if (max_negatives == 0) {
    return false;
  }
  while (static_cast<int>(negative_keyframes_.size()) >= max_negatives) {
    negative_keyframes_.erase(negative_keyframes_.begin());
  }
  negative_keyframes_.push_back(std::move(cached));
  return true;
}

ReturnHomeResult ReturnHomeDetector::detectOnce(const CurrentSonarFrame& current) const {
  ReturnHomeResult result;
  result.scan_id = current.scan_id;
  result.full_scan = current.full_scan;
  result.coverage_deg = current.coverage_deg;

  if (!params_.enabled) {
    result.reason = "detector_disabled";
    return result;
  }
  if (!has_home_ || !home_.descriptor.valid) {
    result.reason = "home_keyframe_unavailable";
    return result;
  }
  if (!current.cloud || current.cloud->empty()) {
    result.reason = "current_cloud_empty";
    return result;
  }
  if (!home_.keyframe.odom_valid || !current.odom_valid) {
    result.reason = "odom_invalid";
    return result;
  }
  if (home_.keyframe.odom_frame_id != current.odom_frame_id ||
      home_.keyframe.odom_epoch != current.odom_epoch) {
    result.reason = "odom_origin_mismatch";
    return result;
  }
  if (!current.stamp.isZero() && !current.odom_stamp.isZero() &&
      params_.max_snapshot_odom_time_delta >= 0.0) {
    const double snapshot_odom_delta =
        std::abs((current.stamp - current.odom_stamp).toSec());
    if (snapshot_odom_delta > params_.max_snapshot_odom_time_delta) {
      result.reason = "current_snapshot_odom_time_delta_too_large";
      return result;
    }
  }

  if (params_.require_full_scan) {
    if (!home_.keyframe.full_scan || home_.keyframe.coverage_deg < params_.min_scan_coverage_deg) {
      result.reason = "home_scan_incomplete";
      return result;
    }
    if (!current.full_scan || current.coverage_deg < params_.min_scan_coverage_deg) {
      result.reason = "current_scan_incomplete";
      return result;
    }
  }

  const double dx = current.odom_pose.x - home_.keyframe.odom_pose.x;
  const double dy = current.odom_pose.y - home_.keyframe.odom_pose.y;
  result.odom_distance = std::hypot(dx, dy);
  result.odom_yaw_delta = normalizeAngle(current.odom_pose.yaw - home_.keyframe.odom_pose.yaw);
  result.estimated_pool_diag = estimatePoolDiagonal();
  result.adaptive_home_radius = adaptiveHomeRadius(result.estimated_pool_diag);
  result.leave_home_radius = leaveHomeRadius(result.adaptive_home_radius);

  const double odom_yaw_limit = degToRad(params_.odom_candidate_yaw_deg);
  const bool inside_home_zone = result.odom_distance <= result.adaptive_home_radius;
  const bool yaw_passed = std::abs(result.odom_yaw_delta) <= odom_yaw_limit;
  result.odom_candidate = inside_home_zone &&
                          (!params_.yaw_is_hard_constraint || yaw_passed);
  if (!params_.sonar_before_odom_gate && !result.odom_candidate) {
    result.reason = "odom_gate_rejected";
    return result;
  }

  PreparedCloud current_cloud = prepareCloud(current.cloud);
  if (current_cloud.points.empty()) {
    result.reason = "current_cloud_has_no_valid_points";
    return result;
  }
  result.effective_sonar_range =
      combinedEffectiveRange(home_.cloud.effective_range,
                             current_cloud.effective_range,
                             params_.max_range);
  if (result.estimated_pool_diag > 0.0 && result.effective_sonar_range > 0.0) {
    result.range_pool_ratio = result.effective_sonar_range / result.estimated_pool_diag;
  }
  result.adaptive_sonar_threshold =
      adaptiveSonarThreshold(result.estimated_pool_diag, result.effective_sonar_range);

  MatchCandidate home_match = matchKeyframe(home_,
                                            current_cloud,
                                            -dx,
                                            -dy,
                                            -result.odom_yaw_delta);
  result.matched_keyframe_id = home_.keyframe.id;
  result.match_score = home_match.score;
  result.fs2d_overlap_score = home_match.fs2d_overlap_score;
  result.fs2d_peak_score = home_match.fs2d_peak_score;
  result.polar_profile_score = home_match.polar_profile_score;
  result.multi_scale_occupancy_score = home_match.multi_scale_occupancy_score;
  result.chamfer_shape_score = home_match.chamfer_shape_score;
  result.correction_translation = std::hypot(home_match.dx, home_match.dy);
  result.correction_yaw = std::abs(home_match.yaw);
  result.sonar_match_valid =
      home_match.unique_cells >= params_.min_unique_matched_cells ||
      (result.match_score >= 0.5 * result.adaptive_sonar_threshold &&
       (result.polar_profile_score > 0.0 ||
        result.multi_scale_occupancy_score > 0.0 ||
        result.chamfer_shape_score > 0.0));

  double best_non_home_score = 0.0;
  for (const CachedKeyframe& negative : negative_keyframes_) {
    if (!negative.descriptor.valid) {
      continue;
    }
    const MatchCandidate negative_match = matchKeyframe(negative,
                                                        current_cloud,
                                                        0.0,
                                                        0.0,
                                                        0.0);
    best_non_home_score = std::max(best_non_home_score, negative_match.score);
  }
  result.best_non_home_score = best_non_home_score;
  result.score_margin = result.match_score - result.best_non_home_score;

  if (!result.odom_candidate) {
    result.reason = params_.yaw_is_hard_constraint && !yaw_passed
                        ? "odom_yaw_gate_rejected"
                        : "outside_adaptive_home_radius";
    return result;
  }
  if (!result.sonar_match_valid) {
    result.reason = home_match.reason.empty() ? "not_enough_unique_matched_cells"
                                              : home_match.reason;
    return result;
  }
  if (result.match_score < result.adaptive_sonar_threshold) {
    result.reason = "match_score_below_threshold";
    return result;
  }
  double margin_threshold = params_.score_margin_threshold;
  if (params_.weaken_margin_inside_home_zone && inside_home_zone) {
    const double relaxed_radius =
        result.adaptive_home_radius *
        std::max(0.0, params_.near_home_margin_relax_radius_ratio);
    const bool near_home_margin_relaxed =
        result.odom_distance <= std::max(0.0, relaxed_radius);
    if (result.estimated_pool_diag >= params_.large_pool_diag_threshold ||
        near_home_margin_relaxed) {
      margin_threshold =
          std::min(margin_threshold, params_.near_home_score_margin_threshold);
    }
  }
  if (result.score_margin < margin_threshold) {
    result.reason = "score_margin_below_threshold";
    return result;
  }
  if (result.estimated_pool_diag < params_.large_pool_diag_threshold &&
      result.correction_translation > params_.correction_translation_threshold) {
    result.reason = "correction_translation_too_large";
    return result;
  }
  if (params_.yaw_is_hard_constraint &&
      result.correction_yaw > degToRad(params_.correction_yaw_threshold_deg)) {
    result.reason = "correction_yaw_too_large";
    return result;
  }

  result.returned = true;
  result.reason = params_.enable_icp_refine ? "matched_icp_refine_not_implemented" : "matched";
  return result;
}

ReturnHomeResult ReturnHomeDetector::detect(const CurrentSonarFrame& current) {
  ReturnHomeResult result = detectOnce(current);

  if (has_counted_scan_ && current.scan_id == last_counted_scan_id_) {
    previous_detect_result_.confirm_hits = countConfirmationHits();
    previous_detect_result_.confirm_window = static_cast<int>(confirmation_window_.size());
    previous_detect_result_.return_check_armed = return_check_armed_;
    previous_detect_result_.scans_since_home = scans_since_home_;
    return previous_detect_result_;
  }

  ++scans_since_home_;
  if (!params_.require_leave_home_before_return) {
    return_check_armed_ = true;
  }
  if (!return_check_armed_ && result.leave_home_radius > 0.0 &&
      result.odom_distance >= result.leave_home_radius) {
    return_check_armed_ = true;
  }

  result.return_check_armed = return_check_armed_;
  result.scans_since_home = scans_since_home_;

  if (result.returned && params_.require_leave_home_before_return && !return_check_armed_) {
    result.returned = false;
    result.reason = "not_armed_waiting_to_leave_home";
  }

  const int min_scans_before_return = std::max(0, params_.min_scans_before_return_check);
  if (result.returned && scans_since_home_ < min_scans_before_return) {
    result.returned = false;
    result.reason = "min_scans_before_return_check";
  }

  const bool single_frame_hit = result.returned;

  const auto existing =
      std::find_if(confirmation_window_.begin(), confirmation_window_.end(),
                   [current](const std::pair<uint64_t, bool>& entry) {
                     return entry.first == current.scan_id;
                   });
  if (existing == confirmation_window_.end()) {
    confirmation_window_.emplace_back(current.scan_id, single_frame_hit);
    const int window_size = safePositiveInt(params_.confirm_window_size, 1);
    while (static_cast<int>(confirmation_window_.size()) > window_size) {
      confirmation_window_.pop_front();
    }
    last_counted_scan_id_ = current.scan_id;
    has_counted_scan_ = true;
  }

  applyConfirmation(&result, single_frame_hit);
  previous_detect_result_ = result;
  return result;
}

bool ReturnHomeDetector::isReturnedToHome(const CurrentSonarFrame& current) {
  return detect(current).returned;
}

ReturnHomeDetector::PreparedCloud ReturnHomeDetector::prepareCloud(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const {
  PreparedCloud prepared;
  if (!cloud) {
    return prepared;
  }

  const double min_range = std::max(0.0, params_.min_range);
  const double max_range = std::max(min_range, params_.max_range);
  const double min_range_sq = min_range * min_range;
  const double max_range_sq = max_range * max_range;
  const double leaf = std::max(0.0, params_.voxel_leaf_size);
  const int max_points = safePositiveInt(params_.max_points_after_filter, 1200);

  std::vector<Point2D> filtered;
  std::vector<double> filtered_ranges;
  filtered.reserve(std::min<std::size_t>(cloud->size(), static_cast<std::size_t>(max_points) * 2U));
  filtered_ranges.reserve(filtered.capacity());
  for (const pcl::PointXYZI& point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.intensity)) {
      continue;
    }
    const double range_sq = static_cast<double>(point.x) * point.x +
                            static_cast<double>(point.y) * point.y;
    if (range_sq < min_range_sq || range_sq > max_range_sq) {
      continue;
    }
    if (point.intensity < params_.min_intensity) {
      continue;
    }
    filtered.push_back(Point2D{point.x, point.y, point.intensity});
    filtered_ranges.push_back(std::sqrt(range_sq));
  }
  prepared.effective_range = effectiveRangeFromSamples(filtered_ranges, min_range, max_range);

  if (leaf > 1.0e-6) {
    struct VoxelAccumulator {
      double x = 0.0;
      double y = 0.0;
      double intensity = 0.0;
      int count = 0;
    };

    std::unordered_map<std::int64_t, VoxelAccumulator> voxels;
    voxels.reserve(filtered.size());
    for (const Point2D& point : filtered) {
      const int ix = static_cast<int>(std::floor(point.x / leaf));
      const int iy = static_cast<int>(std::floor(point.y / leaf));
      const std::int64_t key = (static_cast<std::int64_t>(ix) << 32) ^
                               (static_cast<std::uint32_t>(iy));
      VoxelAccumulator& accumulator = voxels[key];
      accumulator.x += point.x;
      accumulator.y += point.y;
      accumulator.intensity += point.intensity;
      ++accumulator.count;
    }

    prepared.points.reserve(voxels.size());
    for (const auto& item : voxels) {
      const VoxelAccumulator& accumulator = item.second;
      if (accumulator.count <= 0) {
        continue;
      }
      const double inv_count = 1.0 / static_cast<double>(accumulator.count);
      prepared.points.push_back(
          Point2D{static_cast<float>(accumulator.x * inv_count),
                  static_cast<float>(accumulator.y * inv_count),
                  static_cast<float>(accumulator.intensity * inv_count)});
    }
  } else {
    prepared.points = std::move(filtered);
  }

  std::sort(prepared.points.begin(), prepared.points.end(),
            [](const Point2D& lhs, const Point2D& rhs) {
              return lhs.intensity > rhs.intensity;
            });
  if (static_cast<int>(prepared.points.size()) > max_points) {
    prepared.points.resize(static_cast<std::size_t>(max_points));
  }

  if (!prepared.points.empty()) {
    std::unordered_set<int> angle_bins;
    angle_bins.reserve(prepared.points.size());
    for (const Point2D& point : prepared.points) {
      double angle = std::atan2(point.y, point.x) * 180.0 / kPi;
      if (angle < 0.0) {
        angle += 360.0;
      }
      angle_bins.insert(static_cast<int>(std::floor(angle)));
    }
    prepared.coverage_deg = static_cast<double>(angle_bins.size());
    prepared.quality = clamp01(static_cast<double>(prepared.points.size()) /
                               static_cast<double>(std::max(1, params_.min_unique_matched_cells)));
  }

  return prepared;
}

FourierSoft2DDescriptor ReturnHomeDetector::buildDescriptor(const PreparedCloud& cloud) const {
  std::vector<FourierSoft2DPoint> points;
  points.reserve(cloud.points.size());
  for (const Point2D& point : cloud.points) {
    points.push_back(FourierSoft2DPoint{point.x, point.y, point.intensity});
  }
  return FourierSoft2DMatcher(makeFourierParams(0.0, 0.0, 0.0)).buildDescriptor(points);
}

bool ReturnHomeDetector::buildCachedKeyframe(const SonarKeyframe& keyframe,
                                             CachedKeyframe* cached) const {
  if (!cached || !keyframe.cloud || keyframe.cloud->empty()) {
    return false;
  }

  CachedKeyframe candidate;
  candidate.keyframe = keyframe;
  candidate.cloud = prepareCloud(keyframe.cloud);
  const int max_keyframe_points = safePositiveInt(params_.max_points_per_keyframe,
                                                  params_.max_points_after_filter);
  if (static_cast<int>(candidate.cloud.points.size()) > max_keyframe_points) {
    candidate.cloud.points.resize(static_cast<std::size_t>(max_keyframe_points));
  }
  candidate.descriptor = buildDescriptor(candidate.cloud);
  if (!candidate.descriptor.valid) {
    return false;
  }

  if (candidate.keyframe.quality <= 0.0) {
    candidate.keyframe.quality = candidate.cloud.quality;
  }
  if (candidate.keyframe.coverage_deg <= 0.0) {
    candidate.keyframe.coverage_deg = candidate.cloud.coverage_deg;
  }

  *cached = std::move(candidate);
  return true;
}

ReturnHomeDetector::MatchCandidate ReturnHomeDetector::matchKeyframe(
    const CachedKeyframe& reference,
    const PreparedCloud& current,
    double initial_dx,
    double initial_dy,
    double initial_yaw) const {
  MatchCandidate candidate;
  if (!reference.descriptor.valid || reference.cloud.points.empty() || current.points.empty()) {
    candidate.reason = "fourier_descriptor_or_current_empty";
    return candidate;
  }

  std::vector<FourierSoft2DPoint> points;
  points.reserve(current.points.size());
  for (const Point2D& point : current.points) {
    points.push_back(FourierSoft2DPoint{point.x, point.y, point.intensity});
  }
  FourierSoft2DMatcher matcher(makeFourierParams(initial_dx, initial_dy, initial_yaw));
  const FourierSoft2DDescriptor current_descriptor = matcher.buildDescriptor(points);
  const FourierSoft2DMatchResult match = matcher.match(reference.descriptor, current_descriptor);

  candidate.dx = match.dx;
  candidate.dy = match.dy;
  candidate.yaw = match.yaw;
  candidate.fs2d_overlap_score = clamp01(match.match_score);
  candidate.fs2d_peak_score = clamp01(match.peak_score);
  candidate.unique_cells = match.overlap_cells;
  candidate.reason = match.reason;

  const auto build_polar_profile = [this](const PreparedCloud& cloud,
                                          std::vector<double>* ranges,
                                          std::vector<double>* weights) {
    constexpr int kBins = 360;
    ranges->assign(kBins, 0.0);
    weights->assign(kBins, 0.0);
    const double max_range = std::max(params_.max_range, 1.0);
    const double min_intensity = std::max(params_.min_intensity, 1.0);
    for (const Point2D& point : cloud.points) {
      const double range = std::hypot(point.x, point.y);
      if (range <= 0.0) {
        continue;
      }
      double angle = std::atan2(point.y, point.x) * 180.0 / kPi;
      if (angle < 0.0) {
        angle += 360.0;
      }
      const int bin = std::min(359, std::max(0, static_cast<int>(std::floor(angle))));
      const double intensity_weight = std::min(2.0,
                                               std::max(0.1, point.intensity / min_intensity));
      const double range_weight = std::sqrt(std::min(1.0, range / max_range));
      const double weight = intensity_weight * (0.5 + range_weight);
      if (weight > (*weights)[static_cast<std::size_t>(bin)]) {
        (*weights)[static_cast<std::size_t>(bin)] = weight;
        (*ranges)[static_cast<std::size_t>(bin)] = range;
      }
    }
  };

  double polar_yaw = match.yaw;
  if (params_.enable_polar_profile) {
    std::vector<double> ref_ranges;
    std::vector<double> ref_weights;
    std::vector<double> cur_ranges;
    std::vector<double> cur_weights;
    build_polar_profile(reference.cloud, &ref_ranges, &ref_weights);
    build_polar_profile(current, &cur_ranges, &cur_weights);

    double best_score = 0.0;
    int best_shift = 0;
    const double range_sigma = std::max(0.75, std::min(3.0, params_.max_range * 0.06));
    const double two_sigma_sq = 2.0 * range_sigma * range_sigma;
    int ref_valid = 0;
    int cur_valid = 0;
    for (double value : ref_weights) {
      if (value > 0.0) {
        ++ref_valid;
      }
    }
    for (double value : cur_weights) {
      if (value > 0.0) {
        ++cur_valid;
      }
    }
    for (int shift = 0; shift < 360; ++shift) {
      double score_sum = 0.0;
      double weight_sum = 0.0;
      int matched_bins = 0;
      for (int bin = 0; bin < 360; ++bin) {
        const int cur_bin = (bin + shift) % 360;
        const double rw = ref_weights[static_cast<std::size_t>(bin)];
        const double cw = cur_weights[static_cast<std::size_t>(cur_bin)];
        if (rw <= 0.0 || cw <= 0.0) {
          continue;
        }
        const double diff = ref_ranges[static_cast<std::size_t>(bin)] -
                            cur_ranges[static_cast<std::size_t>(cur_bin)];
        const double bin_weight = std::sqrt(rw * cw);
        score_sum += bin_weight * std::exp(-(diff * diff) / two_sigma_sq);
        weight_sum += bin_weight;
        ++matched_bins;
      }
      if (matched_bins == 0 || weight_sum <= 0.0) {
        continue;
      }
      const double profile_score = score_sum / weight_sum;
      const double coverage_score = static_cast<double>(matched_bins) /
                                    static_cast<double>(std::max(1, std::min(ref_valid, cur_valid)));
      const double score = clamp01(0.75 * profile_score + 0.25 * coverage_score);
      if (score > best_score) {
        best_score = score;
        best_shift = shift;
      }
    }
    candidate.polar_profile_score = best_score;
    polar_yaw = normalizeAngle(-degToRad(static_cast<double>(best_shift)));
  }

  const auto rotate_point = [](const Point2D& point, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Point2D rotated;
    rotated.x = static_cast<float>(c * point.x - s * point.y);
    rotated.y = static_cast<float>(s * point.x + c * point.y);
    rotated.intensity = point.intensity;
    return rotated;
  };

  if (params_.enable_multi_scale_matching) {
    const double scales[] = {
        std::min(8.0, std::max(2.0, params_.max_range)),
        std::min(25.0, std::max(4.0, params_.max_range)),
        std::max(8.0, params_.max_range)};
    double weighted_sum = 0.0;
    double total_weight = 0.0;
    for (double extent : scales) {
      const double cell = std::max(0.25, extent / 48.0);
      auto cell_key = [cell](double x, double y) {
        const int ix = static_cast<int>(std::floor(x / cell));
        const int iy = static_cast<int>(std::floor(y / cell));
        return (static_cast<std::int64_t>(ix) << 32) ^ static_cast<std::uint32_t>(iy);
      };
      std::unordered_set<std::int64_t> ref_cells;
      std::unordered_set<std::int64_t> cur_cells;
      for (const Point2D& point : reference.cloud.points) {
        if (std::hypot(point.x, point.y) <= extent) {
          ref_cells.insert(cell_key(point.x, point.y));
        }
      }
      for (const Point2D& point : current.points) {
        const Point2D rotated = rotate_point(point, polar_yaw);
        if (std::hypot(rotated.x, rotated.y) <= extent) {
          cur_cells.insert(cell_key(rotated.x, rotated.y));
        }
      }
      if (ref_cells.empty() || cur_cells.empty()) {
        continue;
      }
      int overlap = 0;
      for (const std::int64_t key : cur_cells) {
        if (ref_cells.find(key) != ref_cells.end()) {
          ++overlap;
        }
      }
      const int union_cells = static_cast<int>(ref_cells.size() + cur_cells.size()) - overlap;
      const double iou = union_cells > 0
                             ? static_cast<double>(overlap) / static_cast<double>(union_cells)
                             : 0.0;
      const double coverage = static_cast<double>(overlap) /
                              static_cast<double>(std::max<std::size_t>(
                                  1U, std::min(ref_cells.size(), cur_cells.size())));
      const double scale_score = clamp01(0.45 * iou + 0.55 * coverage);
      const double scale_weight = extent <= 8.0 ? 0.30 : (extent <= 25.0 ? 0.35 : 0.35);
      weighted_sum += scale_weight * scale_score;
      total_weight += scale_weight;
    }
    candidate.multi_scale_occupancy_score =
        total_weight > 0.0 ? clamp01(weighted_sum / total_weight) : 0.0;
  }

  if (params_.enable_chamfer_score) {
    const int max_points = std::min(240, std::min(static_cast<int>(reference.cloud.points.size()),
                                                  static_cast<int>(current.points.size())));
    if (max_points > 0) {
      auto sample_points = [max_points](const std::vector<Point2D>& points, double yaw) {
        std::vector<Point2D> sampled;
        sampled.reserve(static_cast<std::size_t>(max_points));
        const std::size_t stride =
            points.size() > static_cast<std::size_t>(max_points)
                ? (points.size() + static_cast<std::size_t>(max_points) - 1U) /
                      static_cast<std::size_t>(max_points)
                : 1U;
        for (std::size_t i = 0; i < points.size() &&
                                sampled.size() < static_cast<std::size_t>(max_points);
             i += stride) {
          const double c = std::cos(yaw);
          const double s = std::sin(yaw);
          sampled.push_back(Point2D{
              static_cast<float>(c * points[i].x - s * points[i].y),
              static_cast<float>(s * points[i].x + c * points[i].y),
              points[i].intensity});
        }
        return sampled;
      };
      const std::vector<Point2D> ref_points = sample_points(reference.cloud.points, 0.0);
      const std::vector<Point2D> cur_points = sample_points(current.points, polar_yaw);
      auto directional_score = [](const std::vector<Point2D>& from,
                                  const std::vector<Point2D>& to,
                                  double sigma) {
        if (from.empty() || to.empty()) {
          return 0.0;
        }
        const double two_sigma_sq = 2.0 * sigma * sigma;
        double sum = 0.0;
        for (const Point2D& point : from) {
          double best_sq = std::numeric_limits<double>::infinity();
          for (const Point2D& target : to) {
            const double dx = point.x - target.x;
            const double dy = point.y - target.y;
            best_sq = std::min(best_sq, dx * dx + dy * dy);
          }
          if (std::isfinite(best_sq)) {
            sum += std::exp(-best_sq / two_sigma_sq);
          }
        }
        return sum / static_cast<double>(from.size());
      };
      const double sigma = std::max(0.6, std::min(2.0, params_.max_range * 0.04));
      candidate.chamfer_shape_score =
          clamp01(0.5 * directional_score(ref_points, cur_points, sigma) +
                  0.5 * directional_score(cur_points, ref_points, sigma));
    }
  }

  double weighted_score = 0.0;
  double total_weight = 0.0;
  const auto add_component = [&weighted_score, &total_weight](double score, double weight) {
    if (weight <= 0.0) {
      return;
    }
    weighted_score += weight * clamp01(score);
    total_weight += weight;
  };
  add_component(candidate.fs2d_overlap_score, params_.fs2d_overlap_weight);
  add_component(candidate.fs2d_peak_score, params_.fs2d_peak_weight);
  if (params_.enable_polar_profile) {
    add_component(candidate.polar_profile_score, params_.polar_profile_weight);
  }
  if (params_.enable_multi_scale_matching) {
    add_component(candidate.multi_scale_occupancy_score, params_.multi_scale_occupancy_weight);
  }
  if (params_.enable_chamfer_score) {
    add_component(candidate.chamfer_shape_score, params_.chamfer_shape_weight);
  }
  candidate.score = total_weight > 0.0 ? clamp01(weighted_score / total_weight)
                                       : candidate.fs2d_overlap_score;
  return candidate;
}

FourierSoft2DParams ReturnHomeDetector::makeFourierParams(double initial_dx,
                                                          double initial_dy,
                                                          double initial_yaw) const {
  FourierSoft2DParams params;
  params.dimension = params_.fourier_dimension;
  params.extent = params_.fourier_extent > 0.0 ? params_.fourier_extent : params_.max_range;
  params.draw_sonar_rays = params_.fourier_draw_sonar_rays;
  params.use_initial_guess = params_.fourier_use_initial_guess;
  params.initial_dx = initial_dx;
  params.initial_dy = initial_dy;
  params.initial_yaw = initial_yaw;
  return params;
}

double ReturnHomeDetector::estimatePoolDiagonal() const {
  if (params_.pool_length > 0.0 && params_.pool_width > 0.0) {
    return std::hypot(params_.pool_length, params_.pool_width);
  }

  bool initialized = false;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  const auto add_pose = [&initialized, &min_x, &max_x, &min_y, &max_y](const Pose2D& pose) {
    if (!initialized) {
      min_x = max_x = pose.x;
      min_y = max_y = pose.y;
      initialized = true;
      return;
    }
    min_x = std::min(min_x, pose.x);
    max_x = std::max(max_x, pose.x);
    min_y = std::min(min_y, pose.y);
    max_y = std::max(max_y, pose.y);
  };

  if (has_home_ && home_.keyframe.odom_valid) {
    add_pose(home_.keyframe.odom_pose);
  }
  for (const CachedKeyframe& keyframe : negative_keyframes_) {
    if (keyframe.keyframe.odom_valid) {
      add_pose(keyframe.keyframe.odom_pose);
    }
  }
  if (!initialized) {
    return 0.0;
  }
  return std::hypot(max_x - min_x, max_y - min_y);
}

double ReturnHomeDetector::adaptiveHomeRadius(double pool_diag) const {
  if (!params_.use_adaptive_home_radius) {
    return std::max(0.0, params_.odom_candidate_radius);
  }
  const double radius_from_scale =
      pool_diag > 0.0 ? pool_diag * std::max(0.0, params_.adaptive_home_radius_ratio)
                      : params_.odom_candidate_radius;
  const double min_radius = std::max(0.0, params_.min_home_radius);
  const double max_radius = std::max(min_radius, params_.max_home_radius);
  const double adaptive = std::min(max_radius, std::max(min_radius, radius_from_scale));
  return std::max(params_.odom_candidate_radius, adaptive);
}

double ReturnHomeDetector::leaveHomeRadius(double home_radius) const {
  if (params_.leave_home_radius > 0.0) {
    return std::max(home_radius, params_.leave_home_radius);
  }
  const double by_multiplier =
      home_radius * std::max(1.0, params_.leave_home_radius_multiplier);
  const double by_extra = home_radius + std::max(0.0, params_.leave_home_radius_extra);
  return std::max(home_radius, std::max(by_multiplier, by_extra));
}

double ReturnHomeDetector::adaptiveSonarThreshold(double pool_diag,
                                                  double effective_range) const {
  double threshold = params_.match_score_threshold;
  if (pool_diag >= params_.large_pool_diag_threshold &&
      params_.large_pool_match_score_threshold > 0.0) {
    threshold = params_.large_pool_match_score_threshold;
  }
  if (!params_.use_range_adaptive_match_threshold ||
      pool_diag <= 0.0 ||
      effective_range <= 0.0) {
    return threshold;
  }

  const double ratio = effective_range / pool_diag;
  double relax = 0.0;
  if (ratio < params_.short_range_pool_ratio) {
    relax = std::max(0.0, params_.short_range_threshold_relax);
  } else if (ratio < params_.medium_range_pool_ratio) {
    relax = std::max(0.0, params_.medium_range_threshold_relax);
  }
  return std::max(params_.min_adaptive_match_score_threshold, threshold - relax);
}

bool ReturnHomeDetector::sameKeyframeIdentity(const SonarKeyframe& lhs,
                                              const SonarKeyframe& rhs) const {
  if (lhs.id >= 0 && rhs.id >= 0 && lhs.id == rhs.id) {
    return true;
  }
  if (lhs.scan_id != 0 && rhs.scan_id != 0 && lhs.scan_id == rhs.scan_id) {
    return true;
  }
  return lhs.cloud && rhs.cloud && lhs.cloud.get() == rhs.cloud.get();
}

int ReturnHomeDetector::countConfirmationHits() const {
  return static_cast<int>(std::count_if(
      confirmation_window_.begin(), confirmation_window_.end(),
      [](const std::pair<uint64_t, bool>& entry) { return entry.second; }));
}

void ReturnHomeDetector::applyConfirmation(ReturnHomeResult* result, bool single_frame_hit) {
  if (!result) {
    return;
  }

  const int required_hits = safePositiveInt(params_.confirm_required_hits, 1);
  if (required_hits <= 1) {
    result->confirm_hits = single_frame_hit ? 1 : 0;
    result->confirm_window = confirmation_window_.empty() ? 0 : 1;
    result->returned = single_frame_hit;
    return;
  }

  const int hits = countConfirmationHits();
  result->confirm_hits = hits;
  result->confirm_window = static_cast<int>(confirmation_window_.size());
  result->returned = hits >= required_hits;
  if (result->returned) {
    result->reason = "confirmed";
  } else if (single_frame_hit) {
    result->reason = "awaiting_confirmation";
  }
}

}  // namespace sonar_return_home_file_detector_core
