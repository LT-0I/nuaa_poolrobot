#include <sonar_return_home_file_detector/return_home_file_api.h>

#include "return_home_file_api_internal.h"
#include "sonar_return_home_file_detector/core/return_home_detector.h"

#include <pcl/io/pcd_io.h>

#include <ros/console.h>

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace sonar_return_home_file_detector {
namespace {

namespace fs = std::filesystem;
namespace core = sonar_return_home_file_detector_core;
using Clock = std::chrono::steady_clock;
using PointCloudI = pcl::PointCloud<pcl::PointXYZI>;

struct SnapshotMetadata {
  fs::path metadata_path;
  fs::path pointcloud_path;
  std::uint64_t scan_id = 0;
  int position_number = -1;
  double stamp_sec = 0.0;
  bool full_scan = true;
  double coverage_deg = 360.0;
  std::string cloud_frame_id = "sonar_frame";
  bool odom_valid = false;
  std::string odom_frame_id = "odom";
  std::uint64_t odom_epoch = 0;
  double odom_x = 0.0;
  double odom_y = 0.0;
  double odom_yaw = 0.0;
};

struct RuntimeState {
  std::string directory;
  std::unique_ptr<core::ReturnHomeDetector> detector;
  core::ReturnHomeParams params;
  bool home_set = false;
  std::string last_metadata_file;
  std::uint64_t last_scan_id = 0;
};

std::mutex& stateMutex() {
  static std::mutex mutex;
  return mutex;
}

RuntimeState& state() {
  static RuntimeState runtime_state;
  return runtime_state;
}

double elapsedMs(const Clock::time_point& start, const Clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

long currentMaxRssKb() {
  rusage usage = {};
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_maxrss;
}

std::string trim(const std::string& input) {
  std::size_t first = 0;
  while (first < input.size() &&
         std::isspace(static_cast<unsigned char>(input[first]))) {
    ++first;
  }
  std::size_t last = input.size();
  while (last > first &&
         std::isspace(static_cast<unsigned char>(input[last - 1]))) {
    --last;
  }
  return input.substr(first, last - first);
}

std::string stripQuotes(const std::string& input) {
  std::string value = trim(input);
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

bool parseBool(const std::string& input, bool fallback) {
  std::string value = stripQuotes(input);
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "true" || value == "1" || value == "yes") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no") {
    return false;
  }
  return fallback;
}

template <typename T>
bool parseNumber(const std::string& input, T* output) {
  if (!output) {
    return false;
  }
  std::stringstream stream(stripQuotes(input));
  T value {};
  stream >> value;
  if (!stream.fail()) {
    *output = value;
    return true;
  }
  return false;
}

bool hasReturnHomeYamlSuffix(const fs::path& path) {
  const std::string name = path.filename().string();
  const std::string suffix = ".return_home.yaml";
  return name.size() > suffix.size() &&
         name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool parseSnapshotMetadata(const fs::path& metadata_path,
                           SnapshotMetadata* metadata,
                           std::string* reason) {
  if (!metadata) {
    return false;
  }

  std::ifstream input(metadata_path);
  if (!input) {
    if (reason) {
      *reason = "metadata_open_failed";
    }
    return false;
  }

  SnapshotMetadata parsed;
  parsed.metadata_path = metadata_path;

  bool in_odom = false;
  std::string line;
  while (std::getline(input, line)) {
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    if (trim(line).empty()) {
      continue;
    }

    const bool nested = !line.empty() &&
                        std::isspace(static_cast<unsigned char>(line.front()));
    const std::string stripped = trim(line);
    const std::size_t colon = stripped.find(':');
    if (colon == std::string::npos) {
      continue;
    }

    const std::string key = trim(stripped.substr(0, colon));
    const std::string value = trim(stripped.substr(colon + 1));
    if (!nested && key != "odom") {
      in_odom = false;
    }
    if (!nested && key == "odom") {
      in_odom = true;
      continue;
    }

    if (in_odom && nested) {
      if (key == "valid") {
        parsed.odom_valid = parseBool(value, false);
      } else if (key == "frame_id") {
        parsed.odom_frame_id = stripQuotes(value);
      } else if (key == "epoch") {
        parseNumber(value, &parsed.odom_epoch);
      } else if (key == "x") {
        parseNumber(value, &parsed.odom_x);
      } else if (key == "y") {
        parseNumber(value, &parsed.odom_y);
      } else if (key == "yaw") {
        parseNumber(value, &parsed.odom_yaw);
      }
      continue;
    }

    if (key == "pointcloud") {
      parsed.pointcloud_path = stripQuotes(value);
    } else if (key == "scan_id") {
      parseNumber(value, &parsed.scan_id);
    } else if (key == "position_number") {
      parseNumber(value, &parsed.position_number);
    } else if (key == "stamp") {
      parseNumber(value, &parsed.stamp_sec);
    } else if (key == "full_scan") {
      parsed.full_scan = parseBool(value, true);
    } else if (key == "coverage_deg") {
      parseNumber(value, &parsed.coverage_deg);
    } else if (key == "cloud_frame_id") {
      parsed.cloud_frame_id = stripQuotes(value);
    }
  }

  if (parsed.pointcloud_path.empty()) {
    if (reason) {
      *reason = "metadata_missing_pointcloud";
    }
    return false;
  }
  if (parsed.pointcloud_path.is_absolute()) {
    if (reason) {
      *reason = "absolute_pointcloud_path_rejected";
    }
    return false;
  }
  if (!parsed.odom_valid) {
    if (reason) {
      *reason = "metadata_odom_invalid";
    }
    return false;
  }

  parsed.pointcloud_path = metadata_path.parent_path() / parsed.pointcloud_path;
  *metadata = parsed;
  return true;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool in_quotes = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
        field.push_back('"');
        ++i;
      } else {
        in_quotes = !in_quotes;
      }
      continue;
    }
    if (c == ',' && !in_quotes) {
      fields.push_back(trim(field));
      field.clear();
      continue;
    }
    field.push_back(c);
  }
  fields.push_back(trim(field));
  return fields;
}

bool parseCsvInt(const std::string& value, int* output) {
  return parseNumber(stripQuotes(value), output);
}

bool parseCsvDouble(const std::string& value, double* output) {
  return parseNumber(stripQuotes(value), output);
}

bool resolveCsvLayout(const fs::path& directory,
                      fs::path* root_dir,
                      fs::path* maps_dir,
                      fs::path* csv_path) {
  if (!root_dir || !maps_dir || !csv_path) {
    return false;
  }

  const fs::path direct_csv = directory / "sonar_odom_log.csv";
  const fs::path direct_maps = directory / "maps";
  if (fs::is_regular_file(direct_csv) && fs::is_directory(direct_maps)) {
    *root_dir = directory;
    *maps_dir = direct_maps;
    *csv_path = direct_csv;
    return true;
  }

  const fs::path parent = directory.parent_path();
  const fs::path parent_csv = parent / "sonar_odom_log.csv";
  if (directory.filename() == "maps" &&
      fs::is_regular_file(parent_csv) &&
      fs::is_directory(directory)) {
    *root_dir = parent;
    *maps_dir = directory;
    *csv_path = parent_csv;
    return true;
  }

  return false;
}

fs::path findCsvPointcloud(const fs::path& maps_dir,
                           const std::string& pointcloud_file,
                           int submap_index,
                           int position_number) {
  const std::string relative_file = stripQuotes(pointcloud_file);
  if (!relative_file.empty()) {
    const fs::path relative_path(relative_file);
    if (!relative_path.is_absolute()) {
      const fs::path direct = maps_dir / relative_path;
      if (fs::is_regular_file(direct)) {
        return direct;
      }
    }
  }

  const fs::path mapper_name =
      maps_dir / ("phase_multi_submap_" + std::to_string(submap_index) +
                  "_pos" + std::to_string(position_number) + ".pcd");
  if (fs::is_regular_file(mapper_name)) {
    return mapper_name;
  }

  const std::string prefix =
      "phase_multi_submap_" + std::to_string(submap_index) +
      "_pos" + std::to_string(position_number);
  if (fs::is_directory(maps_dir)) {
    for (const fs::directory_entry& entry : fs::directory_iterator(maps_dir)) {
      if (!entry.is_regular_file() || entry.path().extension() != ".pcd") {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.rfind(prefix, 0) == 0) {
        return entry.path();
      }
    }
  }

  return fs::path();
}

bool findLatestCsvSnapshot(const fs::path& directory,
                           SnapshotMetadata* metadata,
                           ReturnHomeProbeStats* stats) {
  if (!metadata) {
    return false;
  }

  fs::path root_dir;
  fs::path maps_dir;
  fs::path csv_path;
  if (!resolveCsvLayout(directory, &root_dir, &maps_dir, &csv_path)) {
    return false;
  }

  std::ifstream input(csv_path);
  if (!input) {
    if (stats) {
      stats->state = "NoSnapshot";
      stats->reason = "odom_csv_open_failed";
    }
    return false;
  }

  bool found = false;
  SnapshotMetadata latest;
  std::string line;
  std::size_t row_number = 0;
  std::string last_reason = "odom_csv_has_no_valid_rows";
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

  while (std::getline(input, line)) {
    ++row_number;
    if (trim(line).empty()) {
      continue;
    }
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.empty() || fields[0] == "timestamp") {
      continue;
    }
    if (fields.size() < 7U) {
      last_reason = "odom_csv_row_too_short";
      continue;
    }

    int submap_index = 0;
    int position_number = 0;
    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_yaw_deg = 0.0;
    if (!parseCsvInt(fields[1], &submap_index) ||
        !parseCsvInt(fields[2], &position_number) ||
        !parseCsvDouble(fields[3], &odom_x) ||
        !parseCsvDouble(fields[4], &odom_y) ||
        !parseCsvDouble(fields[5], &odom_yaw_deg)) {
      last_reason = "odom_csv_parse_failed";
      continue;
    }

    const fs::path pcd_path =
        findCsvPointcloud(maps_dir, fields[6], submap_index, position_number);
    if (pcd_path.empty()) {
      last_reason = "odom_csv_pointcloud_not_found";
      continue;
    }

    SnapshotMetadata candidate;
    candidate.metadata_path =
        fs::path(csv_path.string() + "#" + std::to_string(row_number));
    candidate.pointcloud_path = pcd_path;
    candidate.scan_id = static_cast<std::uint64_t>(row_number);
    candidate.position_number = position_number;
    candidate.stamp_sec = 0.0;
    candidate.full_scan = true;
    candidate.coverage_deg = 360.0;
    candidate.cloud_frame_id = "sonar_frame";
    candidate.odom_valid = true;
    candidate.odom_frame_id = "odom";
    candidate.odom_epoch = 0;
    candidate.odom_x = odom_x;
    candidate.odom_y = odom_y;
    candidate.odom_yaw = odom_yaw_deg * kDegToRad;
    latest = std::move(candidate);
    found = true;
  }

  if (!found) {
    if (stats) {
      stats->state = "NoSnapshot";
      stats->reason = last_reason;
    }
    return false;
  }

  *metadata = std::move(latest);
  return true;
}

bool isNewerSnapshot(const SnapshotMetadata& lhs, const SnapshotMetadata& rhs) {
  if (lhs.scan_id != 0 || rhs.scan_id != 0) {
    if (lhs.scan_id != rhs.scan_id) {
      return lhs.scan_id > rhs.scan_id;
    }
  }
  if (lhs.stamp_sec != rhs.stamp_sec) {
    return lhs.stamp_sec > rhs.stamp_sec;
  }
  return lhs.metadata_path.filename().string() > rhs.metadata_path.filename().string();
}

bool findLatestSnapshot(const fs::path& directory,
                        SnapshotMetadata* metadata,
                        ReturnHomeProbeStats* stats) {
  if (!metadata) {
    return false;
  }
  if (!fs::is_directory(directory)) {
    if (stats) {
      stats->state = "NoSnapshot";
      stats->reason = "directory_not_found";
    }
    return false;
  }

  fs::path csv_root_dir;
  fs::path csv_maps_dir;
  fs::path csv_path;
  if (resolveCsvLayout(directory, &csv_root_dir, &csv_maps_dir, &csv_path)) {
    return findLatestCsvSnapshot(directory, metadata, stats);
  }

  bool found = false;
  SnapshotMetadata best;
  std::string last_parse_reason;
  for (const fs::directory_entry& entry : fs::directory_iterator(directory)) {
    if (!entry.is_regular_file() || !hasReturnHomeYamlSuffix(entry.path())) {
      continue;
    }

    SnapshotMetadata candidate;
    std::string parse_reason;
    const auto metadata_start = Clock::now();
    if (!parseSnapshotMetadata(entry.path(), &candidate, &parse_reason)) {
      if (stats) {
        stats->metadata_ms += elapsedMs(metadata_start, Clock::now());
      }
      last_parse_reason = parse_reason;
      continue;
    }
    if (stats) {
      stats->metadata_ms += elapsedMs(metadata_start, Clock::now());
    }
    if (!found || isNewerSnapshot(candidate, best)) {
      best = std::move(candidate);
      found = true;
    }
  }

  if (!found) {
    if (stats) {
      stats->state = "NoSnapshot";
      stats->reason = last_parse_reason.empty() ? "no_return_home_metadata"
                                                : last_parse_reason;
    }
    return false;
  }

  *metadata = std::move(best);
  return true;
}

ros::Time rosTimeFromSec(double stamp_sec) {
  ros::Time stamp;
  if (stamp_sec > 0.0) {
    stamp.fromSec(stamp_sec);
  }
  return stamp;
}

core::Pose2D makePose(const SnapshotMetadata& metadata) {
  core::Pose2D pose;
  pose.x = metadata.odom_x;
  pose.y = metadata.odom_y;
  pose.yaw = metadata.odom_yaw;
  return pose;
}

core::SonarKeyframe makeHomeKeyframe(const SnapshotMetadata& metadata,
                                     const PointCloudI::ConstPtr& cloud) {
  core::SonarKeyframe keyframe;
  keyframe.id = static_cast<int>(metadata.scan_id);
  keyframe.position_number = metadata.position_number;
  keyframe.stamp = rosTimeFromSec(metadata.stamp_sec);
  keyframe.odom_stamp = keyframe.stamp;
  keyframe.cloud = cloud;
  keyframe.odom_pose = makePose(metadata);
  keyframe.odom_valid = metadata.odom_valid;
  keyframe.odom_frame_id = metadata.odom_frame_id;
  keyframe.odom_epoch = metadata.odom_epoch;
  keyframe.scan_id = metadata.scan_id;
  keyframe.full_scan = metadata.full_scan;
  keyframe.coverage_deg = metadata.coverage_deg;
  keyframe.frame_id = metadata.cloud_frame_id;
  keyframe.cloud_source = core::ProcessedCloudSource::ExternalProcessedCloud;
  keyframe.quality = 1.0;
  return keyframe;
}

core::CurrentSonarFrame makeCurrentFrame(const SnapshotMetadata& metadata,
                                         const PointCloudI::ConstPtr& cloud) {
  core::CurrentSonarFrame frame;
  frame.stamp = rosTimeFromSec(metadata.stamp_sec);
  frame.odom_stamp = frame.stamp;
  frame.cloud = cloud;
  frame.odom_pose = makePose(metadata);
  frame.odom_valid = metadata.odom_valid;
  frame.odom_frame_id = metadata.odom_frame_id;
  frame.odom_epoch = metadata.odom_epoch;
  frame.scan_id = metadata.scan_id;
  frame.full_scan = metadata.full_scan;
  frame.coverage_deg = metadata.coverage_deg;
  frame.frame_id = metadata.cloud_frame_id;
  frame.cloud_source = core::ProcessedCloudSource::ExternalProcessedCloud;
  return frame;
}

core::SonarKeyframe makeHistoryKeyframe(const SnapshotMetadata& metadata,
                                        const PointCloudI::ConstPtr& cloud) {
  core::SonarKeyframe keyframe = makeHomeKeyframe(metadata, cloud);
  keyframe.quality = 0.8;
  return keyframe;
}

void ensureDetector(RuntimeState* runtime_state,
                    const std::string& directory,
                    ReturnHomeProbeStats* stats) {
  if (!runtime_state) {
    return;
  }
  if (runtime_state->detector && runtime_state->directory == directory) {
    return;
  }

  core::ReturnHomeParams params;
  params.enabled = true;
  params.confirm_window_size = 1;
  params.confirm_required_hits = 1;
  params.yaw_is_hard_constraint = false;

  runtime_state->directory = directory;
  runtime_state->params = params;
  runtime_state->detector.reset(new core::ReturnHomeDetector(params));
  runtime_state->home_set = false;
  runtime_state->last_metadata_file.clear();
  runtime_state->last_scan_id = 0;
  if (stats) {
    stats->state = "DetectorInitialized";
  }
}

}  // namespace

bool detectReturnHomeFromDirectory(const std::string& directory) {
  return detectReturnHomeFromDirectoryForProbe(directory).returned;
}

ReturnHomeProbeStats detectReturnHomeFromDirectoryForProbe(const std::string& directory) {
  ReturnHomeProbeStats stats;
  stats.directory = directory;
  const auto total_start = Clock::now();

  std::lock_guard<std::mutex> lock(stateMutex());
  RuntimeState& runtime_state = state();

  const auto init_start = Clock::now();
  ensureDetector(&runtime_state, directory, &stats);
  const auto init_end = Clock::now();
  stats.init_ms = elapsedMs(init_start, init_end);

  const auto scan_start = Clock::now();
  SnapshotMetadata metadata;
  if (!findLatestSnapshot(fs::path(directory), &metadata, &stats)) {
    const auto total_end = Clock::now();
    stats.scan_dir_ms = elapsedMs(scan_start, total_end);
    stats.total_ms = elapsedMs(total_start, total_end);
    stats.max_rss_kb = currentMaxRssKb();
    return stats;
  }
  const auto scan_end = Clock::now();
  stats.scan_dir_ms = elapsedMs(scan_start, scan_end);
  stats.metadata_file = metadata.metadata_path.string();
  stats.pointcloud_file = metadata.pointcloud_path.string();
  stats.scan_id = metadata.scan_id;
  stats.position_number = metadata.position_number;

  const std::string metadata_file = metadata.metadata_path.string();
  if (runtime_state.last_metadata_file == metadata_file) {
    stats.state = "NoNewSnapshot";
    stats.reason = "latest_snapshot_already_processed";
    const auto total_end = Clock::now();
    stats.total_ms = elapsedMs(total_start, total_end);
    stats.max_rss_kb = currentMaxRssKb();
    return stats;
  }
  if (metadata.scan_id != 0 && runtime_state.last_scan_id != 0 &&
      metadata.scan_id <= runtime_state.last_scan_id) {
    stats.state = "NoNewSnapshot";
    stats.reason = "scan_id_not_newer_than_last_processed";
    const auto total_end = Clock::now();
    stats.total_ms = elapsedMs(total_start, total_end);
    stats.max_rss_kb = currentMaxRssKb();
    return stats;
  }

  const auto pcd_start = Clock::now();
  PointCloudI::Ptr cloud(new PointCloudI);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(metadata.pointcloud_path.string(), *cloud) != 0) {
    stats.state = "LoadFailed";
    stats.reason = "pcd_load_failed";
    const auto total_end = Clock::now();
    stats.pcd_load_ms = elapsedMs(pcd_start, total_end);
    stats.total_ms = elapsedMs(total_start, total_end);
    stats.max_rss_kb = currentMaxRssKb();
    ROS_WARN("回到起点目录检测：读取点云失败: %s",
             metadata.pointcloud_path.string().c_str());
    return stats;
  }
  const auto pcd_end = Clock::now();
  stats.pcd_load_ms = elapsedMs(pcd_start, pcd_end);
  stats.point_count = cloud->size();

  if (cloud->empty()) {
    stats.state = "LoadFailed";
    stats.reason = "pcd_empty";
    const auto total_end = Clock::now();
    stats.total_ms = elapsedMs(total_start, total_end);
    stats.max_rss_kb = currentMaxRssKb();
    return stats;
  }

  if (!runtime_state.home_set) {
    const auto detect_start = Clock::now();
    runtime_state.detector->setHomeKeyframe(makeHomeKeyframe(metadata, cloud));
    const auto detect_end = Clock::now();
    runtime_state.home_set = true;
    runtime_state.last_metadata_file = metadata_file;
    runtime_state.last_scan_id = metadata.scan_id;
    stats.home_initialized = true;
    stats.processed_new_snapshot = true;
    stats.state = "HomeInitialized";
    stats.reason = "first_snapshot_used_as_home";
    stats.detect_ms = elapsedMs(detect_start, detect_end);
    const auto total_end = Clock::now();
    stats.total_ms = elapsedMs(total_start, total_end);
    stats.max_rss_kb = currentMaxRssKb();
    ROS_INFO("回到起点目录检测：已初始化起点 scan_id=%lu points=%zu file=%s",
             static_cast<unsigned long>(metadata.scan_id),
             cloud->size(),
             metadata.pointcloud_path.filename().string().c_str());
    return stats;
  }

  const auto detect_start = Clock::now();
  const core::ReturnHomeResult result =
      runtime_state.detector->detect(makeCurrentFrame(metadata, cloud));
  const auto detect_end = Clock::now();

  if (!result.returned) {
    const double add_history_radius =
        std::max(runtime_state.params.negative_keyframe_min_distance,
                 std::max(result.adaptive_home_radius,
                          result.leave_home_radius *
                              std::max(1.0, runtime_state.params.negative_keyframe_min_leave_radius_multiplier)));
    if (result.odom_distance >= add_history_radius) {
      runtime_state.detector->addHistoryKeyframe(makeHistoryKeyframe(metadata, cloud));
    }
  }

  runtime_state.last_metadata_file = metadata_file;
  runtime_state.last_scan_id = metadata.scan_id;
  stats.processed_new_snapshot = true;
  stats.returned = result.returned;
  stats.state = "Detected";
  stats.reason = result.reason;
  stats.detect_ms = elapsedMs(detect_start, detect_end);

  ROS_INFO("回到起点目录检测：scan_id=%lu pos=%d returned=%s score=%.3f odom=%.3f reason=%s",
           static_cast<unsigned long>(metadata.scan_id),
           metadata.position_number,
           result.returned ? "true" : "false",
           result.match_score,
           result.odom_distance,
           result.reason.c_str());

  const auto total_end = Clock::now();
  stats.total_ms = elapsedMs(total_start, total_end);
  stats.max_rss_kb = currentMaxRssKb();
  return stats;
}

void resetReturnHomeFileDetectorForProbe() {
  std::lock_guard<std::mutex> lock(stateMutex());
  state() = RuntimeState();
}

}  // namespace sonar_return_home_file_detector
