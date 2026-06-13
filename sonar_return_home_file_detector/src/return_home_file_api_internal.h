#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace sonar_return_home_file_detector {

struct ReturnHomeProbeStats {
  bool returned = false;
  bool processed_new_snapshot = false;
  bool home_initialized = false;
  std::string state;
  std::string reason;
  std::string directory;
  std::string metadata_file;
  std::string pointcloud_file;
  std::uint64_t scan_id = 0;
  int position_number = -1;
  std::size_t point_count = 0;
  double init_ms = 0.0;
  double scan_dir_ms = 0.0;
  double metadata_ms = 0.0;
  double pcd_load_ms = 0.0;
  double detect_ms = 0.0;
  double total_ms = 0.0;
  long max_rss_kb = 0;
};

ReturnHomeProbeStats detectReturnHomeFromDirectoryForProbe(const std::string& directory);
void resetReturnHomeFileDetectorForProbe();

}  // namespace sonar_return_home_file_detector
