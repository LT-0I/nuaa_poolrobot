#pragma once

#include <string>

namespace sonar_return_home_file_detector {

// The only public call-in API.
//
// Product mapper mode:
//   directory/sonar_odom_log.csv
//   directory/maps/<base>.pcd
//
// Legacy sidecar mode is also accepted:
//   directory/<base>.pcd
//   directory/<base>.return_home.yaml
//
// First valid snapshot initializes the home keyframe and returns false.
// Later new snapshots are checked against home; true means returned to home.
bool detectReturnHomeFromDirectory(const std::string& directory);

}  // namespace sonar_return_home_file_detector
