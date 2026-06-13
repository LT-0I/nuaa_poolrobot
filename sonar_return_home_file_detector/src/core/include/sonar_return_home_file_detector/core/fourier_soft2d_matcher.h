#ifndef SONAR_RETURN_HOME_FILE_DETECTOR_CORE_FOURIER_SOFT2D_MATCHER_H
#define SONAR_RETURN_HOME_FILE_DETECTOR_CORE_FOURIER_SOFT2D_MATCHER_H

#include <cstdint>
#include <string>
#include <vector>

namespace sonar_return_home_file_detector_core {

struct FourierSoft2DPoint {
  float x = 0.0F;
  float y = 0.0F;
  float intensity = 0.0F;
};

struct FourierSoft2DParams {
  int dimension = 64;
  double extent = 8.0;
  bool draw_sonar_rays = true;
  bool use_initial_guess = true;
  double initial_dx = 0.0;
  double initial_dy = 0.0;
  double initial_yaw = 0.0;
};

struct FourierSoft2DDescriptor {
  bool valid = false;
  int dimension = 0;
  double extent = 0.0;
  double cell_size = 0.0;
  std::vector<double> voxel;
  std::vector<uint8_t> occupied;
  int occupied_cells = 0;
};

struct FourierSoft2DMatchResult {
  bool valid = false;
  double match_score = 0.0;
  double peak_score = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double yaw = 0.0;
  int overlap_cells = 0;
  int union_cells = 0;
  int best_solution_index = -1;
  std::string reason;
};

class FourierSoft2DMatcher {
 public:
  explicit FourierSoft2DMatcher(const FourierSoft2DParams& params = FourierSoft2DParams());

  FourierSoft2DDescriptor buildDescriptor(const std::vector<FourierSoft2DPoint>& points) const;
  FourierSoft2DMatchResult match(const FourierSoft2DDescriptor& reference,
                                 const FourierSoft2DDescriptor& current) const;

  bool backendAvailable() const;
  const FourierSoft2DParams& params() const;

 private:
  FourierSoft2DParams params_;
};

}  // namespace sonar_return_home_file_detector_core

#endif  // SONAR_RETURN_HOME_FILE_DETECTOR_CORE_FOURIER_SOFT2D_MATCHER_H
