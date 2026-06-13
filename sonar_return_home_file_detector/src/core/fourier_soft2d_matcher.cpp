#include "sonar_return_home_file_detector/core/fourier_soft2d_matcher.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_set>

#if SONAR_RETURN_HOME_ENABLE_FOURIER_SOFT2D
#include <Eigen/Dense>
#include "softDescriptorRegistration.h"
#endif

namespace sonar_return_home_file_detector_core {
namespace {

#if SONAR_RETURN_HOME_ENABLE_FOURIER_SOFT2D
constexpr double kPi = 3.14159265358979323846;

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
#endif

int safeDimension(int dimension) {
  if (dimension < 16) {
    return 16;
  }
  int power = 1;
  while (power * 2 <= dimension) {
    power *= 2;
  }
  return std::max(16, power);
}

bool inBounds(int x, int y, int dimension) {
  return x >= 0 && y >= 0 && x < dimension && y < dimension;
}

int toIndex(int x, int y, int dimension) {
  return y * dimension + x;
}

void markCell(FourierSoft2DDescriptor* descriptor, int x, int y, double value) {
  if (!descriptor || !inBounds(x, y, descriptor->dimension)) {
    return;
  }
  const int index = toIndex(x, y, descriptor->dimension);
  descriptor->voxel[static_cast<std::size_t>(index)] =
      std::max(descriptor->voxel[static_cast<std::size_t>(index)], value);
  if (!descriptor->occupied[static_cast<std::size_t>(index)] && value >= 0.5) {
    descriptor->occupied[static_cast<std::size_t>(index)] = 1U;
    ++descriptor->occupied_cells;
  }
}

void drawRay(FourierSoft2DDescriptor* descriptor, int x0, int y0, int x1, int y1) {
  if (!descriptor) {
    return;
  }
  int dx = std::abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  int x = x0;
  int y = y0;
  while (true) {
    markCell(descriptor, x, y, 0.01);
    if (x == x1 && y == y1) {
      break;
    }
    const int error2 = 2 * error;
    if (error2 >= dy) {
      error += dy;
      x += sx;
    }
    if (error2 <= dx) {
      error += dx;
      y += sy;
    }
  }
}

struct OverlapScore {
  double score = 0.0;
  int overlap = 0;
  int union_cells = 0;
};

#if SONAR_RETURN_HOME_ENABLE_FOURIER_SOFT2D
struct RegistrationCache {
  int dimension = 0;
  std::unique_ptr<softDescriptorRegistration> registration;
};

softDescriptorRegistration& getRegistration(int dimension) {
  thread_local RegistrationCache cache;
  if (!cache.registration || cache.dimension != dimension) {
    cache.dimension = dimension;
    cache.registration.reset(new softDescriptorRegistration(dimension,
                                                            dimension / 2,
                                                            dimension / 2,
                                                            dimension / 2 - 1));
  }
  return *cache.registration;
}

std::mutex& fourierBackendMutex() {
  static std::mutex mutex;
  return mutex;
}

OverlapScore scoreOverlap(const FourierSoft2DDescriptor& reference,
                          const FourierSoft2DDescriptor& current,
                          double dx,
                          double dy,
                          double yaw) {
  OverlapScore score;
  if (!reference.valid || !current.valid ||
      reference.dimension != current.dimension ||
      reference.cell_size <= 0.0) {
    return score;
  }

  const int dimension = reference.dimension;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  std::vector<uint8_t> transformed(current.occupied.size(), 0U);
  int transformed_count = 0;

  for (int y = 0; y < dimension; ++y) {
    for (int x = 0; x < dimension; ++x) {
      const int index = toIndex(x, y, dimension);
      if (!current.occupied[static_cast<std::size_t>(index)]) {
        continue;
      }

      const double local_x =
          (static_cast<double>(x) + 0.5) * current.cell_size - current.extent;
      const double local_y =
          (static_cast<double>(y) + 0.5) * current.cell_size - current.extent;
      const double aligned_x = cos_yaw * local_x - sin_yaw * local_y + dx;
      const double aligned_y = sin_yaw * local_x + cos_yaw * local_y + dy;
      const int rx = static_cast<int>(std::floor((aligned_x + reference.extent) /
                                                 reference.cell_size));
      const int ry = static_cast<int>(std::floor((aligned_y + reference.extent) /
                                                 reference.cell_size));
      if (!inBounds(rx, ry, dimension)) {
        continue;
      }
      const int aligned_index = toIndex(rx, ry, dimension);
      if (!transformed[static_cast<std::size_t>(aligned_index)]) {
        transformed[static_cast<std::size_t>(aligned_index)] = 1U;
        ++transformed_count;
      }
    }
  }

  int overlap = 0;
  for (std::size_t i = 0; i < transformed.size(); ++i) {
    if (reference.occupied[i] && transformed[i]) {
      ++overlap;
    }
  }
  const int union_cells = reference.occupied_cells + transformed_count - overlap;
  const double iou = union_cells > 0 ? static_cast<double>(overlap) /
                                           static_cast<double>(union_cells)
                                     : 0.0;
  const int min_cells = std::max(1, std::min(reference.occupied_cells, transformed_count));
  const double coverage = static_cast<double>(overlap) / static_cast<double>(min_cells);

  score.overlap = overlap;
  score.union_cells = union_cells;
  score.score = clamp01(0.5 * iou + 0.5 * coverage);
  return score;
}
#endif

}  // namespace

FourierSoft2DMatcher::FourierSoft2DMatcher(const FourierSoft2DParams& params)
    : params_(params) {
  params_.dimension = safeDimension(params_.dimension);
  params_.extent = std::max(1.0, params_.extent);
}

FourierSoft2DDescriptor FourierSoft2DMatcher::buildDescriptor(
    const std::vector<FourierSoft2DPoint>& points) const {
  FourierSoft2DDescriptor descriptor;
  descriptor.dimension = params_.dimension;
  descriptor.extent = params_.extent;
  descriptor.cell_size = 2.0 * descriptor.extent / static_cast<double>(descriptor.dimension);
  const std::size_t total_cells =
      static_cast<std::size_t>(descriptor.dimension) *
      static_cast<std::size_t>(descriptor.dimension);
  descriptor.voxel.assign(total_cells, 0.0);
  descriptor.occupied.assign(total_cells, 0U);

  const int center = descriptor.dimension / 2;
  for (const FourierSoft2DPoint& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    const int x = static_cast<int>(std::floor((static_cast<double>(point.x) +
                                               descriptor.extent) /
                                              descriptor.cell_size));
    const int y = static_cast<int>(std::floor((static_cast<double>(point.y) +
                                               descriptor.extent) /
                                              descriptor.cell_size));
    if (!inBounds(x, y, descriptor.dimension)) {
      continue;
    }
    if (params_.draw_sonar_rays) {
      drawRay(&descriptor, center, center, x, y);
    }
    markCell(&descriptor, x, y, 1.0);
  }

  descriptor.valid = descriptor.occupied_cells > 0;
  return descriptor;
}

FourierSoft2DMatchResult FourierSoft2DMatcher::match(
    const FourierSoft2DDescriptor& reference,
    const FourierSoft2DDescriptor& current) const {
  FourierSoft2DMatchResult result;
  if (!reference.valid || !current.valid) {
    result.reason = "fourier_descriptor_invalid";
    return result;
  }
  if (reference.dimension != current.dimension ||
      std::abs(reference.cell_size - current.cell_size) > 1.0e-9) {
    result.reason = "fourier_descriptor_geometry_mismatch";
    return result;
  }

#if SONAR_RETURN_HOME_ENABLE_FOURIER_SOFT2D
  try {
    std::lock_guard<std::mutex> backend_lock(fourierBackendMutex());
    softDescriptorRegistration& registration = getRegistration(reference.dimension);
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    const double cos_yaw = std::cos(params_.initial_yaw);
    const double sin_yaw = std::sin(params_.initial_yaw);
    initial_guess(0, 0) = cos_yaw;
    initial_guess(0, 1) = -sin_yaw;
    initial_guess(1, 0) = sin_yaw;
    initial_guess(1, 1) = cos_yaw;
    initial_guess(0, 3) = params_.initial_dx;
    initial_guess(1, 3) = params_.initial_dy;

    int best_solution_index = -1;
    std::vector<double> ref_voxel = reference.voxel;
    std::vector<double> cur_voxel = current.voxel;
    const Eigen::Matrix4d transform = registration.registrationOfTwoVoxelsSOFTFast(
        ref_voxel.data(),
        cur_voxel.data(),
        initial_guess,
        params_.use_initial_guess,
        params_.use_initial_guess,
        reference.cell_size,
        "",
        best_solution_index,
        false);
    const double raw_peak = registration.lastMaximumHeightPeak();

    const double yaw = normalizeAngle(std::atan2(transform(1, 0), transform(0, 0)));
    const double dx = transform(0, 3);
    const double dy = transform(1, 3);
    const OverlapScore direct = scoreOverlap(reference, current, dx, dy, yaw);
    const OverlapScore inverse = scoreOverlap(reference, current, -dx, -dy, -yaw);
    const bool use_inverse = inverse.score > direct.score;
    const OverlapScore& best = use_inverse ? inverse : direct;

    result.valid = true;
    result.match_score = best.score;
    const double peak_normalizer =
        std::max(1.0, std::sqrt(static_cast<double>(std::max(1, reference.occupied_cells)) *
                                static_cast<double>(std::max(1, current.occupied_cells))));
    result.peak_score = clamp01(raw_peak / peak_normalizer);
    result.dx = use_inverse ? -dx : dx;
    result.dy = use_inverse ? -dy : dy;
    result.yaw = use_inverse ? std::abs(normalizeAngle(-yaw)) : std::abs(yaw);
    result.overlap_cells = best.overlap;
    result.union_cells = best.union_cells;
    result.best_solution_index = best_solution_index;
    result.reason = "fourier_soft2d_matched";
    return result;
  } catch (...) {
    result.reason = "fourier_soft2d_exception";
    return result;
  }
#else
  result.reason = "fourier_soft2d_backend_unavailable";
  return result;
#endif
}

bool FourierSoft2DMatcher::backendAvailable() const {
#if SONAR_RETURN_HOME_ENABLE_FOURIER_SOFT2D
  return true;
#else
  return false;
#endif
}

const FourierSoft2DParams& FourierSoft2DMatcher::params() const {
  return params_;
}

}  // namespace sonar_return_home_file_detector_core
