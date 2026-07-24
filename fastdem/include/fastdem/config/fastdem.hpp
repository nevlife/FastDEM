// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_CONFIG_FASTDEM_HPP
#define FASTDEM_CONFIG_FASTDEM_HPP

#include <limits>
#include <string>

namespace YAML {
class Node;
}

#include "fastdem/config/mapping.hpp"
#include "fastdem/config/postprocess.hpp"
#include "fastdem/config/sensor_model.hpp"

namespace fastdem {

namespace config {

/// Point filtering bounds (z-range and distance).
struct PointFilter {
  float z_min = -std::numeric_limits<float>::max();
  float z_max = std::numeric_limits<float>::max();
  float range_min = 0.0f;
  float range_max = std::numeric_limits<float>::max();
};

/// Robot-pose uncertainty propagated into the per-point height measurement variance.
///
/// The sensor model only accounts for LiDAR range/angular noise. On slope crests/toes a
/// tilting vehicle has a transient attitude (roll/pitch) and z error, which places far
/// points at the wrong height and fabricates roughness. Propagating that pose uncertainty
/// through the measurement geometry inflates a point's map-frame z variance by
///     r_horizontal^2 * tilt_variance + z_variance
/// so far/oblique points (the ones most corrupted by tilt) get down-weighted in the Kalman
/// fusion instead of baked in as confident bumps. tilt/z variance are a static fallback;
/// FastDEM::setPoseNoise() can override them per-scan from live odometry covariance.
struct PoseNoise {
  bool enable = false;
  float tilt_variance = 0.0f;  ///< roll+pitch variance [rad^2]
  float z_variance = 0.0f;     ///< height variance [m^2]
};

}  // namespace config

/// Pipeline configuration for FastDEM.
struct Config {
  config::PointFilter point_filter;
  config::SensorModel sensor_model;
  config::Mapping mapping;
  config::Raycasting raycasting;
  config::PoseNoise pose_noise;
};

Config parseConfig(const YAML::Node& root);
Config loadConfig(const std::string& path);

}  // namespace fastdem

#endif  // FASTDEM_CONFIG_FASTDEM_HPP
