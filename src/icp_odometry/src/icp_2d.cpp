#include "icp_odometry/icp_2d.hpp"

#include <cmath>
#include <limits>

namespace icp_odometry
{

std::vector<Point2D> laserScanToPoints(
  const std::vector<float> & ranges,
  float angle_min,
  float angle_increment,
  int stride)
{
  std::vector<Point2D> points;
  if (stride < 1) {
    stride = 1;
  }
  points.reserve(ranges.size() / static_cast<std::size_t>(stride));
  for (std::size_t index = 0; index < ranges.size(); index += static_cast<std::size_t>(stride)) {
    const float range = ranges[index];
    if (!std::isfinite(range)) {
      continue;
    }
    const double angle = angle_min + static_cast<double>(index) * angle_increment;
    points.push_back({range * std::cos(angle), range * std::sin(angle)});
  }
  return points;
}

std::vector<std::pair<int, int>> findCorrespondences(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target)
{
  std::vector<std::pair<int, int>> pairs;
  pairs.reserve(source.size());
  for (int source_index = 0; source_index < static_cast<int>(source.size()); ++source_index) {
    double best_distance = std::numeric_limits<double>::max();
    int best_target = -1;
    for (int target_index = 0; target_index < static_cast<int>(target.size()); ++target_index) {
      const double dx = source[source_index].x - target[target_index].x;
      const double dy = source[source_index].y - target[target_index].y;
      const double distance = dx * dx + dy * dy;
      if (distance < best_distance) {
        best_distance = distance;
        best_target = target_index;
      }
    }
    if (best_target >= 0) {
      pairs.emplace_back(source_index, best_target);
    }
  }
  return pairs;
}

std::vector<std::pair<int, int>> rejectOutliers(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target,
  const std::vector<std::pair<int, int>> & pairs,
  double max_distance)
{
  const double max_distance_sq = max_distance * max_distance;
  std::vector<std::pair<int, int>> filtered;
  filtered.reserve(pairs.size());
  for (const auto & pair : pairs) {
    const double dx = source[pair.first].x - target[pair.second].x;
    const double dy = source[pair.first].y - target[pair.second].y;
    if ((dx * dx + dy * dy) <= max_distance_sq) {
      filtered.push_back(pair);
    }
  }
  return filtered;
}

Eigen::Isometry2d estimateRigidTransform(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target,
  const std::vector<std::pair<int, int>> & pairs)
{
  if (pairs.empty()) {
    return Eigen::Isometry2d::Identity();
  }

  Eigen::Vector2d source_centroid(0.0, 0.0);
  Eigen::Vector2d target_centroid(0.0, 0.0);
  for (const auto & pair : pairs) {
    source_centroid += Eigen::Vector2d(source[pair.first].x, source[pair.first].y);
    target_centroid += Eigen::Vector2d(target[pair.second].x, target[pair.second].y);
  }
  source_centroid /= static_cast<double>(pairs.size());
  target_centroid /= static_cast<double>(pairs.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto & pair : pairs) {
    const Eigen::Vector2d source_centered(
      source[pair.first].x - source_centroid.x(),
      source[pair.first].y - source_centroid.y());
    const Eigen::Vector2d target_centered(
      target[pair.second].x - target_centroid.x(),
      target[pair.second].y - target_centroid.y());
    covariance += target_centered * source_centered.transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d rotation = svd.matrixU() * svd.matrixV().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix2d fix = Eigen::Matrix2d::Identity();
    fix(1, 1) = -1.0;
    rotation = svd.matrixU() * fix * svd.matrixV().transpose();
  }

  const Eigen::Vector2d translation = target_centroid - rotation * source_centroid;
  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
  transform.linear() = rotation;
  transform.translation() = translation;
  return transform;
}

Point2D applyTransform(const Point2D & point, const Eigen::Isometry2d & transform)
{
  const Eigen::Vector2d transformed =
    transform * Eigen::Vector2d(point.x, point.y);
  return {transformed.x(), transformed.y()};
}

bool hasConverged(
  const Eigen::Isometry2d & previous,
  const Eigen::Isometry2d & current,
  const IcpParameters & params)
{
  const Eigen::Vector2d delta_translation =
    current.translation() - previous.translation();
  const double translation_delta = delta_translation.norm();
  const double previous_yaw = std::atan2(previous.linear()(1, 0), previous.linear()(0, 0));
  const double current_yaw = std::atan2(current.linear()(1, 0), current.linear()(0, 0));
  const double rotation_delta = std::abs(std::atan2(
      std::sin(current_yaw - previous_yaw),
      std::cos(current_yaw - previous_yaw)));
  return translation_delta < params.convergence_translation &&
         rotation_delta < params.convergence_rotation;
}

IcpResult runIcp(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target,
  const IcpParameters & params)
{
  IcpResult result;
  if (source.size() < static_cast<std::size_t>(params.minimum_correspondences) ||
    target.size() < static_cast<std::size_t>(params.minimum_correspondences))
  {
    return result;
  }

  std::vector<Point2D> transformed = source;
  Eigen::Isometry2d cumulative = Eigen::Isometry2d::Identity();

  for (int iteration = 0; iteration < params.max_iterations; ++iteration) {
    auto pairs = findCorrespondences(transformed, target);
    pairs = rejectOutliers(transformed, target, pairs, params.correspondence_distance);
    result.correspondences = static_cast<int>(pairs.size());
    if (pairs.size() < static_cast<std::size_t>(params.minimum_correspondences)) {
      result.transform = cumulative;
      return result;
    }

    const Eigen::Isometry2d previous = cumulative;
    const Eigen::Isometry2d delta = estimateRigidTransform(transformed, target, pairs);
    cumulative = delta * cumulative;
    for (auto & point : transformed) {
      point = applyTransform(point, delta);
    }

    result.iterations = iteration + 1;
    if (hasConverged(previous, cumulative, params)) {
      result.converged = true;
      break;
    }
  }

  result.valid = true;
  result.transform = cumulative;
  return result;
}

}  // namespace icp_odometry
