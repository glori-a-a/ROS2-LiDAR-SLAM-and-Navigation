#pragma once

#include <Eigen/Dense>
#include <utility>
#include <vector>

namespace icp_odometry
{

struct Point2D
{
  double x;
  double y;
};

struct IcpParameters
{
  int max_iterations{20};
  double correspondence_distance{2.0};
  double convergence_translation{1e-3};
  double convergence_rotation{1e-3};
  int minimum_correspondences{10};
};

struct IcpResult
{
  bool converged{false};
  bool valid{false};
  Eigen::Isometry2d transform{Eigen::Isometry2d::Identity()};
  int iterations{0};
  int correspondences{0};
};

std::vector<Point2D> laserScanToPoints(
  const std::vector<float> & ranges,
  float angle_min,
  float angle_increment,
  int stride);

std::vector<std::pair<int, int>> findCorrespondences(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target);

std::vector<std::pair<int, int>> rejectOutliers(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target,
  const std::vector<std::pair<int, int>> & pairs,
  double max_distance);

Eigen::Isometry2d estimateRigidTransform(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target,
  const std::vector<std::pair<int, int>> & pairs);

Point2D applyTransform(const Point2D & point, const Eigen::Isometry2d & transform);

bool hasConverged(
  const Eigen::Isometry2d & previous,
  const Eigen::Isometry2d & current,
  const IcpParameters & params);

IcpResult runIcp(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target,
  const IcpParameters & params);

double normalizeYaw(double yaw);

Eigen::Isometry2d isometryFromXYYaw(double x, double y, double yaw);

void xyYawFromIsometry(const Eigen::Isometry2d & pose, double & x, double & y, double & yaw);

Eigen::Isometry2d composePoses(
  const Eigen::Isometry2d & parent,
  const Eigen::Isometry2d & child);

Eigen::Isometry2d relativePose(
  const Eigen::Isometry2d & from,
  const Eigen::Isometry2d & to);

}  // namespace icp_odometry
