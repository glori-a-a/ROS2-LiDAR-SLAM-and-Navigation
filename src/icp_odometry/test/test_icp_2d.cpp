#include <cmath>
#include <gtest/gtest.h>

#include "icp_odometry/icp_2d.hpp"

using icp_odometry::IcpParameters;
using icp_odometry::Point2D;
using icp_odometry::applyTransform;
using icp_odometry::estimateRigidTransform;
using icp_odometry::findCorrespondences;
using icp_odometry::hasConverged;
using icp_odometry::rejectOutliers;
using icp_odometry::runIcp;

static std::vector<Point2D> transformPoints(
  const std::vector<Point2D> & input,
  const Eigen::Isometry2d & transform)
{
  std::vector<Point2D> output;
  output.reserve(input.size());
  for (const auto & point : input) {
    output.push_back(applyTransform(point, transform));
  }
  return output;
}

TEST(Icp2d, IdentityTransform)
{
  std::vector<Point2D> target{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
  const auto pairs = findCorrespondences(target, target);
  const auto transform = estimateRigidTransform(target, target, pairs);
  EXPECT_NEAR(transform.translation().x(), 0.0, 1e-6);
  EXPECT_NEAR(transform.translation().y(), 0.0, 1e-6);
}

TEST(Icp2d, EstimateTranslationDirect)
{
  std::vector<Point2D> source{{0.0, 0.0}, {2.0, 0.0}, {0.0, 1.0}};
  Eigen::Isometry2d expected = Eigen::Isometry2d::Identity();
  expected.translation() = Eigen::Vector2d(0.5, -0.25);
  const auto target = transformPoints(source, expected);
  const auto pairs = findCorrespondences(source, target);
  const auto transform = estimateRigidTransform(source, target, pairs);
  EXPECT_NEAR(transform.translation().x(), 0.5, 1e-5);
  EXPECT_NEAR(transform.translation().y(), -0.25, 1e-5);
}

TEST(Icp2d, KnownTranslation)
{
  std::vector<Point2D> source{{0.0, 0.0}, {2.0, 0.0}, {0.0, 1.0}};
  Eigen::Isometry2d expected = Eigen::Isometry2d::Identity();
  expected.translation() = Eigen::Vector2d(0.5, -0.25);
  const auto target = transformPoints(source, expected);
  IcpParameters params;
  params.minimum_correspondences = 3;
  const auto result = runIcp(source, target, params);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.transform.translation().x(), 0.5, 0.02);
  EXPECT_NEAR(result.transform.translation().y(), -0.25, 0.02);
}

TEST(Icp2d, KnownRotation)
{
  std::vector<Point2D> source{{0.5, 0.0}, {1.0, 0.0}, {0.0, 0.8}};
  Eigen::Isometry2d expected = Eigen::Isometry2d::Identity();
  expected.linear() = Eigen::Rotation2Dd(M_PI / 4.0).matrix();
  const auto target = transformPoints(source, expected);
  IcpParameters params;
  params.minimum_correspondences = 3;
  const auto result = runIcp(source, target, params);
  ASSERT_TRUE(result.valid);
  const double yaw = std::atan2(result.transform.linear()(1, 0), result.transform.linear()(0, 0));
  EXPECT_NEAR(yaw, M_PI / 4.0, 0.05);
}

TEST(Icp2d, InsufficientCorrespondences)
{
  std::vector<Point2D> source{{0.0, 0.0}};
  std::vector<Point2D> target{{5.0, 5.0}, {6.0, 5.0}, {5.0, 6.0}};
  IcpParameters params;
  params.minimum_correspondences = 5;
  const auto result = runIcp(source, target, params);
  EXPECT_FALSE(result.valid);
}

TEST(Icp2d, OutlierRejection)
{
  std::vector<Point2D> source{{0.0, 0.0}, {1.0, 0.0}};
  std::vector<Point2D> target{{0.0, 0.0}, {10.0, 10.0}};
  auto pairs = findCorrespondences(source, target);
  pairs = rejectOutliers(source, target, pairs, 0.5);
  EXPECT_EQ(pairs.size(), 1u);
}

TEST(Icp2d, ConvergenceThreshold)
{
  IcpParameters params;
  params.convergence_translation = 0.1;
  params.convergence_rotation = 0.1;
  Eigen::Isometry2d a = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d b = Eigen::Isometry2d::Identity();
  b.translation() = Eigen::Vector2d(0.01, 0.0);
  EXPECT_TRUE(hasConverged(a, b, params));
}
