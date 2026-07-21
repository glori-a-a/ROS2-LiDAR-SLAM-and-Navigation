#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "icp_odometry/icp_2d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace icp_odometry
{

class IcpOdometryNode : public rclcpp::Node
{
public:
  IcpOdometryNode()
  : Node("icp_odometry")
  {
    declare_parameter("input_topic", "/scan");
    declare_parameter("output_topic", "/icp_odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("odom_frame", "odom");
    declare_parameter("max_iterations", 20);
    declare_parameter("correspondence_distance", 0.5);
    declare_parameter("convergence_translation", 1e-3);
    declare_parameter("convergence_rotation", 1e-3);
    declare_parameter("minimum_correspondences", 20);
    declare_parameter("scan_stride", 2);
    declare_parameter("pose_covariance_xy", 0.05);
    declare_parameter("pose_covariance_yaw", 0.02);
    declare_parameter("twist_covariance_vx", 0.02);
    declare_parameter("twist_covariance_wz", 0.01);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    params_.max_iterations = get_parameter("max_iterations").as_int();
    params_.correspondence_distance = get_parameter("correspondence_distance").as_double();
    params_.convergence_translation = get_parameter("convergence_translation").as_double();
    params_.convergence_rotation = get_parameter("convergence_rotation").as_double();
    params_.minimum_correspondences = get_parameter("minimum_correspondences").as_int();
    scan_stride_ = get_parameter("scan_stride").as_int();
    pose_cov_xy_ = get_parameter("pose_covariance_xy").as_double();
    pose_cov_yaw_ = get_parameter("pose_covariance_yaw").as_double();
    twist_cov_vx_ = get_parameter("twist_covariance_vx").as_double();
    twist_cov_wz_ = get_parameter("twist_covariance_wz").as_double();

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&IcpOdometryNode::scanCallback, this, std::placeholders::_1));
  }

private:
  static void fillPoseCovariance(nav_msgs::msg::Odometry & odom, double xy, double yaw)
  {
    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0] = xy;
    odom.pose.covariance[7] = xy;
    odom.pose.covariance[35] = yaw;
  }

  static void fillTwistCovariance(nav_msgs::msg::Odometry & odom, double vx, double wz)
  {
    odom.twist.covariance.fill(0.0);
    odom.twist.covariance[0] = vx;
    odom.twist.covariance[35] = wz;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    std::vector<float> ranges(scan->ranges.begin(), scan->ranges.end());
    const auto points = laserScanToPoints(
      ranges, scan->angle_min, scan->angle_increment, scan_stride_);

    if (previous_points_.empty()) {
      previous_points_ = points;
      last_stamp_ = rclcpp::Time(scan->header.stamp);
      global_pose_ = Eigen::Isometry2d::Identity();
      return;
    }

    const auto result = runIcp(points, previous_points_, params_);
    previous_points_ = points;
    if (!result.valid) {
      RCLCPP_WARN(get_logger(), "ICP failed: correspondences=%d", result.correspondences);
      return;
    }

    const Eigen::Isometry2d previous_pose = global_pose_;
    global_pose_ = composePoses(global_pose_, result.transform);

    const rclcpp::Time current_stamp(scan->header.stamp);
    const double dt = (current_stamp - last_stamp_).seconds();
    last_stamp_ = current_stamp;

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    xyYawFromIsometry(global_pose_, x, y, yaw);
    yaw = normalizeYaw(yaw);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = scan->header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);
    fillPoseCovariance(odom, pose_cov_xy_, pose_cov_yaw_);

    if (dt > 1e-4) {
      double prev_x = 0.0;
      double prev_y = 0.0;
      double prev_yaw = 0.0;
      xyYawFromIsometry(previous_pose, prev_x, prev_y, prev_yaw);
      const double dx = x - prev_x;
      const double dy = y - prev_y;
      const double vx_body =
        (std::cos(-prev_yaw) * dx - std::sin(-prev_yaw) * dy) / dt;
      const double delta_yaw = normalizeYaw(yaw - prev_yaw);
      odom.twist.twist.linear.x = vx_body;
      odom.twist.twist.angular.z = delta_yaw / dt;
    }
    fillTwistCovariance(odom, twist_cov_vx_, twist_cov_wz_);
    publisher_->publish(odom);
  }

  IcpParameters params_;
  int scan_stride_{2};
  std::string base_frame_;
  std::string odom_frame_;
  double pose_cov_xy_{0.05};
  double pose_cov_yaw_{0.02};
  double twist_cov_vx_{0.02};
  double twist_cov_wz_{0.01};
  std::vector<Point2D> previous_points_;
  rclcpp::Time last_stamp_;
  Eigen::Isometry2d global_pose_{Eigen::Isometry2d::Identity()};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace icp_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<icp_odometry::IcpOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
