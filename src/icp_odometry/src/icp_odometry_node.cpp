#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "icp_odometry/icp_2d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&IcpOdometryNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ICP odometry listening on %s, publishing %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto start = std::chrono::steady_clock::now();
    std::vector<float> ranges(scan->ranges.begin(), scan->ranges.end());
    const auto points = laserScanToPoints(
      ranges, scan->angle_min, scan->angle_increment, scan_stride_);

    if (previous_points_.empty()) {
      previous_points_ = points;
      stamp_ = scan->header.stamp;
      return;
    }

    const auto result = runIcp(points, previous_points_, params_);
    if (!result.valid) {
      RCLCPP_WARN(get_logger(), "ICP failed: correspondences=%d", result.correspondences);
      previous_points_ = points;
      return;
    }

    const Eigen::Isometry2d delta = result.transform.inverse();
    const double delta_x = delta.translation().x();
    const double delta_y = delta.translation().y();
    const double delta_yaw = std::atan2(
      delta.linear()(1, 0), delta.linear()(0, 0));

    pose_x_ += delta_x;
    pose_y_ += delta_y;
    pose_yaw_ += delta_yaw;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = scan->header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose_x_;
    odom.pose.pose.position.y = pose_y_;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose_yaw_);
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = delta_x / 0.1;
    odom.twist.twist.angular.z = delta_yaw / 0.1;
    publisher_->publish(odom);

    previous_points_ = points;
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double ms = std::chrono::duration<double, std::milli>(elapsed).count();
    RCLCPP_INFO(
      get_logger(),
      "ICP ok iter=%d corr=%d dt=%.1fms pose=(%.2f, %.2f, %.2f)",
      result.iterations, result.correspondences, ms, pose_x_, pose_y_, pose_yaw_);
  }

  IcpParameters params_;
  int scan_stride_{2};
  std::string base_frame_;
  std::string odom_frame_;
  std::vector<Point2D> previous_points_;
  rclcpp::Time stamp_;
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
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
