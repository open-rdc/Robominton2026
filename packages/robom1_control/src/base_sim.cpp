#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "robom1_control/kinematics.hpp"

class BaseSim : public rclcpp::Node
{
public:
  BaseSim() : Node("base_sim")
  {
    wheelbase_ = declare_parameter("wheelbase", 0.52);
    track_ = declare_parameter("track", 0.48);
    radius_ = declare_parameter("wheel_radius", 0.09);
    max_wheel_speed_ = declare_parameter("max_wheel_speed", 40.0);
    max_speed_ = declare_parameter("max_speed", 1.5);
    max_yaw_ = declare_parameter("max_yaw_rate", 1.5);
    acceleration_ = declare_parameter("acceleration", 2.0);
    yaw_acceleration_ = declare_parameter("yaw_acceleration", 3.0);
    timeout_ = declare_parameter("command_timeout", 0.3);
    x_ = declare_parameter("initial_x", 1.0);
    y_ = declare_parameter("initial_y", 0.0);
    const double rate = declare_parameter("update_rate", 100.0);
    if (!(wheelbase_ > 0 && track_ > 0 && radius_ > 0 && max_speed_ > 0 &&
      max_yaw_ > 0 && max_wheel_speed_ > 0 && acceleration_ > 0 && yaw_acceleration_ > 0 && timeout_ > 0 && rate > 0))
    {throw std::invalid_argument("Geometry, rates, and limits must be positive");}
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
          !std::isfinite(msg->angular.z)) {return;}
        target_ = *msg;
        const double speed = std::hypot(target_.linear.x, target_.linear.y);
        if (speed > max_speed_) {
          target_.linear.x *= max_speed_ / speed;
          target_.linear.y *= max_speed_ / speed;
        }
        target_.angular.z = std::clamp(target_.angular.z, -max_yaw_, max_yaw_);
        last_command_ = now();
      });
    last_ = last_command_ = now();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
      std::bind(&BaseSim::step, this));
  }
private:
  void step()
  {
    const auto stamp = now();
    const double dt = (stamp - last_).seconds();
    last_ = stamp;
    if (dt <= 0 || dt > 0.5) {return;}
    if ((stamp - last_command_).seconds() > timeout_) {target_ = geometry_msgs::msg::Twist();}
    const double dx = target_.linear.x - velocity_.linear.x;
    const double dy = target_.linear.y - velocity_.linear.y;
    const double delta = std::hypot(dx, dy);
    const double scale = delta > 0 ? std::min(1.0, acceleration_ * dt / delta) : 0.0;
    velocity_.linear.x += dx * scale;
    velocity_.linear.y += dy * scale;
    velocity_.angular.z += std::clamp(target_.angular.z - velocity_.angular.z,
      -yaw_acceleration_ * dt, yaw_acceleration_ * dt);
    const double drive_scale = robom1_control::drive_scale(velocity_.linear.x, velocity_.linear.y,
      velocity_.angular.z, wheelbase_, track_, radius_, max_wheel_speed_);
    velocity_.linear.x *= drive_scale;
    velocity_.linear.y *= drive_scale;
    velocity_.angular.z *= drive_scale;
    robom1_control::integrate(x_, y_, yaw_, velocity_.linear.x, velocity_.linear.y,
      velocity_.angular.z, dt);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(yaw_ / 2);
    odom.pose.pose.orientation.w = std::cos(yaw_ / 2);
    odom.twist.twist = velocity_;
    odom_pub_->publish(odom);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_->sendTransform(transform);
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    const std::array<std::string, 4> names = {"front_left", "front_right", "rear_left", "rear_right"};
    for (size_t i = 0; i < names.size(); ++i) {
      const double wx = (i < 2 ? 1 : -1) * wheelbase_ / 2;
      const double wy = (i % 2 == 0 ? 1 : -1) * track_ / 2;
      const auto wheel = robom1_control::inverse(velocity_.linear.x, velocity_.linear.y,
        velocity_.angular.z, wx, wy, radius_, steering_[i]);
      steering_[i] = wheel.steering;
      rotation_[i] = robom1_control::wrap(rotation_[i] + wheel.velocity * dt);
      joints.name.push_back(names[i] + "_steer_joint");
      joints.position.push_back(steering_[i]);
      joints.name.push_back(names[i] + "_wheel_joint");
      joints.position.push_back(rotation_[i]);
    }
    joints_pub_->publish(joints);
  }
  double wheelbase_, track_, radius_, max_wheel_speed_, max_speed_, max_yaw_, acceleration_, yaw_acceleration_, timeout_;
  double x_, y_, yaw_{0};
  std::array<double, 4> steering_{}, rotation_{};
  geometry_msgs::msg::Twist target_, velocity_;
  rclcpp::Time last_, last_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseSim>());
  rclcpp::shutdown();
  return 0;
}
