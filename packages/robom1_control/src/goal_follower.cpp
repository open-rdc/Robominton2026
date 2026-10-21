#include <algorithm>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Proportional point follower, not an obstacle-avoiding path planner.
class GoalFollower : public rclcpp::Node
{
public:
  GoalFollower() : Node("goal_follower")
  {
    gain_ = declare_parameter("gain", 1.2);
    speed_ = declare_parameter("max_speed", 1.0);
    demo_ = declare_parameter("demo", true);
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_ = *msg; have_odom_ = true; odom_received_ = now();
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (!demo_ && msg->header.frame_id == "odom") {set_goal(msg->pose.position);}
      });
    shuttle_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("shuttle/ground_truth", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        if (demo_ && msg->header.frame_id == "odom") {
          auto point = msg->point;
          point.x = std::clamp(point.x - 0.7, 0.5, 5.7);
          point.y = std::clamp(point.y, -2.5, 2.5);
          set_goal(point);
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {step();});
  }
private:
  void set_goal(const geometry_msgs::msg::Point & point)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {return;}
    goal_ = point; have_goal_ = true; goal_received_ = now();
  }
  void step()
  {
    geometry_msgs::msg::Twist cmd;
    if (have_odom_ && have_goal_ && (now() - odom_received_).seconds() < 0.5 &&
      (!demo_ || (now() - goal_received_).seconds() < 0.5))
    {
      const double dx = goal_.x - odom_.pose.pose.position.x;
      const double dy = goal_.y - odom_.pose.pose.position.y;
      const auto & q = odom_.pose.pose.orientation;
      const double yaw = std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
      const double distance = std::hypot(dx, dy);
      if (distance > 0.05) {
        const double scale = std::min(gain_, speed_ / distance);
        cmd.linear.x = scale * (std::cos(yaw) * dx + std::sin(yaw) * dy);
        cmd.linear.y = scale * (-std::sin(yaw) * dx + std::cos(yaw) * dy);
      }
    }
    pub_->publish(cmd);
  }
  double gain_, speed_;
  bool demo_, have_odom_{false}, have_goal_{false};
  rclcpp::Time odom_received_{0}, goal_received_{0};
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::Point goal_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr shuttle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalFollower>());
  rclcpp::shutdown();
  return 0;
}
