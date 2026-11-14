#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control")
    , control_(robot::ControlCore(this->get_logger())) {
  // init parameters
  lookahead_distance_ = 1.0;
  goal_tolerance_ = 0.1;
  linear_speed_ = 0.5;

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        current_path_ = msg;
      });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_odom_ = msg;
      });

  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                           [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // skip control if no path or odemtry data available
  if (!current_path_ || !robot_odom_) {
    return;
  }

  // find lookahead_point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    // TODO? no lookahead maybe cuz theres a new path thats too far from
    // lookahead distance?
    return; // no valid lookahead point found
  }

  // compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // publish velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped>
ControlNode::findLookaheadPoint() {
  // TODO: Implement logic to find the lookahead point on the path
  return std::nullopt; // Replace with a valid point when implemented
}

geometry_msgs::msg::Twist
ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  // TODO: Implement logic to compute velocity commands
  geometry_msgs::msg::Twist cmd_vel;
  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a,
                                    const geometry_msgs::msg::Point &b) {
  // TODO: Implement distance calculation between two points
  return 0.0;
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // TODO: Implement quaternion to yaw conversion
  return 0.0;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
