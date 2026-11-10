#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // publisher
  path_pub_ = this->create_publisher<nav_msgs:msg::Path>("/path", 10);

  // timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

}

/*
A* notes
G-cost: distance from origin
H-cost: distance from goal
F-cost: g + h

at every step, explore path with smallest F-cost. once that path is explored, it is blocked and marked as such
- if >1 path have same F-cost, pick the one with the smallest H-cost
- same h-cost, explore either one

we are exploring smallest F-cost across ALL THE possible options we've computed. 
possibly popping off of a min heap or smt?

5min: options we explore also update past options since we might have discovered a new shortest path
so each node also stores its origin (since diagonal is not same cost as horiz/vert)
kind of a linked list so we can retrace path
*/


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
