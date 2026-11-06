#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory")
    , map_memory_(robot::MapMemoryCore(this->get_logger()))
    , lastX_(0.0)
    , lastY_(0.0)
    , distance_threshold_(1.5) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&MapMemoryNode::updateMap, this));
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store the latest costmap
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // Compute distance traveled
  double distance =
      std::sqrt((x - lastX_) * (x - lastX_) + (y - lastY_) * (y - lastY_));
  if (distance >= distance_threshold_) {
    lastX_ = x;
    lastY_ = y;
    should_update_map_ = true;
  }
}

// Timer-based map update
void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
  }
}

// Integrate the latest costmap into the global map
void MapMemoryNode::integrateCostmap() {
  // Transform the latest costmap into the global map

  // integrate latest costmap into global map
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
