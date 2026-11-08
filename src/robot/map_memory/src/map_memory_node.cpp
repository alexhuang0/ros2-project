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

  // publish initial empty map to help planner node
  global_map_.info.resolution = RESOLUTION;
  global_map_.info.width = GRIDSIZE;
  global_map_.info.height = GRIDSIZE;
  global_map_.data.assign(GRIDSIZE * GRIDSIZE, 0);
  // possibly need to set Header and Info?
  map_pub_->publish(global_map_);
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
    orientation_ = msg->pose.pose.orientation;
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

double extractYaw(geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Integrate the latest costmap into the global map
void MapMemoryNode::integrateCostmap() {
  // Transform the latest costmap into the global map
  double yaw{extractYaw(this->orientation_)};

  nav_msgs::msg::OccupancyGrid costmap_world_pov{this->latest_costmap_};

  for (int i{0}, rowCtr{0}; i < costmap_world_pov.data.size(); ++i) {
    if ((rowCtr + 1) % costmap_world_pov.info.height == 0) {
      rowCtr = 0;
    }
    int obstacle_x{i - rowCtr * static_cast<int>(costmap_world_pov.info.width)},
        obstacle_y{(i - obstacle_x) /
                   static_cast<int>(costmap_world_pov.info.width)};

    // translation + rotation matrix
    int world_x{static_cast<int>(std::round(
        lastX_ + std::cos(yaw) * obstacle_x - std::sin(yaw) * obstacle_y))};
    int world_y{static_cast<int>(std::round(
        lastY_ + std::sin(yaw) * obstacle_x + std::cos(yaw) * obstacle_y))};
    if (world_x < 0)
      world_x = std::max(world_x, -GRIDSIZE);
    else {
      world_x = std::min(world_x, GRIDSIZE);
    }

    if (world_y < 0)
      world_y = std::max(world_y, -GRIDSIZE);
    else {
      world_y = std::min(world_y, GRIDSIZE);
    }

    // integrate latest costmap into world map
    int global_idx = world_x + world_y * GRIDSIZE; // row-major
    global_map_.data[global_idx] = latest_costmap_.data[i];
    // if (global_map_.data[global_idx] != -1) {
    // }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
