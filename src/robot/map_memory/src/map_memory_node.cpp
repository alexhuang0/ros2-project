#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory")
    , map_memory_(robot::MapMemoryCore(this->get_logger()))
    , lastX_(0.0)
    , lastY_(0.0)
    , currentX_(0.0)
    , currentY_(0.0)
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
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = RESOLUTION;
  global_map_.info.width = GRIDSIZE;
  global_map_.info.height = GRIDSIZE;

  global_map_.info.origin.position.x = -SIZE / 2;
  global_map_.info.origin.position.y = -SIZE / 2;

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
  currentX_ = msg->pose.pose.position.x;
  currentY_ = msg->pose.pose.position.y;
  orientation_ = msg->pose.pose.orientation;

  // Compute distance traveled
  double distance = std::sqrt((currentX_ - lastX_) * (currentX_ - lastX_) +
                              (currentY_ - lastY_) * (currentY_ - lastY_));
  if (distance >= distance_threshold_) {
    lastX_ = currentX_;
    lastY_ = currentY_;
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

  // nav_msgs::msg::OccupancyGrid costmap_world_pov{this->latest_costmap_};
  // int nObstacle = 0, nNonZero = 0, maxVal = 0;
  // for (int8_t v : latest_costmap_.data) {
  //   if (v == 100)
  //     nObstacle++;
  //   if (v != 0)
  //     nNonZero++;
  //   maxVal = std::max(maxVal, static_cast<int>(v));
  // }
  // RCLCPP_INFO(this->get_logger(), "new costmap: obstacles=%d nonzero=%d
  // max=%d",
  //             nObstacle, nNonZero, maxVal);

  for (int i{0}, rowCtr{-1}; i < latest_costmap_.data.size(); ++i) {
    if (i % latest_costmap_.info.width == 0) {
      rowCtr += 1;
    }
    // obstacle coords in ROBOT'S frame (initially they are in costmap frame,
    // with origin at (-20m, -20m from robot), aka (-200 cells, -200 cells)).
    // These coords are in cells
    int obstacle_x{i - rowCtr * static_cast<int>(latest_costmap_.info.width)},
        obstacle_y{rowCtr};
    // shift coords to have robot at origin
    obstacle_x -= static_cast<int>(latest_costmap_.info.width) / 2;
    obstacle_y -= static_cast<int>(latest_costmap_.info.height) / 2;

    // translation + rotation matrix
    // coords are in cells
    // origin is center of map
    int world_x{static_cast<int>(std::round(currentX_ / RESOLUTION +
                                            std::cos(yaw) * obstacle_x -
                                            std::sin(yaw) * obstacle_y))};
    int world_y{static_cast<int>(std::round(currentY_ / RESOLUTION +
                                            std::sin(yaw) * obstacle_x +
                                            std::cos(yaw) * obstacle_y))};

    // world coords but with origin at (-15m, -15m), so (-150 cells, -150 cells)
    int world_x_cells{world_x + GRIDSIZE / 2};
    int world_y_cells{world_y + GRIDSIZE / 2};

    // integrate latest costmap into world map
    if (world_x_cells >= 0 && world_x_cells < GRIDSIZE && world_y_cells >= 0 &&
        world_y_cells < GRIDSIZE) {
      int global_idx = world_x_cells + world_y_cells * GRIDSIZE;
      if (latest_costmap_.data[i] != 0) {
        global_map_.data[global_idx] =
            std::max(latest_costmap_.data[i], global_map_.data[global_idx]);
      }
    }
  }

  // int obstacles = 0, nonzero = 0, maxVal2 = 0;
  // for (int8_t v : global_map_.data) {
  //   if (v == 100)
  //     obstacles++;
  //   if (v != 0)
  //     nonzero++;
  //   maxVal2 = std::max(maxVal2, static_cast<int>(v));
  // }
  // RCLCPP_INFO(this->get_logger(),
  //             "integrated costmap: obstacles=%d nonzero=%d max=%d",
  //             obstacles, nonzero, maxVal2);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
