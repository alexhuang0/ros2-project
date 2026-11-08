#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap")
    , costmap_(robot::CostmapCore(this->get_logger())) {
  using std::placeholders::_1;

  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 5, std::bind(&CostmapNode::lidar_callback, this, _1));
  publish_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidar_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // RCLCPP_INFO(this->get_logger(), "Received a scan: %zu scan numbers",
  //             scan->ranges.size());

  // Step 1: Initialize costmap
  initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      // Calculate grid coordinates
      int x_grid{0}, y_grid{0};
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  // Step 3: Inflate obstacles
  inflateObstacles();

  // Step 4: Publish costmap
  publishCostmap(scan);
}

void CostmapNode::initializeCostmap() {
  for (auto &row : occupancyGrid) {
    row.fill(0); // Fills each inner array with 0
  }
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid,
                                int &y_grid) {
  x_grid = std::round(range * std::cos(angle) / RESOLUTION + GRIDSIZE / 2);
  y_grid = std::round(range * std::sin(angle) / RESOLUTION + GRIDSIZE / 2);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {

  if (x_grid >= 0 && x_grid < CostmapNode::GRIDSIZE && y_grid >= 0 &&
      y_grid < CostmapNode::GRIDSIZE) {
    // can simply store into y_grid even if array's "y-axis" grows from top to
    // bottom. This is bc we tell Foxglove that the costmap's origin sits at
    // (-20, -20) from robot so when Foxglove reads the data array, when y
    // increases, it is increasing from BOTTOM to top, in RESOLUTION steps
    // (0.1m/cell)
    this->occupancyGrid[y_grid][x_grid] = 100;
  }
}

void CostmapNode::inflateObstacles() {
  constexpr int max_cost{100};
  constexpr double inflation_radius{1.0 / CostmapNode::RESOLUTION};

  for (int i{0}; i < CostmapNode::GRIDSIZE; ++i) {
    for (int j{0}; j < CostmapNode::GRIDSIZE; ++j) {
      if (this->occupancyGrid[i][j] != 100)
        continue;

      // bounding box
      int xMin = std::max(0, static_cast<int>(j - inflation_radius));
      int xMax = std::min(CostmapNode::GRIDSIZE - 1,
                          static_cast<int>(j + inflation_radius));
      int yMin = std::max(0, static_cast<int>(i - inflation_radius));
      int yMax = std::min(CostmapNode::GRIDSIZE - 1,
                          static_cast<int>(i + inflation_radius));

      for (int x{xMin}; x <= xMax; ++x) {
        for (int y{yMin}; y <= yMax; ++y) {
          double distance = std::sqrt((x - j) * (x - j) + (y - i) * (y - i));

          // if distance from (i, j) > inflation_radius, disregard
          if (distance > inflation_radius)
            continue;

          int cost = std::round(max_cost * (1 - (distance / inflation_radius)));

          if (cost > this->occupancyGrid[y][x]) {
            this->occupancyGrid[y][x] = cost;
          }
        }
      }
    }
  }
}

template <size_t ROWS, size_t COLS>
void flatten(std::array<std::array<int, COLS>, ROWS> &arr,
             std::vector<int8_t> &flattened) {
  for (size_t i{0}; i < ROWS; ++i) {
    for (size_t j{0}; j < COLS; ++j) {
      // flattened[j + i * COLS] = arr[i][j];
      flattened.push_back(arr[i][j]);
    }
  }
}

void CostmapNode::publishCostmap(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  auto occGridMsg = nav_msgs::msg::OccupancyGrid();

  // HEADER
  occGridMsg.header = scan->header;

  // INFO
  auto info = nav_msgs::msg::MapMetaData();
  info.resolution = CostmapNode::RESOLUTION;
  info.width = CostmapNode::GRIDSIZE;
  info.height = CostmapNode::GRIDSIZE;

  // origin (where is bottom left corner based on robot, in meters). i.e. where
  // is the world's (0,0) origin is publised in robot's frame, so this is a
  // CONSTANT offset
  auto origin = geometry_msgs::msg::Pose();
  origin.position.x = -SIZE / 2;
  origin.position.y = -SIZE / 2;
  origin.position.z = 0;
  origin.orientation.x = 0;
  origin.orientation.y = 0;
  origin.orientation.z = 0;
  origin.orientation.w = 1;
  info.origin = origin;

  occGridMsg.info = info;

  // DATA
  std::vector<int8_t> flattened{};
  flatten(this->occupancyGrid, flattened);

  occGridMsg.data = flattened;

  // int nObstacle = 0, nNonZero = 0, maxVal = 0;
  // for (auto &row : occupancyGrid)
  //   for (int v : row) {
  //     if (v == 100)
  //       nObstacle++;
  //     if (v != 0)
  //       nNonZero++;
  //     maxVal = std::max(maxVal, v);
  //   }
  // RCLCPP_INFO(this->get_logger(), "obstacles=%d nonzero=%d max=%d",
  // nObstacle,
  //             nNonZero, maxVal);

  publish_->publish(occGridMsg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}