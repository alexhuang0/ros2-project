#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include "costmap_core.hpp"

#include <array>

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

  static constexpr double RESOLUTION{0.1}; // in meters
  static constexpr int SIZE{40};           // in meters (width and height)
  static constexpr int GRIDSIZE{static_cast<int>(
      SIZE * RESOLUTION * 100)}; // in cells, determines gsize x gsize grid
  std::array<std::array<int, GRIDSIZE>, GRIDSIZE> occupancyGrid{};

  void initializeCostmap();

  void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
  void markObstacle(int x_grid, int y_grid);
  void inflateObstacles();
  void publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:
  robot::CostmapCore costmap_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publish_;
};

#endif