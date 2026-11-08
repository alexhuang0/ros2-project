#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

  static constexpr double RESOLUTION{0.1}; // in m/cell
  static constexpr int SIZE{30};           // in meters (width and height)
  static constexpr int GRIDSIZE{static_cast<int>(
      SIZE / RESOLUTION)}; // in cells, determines gsize x gsize grid

private:
  robot::MapMemoryCore map_memory_;

  // subscribers and publisher
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // global map and robot position
  nav_msgs::msg::OccupancyGrid global_map_;
  double lastX_, lastY_;
  geometry_msgs::msg::Quaternion orientation_;
  const double distance_threshold_;
  bool costmap_updated_ = false;

  // callback for costmap updates
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void updateMap();

  // integrate latest costmap into the global map
  void integrateCostmap();

  // flags
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  bool should_update_map_ = false;
};

#endif
