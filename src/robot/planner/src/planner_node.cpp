#include "planner_node.hpp"

#include <limits>
#include <numbers>
#include <queue>
#include <unordered_set>
#include <vector>

PlannerNode::PlannerNode()
    : Node("planner")
    , planner_(robot::PlannerCore(this->get_logger())) {
  // subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // timer
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (this->state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx{goal_.point.x - robot_pose_.position.x};
  double dy{goal_.point.y - robot_pose_.position.y};

  // distance from goal is less than sqr(0.7) ~= 0.5
  return dx * dx + dy * dy <= 0.7;
}

std::vector<AStarNode> PlannerNode::getNeighbours(AStarNode &current) {
  std::vector<AStarNode> neighbours{};

  constexpr double SQRT2{1.414213562373095048801688724209698079};

  for (int dx{-1}; dx <= 1; ++dx) {
    for (int dy{-1}; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0)
        continue;

      int new_x{current.index.x + dx};
      int new_y{current.index.y + dy};

      // out of bounds
      int mapSize{static_cast<int>(current_map_.info.width)};
      if (new_x < 0 || new_x >= mapSize || new_y < 0 || new_y >= mapSize) {
        continue;
      }

      int map_idx{new_x + (new_y * mapSize)};

      // is an obstacle
      if (current_map_.data[map_idx] > 0) {
        continue;
      }

      AStarNode neighbour{};

      // we have a valid neighbour now
      if (dx != 0 && dy != 0) {
        neighbour.g_score_delta += SQRT2;
      } else {
        neighbour.g_score_delta += 1;
      }

      CellIndex cIdx{new_x, new_y};

      neighbours.emplace_back(cIdx, 0);
    }
  }

  return neighbours;
}

double PlannerNode::calc_h_cost(AStarNode &current) {
  int dx{worldToGrid(goal_.point.x) - current.index.x},
      dy{worldToGrid(goal_.point.y) - current.index.y};
  return std::sqrt(dx * dx + dy * dy);
}

void PlannerNode::setPath(nav_msgs::msg::Path &path,
                          std::vector<std::vector<CellIndex>> &came_from_map,
                          CellIndex &end, CellIndex &robot_cell) {
  CellIndex cur{end};

  std::vector<geometry_msgs::msg::PoseStamped> path_poses;

  while (cur != robot_cell) {
    double world_x{gridToWorld(cur.x)};
    double world_y{gridToWorld(cur.y)};

    geometry_msgs::msg::PoseStamped pose_stamped;

    pose_stamped.header.set__frame_id("map");
    pose_stamped.pose.position.set__x(world_x);
    pose_stamped.pose.position.set__y(world_y);
    path_poses.emplace_back(pose_stamped);

    cur = came_from_map[cur.y][cur.x];
  }

  std::reverse(path_poses.begin(), path_poses.end());
  path.set__poses(path_poses);
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_INFO(this->get_logger(), "Can't plan a path: missing map or goal");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  // A* implementation

  std::unordered_set<CellIndex, CellIndexHash>
      closed; // set of nodes already evaluated
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;

  // each cell index holds its parent cell index in the path
  std::vector<std::vector<CellIndex>> came_from_map(
      current_map_.info.height,
      std::vector<CellIndex>(current_map_.info.width, CellIndex(-1, -1)));

  std::vector<std::vector<double>> g_score_map(
      current_map_.info.height,
      std::vector<double>(current_map_.info.width,
                          std::numeric_limits<double>::max())); // g_scores

  int robot_x_grid{worldToGrid(robot_pose_.position.x)};
  int robot_y_grid{worldToGrid(robot_pose_.position.y)};

  CellIndex start(robot_x_grid, robot_y_grid);
  open.push(
      {start, 0}); // initialized w/ 0 f-cost cuz it doesn't matter for 1st node
  g_score_map[robot_y_grid][robot_x_grid] = 0;

  while (!open.empty()) {
    AStarNode cur = open.top();
    open.pop();
    closed.insert(cur.index);

    if (goalReached()) {
      setPath(path, came_from_map, cur.index, start);
    }

    for (AStarNode neighbour : getNeighbours(cur)) {
      if (closed.find(neighbour.index) != closed.end()) {
        continue;
      }

      double neighbour_g_score{g_score_map[cur.index.y][cur.index.x] +
                               neighbour.g_score_delta};

      // new path to neighbour is shorter
      if (neighbour_g_score <
          g_score_map[neighbour.index.y][neighbour.index.x]) {

        // cost of the path ALREADY TRAVELLED to this node (known, exact)
        g_score_map[neighbour.index.y][neighbour.index.x] = neighbour_g_score;

        // estimated euclidean distance to goal (guessed, heuristic)
        double h_score = calc_h_cost(neighbour);
        neighbour.f_score = neighbour_g_score + h_score;

        came_from_map[neighbour.index.y][neighbour.index.x] = cur.index;

        open.emplace(neighbour);
      }
    }
  }

  path_pub_->publish(path);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
