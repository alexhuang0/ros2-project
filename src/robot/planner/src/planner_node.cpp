#include "planner_node.hpp"

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
                  "Replanning due to timeout or progress stall...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx{goal_.point.x - robot_pose_.position.x};
  double dy{goal_.point.y - robot_pose_.position.y};

  // distance from goal is less than sqr(0.7) ~= 0.5
  return dx * dx - dy * dy <= 0.7;
}

void calc_g_score(AStarNode &current, AStarNode &neighbour) {

  // if (current.index.x == neighbour.index.x
  // || current.index.y == neighbour.index.y) {
  //   neighbour.g_score +=
  // }
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
      if (new_x < 0 || new_x >= current_map_.info.width || new_y < 0 ||
          new_y >= current_map_.info.width) {
        continue;
      }

      int map_idx{new_x + (new_y * current_map_.info.width)};

      // is an obstacle
      if (current_map_.data[map_idx] > 0) {
        continue;
      }

      AStarNode neighbour{};

      // we have a valid neighbour now
      if (dx != 0 && dy != 0) {
        neighbour.g_score += SQRT2;
      } else {
        neighbour.g_score += 1;
      }

      CellIndex cIdx{new_x, new_y};

      neighbours.emplace_back(cIdx, 0);
    }
  }
}

double PlannerNode::calc_f_cost(AStarNode &current) {
  double dx{goal_.point.x - current.index.x},
      dy{goal_.point.y - current.index.y};
  return std::sqrt(dx * dx + dy * dy);
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

  /*
A* notes
G-cost: distance from origin
H-cost: distance from goal
F-cost: g + h

at every step, explore path with smallest F-cost. once that path is explored, it
is blocked and marked as such
- if >1 path have same F-cost, pick the one with the smallest H-cost
- same h-cost, explore either one

we are exploring smallest F-cost across ALL THE possible options we've computed.
possibly popping off of a min heap or smt?

5min: options we explore also update past options since we might have discovered
a new shortest path so each node also stores its origin (since diagonal is not
same cost as horiz/vert) kind of a linked list so we can retrace path
*/

  // requires more std::unordered_map<CellIndex, int, CellIndexHash> visitCount;
  std::unordered_set<AStarNode> closed; // set of nodes already evaluated
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::vector<std::vector<CellIndex>>
      came_from_map; // each cell index holds its parent cell index in the path
  std::vector<std::vector<double>> g_score_map; // g_scores

  int robot_x_grid{static_cast<int>(
      std::round(robot_pose_.position.x / current_map_.info.resolution))};
  int robot_y_grid{static_cast<int>(
      std::round(robot_pose_.position.y / current_map_.info.resolution))};

  CellIndex start(robot_x_grid, robot_y_grid);
  open.push(
      {start, 0}); // initialized w/ 0 f-cost cuz it doesn't matter for 1st node

  while (!open.empty()) {
    AStarNode cur = open.top();
    open.pop();
    closed.insert(cur);

    if (goalReached()) {
      // backtracking
      // fill path.poses with path waypoints
    }

    for (AStarNode neighbour : getNeighbours(cur)) {
      if (closed.find(neighbour) != closed.end() || isObstacle(neighbour)) {
        continue;
      }

      // neighbour isn't in OPEN (haven't calculated f_cost yet) or new path to
      // neighbour is shorter
      if (open.find(neighbour) != open.end() ||) {
        neighbour.f_score = calc_f_cost(neighbour);
        came_from_map[neighbour.index.y][neighbour.index.x] = cur.index;

        if (open.find(neighbour) != open.end()) {
          open.emplace(neighbour);
        }
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
