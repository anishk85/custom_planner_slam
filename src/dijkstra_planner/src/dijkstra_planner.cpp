#include "dijkstra_planner/dijkstra_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

namespace dijkstra_planner
{

void DijkstraPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  name_ = name;
  
  // Get parameters (compatible with ROS 2 Humble+)
  if (!node->has_parameter(name_ + ".tolerance")) {
    node->declare_parameter(name_ + ".tolerance", rclcpp::ParameterValue(0.5));
  }
  if (!node->has_parameter(name_ + ".use_astar")) {
    node->declare_parameter(name_ + ".use_astar", rclcpp::ParameterValue(false));
  }
    
  node->get_parameter(name_ + ".tolerance", tolerance_);
  node->get_parameter(name_ + ".use_astar", use_astar_);
  
  initialized_ = true;
  
  RCLCPP_INFO(logger_, "Dijkstra planner configured with tolerance: %.2f", tolerance_);
}

void DijkstraPlanner::cleanup()
{
  initialized_ = false;
  RCLCPP_INFO(logger_, "Cleaning up Dijkstra planner");
}

void DijkstraPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating Dijkstra planner");
}

void DijkstraPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating Dijkstra planner");
}

nav_msgs::msg::Path DijkstraPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = start.header.frame_id;
  path.header.stamp = start.header.stamp;

  if (!initialized_) {
    RCLCPP_ERROR(logger_, "Planner not initialized");
    return path;
  }

  // Convert world coordinates to map coordinates
  int start_x, start_y, goal_x, goal_y;
  worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
  worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);

  // Validate start and goal
  if (!isValid(start_x, start_y) || !isValid(goal_x, goal_y)) {
    RCLCPP_ERROR(logger_, "Start or goal position is invalid");
    return path;
  }

  RCLCPP_INFO(logger_, "Planning from (%d,%d) to (%d,%d)", start_x, start_y, goal_x, goal_y);

  // Dijkstra's Algorithm Implementation
  int width = costmap_->getSizeInCellsX();
  int height = costmap_->getSizeInCellsY();
  
  // Distance matrix - initialize with infinity
  std::vector<std::vector<double>> dist(width, std::vector<double>(height, std::numeric_limits<double>::infinity()));
  std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
  std::vector<std::vector<std::pair<int, int>>> parent(width, std::vector<std::pair<int, int>>(height, {-1, -1}));

  // Priority queue for Dijkstra's algorithm
  std::priority_queue<Node, std::vector<Node>, NodeComparator> pq;

  // Initialize start node
  dist[start_x][start_y] = 0.0;
  pq.push(Node(start_x, start_y, 0.0));

  // Dijkstra's main loop
  while (!pq.empty()) {
    Node current = pq.top();
    pq.pop();

    // Skip if already visited
    if (visited[current.x][current.y]) {
      continue;
    }

    visited[current.x][current.y] = true;

    // Check if we reached the goal
    if (current.x == goal_x && current.y == goal_y) {
      RCLCPP_INFO(logger_, "Path found! Total cost: %.2f", current.cost);
      break;
    }

    // Explore neighbors
    std::vector<Node> neighbors = getNeighbors(current);
    for (const auto& neighbor : neighbors) {
      if (!isValid(neighbor.x, neighbor.y) || visited[neighbor.x][neighbor.y]) {
        continue;
      }

      double edge_cost = getCost(neighbor.x, neighbor.y);
      
      // Skip if obstacle (high cost)
      if (edge_cost >= 254) {
        continue;
      }

      // Calculate distance (Euclidean distance between cells)
      double dx = neighbor.x - current.x;
      double dy = neighbor.y - current.y;
      double movement_cost = std::sqrt(dx*dx + dy*dy);
      
      double new_cost = dist[current.x][current.y] + movement_cost + edge_cost * 0.01;

      if (new_cost < dist[neighbor.x][neighbor.y]) {
        dist[neighbor.x][neighbor.y] = new_cost;
        parent[neighbor.x][neighbor.y] = {current.x, current.y};
        
        // Add heuristic for A* variant (optional)
        double priority = new_cost;
        if (use_astar_) {
          double heuristic = std::sqrt(
            std::pow(neighbor.x - goal_x, 2) + 
            std::pow(neighbor.y - goal_y, 2)
          );
          priority += heuristic;
        }
        
        pq.push(Node(neighbor.x, neighbor.y, priority));
      }
    }
  }

  // Reconstruct path
  if (dist[goal_x][goal_y] == std::numeric_limits<double>::infinity()) {
    RCLCPP_ERROR(logger_, "No path found to goal");
    return path;
  }

  // Backtrack to create path
  std::vector<std::pair<int, int>> map_path;
  int current_x = goal_x, current_y = goal_y;
  
  while (current_x != -1 && current_y != -1) {
    map_path.push_back({current_x, current_y});
    auto p = parent[current_x][current_y];
    current_x = p.first;
    current_y = p.second;
  }

  std::reverse(map_path.begin(), map_path.end());

  // Convert back to world coordinates and create ROS path
  for (const auto& point : map_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = start.header.frame_id;
    pose.header.stamp = start.header.stamp;
    
    mapToWorld(point.first, point.second, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    
    path.poses.push_back(pose);
  }

  RCLCPP_INFO(logger_, "Generated path with %zu points", path.poses.size());
  return path;
}

bool DijkstraPlanner::isValid(int x, int y)
{
  return x >= 0 && x < static_cast<int>(costmap_->getSizeInCellsX()) &&
         y >= 0 && y < static_cast<int>(costmap_->getSizeInCellsY());
}

double DijkstraPlanner::getCost(int x, int y)
{
  if (!isValid(x, y)) {
    return 255.0;  // Invalid cell
  }
  return static_cast<double>(costmap_->getCost(x, y));
}

std::vector<DijkstraPlanner::Node> DijkstraPlanner::getNeighbors(const Node& current)
{
  std::vector<Node> neighbors;
  
  // 8-connected neighborhood
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;  // Skip current cell
      
      int new_x = current.x + dx;
      int new_y = current.y + dy;
      
      if (isValid(new_x, new_y)) {
        neighbors.push_back(Node(new_x, new_y, 0.0));
      }
    }
  }
  
  return neighbors;
}

void DijkstraPlanner::worldToMap(double wx, double wy, int& mx, int& my)
{
  unsigned int umx, umy;
  if (!costmap_->worldToMap(wx, wy, umx, umy)) {
    RCLCPP_WARN(logger_, "Failed to convert world coordinates to map coordinates");
    mx = -1;
    my = -1;
  } else {
    mx = static_cast<int>(umx);
    my = static_cast<int>(umy);
  }
}

void DijkstraPlanner::mapToWorld(int mx, int my, double& wx, double& wy)
{
  costmap_->mapToWorld(mx, my, wx, wy);
}

}  // namespace dijkstra_planner

// Register the planner as a plugin
PLUGINLIB_EXPORT_CLASS(dijkstra_planner::DijkstraPlanner, nav2_core::GlobalPlanner)