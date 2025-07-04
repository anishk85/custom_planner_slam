#ifndef DIJKSTRA_PLANNER_HPP_
#define DIJKSTRA_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"

namespace dijkstra_planner
{

class DijkstraPlanner : public nav2_core::GlobalPlanner
{
public:
  DijkstraPlanner() = default;
  ~DijkstraPlanner() = default;

  // Configure the planner
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // Cleanup
  void cleanup() override;

  // Activate
  void activate() override;

  // Deactivate
  void deactivate() override;

  // Create plan using Dijkstra's algorithm
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Node structure for Dijkstra's algorithm
  struct Node {
    int x, y;
    double cost;
    int parent_x, parent_y;
    
    Node(int x_, int y_, double cost_, int px = -1, int py = -1) 
      : x(x_), y(y_), cost(cost_), parent_x(px), parent_y(py) {}
  };

  // Comparator for priority queue
  struct NodeComparator {
    bool operator()(const Node& a, const Node& b) {
      return a.cost > b.cost;  // Min heap
    }
  };

  // Helper functions
  bool isValid(int x, int y);
  double getCost(int x, int y);
  std::vector<Node> getNeighbors(const Node& current);
  nav_msgs::msg::Path reconstructPath(
    const std::vector<std::vector<Node>>& came_from,
    const Node& start, const Node& goal);
  void worldToMap(double wx, double wy, int& mx, int& my);
  void mapToWorld(int mx, int my, double& wx, double& wy);

  // Member variables
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("DijkstraPlanner")};
  std::string name_;
  bool initialized_;

  // Parameters
  double tolerance_;
  bool use_astar_;  // Option to use A* heuristic (making it A* instead of pure Dijkstra)
};

}  // namespace dijkstra_planner

#endif  // DIJKSTRA_PLANNER_HPP_