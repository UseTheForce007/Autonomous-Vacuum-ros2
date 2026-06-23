#ifndef VACUUM_COVERAGE__BOUSTROPHEDON_COVERAGE_PLANNER_HPP_
#define VACUUM_COVERAGE__BOUSTROPHEDON_COVERAGE_PLANNER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>

namespace vacuum_coverage
{

using Cell = std::pair<int, int>;
using Grid = std::vector<std::vector<uint8_t>>;
using Room = std::set<Cell>;

class BoustrophedonCoveragePlanner : public nav2_core::GlobalPlanner
{
public:
  BoustrophedonCoveragePlanner();
  ~BoustrophedonCoveragePlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  // Algorithm methods (ported from coverage_standalone.py)
  Grid inflateGrid(const Grid & grid, int r) const;

  std::vector<Room> findRooms(const Grid & grid, int min_cells) const;

  std::vector<Cell> astarNoCC(
    const Grid & grid, Cell start, Cell goal,
    std::function<bool()> cancel_checker) const;

  std::vector<Cell> astarCC(
    const Grid & grid, Cell start, Cell goal,
    std::function<bool()> cancel_checker) const;

  std::vector<Cell> coverRoom(
    const Grid & grid, const Room & cells, int stride,
    std::function<bool()> cancel_checker) const;

  std::vector<std::string> validate(
    const std::vector<Cell> & path, const Grid & occ_grid) const;

  // World coordinate conversion
  Cell worldToCell(double wx, double wy) const;
  void cellToWorld(int cx, int cy, double & wx, double & wy) const;

  // Orientation
  void orientPath(nav_msgs::msg::Path & path) const;

  nav_msgs::msg::Path makePath(const std::vector<Cell> & cells) const;

  // Compute full coverage path (called once, cached)
  nav_msgs::msg::Path computeCoverage(
    std::function<bool()> cancel_checker);

  // Parameters
  double spacing_{0.3};
  double robot_radius_{0.2};
  double min_room_area_{0.5};
  std::string name_;
  bool coverage_computed_{false};

  // ROS interfaces
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_{"map"};
  rclcpp::Logger logger_{rclcpp::get_logger("boustrophedon_coverage_planner")};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_srv_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Cached coverage path
  nav_msgs::msg::Path cached_path_;
};

}  // namespace vacuum_coverage

#endif  // VACUUM_COVERAGE__BOUSTROPHEDON_COVERAGE_PLANNER_HPP_
