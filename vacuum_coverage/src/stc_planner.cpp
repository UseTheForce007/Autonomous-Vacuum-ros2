#include "vacuum_coverage/stc_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace vacuum_coverage
{

int STCPlanner::find_nearest_node(
  const Graph & graph, double world_x, double world_y) const
{
  int best_id = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < graph.node_count(); ++i) {
    const auto & node = graph.get_node(i);
    if (node.is_occupied) {
      continue;
    }
    double dx = node.world_x - world_x;
    double dy = node.world_y - world_y;
    double d = dx * dx + dy * dy;
    if (d < best_dist) {
      best_dist = d;
      best_id = i;
    }
  }
  return best_id;
}

void STCPlanner::build_spanning_tree(int start_id, const Graph & graph)
{
  int n = graph.node_count();
  visited_.assign(n, false);
  children_.assign(n, {});
  tree_edges_.clear();

  std::vector<int> stack;
  stack.push_back(start_id);
  visited_[start_id] = true;

  while (!stack.empty()) {
    int cur = stack.back();
    stack.pop_back();

    auto neighbors = graph.get_neighbors(cur);
    for (int nb : neighbors) {
      if (visited_[nb]) {
        continue;
      }
      visited_[nb] = true;
      children_[cur].push_back(nb);
      tree_edges_.emplace_back(cur, nb);
      stack.push_back(nb);
    }
  }
}

void STCPlanner::traverse_tree(
  int node_id, int parent_id,
  const Graph & graph,
  nav_msgs::msg::Path & path)
{
  const auto & node = graph.get_node(node_id);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = path.header.frame_id;
  pose.pose.position.x = node.world_x;
  pose.pose.position.y = node.world_y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  for (int child : children_[node_id]) {
    if (child == parent_id) {
      continue;
    }
    traverse_tree(child, node_id, graph, path);

    const auto & return_node = graph.get_node(node_id);
    geometry_msgs::msg::PoseStamped return_pose;
    return_pose.header.frame_id = path.header.frame_id;
    return_pose.pose.position.x = return_node.world_x;
    return_pose.pose.position.y = return_node.world_y;
    return_pose.pose.position.z = 0.0;
    return_pose.pose.orientation.w = 1.0;
    path.poses.push_back(return_pose);
  }
}

nav_msgs::msg::Path STCPlanner::plan(
  const Graph & graph,
  double start_x,
  double start_y,
  const std::string & frame_id)
{
  if (graph.node_count() == 0) {
    return nav_msgs::msg::Path{};
  }

  int start_id = find_nearest_node(graph, start_x, start_y);

  build_spanning_tree(start_id, graph);

  int visited_count = 0;
  for (bool v : visited_) {
    if (v) {
      ++visited_count;
    }
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id.empty() ? "map" : frame_id;
  path.header.stamp.sec = 0;

  traverse_tree(start_id, -1, graph, path);

  return path;
}

}  // namespace vacuum_coverage
