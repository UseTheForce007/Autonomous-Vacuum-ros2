#ifndef VACUUM_COVERAGE__STC_PLANNER_HPP_
#define VACUUM_COVERAGE__STC_PLANNER_HPP_

#include "vacuum_coverage/graph.hpp"

#include <utility>
#include <vector>

#include <nav_msgs/msg/path.hpp>

namespace vacuum_coverage
{

class STCPlanner
{
public:
  STCPlanner() = default;

  nav_msgs::msg::Path plan(
    const Graph & graph,
    double start_x,
    double start_y,
    const std::string & frame_id);

  const std::vector<std::pair<int, int>> & get_spanning_tree() const
  {
    return tree_edges_;
  }

private:
  int find_nearest_node(const Graph & graph, double world_x, double world_y) const;

  void build_spanning_tree(int start_id, const Graph & graph);

  void traverse_tree(
    int node_id,
    int parent_id,
    const Graph & graph,
    nav_msgs::msg::Path & path);

  std::vector<std::pair<int, int>> tree_edges_;
  std::vector<bool> visited_;
  std::vector<std::vector<int>> children_;
};

}  // namespace vacuum_coverage

#endif  // VACUUM_COVERAGE__STC_PLANNER_HPP_
