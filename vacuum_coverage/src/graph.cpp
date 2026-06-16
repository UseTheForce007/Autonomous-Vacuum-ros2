#include "vacuum_coverage/graph.hpp"

#include <stdexcept>

namespace vacuum_coverage
{

int Graph::add_node(int grid_x, int grid_y, double world_x, double world_y, bool occupied)
{
  if (has_node_at(grid_x, grid_y)) {
    return node_id_at(grid_x, grid_y);
  }
  int id = nodes_.size();
  nodes_.push_back({id, grid_x, grid_y, world_x, world_y, occupied});
  adjacency_.emplace_back();
  grid_to_node_[{grid_x, grid_y}] = id;
  return id;
}

void Graph::add_edge(int from, int to, double cost)
{
  edges_.push_back({from, to, cost});
  adjacency_[from].push_back(to);
  adjacency_[to].push_back(from);
}

void Graph::clear()
{
  nodes_.clear();
  edges_.clear();
  adjacency_.clear();
  grid_to_node_.clear();
}

const Node & Graph::get_node(int id) const
{
  return nodes_.at(id);
}

const std::vector<Edge> & Graph::get_edges() const
{
  return edges_;
}

std::vector<int> Graph::get_neighbors(int node_id) const
{
  return adjacency_.at(node_id);
}

int Graph::node_count() const
{
  return nodes_.size();
}

int Graph::edge_count() const
{
  return edges_.size();
}

bool Graph::has_node_at(int grid_x, int grid_y) const
{
  return grid_to_node_.count({grid_x, grid_y}) > 0;
}

int Graph::node_id_at(int grid_x, int grid_y) const
{
  return grid_to_node_.at({grid_x, grid_y});
}

}  // namespace vacuum_coverage
