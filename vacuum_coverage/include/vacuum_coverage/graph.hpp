#ifndef VACUUM_COVERAGE__GRAPH_HPP_
#define VACUUM_COVERAGE__GRAPH_HPP_

#include <cstddef>
#include <map>
#include <utility>
#include <vector>

namespace vacuum_coverage
{

struct Node
{
  int id;
  int grid_x;
  int grid_y;
  double world_x;
  double world_y;
  bool is_occupied;
};

struct Edge
{
  int from;
  int to;
  double cost;
};

class Graph
{
public:
  int add_node(int grid_x, int grid_y, double world_x, double world_y, bool occupied);
  void add_edge(int from, int to, double cost);
  void clear();

  const Node & get_node(int id) const;
  const std::vector<Edge> & get_edges() const;
  std::vector<int> get_neighbors(int node_id) const;
  int node_count() const;
  int edge_count() const;
  bool has_node_at(int grid_x, int grid_y) const;
  int node_id_at(int grid_x, int grid_y) const;

private:
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  std::vector<std::vector<int>> adjacency_;
  std::map<std::pair<int, int>, int> grid_to_node_;
};

}  // namespace vacuum_coverage

#endif  // VACUUM_COVERAGE__GRAPH_HPP_
