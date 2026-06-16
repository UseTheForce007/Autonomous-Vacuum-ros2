#ifndef VACUUM_COVERAGE__MAP_TO_GRAPH_HPP_
#define VACUUM_COVERAGE__MAP_TO_GRAPH_HPP_

#include "vacuum_coverage/graph.hpp"

#include <memory>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace vacuum_coverage
{

class MapToGraph
{
public:
  explicit MapToGraph(
    double cell_size = 0.3,
    int occupancy_threshold = 50,
    double free_threshold_ratio = 0.9);

  Graph convert(const nav_msgs::msg::OccupancyGrid & map);

  double cell_size() const { return cell_size_; }
  int decomposed_width() const { return decomposed_width_; }
  int decomposed_height() const { return decomposed_height_; }

private:
  std::vector<std::vector<int>> build_downsampled_grid(
    const nav_msgs::msg::OccupancyGrid & map,
    int stride);

  Graph build_graph(
    const std::vector<std::vector<int>> & grid,
    int stride,
    double origin_x,
    double origin_y,
    double resolution);

  double cell_size_;
  int occupancy_threshold_;
  double free_threshold_ratio_;
  int decomposed_width_;
  int decomposed_height_;
};

}  // namespace vacuum_coverage

#endif  // VACUUM_COVERAGE__MAP_TO_GRAPH_HPP_
