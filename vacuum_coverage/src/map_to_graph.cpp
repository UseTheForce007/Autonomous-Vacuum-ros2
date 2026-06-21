#include "vacuum_coverage/map_to_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace vacuum_coverage
{

MapToGraph::MapToGraph(
  double cell_size,
  int occupancy_threshold,
  double free_threshold_ratio)
: cell_size_(cell_size),
  occupancy_threshold_(occupancy_threshold),
  free_threshold_ratio_(free_threshold_ratio),
  decomposed_width_(0),
  decomposed_height_(0)
{
}

std::vector<int> MapToGraph::build_downsampled_grid(
  const nav_msgs::msg::OccupancyGrid & map,
  int stride)
{
  unsigned int width = map.info.width;
  unsigned int height = map.info.height;

  decomposed_width_ = std::ceil(static_cast<double>(width) / stride);
  decomposed_height_ = std::ceil(static_cast<double>(height) / stride);

  std::vector<int> grid(decomposed_width_ * decomposed_height_, 0);

  auto threshold = static_cast<int8_t>(occupancy_threshold_);

  for (int dy = 0; dy < decomposed_height_; ++dy) {
    for (int dx = 0; dx < decomposed_width_; ++dx) {
      int free_count = 0;
      int total = 0;
      for (int sy = 0; sy < stride; ++sy) {
        for (int sx = 0; sx < stride; ++sx) {
          int px = dx * stride + sx;
          int py = dy * stride + sy;
          if (px >= static_cast<int>(width) || py >= static_cast<int>(height)) {
            continue;
          }
          int8_t cell = map.data[py * width + px];
          if (cell >= 0 && cell <= threshold) {
            ++free_count;
          }
          ++total;
        }
      }
      int idx = dy * decomposed_width_ + dx;
      if (total == 0) {
        grid[idx] = -1;
      } else {
        int required = static_cast<int>(total * free_threshold_ratio_);
        grid[idx] = (free_count >= required) ? 0 : -1;
      }
    }
  }

  return grid;
}

Graph MapToGraph::build_graph(
  const std::vector<int> & grid,
  int stride,
  double origin_x,
  double origin_y,
  double resolution)
{
  Graph graph;

  std::vector<int> node_ids(decomposed_width_ * decomposed_height_, -1);

  for (int dy = 0; dy < decomposed_height_; ++dy) {
    for (int dx = 0; dx < decomposed_width_; ++dx) {
      int idx = dy * decomposed_width_ + dx;
      if (grid[idx] != 0) {
        continue;
      }

      double wx = origin_x + (dx * stride + stride / 2.0) * resolution;
      double wy = origin_y + (dy * stride + stride / 2.0) * resolution;
      int id = graph.add_node(dx, dy, wx, wy, false);
      node_ids[idx] = id;
    }
  }

  const int dirs[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  for (int dy = 0; dy < decomposed_height_; ++dy) {
    for (int dx = 0; dx < decomposed_width_; ++dx) {
      int from_idx = dy * decomposed_width_ + dx;
      int from_id = node_ids[from_idx];
      if (from_id == -1) {
        continue;
      }

      for (auto & dir : dirs) {
        int nx = dx + dir[0];
        int ny = dy + dir[1];
        if (nx < 0 || nx >= decomposed_width_ ||
          ny < 0 || ny >= decomposed_height_)
        {
          continue;
        }
        int to_idx = ny * decomposed_width_ + nx;
        int to_id = node_ids[to_idx];
        if (to_id == -1) {
          continue;
        }
        if (from_id < to_id) {
          graph.add_edge(from_id, to_id, cell_size_);
        }
      }
    }
  }

  return graph;
}

Graph MapToGraph::convert(const nav_msgs::msg::OccupancyGrid & map)
{
  int stride = std::max(1, static_cast<int>(std::round(cell_size_ / map.info.resolution)));
  auto grid = build_downsampled_grid(map, stride);
  return build_graph(
    grid, stride,
    map.info.origin.position.x,
    map.info.origin.position.y,
    map.info.resolution);
}

}  // namespace vacuum_coverage
