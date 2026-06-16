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

std::vector<std::vector<int>> MapToGraph::build_downsampled_grid(
  const nav_msgs::msg::OccupancyGrid & map,
  int stride)
{
  unsigned int width = map.info.width;
  unsigned int height = map.info.height;

  decomposed_width_ = std::ceil(static_cast<double>(width) / stride);
  decomposed_height_ = std::ceil(static_cast<double>(height) / stride);

  std::vector<std::vector<int>> grid(
    decomposed_height_, std::vector<int>(decomposed_width_, 0));

  auto threshold = static_cast<int8_t>(occupancy_threshold_);
  int cells_per_block = stride * stride;
  int min_free_cells = static_cast<int>(cells_per_block * free_threshold_ratio_);

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
      if (total == 0) {
        grid[dy][dx] = -1;
      } else {
        int required = static_cast<int>(total * free_threshold_ratio_);
        grid[dy][dx] = (free_count >= required) ? 0 : -1;
      }
    }
  }

  return grid;
}

Graph MapToGraph::build_graph(
  const std::vector<std::vector<int>> & grid,
  int stride,
  double origin_x,
  double origin_y,
  double resolution)
{
  Graph graph;

  for (int dy = 0; dy < decomposed_height_; ++dy) {
    for (int dx = 0; dx < decomposed_width_; ++dx) {
      if (grid[dy][dx] != 0) {
        continue;
      }

      double wx = origin_x + (dx * stride + stride / 2.0) * resolution;
      double wy = origin_y + (dy * stride + stride / 2.0) * resolution;
      graph.add_node(dx, dy, wx, wy, false);
    }
  }

  const int dirs[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  for (int dy = 0; dy < decomposed_height_; ++dy) {
    for (int dx = 0; dx < decomposed_width_; ++dx) {
      if (grid[dy][dx] != 0) {
        continue;
      }
      int from_id = graph.node_id_at(dx, dy);

      for (auto & dir : dirs) {
        int nx = dx + dir[0];
        int ny = dy + dir[1];
        if (nx < 0 || nx >= decomposed_width_ ||
          ny < 0 || ny >= decomposed_height_)
        {
          continue;
        }
        if (grid[ny][nx] != 0) {
          continue;
        }
        if (!graph.has_node_at(nx, ny)) {
          continue;
        }
        int to_id = graph.node_id_at(nx, ny);
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
