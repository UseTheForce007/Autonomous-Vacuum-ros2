#include "vacuum_coverage/boustrophedon_coverage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace vacuum_coverage
{

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

BoustrophedonCoveragePlanner::BoustrophedonCoveragePlanner()
: logger_(rclcpp::get_logger("boustrophedon_coverage_planner"))
{
}

BoustrophedonCoveragePlanner::~BoustrophedonCoveragePlanner()
{
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void BoustrophedonCoveragePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock parent node in configure");
  }

  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  spacing_ = node->declare_parameter(name + ".spacing", 0.3);
  robot_radius_ = node->declare_parameter(name + ".robot_radius", 0.2);
  min_room_area_ = node->declare_parameter(name + ".min_room_area", 0.5);

  RCLCPP_INFO(logger_,
    "Configured: spacing=%.2f m  robot_radius=%.2f m  min_room_area=%.2f m²",
    spacing_, robot_radius_, min_room_area_);

  coverage_computed_ = false;

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "~/coverage_path", rclcpp::QoS(1).transient_local());

  replan_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/replan",
    [this, node](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      RCLCPP_INFO(logger_, "Computing coverage path...");
      auto cancel = []() { return false; };
      cached_path_ = computeCoverage(cancel);
      coverage_computed_ = true;
      if (!cached_path_.poses.empty()) {
        cached_path_.header.stamp = node->now();
        path_pub_->publish(cached_path_);
        RCLCPP_INFO(logger_, "Coverage path published (%zu poses)",
          cached_path_.poses.size());
      }
      response->success = !cached_path_.poses.empty();
      response->message = cached_path_.poses.empty()
        ? "Coverage computation failed — no path generated"
        : "Coverage path computed and published";
    });
}

void BoustrophedonCoveragePlanner::cleanup()
{
  cached_path_ = nav_msgs::msg::Path();
  coverage_computed_ = false;
}

void BoustrophedonCoveragePlanner::activate()
{
}

void BoustrophedonCoveragePlanner::deactivate()
{
}

// ---------------------------------------------------------------------------
// Coordinate helpers
// ---------------------------------------------------------------------------

Cell BoustrophedonCoveragePlanner::worldToCell(double wx, double wy) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return {-1, -1};
  }
  return {static_cast<int>(mx), static_cast<int>(my)};
}

void BoustrophedonCoveragePlanner::cellToWorld(int cx, int cy, double & wx, double & wy) const
{
  costmap_->mapToWorld(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy), wx, wy);
}

// ---------------------------------------------------------------------------
// Inflate grid  (Chebyshev-disk dilation, matching the Python version)
// ---------------------------------------------------------------------------

Grid BoustrophedonCoveragePlanner::inflateGrid(const Grid & grid, int r) const
{
  if (r <= 0) {
    return grid;
  }
  int h = static_cast<int>(grid.size());
  int w = static_cast<int>(h > 0 ? grid[0].size() : 0);
  Grid out(h, std::vector<uint8_t>(w, 0));

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (!grid[y][x]) {
        continue;
      }
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (dx * dx + dy * dy <= r * r) {
            int ny = y + dy;
            int nx = x + dx;
            if (ny >= 0 && ny < h && nx >= 0 && nx < w) {
              out[ny][nx] = 1;
            }
          }
        }
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Room decomposition  (8-connectivity, matching the Python version)
// ---------------------------------------------------------------------------

std::vector<Room> BoustrophedonCoveragePlanner::findRooms(
  const Grid & grid, int min_cells) const
{
  int h = static_cast<int>(grid.size());
  int w = static_cast<int>(h > 0 ? grid[0].size() : 0);
  std::vector<std::vector<bool>> seen(h, std::vector<bool>(w, false));
  std::vector<Room> rooms;

  const int dirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (grid[y][x] || seen[y][x]) {
        continue;
      }
      // BFS
      std::vector<Cell> stack;
      stack.emplace_back(x, y);
      Room comp;
      while (!stack.empty()) {
        auto cx = stack.back().first;
        auto cy = stack.back().second;
        stack.pop_back();
        if (cx < 0 || cx >= w || cy < 0 || cy >= h) {
          continue;
        }
        if (seen[cy][cx] || grid[cy][cx]) {
          continue;
        }
        seen[cy][cx] = true;
        comp.emplace(cx, cy);
        for (const auto & d : dirs) {
          stack.emplace_back(cx + d[0], cy + d[1]);
        }
      }
      if (static_cast<int>(comp.size()) >= min_cells) {
        rooms.push_back(std::move(comp));
      }
    }
  }
  std::sort(rooms.begin(), rooms.end(),
    [](const Room & a, const Room & b) {
      return a.size() > b.size();
    });
  return rooms;
}

// ---------------------------------------------------------------------------
// A* helpers
// ---------------------------------------------------------------------------

struct AStarNode
{
  double f;
  int x;
  int y;
};

struct AStarCompare
{
  bool operator()(const AStarNode & a, const AStarNode & b) const
  {
    return a.f > b.f;
  }
};

using AStarPQ = std::priority_queue<AStarNode, std::vector<AStarNode>, AStarCompare>;

static double astarHeuristic(int ax, int ay, int bx, int by)
{
  return std::hypot(static_cast<double>(bx - ax), static_cast<double>(by - ay));
}

static std::vector<Cell> reconstructPath(
  const std::vector<std::vector<std::pair<int, int>>> & came_from,
  Cell start, Cell goal)
{
  std::vector<Cell> path;
  int cx = goal.first;
  int cy = goal.second;
  while (!(cx == start.first && cy == start.second)) {
    path.emplace_back(cx, cy);
    auto p = came_from[cy][cx];
    cx = p.first;
    cy = p.second;
  }
  path.emplace_back(start.first, start.second);
  std::reverse(path.begin(), path.end());
  return path;
}

// ---------------------------------------------------------------------------
// A* without corner-cutting (original grid – used for inter-room connectors)
// ---------------------------------------------------------------------------

std::vector<Cell> BoustrophedonCoveragePlanner::astarNoCC(
  const Grid & grid, Cell start, Cell goal,
  std::function<bool()> cancel_checker) const
{
  int h = static_cast<int>(grid.size());
  int w = static_cast<int>(h > 0 ? grid[0].size() : 0);
  int sx = start.first;
  int sy = start.second;
  int gx = goal.first;
  int gy = goal.second;

  if (sx < 0 || sx >= w || sy < 0 || sy >= h ||
    gx < 0 || gx >= w || gy < 0 || gy >= h)
  {
    return {};
  }
  if (grid[sy][sx] || grid[gy][gx]) {
    return {};
  }

  const int dirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  std::vector<std::vector<double>> g_score(
    h, std::vector<double>(w, std::numeric_limits<double>::infinity()));
  std::vector<std::vector<std::pair<int, int>>> came_from(
    h, std::vector<std::pair<int, int>>(w, {-1, -1}));
  std::vector<std::vector<bool>> closed(
    h, std::vector<bool>(w, false));

  AStarPQ open_set;
  g_score[sy][sx] = 0.0;
  open_set.push({astarHeuristic(sx, sy, gx, gy), sx, sy});

  while (!open_set.empty()) {
    if (cancel_checker()) {
      RCLCPP_DEBUG(logger_, "astarNoCC cancelled");
      return {};
    }
    auto cur = open_set.top();
    open_set.pop();
    int cx = cur.x;
    int cy = cur.y;

    if (closed[cy][cx]) {
      continue;
    }
    closed[cy][cx] = true;

    if (cx == gx && cy == gy) {
      return reconstructPath(came_from, start, goal);
    }

    for (const auto & d : dirs) {
      int nx = cx + d[0];
      int ny = cy + d[1];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
        continue;
      }
      if (grid[ny][nx]) {
        continue;
      }
      double step_cost = std::hypot(static_cast<double>(d[0]), static_cast<double>(d[1]));
      double tentative = g_score[cy][cx] + step_cost;
      if (tentative < g_score[ny][nx]) {
        came_from[ny][nx] = {cx, cy};
        g_score[ny][nx] = tentative;
        open_set.push({tentative + astarHeuristic(nx, ny, gx, gy), nx, ny});
      }
    }
  }
  return {};
}

// ---------------------------------------------------------------------------
// A* with corner-cutting check (inflated grid – used for intra-room sweeps)
// ---------------------------------------------------------------------------

std::vector<Cell> BoustrophedonCoveragePlanner::astarCC(
  const Grid & grid, Cell start, Cell goal,
  std::function<bool()> cancel_checker) const
{
  int h = static_cast<int>(grid.size());
  int w = static_cast<int>(h > 0 ? grid[0].size() : 0);
  int sx = start.first;
  int sy = start.second;
  int gx = goal.first;
  int gy = goal.second;

  if (sx < 0 || sx >= w || sy < 0 || sy >= h ||
    gx < 0 || gx >= w || gy < 0 || gy >= h)
  {
    return {};
  }
  if (grid[sy][sx] || grid[gy][gx]) {
    return {};
  }

  const int dirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  std::vector<std::vector<double>> g_score(
    h, std::vector<double>(w, std::numeric_limits<double>::infinity()));
  std::vector<std::vector<std::pair<int, int>>> came_from(
    h, std::vector<std::pair<int, int>>(w, {-1, -1}));
  std::vector<std::vector<bool>> closed(
    h, std::vector<bool>(w, false));

  AStarPQ open_set;
  g_score[sy][sx] = 0.0;
  open_set.push({astarHeuristic(sx, sy, gx, gy), sx, sy});

  while (!open_set.empty()) {
    if (cancel_checker()) {
      RCLCPP_DEBUG(logger_, "astarCC cancelled");
      return {};
    }
    auto cur = open_set.top();
    open_set.pop();
    int cx = cur.x;
    int cy = cur.y;

    if (closed[cy][cx]) {
      continue;
    }
    closed[cy][cx] = true;

    if (cx == gx && cy == gy) {
      return reconstructPath(came_from, start, goal);
    }

    for (const auto & d : dirs) {
      int nx = cx + d[0];
      int ny = cy + d[1];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
        continue;
      }
      if (grid[ny][nx]) {
        continue;
      }
      // Corner-cutting: reject diagonal when both orthogonal neighbours
      // are obstacles (would clip a convex corner).
      if (std::abs(d[0]) == 1 && std::abs(d[1]) == 1 &&
        grid[cy][nx] && grid[ny][cx])
      {
        continue;
      }
      double step_cost = std::hypot(static_cast<double>(d[0]), static_cast<double>(d[1]));
      double tentative = g_score[cy][cx] + step_cost;
      if (tentative < g_score[ny][nx]) {
        came_from[ny][nx] = {cx, cy};
        g_score[ny][nx] = tentative;
        open_set.push({tentative + astarHeuristic(nx, ny, gx, gy), nx, ny});
      }
    }
  }
  return {};
}

// ---------------------------------------------------------------------------
// Boustrophedon coverage for a single room
// ---------------------------------------------------------------------------

std::vector<Cell> BoustrophedonCoveragePlanner::coverRoom(
  const Grid & grid, const Room & cells, int stride,
  std::function<bool()> cancel_checker) const
{
  if (cells.empty()) {
    return {};
  }

  int min_x = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  int min_y = std::numeric_limits<int>::max();
  int max_y = std::numeric_limits<int>::min();

  for (const auto & c : cells) {
    min_x = std::min(min_x, c.first);
    max_x = std::max(max_x, c.first);
    min_y = std::min(min_y, c.second);
    max_y = std::max(max_y, c.second);
  }

  bool horiz = (max_x - min_x) >= (max_y - min_y);
  std::vector<Cell> wp;

  if (horiz) {
    std::vector<int> rows;
    for (int y = min_y; y <= max_y; y += stride) {
      rows.push_back(y);
    }
    // Pre-compute free cells per row
    struct RowData
    {
      int y;
      std::vector<int> xs;
    };
    std::vector<RowData> row_data;
    for (int y : rows) {
      std::vector<int> xs;
      for (int x = min_x; x <= max_x; ++x) {
        if (cells.count({x, y}) && !grid[y][x]) {
          xs.push_back(x);
        }
      }
      row_data.push_back({y, std::move(xs)});
    }

    for (std::size_t ri = 0; ri < row_data.size(); ++ri) {
      if (cancel_checker()) {
        RCLCPP_DEBUG(logger_, "coverRoom cancelled (horiz)");
        return wp;
      }
      const auto & here = row_data[ri].xs;
      if (here.empty()) {
        continue;
      }
      int y = row_data[ri].y;

      if (ri % 2 == 0) {
        for (int px : here) {
          if (!wp.empty() && std::abs(px - wp.back().first) > 1) {
            auto p = astarCC(grid, wp.back(), {px, y}, cancel_checker);
            if (!p.empty()) {
              wp.insert(wp.end(), p.begin() + 1, p.end());
            } else {
              wp.emplace_back(px, y);
            }
          } else {
            wp.emplace_back(px, y);
          }
        }
      } else {
        for (auto it = here.rbegin(); it != here.rend(); ++it) {
          int px = *it;
          if (!wp.empty() && std::abs(px - wp.back().first) > 1) {
            auto p = astarCC(grid, wp.back(), {px, y}, cancel_checker);
            if (!p.empty()) {
              wp.insert(wp.end(), p.begin() + 1, p.end());
            } else {
              wp.emplace_back(px, y);
            }
          } else {
            wp.emplace_back(px, y);
          }
        }
      }

      // Transition to next row
      if (ri < row_data.size() - 1) {
        const auto & nxt = row_data[ri + 1].xs;
        if (!nxt.empty()) {
          int nx = (ri + 1) % 2 == 0 ? nxt.front() : nxt.back();
          int ny_val = row_data[ri + 1].y;
          auto p = astarCC(grid, wp.back(), {nx, ny_val}, cancel_checker);
          if (!p.empty()) {
            wp.insert(wp.end(), p.begin() + 1, p.end());
          }
        }
      }
    }
  } else {
    // Vertical sweeps
    std::vector<int> cols;
    for (int x = min_x; x <= max_x; x += stride) {
      cols.push_back(x);
    }
    struct ColData
    {
      int x;
      std::vector<int> ys;
    };
    std::vector<ColData> col_data;
    for (int x : cols) {
      std::vector<int> ys;
      for (int y = min_y; y <= max_y; ++y) {
        if (cells.count({x, y}) && !grid[y][x]) {
          ys.push_back(y);
        }
      }
      col_data.push_back({x, std::move(ys)});
    }

    for (std::size_t ci = 0; ci < col_data.size(); ++ci) {
      if (cancel_checker()) {
        RCLCPP_DEBUG(logger_, "coverRoom cancelled (vert)");
        return wp;
      }
      const auto & here = col_data[ci].ys;
      if (here.empty()) {
        continue;
      }
      int x = col_data[ci].x;

      if (ci % 2 == 0) {
        for (int py : here) {
          if (!wp.empty() && std::abs(py - wp.back().second) > 1) {
            auto p = astarCC(grid, wp.back(), {x, py}, cancel_checker);
            if (!p.empty()) {
              wp.insert(wp.end(), p.begin() + 1, p.end());
            } else {
              wp.emplace_back(x, py);
            }
          } else {
            wp.emplace_back(x, py);
          }
        }
      } else {
        for (auto it = here.rbegin(); it != here.rend(); ++it) {
          int py = *it;
          if (!wp.empty() && std::abs(py - wp.back().second) > 1) {
            auto p = astarCC(grid, wp.back(), {x, py}, cancel_checker);
            if (!p.empty()) {
              wp.insert(wp.end(), p.begin() + 1, p.end());
            } else {
              wp.emplace_back(x, py);
            }
          } else {
            wp.emplace_back(x, py);
          }
        }
      }

      // Transition to next column
      if (ci < col_data.size() - 1) {
        const auto & nxt = col_data[ci + 1].ys;
        if (!nxt.empty()) {
          int ny = (ci + 1) % 2 == 0 ? nxt.front() : nxt.back();
          int nx_val = col_data[ci + 1].x;
          auto p = astarCC(grid, wp.back(), {nx_val, ny}, cancel_checker);
          if (!p.empty()) {
            wp.insert(wp.end(), p.begin() + 1, p.end());
          }
        }
      }
    }
  }
  return wp;
}

// ---------------------------------------------------------------------------
// Validation – check every waypoint and transition against the occupancy grid
// ---------------------------------------------------------------------------

std::vector<std::string> BoustrophedonCoveragePlanner::validate(
  const std::vector<Cell> & path, const Grid & occ_grid) const
{
  int h = static_cast<int>(occ_grid.size());
  int w = static_cast<int>(h > 0 ? occ_grid[0].size() : 0);
  std::vector<std::string> bad;

  for (std::size_t i = 0; i < path.size(); ++i) {
    int px = path[i].first;
    int py = path[i].second;
    if (px < 0 || px >= w || py < 0 || py >= h) {
      bad.push_back("WP_OOB:" + std::to_string(i));
    } else if (occ_grid[py][px]) {
      bad.push_back("WP_OBS:" + std::to_string(i));
    }
  }

  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    int x1 = path[i].first;
    int y1 = path[i].second;
    int x2 = path[i + 1].first;
    int y2 = path[i + 1].second;
    int steps = std::max(std::abs(x2 - x1), std::abs(y2 - y1));
    if (steps <= 1) {
      continue;
    }
    for (int t = 1; t < steps; ++t) {
      int x = x1 + (x2 - x1) * t / steps;
      int y = y1 + (y2 - y1) * t / steps;
      if (x < 0 || x >= w || y < 0 || y >= h) {
        continue;
      }
      if (occ_grid[y][x]) {
        bad.push_back("TRANS_OBS:" + std::to_string(i));
        break;
      }
    }
  }
  return bad;
}

// ---------------------------------------------------------------------------
// Orientation
// ---------------------------------------------------------------------------

void BoustrophedonCoveragePlanner::orientPath(nav_msgs::msg::Path & path) const
{
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    double yaw = std::atan2(dy, dx);
    path.poses[i].pose.orientation.z = std::sin(yaw * 0.5);
    path.poses[i].pose.orientation.w = std::cos(yaw * 0.5);
  }
  if (path.poses.size() >= 2) {
    path.poses.back().pose.orientation =
      path.poses[path.poses.size() - 2].pose.orientation;
  } else if (path.poses.size() == 1) {
    path.poses[0].pose.orientation.w = 1.0;
  }
}

// ---------------------------------------------------------------------------
// Make a nav_msgs::msg::Path from a vector of cell coordinates
// ---------------------------------------------------------------------------

nav_msgs::msg::Path BoustrophedonCoveragePlanner::makePath(
  const std::vector<Cell> & cells) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  if (cells.empty()) {
    return path;
  }

  for (const auto & c : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    cellToWorld(c.first, c.second, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    path.poses.push_back(pose);
  }

  orientPath(path);
  return path;
}

// ---------------------------------------------------------------------------
// Compute the full coverage path (cached)
// ---------------------------------------------------------------------------

nav_msgs::msg::Path BoustrophedonCoveragePlanner::computeCoverage(
  std::function<bool()> cancel_checker)
{
  if (!costmap_) {
    RCLCPP_WARN(logger_, "No costmap available, cannot compute coverage");
    return nav_msgs::msg::Path();
  }

  unsigned int w = costmap_->getSizeInCellsX();
  unsigned int h = costmap_->getSizeInCellsY();
  double res = costmap_->getResolution();

  if (w == 0 || h == 0) {
    RCLCPP_WARN(logger_, "Empty costmap");
    return nav_msgs::msg::Path();
  }

  // --- Build binary occupancy grid from costmap ---
  Grid occ_grid(h, std::vector<uint8_t>(w, 0));
  int free_total = 0;
  for (unsigned int y = 0; y < h; ++y) {
    for (unsigned int x = 0; x < w; ++x) {
      unsigned char cost = costmap_->getCost(x, y);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        occ_grid[y][x] = 1;
      } else {
        ++free_total;
      }
    }
  }

  if (free_total == 0) {
    RCLCPP_WARN(logger_, "No free cells in costmap");
    return nav_msgs::msg::Path();
  }

  int rpx = std::max(1, static_cast<int>(robot_radius_ / res));
  int spx = std::max(1, static_cast<int>(spacing_ / res));
  int min_px = static_cast<int>(min_room_area_ / (res * res));

  RCLCPP_INFO(logger_,
    "Costmap: %dx%d  res=%.3f m  inflation=%d px  stride=%d px",
    w, h, res, rpx, spx);

  // --- Inflated grid for intra-room safety ---
  Grid inf_grid = inflateGrid(occ_grid, rpx);
  int free_inf = 0;
  for (const auto & row : inf_grid) {
    for (auto v : row) {
      if (v == 0) ++free_inf;
    }
  }
  RCLCPP_INFO(logger_, "Free cells: %d -> %d (inflated)", free_total, free_inf);

  // --- Room decomposition on ORIGINAL grid (8-conn) ---
  std::vector<Room> orig_rooms = findRooms(occ_grid, min_px);
  RCLCPP_INFO(logger_, "Connected components (original): %zu", orig_rooms.size());

  // --- Sub-room decomposition on INFLATED grid ---
  std::vector<Room> inf_rooms = findRooms(inf_grid, min_px);
  RCLCPP_INFO(logger_, "Inflated components: %zu", inf_rooms.size());

  std::vector<Cell> full_path;
  int total_violations = 0;

  // Process the largest original room (interior)
  if (orig_rooms.empty()) {
    RCLCPP_WARN(logger_, "No interior room found");
    return nav_msgs::msg::Path();
  }

  const Room & interior = orig_rooms[0];
  {
    int min_x_i = std::numeric_limits<int>::max();
    int max_x_i = std::numeric_limits<int>::min();
    int min_y_i = std::numeric_limits<int>::max();
    int max_y_i = std::numeric_limits<int>::min();
    for (const auto & c : interior) {
      min_x_i = std::min(min_x_i, c.first);
      max_x_i = std::max(max_x_i, c.first);
      min_y_i = std::min(min_y_i, c.second);
      max_y_i = std::max(max_y_i, c.second);
    }
    double area_m2 = interior.size() * res * res;
    RCLCPP_INFO(logger_,
      "Interior room: %zu cells  %.1f m²  bbox=[%d-%d],[%d-%d]",
      interior.size(), area_m2, min_x_i, max_x_i, min_y_i, max_y_i);
  }

  // Find overlapping inflated sub-rooms
  using OverlapEntry = std::tuple<std::size_t, const Room *, int>;
  std::vector<OverlapEntry> overlapping;
  for (std::size_t ir_idx = 0; ir_idx < inf_rooms.size(); ++ir_idx) {
    int overlap = 0;
    const auto & ir = inf_rooms[ir_idx];
    for (const auto & c : ir) {
      if (interior.count(c)) {
        ++overlap;
      }
    }
    if (overlap > 0) {
      overlapping.emplace_back(ir_idx, &ir, overlap);
    }
  }
  std::sort(overlapping.begin(), overlapping.end(),
    [](const OverlapEntry & a, const OverlapEntry & b) {
      return std::get<2>(a) > std::get<2>(b);
    });

  RCLCPP_INFO(logger_, "Interior contains %zu inflated sub-room(s)", overlapping.size());

  // Generate coverage for each sub-room
  std::vector<std::vector<Cell>> sub_paths;
  for (const auto & entry : overlapping) {
    std::size_t ir_idx = std::get<0>(entry);
    const Room * ir = std::get<1>(entry);
    double area_ir = ir->size() * res * res;
    std::vector<Cell> cells_vec(ir->begin(), ir->end());
    std::vector<Cell> wp = coverRoom(inf_grid, *ir, spx, cancel_checker);
    auto violations = validate(wp, occ_grid);
    total_violations += static_cast<int>(violations.size());
    RCLCPP_INFO(logger_, "  Inflated sub-room %zu: %zu cells  %.1f m² -> %zu wp  %zu viol(s)",
      ir_idx, ir->size(), area_ir, wp.size(), violations.size());
    sub_paths.push_back(std::move(wp));
  }

  // Connect sub-rooms using A* on the ORIGINAL grid
  std::vector<Cell> merged;
  for (std::size_t i = 0; i < sub_paths.size(); ++i) {
    if (sub_paths[i].empty()) {
      continue;
    }
    if (!merged.empty()) {
      std::vector<Cell> connector = astarNoCC(
        occ_grid, merged.back(), sub_paths[i].front(), cancel_checker);
      if (!connector.empty()) {
        auto vc = validate(connector, occ_grid);
        total_violations += static_cast<int>(vc.size());
        RCLCPP_DEBUG(logger_, "    Connector: %zu steps  %zu viol(s)",
          connector.size(), vc.size());
        merged.insert(merged.end(), connector.begin() + 1, connector.end());
      } else {
        RCLCPP_WARN(logger_, "    Connector: A* FAILED between sub-rooms");
      }
    }
    merged.insert(merged.end(), sub_paths[i].begin(), sub_paths[i].end());
  }
  full_path = std::move(merged);

  // --- Summary ---
  double plen = 0.0;
  for (std::size_t i = 1; i < full_path.size(); ++i) {
    double dx = (full_path[i].first - full_path[i - 1].first) * res;
    double dy = (full_path[i].second - full_path[i - 1].second) * res;
    plen += std::hypot(dx, dy);
  }

  // Count unique covered cells
  std::set<Cell> covered(full_path.begin(), full_path.end());
  double cov_pct = free_total > 0
    ? 100.0 * static_cast<double>(covered.size()) / static_cast<double>(free_total)
    : 0.0;

  auto final_violations = validate(full_path, occ_grid);

  RCLCPP_INFO(logger_,
    "=== Coverage summary ===");
  RCLCPP_INFO(logger_,
    "Total waypoints      : %zu", full_path.size());
  RCLCPP_INFO(logger_,
    "Unique cells covered : %zu / %d free (%.1f%%)",
    covered.size(), free_total, cov_pct);
  RCLCPP_INFO(logger_,
    "Path length          : %.1f m", plen);
  RCLCPP_INFO(logger_,
    "Obstacle crossings   : %zu", final_violations.size());

  return makePath(full_path);
}

// ---------------------------------------------------------------------------
// createPlan – the main entry point called by Nav2's planner server
// ---------------------------------------------------------------------------

nav_msgs::msg::Path BoustrophedonCoveragePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & /*goal*/,
  std::function<bool()> cancel_checker)
{
  if (!coverage_computed_) {
    auto node = node_.lock();
    if (!node) {
      RCLCPP_WARN(logger_, "Parent node unavailable");
      return nav_msgs::msg::Path();
    }
    RCLCPP_INFO(logger_, "Computing coverage path (first call)...");
    cached_path_ = computeCoverage(cancel_checker);
    coverage_computed_ = true;
    if (!cached_path_.poses.empty()) {
      cached_path_.header.stamp = node->now();
    }
  }

  if (cached_path_.poses.empty()) {
    return cached_path_;
  }

  // Find the waypoint closest to the start pose
  auto node = node_.lock();
  double sx = start.pose.position.x;
  double sy = start.pose.position.y;
  std::size_t best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < cached_path_.poses.size(); ++i) {
    double dx = cached_path_.poses[i].pose.position.x - sx;
    double dy = cached_path_.poses[i].pose.position.y - sy;
    double d = dx * dx + dy * dy;
    if (d < best_dist) {
      best_dist = d;
      best_idx = i;
    }
  }

  // Build result path from best_idx to end of cached path
  nav_msgs::msg::Path result;
  result.header = cached_path_.header;
  if (node) {
    result.header.stamp = node->now();
  }
  for (std::size_t i = best_idx; i < cached_path_.poses.size(); ++i) {
    result.poses.push_back(cached_path_.poses[i]);
  }

  orientPath(result);
  return result;
}

}  // namespace vacuum_coverage

PLUGINLIB_EXPORT_CLASS(
  vacuum_coverage::BoustrophedonCoveragePlanner,
  nav2_core::GlobalPlanner)
