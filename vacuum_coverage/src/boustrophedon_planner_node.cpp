#include "vacuum_coverage/map_to_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace vacuum_coverage
{

struct Segment
{
  int y;
  int start_x;
  int end_x;
};

class BoustrophedonPlannerNode : public rclcpp::Node
{
public:
  BoustrophedonPlannerNode()
  : Node("boustrophedon_planner_node")
  {
    double cell_size = declare_parameter("cell_size", 0.3);
    int occupancy_threshold = declare_parameter("occupancy_threshold", 50);
    double free_threshold_ratio = declare_parameter("free_threshold_ratio", 0.9);
    std::string map_topic = declare_parameter("map_topic", "/map");
    double start_x = declare_parameter("start_x", 0.0);
    double start_y = declare_parameter("start_y", 0.0);
    bool auto_plan = declare_parameter("auto_plan", false);

    converter_ = std::make_shared<MapToGraph>(cell_size, occupancy_threshold, free_threshold_ratio);
    start_x_ = start_x;
    start_y_ = start_y;
    auto_plan_ = auto_plan;

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local(),
      std::bind(&BoustrophedonPlannerNode::map_callback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/coverage_path", rclcpp::QoS(1).transient_local());

    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/coverage_visualization", rclcpp::QoS(1).transient_local());

    replan_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/replan",
      std::bind(&BoustrophedonPlannerNode::replan_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "BoustrophedonPlannerNode started. cell_size=%.2f, start=(%.2f, %.2f), auto_plan=%s",
      cell_size, start_x, start_y, auto_plan ? "true" : "false");
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    last_map_ = msg;
    if (auto_plan_) {
      run_planner();
    }
  }

  void replan_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    if (!last_map_) {
      response->success = false;
      response->message = "No map received yet";
      return;
    }
    run_planner();
    response->success = true;
    response->message = "Coverage path re-planned and published";
  }

  void run_planner()
  {
    if (!last_map_) {
      return;
    }

    Graph graph = converter_->convert(*last_map_);
    RCLCPP_INFO(get_logger(), "Graph built: %d nodes, %d edges, grid=%dx%d",
      graph.node_count(), graph.edge_count(),
      converter_->decomposed_width(), converter_->decomposed_height());

    if (graph.node_count() == 0) {
      RCLCPP_WARN(get_logger(), "No free nodes, skipping planning");
      return;
    }

    nav_msgs::msg::Path path = plan_boustrophedon(graph);
    RCLCPP_INFO(get_logger(), "Coverage path: %zu poses", path.poses.size());

    path.header.stamp = now();
    path_pub_->publish(path);

    publish_visualization(graph, path);
  }

  nav_msgs::msg::Path plan_boustrophedon(const Graph & graph)
  {
    int width = converter_->decomposed_width();
    int height = converter_->decomposed_height();

    int start_id = find_nearest_node(graph, start_x_, start_y_);
    const auto & start_node = graph.get_node(start_id);

    std::string frame_id = last_map_->header.frame_id.empty() ? "map" : last_map_->header.frame_id;

    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;

    std::vector<int> row_order;
    for (int dy = 0; dy < height; ++dy) {
      int row = start_node.grid_y + ((dy % 2 == 0) ? -dy / 2 : (dy + 1) / 2);
      if (row >= 0 && row < height) {
        row_order.push_back(row);
      }
    }

    bool left_to_right = true;
    bool first_segment = true;

    for (int row_y : row_order) {
      auto segments = find_segments(row_y, graph, width);

      if (segments.empty()) {
        left_to_right = !left_to_right;
        continue;
      }

      if (left_to_right) {
        std::sort(segments.begin(), segments.end(),
          [](const Segment & a, const Segment & b) {
            return a.start_x < b.start_x;
          });
      } else {
        std::sort(segments.begin(), segments.end(),
          [](const Segment & a, const Segment & b) {
            return a.end_x > b.end_x;
          });
      }

      for (const auto & seg : segments) {
        if (first_segment) {
          bool found_start = (seg.y == start_node.grid_y &&
            seg.start_x <= start_node.grid_x &&
            start_node.grid_x <= seg.end_x);

          if (found_start) {
            if (left_to_right) {
              for (int x = start_node.grid_x; x <= seg.end_x; ++x) {
                add_pose(path, graph, x, seg.y, frame_id);
              }
            } else {
              for (int x = start_node.grid_x; x >= seg.start_x; --x) {
                add_pose(path, graph, x, seg.y, frame_id);
              }
            }
            first_segment = false;
            continue;
          }
        }

        if (left_to_right) {
          for (int x = seg.start_x; x <= seg.end_x; ++x) {
            add_pose(path, graph, x, seg.y, frame_id);
          }
        } else {
          for (int x = seg.end_x; x >= seg.start_x; --x) {
            add_pose(path, graph, x, seg.y, frame_id);
          }
        }
      }

      left_to_right = !left_to_right;
    }

    orient_path(path);
    return path;
  }

  void orient_path(nav_msgs::msg::Path & path)
  {
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
      double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
      double yaw = std::atan2(dy, dx);
      path.poses[i].pose.orientation.z = std::sin(yaw * 0.5);
      path.poses[i].pose.orientation.w = std::cos(yaw * 0.5);
    }
    if (path.poses.size() >= 2) {
      path.poses.back().pose.orientation =
        path.poses[path.poses.size() - 2].pose.orientation;
    }
  }

  std::vector<Segment> find_segments(int row_y, const Graph & graph, int width)
  {
    std::vector<Segment> segments;
    int seg_start = -1;

    for (int x = 0; x < width; ++x) {
      bool free = graph.has_node_at(x, row_y);
      if (free && seg_start < 0) {
        seg_start = x;
      } else if (!free && seg_start >= 0) {
        segments.push_back({row_y, seg_start, x - 1});
        seg_start = -1;
      }
    }
    if (seg_start >= 0) {
      segments.push_back({row_y, seg_start, width - 1});
    }

    return segments;
  }

  void add_pose(
    nav_msgs::msg::Path & path,
    const Graph & graph,
    int grid_x, int grid_y,
    const std::string & frame_id)
  {
    if (!graph.has_node_at(grid_x, grid_y)) {
      return;
    }
    const auto & node = graph.get_node(graph.node_id_at(grid_x, grid_y));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = node.world_x;
    pose.pose.position.y = node.world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  int find_nearest_node(const Graph & graph, double world_x, double world_y)
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

  void publish_visualization(const Graph & graph, const nav_msgs::msg::Path & path)
  {
    visualization_msgs::msg::MarkerArray markers;
    std::string fid = last_map_->header.frame_id.empty() ? "map" : last_map_->header.frame_id;

    visualization_msgs::msg::Marker sweep_marker;
    sweep_marker.header.frame_id = fid;
    sweep_marker.header.stamp = now();
    sweep_marker.ns = "sweep_path";
    sweep_marker.id = 0;
    sweep_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sweep_marker.action = visualization_msgs::msg::Marker::ADD;
    sweep_marker.scale.x = 0.04;
    sweep_marker.color.a = 0.9;
    sweep_marker.color.b = 1.0;
    sweep_marker.color.r = 1.0;

    for (const auto & pose : path.poses) {
      sweep_marker.points.push_back(pose.pose.position);
    }
    markers.markers.push_back(sweep_marker);

    visualization_msgs::msg::Marker start_marker;
    start_marker.header.frame_id = fid;
    start_marker.header.stamp = now();
    start_marker.ns = "start";
    start_marker.id = 1;
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.scale.x = converter_->cell_size() * 0.6;
    start_marker.scale.y = converter_->cell_size() * 0.6;
    start_marker.scale.z = converter_->cell_size() * 0.6;
    start_marker.color.a = 1.0;
    start_marker.color.g = 1.0;

    if (!path.poses.empty()) {
      start_marker.pose.position = path.poses.front().pose.position;
    }
    markers.markers.push_back(start_marker);

    viz_pub_->publish(markers);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_srv_;
  std::shared_ptr<MapToGraph> converter_;
  double start_x_;
  double start_y_;
  bool auto_plan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
};

}  // namespace vacuum_coverage

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vacuum_coverage::BoustrophedonPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
