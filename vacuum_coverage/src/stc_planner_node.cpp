#include "vacuum_coverage/map_to_graph.hpp"
#include "vacuum_coverage/stc_planner.hpp"

#include <memory>
#include <string>
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

class STCPlannerNode : public rclcpp::Node
{
public:
  STCPlannerNode()
  : Node("stc_planner_node")
  {
    double cell_size = declare_parameter("cell_size", 0.3);
    int occupancy_threshold = declare_parameter("occupancy_threshold", 50);
    double free_threshold_ratio = declare_parameter("free_threshold_ratio", 0.9);
    std::string map_topic = declare_parameter("map_topic", "/map");
    double start_x = declare_parameter("start_x", 0.0);
    double start_y = declare_parameter("start_y", 0.0);
    bool auto_plan = declare_parameter("auto_plan", false);

    converter_ = std::make_shared<MapToGraph>(cell_size, occupancy_threshold, free_threshold_ratio);
    planner_ = std::make_shared<STCPlanner>();

    start_x_ = start_x;
    start_y_ = start_y;
    auto_plan_ = auto_plan;

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local(),
      std::bind(&STCPlannerNode::map_callback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/coverage_path", rclcpp::QoS(1).transient_local());

    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/stc_visualization", rclcpp::QoS(1).transient_local());

    replan_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/replan",
      std::bind(&STCPlannerNode::replan_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "STCPlannerNode started. cell_size=%.2f, start=(%.2f, %.2f), auto_plan=%s",
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
      RCLCPP_WARN(get_logger(), "Replan requested but no map stored");
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
    RCLCPP_INFO(get_logger(), "Graph built: %d nodes, %d edges",
      graph.node_count(), graph.edge_count());

    if (graph.node_count() == 0) {
      RCLCPP_WARN(get_logger(), "No free nodes in graph, skipping coverage planning");
      return;
    }

    nav_msgs::msg::Path path = planner_->plan(
      graph, start_x_, start_y_, last_map_->header.frame_id);

    RCLCPP_INFO(get_logger(), "Coverage path generated: %zu poses",
      path.poses.size());

    path.header.stamp = now();
    path_pub_->publish(path);

    publish_stc_visualization(graph, path, last_map_->header.frame_id);
  }

  void publish_stc_visualization(
    const Graph & graph,
    const nav_msgs::msg::Path & path,
    const std::string & frame_id)
  {
    visualization_msgs::msg::MarkerArray markers;

    std::string fid = frame_id.empty() ? "map" : frame_id;

    visualization_msgs::msg::Marker tree_marker;
    tree_marker.header.frame_id = fid;
    tree_marker.header.stamp = now();
    tree_marker.ns = "spanning_tree";
    tree_marker.id = 0;
    tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree_marker.action = visualization_msgs::msg::Marker::ADD;
    tree_marker.scale.x = 0.03;
    tree_marker.color.a = 0.8;
    tree_marker.color.g = 1.0;

    const auto & tree = planner_->get_spanning_tree();
    for (const auto & edge : tree) {
      const auto & from = graph.get_node(edge.first);
      const auto & to = graph.get_node(edge.second);

      geometry_msgs::msg::Point p1;
      p1.x = from.world_x;
      p1.y = from.world_y;
      p1.z = 0.0;
      tree_marker.points.push_back(p1);

      geometry_msgs::msg::Point p2;
      p2.x = to.world_x;
      p2.y = to.world_y;
      p2.z = 0.0;
      tree_marker.points.push_back(p2);
    }
    markers.markers.push_back(tree_marker);

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
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;

    if (!path.poses.empty()) {
      start_marker.pose.position.x = path.poses.front().pose.position.x;
      start_marker.pose.position.y = path.poses.front().pose.position.y;
      start_marker.pose.position.z = 0.0;
    }
    markers.markers.push_back(start_marker);

    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = fid;
    path_marker.header.stamp = now();
    path_marker.ns = "path";
    path_marker.id = 2;
    path_marker.type = visualization_msgs::msg::Marker::ARROW;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = converter_->cell_size() * 0.3;
    path_marker.scale.y = converter_->cell_size() * 0.15;
    path_marker.scale.z = converter_->cell_size() * 0.15;
    path_marker.color.a = 0.9;
    path_marker.color.b = 1.0;
    path_marker.color.r = 1.0;

    if (path.poses.size() >= 2) {
      path_marker.pose.position.x = path.poses[1].pose.position.x;
      path_marker.pose.position.y = path.poses[1].pose.position.y;
      path_marker.pose.position.z = 0.0;

      double dx = path.poses[1].pose.position.x - path.poses[0].pose.position.x;
      double dy = path.poses[1].pose.position.y - path.poses[0].pose.position.y;
      double yaw = std::atan2(dy, dx);
      path_marker.pose.orientation.z = std::sin(yaw * 0.5);
      path_marker.pose.orientation.w = std::cos(yaw * 0.5);
    }
    markers.markers.push_back(path_marker);

    viz_pub_->publish(markers);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_srv_;
  std::shared_ptr<MapToGraph> converter_;
  std::shared_ptr<STCPlanner> planner_;
  double start_x_;
  double start_y_;
  bool auto_plan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
};

}  // namespace vacuum_coverage

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vacuum_coverage::STCPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
