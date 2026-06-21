#include "vacuum_coverage/map_to_graph.hpp"

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace vacuum_coverage
{

class MapToGraphNode : public rclcpp::Node
{
public:
  MapToGraphNode()
  : Node("map_to_graph_node")
  {
    double cell_size = declare_parameter("cell_size", 0.3);
    int occupancy_threshold = declare_parameter("occupancy_threshold", 50);
    double free_threshold_ratio = declare_parameter("free_threshold_ratio", 0.9);
    std::string map_topic = declare_parameter("map_topic", "/map");
    bool auto_plan = declare_parameter("auto_plan", false);

    converter_ = std::make_shared<MapToGraph>(cell_size, occupancy_threshold, free_threshold_ratio);
    auto_plan_ = auto_plan;

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local(),
      std::bind(&MapToGraphNode::map_callback, this, std::placeholders::_1));

    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/graph_visualization", rclcpp::QoS(1).transient_local());

    processed_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/processed_map", rclcpp::QoS(1).transient_local());

    replan_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/replan",
      std::bind(&MapToGraphNode::replan_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "MapToGraphNode started. cell_size=%.2f, threshold=%d, free_ratio=%.2f, auto_plan=%s",
      cell_size, occupancy_threshold, free_threshold_ratio, auto_plan ? "true" : "false");
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    last_map_ = msg;
    if (auto_plan_) {
      convert_and_publish();
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
    convert_and_publish();
    response->success = true;
    response->message = "Graph rebuilt and published";
  }

  void convert_and_publish()
  {
    if (!last_map_) {
      return;
    }

    Graph graph = converter_->convert(*last_map_);
    RCLCPP_INFO(get_logger(), "Graph built: %d nodes, %d edges",
      graph.node_count(), graph.edge_count());

    publish_graph_visualization(graph, last_map_->header.frame_id);
    publish_processed_map(*last_map_, graph);
  }

  void publish_graph_visualization(const Graph & graph, const std::string & frame_id)
  {
    visualization_msgs::msg::MarkerArray markers;

    std::string fid = frame_id.empty() ? "map" : frame_id;

    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = fid;
    node_marker.header.stamp = now();
    node_marker.ns = "nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.scale.x = converter_->cell_size();
    node_marker.scale.y = converter_->cell_size();
    node_marker.scale.z = 0.02;
    node_marker.color.a = 0.8;
    node_marker.color.g = 1.0;

    for (int i = 0; i < graph.node_count(); ++i) {
      const auto & node = graph.get_node(i);
      if (node.is_occupied) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = node.world_x;
      p.y = node.world_y;
      p.z = 0.0;
      node_marker.points.push_back(p);
    }
    markers.markers.push_back(node_marker);

    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = node_marker.header.frame_id;
    edge_marker.header.stamp = now();
    edge_marker.ns = "edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.scale.x = 0.02;
    edge_marker.color.a = 0.5;
    edge_marker.color.b = 1.0;
    edge_marker.color.r = 1.0;

    for (int i = 0; i < graph.edge_count(); ++i) {
      const auto & edge = graph.get_edges()[i];
      const auto & from = graph.get_node(edge.from);
      const auto & to = graph.get_node(edge.to);

      geometry_msgs::msg::Point p1;
      p1.x = from.world_x;
      p1.y = from.world_y;
      p1.z = 0.0;
      edge_marker.points.push_back(p1);

      geometry_msgs::msg::Point p2;
      p2.x = to.world_x;
      p2.y = to.world_y;
      p2.z = 0.0;
      edge_marker.points.push_back(p2);
    }
    markers.markers.push_back(edge_marker);

    viz_pub_->publish(markers);
  }

  void publish_processed_map(const nav_msgs::msg::OccupancyGrid & original, const Graph & graph)
  {
    nav_msgs::msg::OccupancyGrid out;
    out.header = original.header;
    out.info.resolution = converter_->cell_size();
    out.info.width = converter_->decomposed_width();
    out.info.height = converter_->decomposed_height();
    out.info.origin = original.info.origin;

    out.data.resize(out.info.width * out.info.height, -1);

    for (int dy = 0; dy < static_cast<int>(out.info.height); ++dy) {
      for (int dx = 0; dx < static_cast<int>(out.info.width); ++dx) {
        if (graph.has_node_at(dx, dy)) {
          out.data[dy * out.info.width + dx] = 0;
        } else {
          out.data[dy * out.info.width + dx] = 100;
        }
      }
    }

    processed_map_pub_->publish(out);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr processed_map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_srv_;
  std::shared_ptr<MapToGraph> converter_;
  bool auto_plan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
};

}  // namespace vacuum_coverage

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vacuum_coverage::MapToGraphNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
