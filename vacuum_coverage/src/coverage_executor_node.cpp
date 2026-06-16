#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace vacuum_coverage
{

using ActionT = nav2_msgs::action::FollowWaypoints;

class CoverageExecutorNode : public rclcpp::Node
{
public:
  CoverageExecutorNode()
  : Node("coverage_executor_node")
  {
    std::string waypoints_topic = declare_parameter("waypoints_topic", "/stc_planner_node/coverage_path");

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      waypoints_topic, rclcpp::QoS(1).transient_local(),
      std::bind(&CoverageExecutorNode::path_callback, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<ActionT>(this, "/follow_waypoints");

    replan_client_ = create_client<std_srvs::srv::Trigger>("/stc_planner/replan");

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/start_coverage",
      std::bind(&CoverageExecutorNode::start_coverage_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/execution_status", rclcpp::QoS(1).transient_local());

    progress_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/coverage_progress", rclcpp::QoS(1).transient_local());

    publish_status("idle");

    RCLCPP_INFO(get_logger(), "CoverageExecutorNode started. Waiting for path and /start_coverage trigger.");
  }

private:
  enum class State { IDLE, READY, EXECUTING, COMPLETE };

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_ = msg;
    if (state_ == State::IDLE || state_ == State::COMPLETE) {
      state_ = State::READY;
      publish_status("ready");
      RCLCPP_INFO(get_logger(), "Coverage path received: %zu poses.",
        msg->poses.size());

      if (auto_start_) {
        RCLCPP_INFO(get_logger(), "Auto-starting coverage...");
        auto_start_ = false;
        try_start_coverage();
      } else {
        RCLCPP_INFO(get_logger(), "Call /start_coverage to begin.");
      }
    } else {
      RCLCPP_WARN(get_logger(), "Path received while %s, ignoring.",
        state_string().c_str());
    }
  }

  void start_coverage_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (state_ == State::EXECUTING) {
      response->success = false;
      response->message = "Already executing";
      return;
    }

    if (latest_path_) {
      try_start_coverage();
      response->success = true;
      response->message = "Coverage started";
      return;
    }

    if (!replan_client_->wait_for_service(std::chrono::seconds(1))) {
      response->success = false;
      response->message = "STC planner replan service not available. Is stc_planner running?";
      RCLCPP_ERROR(get_logger(), "Replan service not available");
      return;
    }

    RCLCPP_INFO(get_logger(), "No path yet. Triggering replan and waiting...");
    auto_start_ = true;

    auto replan_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto replan_future = replan_client_->async_send_request(replan_request);

    if (replan_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
      auto replan_result = replan_future.get();
      if (replan_result->success) {
        response->success = true;
        response->message = "Replan triggered. Coverage will start when path arrives.";
      } else {
        auto_start_ = false;
        response->success = false;
        response->message = "Replan failed: " + replan_result->message;
      }
    } else {
      auto_start_ = false;
      response->success = false;
      response->message = "Replan timed out after 5s";
    }
  }

  void try_start_coverage()
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "FollowWaypoints action server not found");
      state_ = State::READY;
      publish_status("ready");
      return;
    }
    send_coverage_goal();
  }

  void send_coverage_goal()
  {
    auto goal = ActionT::Goal();
    goal.poses = latest_path_->poses;
    goal.number_of_loops = 1;

    RCLCPP_INFO(get_logger(), "Sending coverage goal with %zu waypoints",
      goal.poses.size());

    auto send_goal_options =
      rclcpp_action::Client<ActionT>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&CoverageExecutorNode::goal_response_callback, this,
        std::placeholders::_1);

    send_goal_options.feedback_callback =
      std::bind(&CoverageExecutorNode::feedback_callback, this,
        std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(&CoverageExecutorNode::result_callback, this,
        std::placeholders::_1);

    action_client_->async_send_goal(goal, send_goal_options);
  }

  void goal_response_callback(
    const rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Coverage goal was rejected by Nav2");
      state_ = State::READY;
      publish_status("ready");
      return;
    }
    RCLCPP_INFO(get_logger(), "Coverage goal accepted, executing...");
    state_ = State::EXECUTING;
    publish_status("executing");
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
    const std::shared_ptr<const ActionT::Feedback> feedback)
  {
    current_waypoint_ = feedback->current_waypoint;
    publish_coverage_progress();
  }

  void result_callback(
    const rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Coverage complete");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Coverage was aborted: %s",
          result.result->error_msg.c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Coverage was canceled");
        break;
      default:
        break;
    }

    const auto & missed = result.result->missed_waypoints;
    if (!missed.empty()) {
      RCLCPP_WARN(get_logger(), "%zu waypoints were missed", missed.size());
      for (const auto & wp : missed) {
        RCLCPP_WARN(get_logger(), "  Missed waypoint %u, error %u", wp.index, wp.error_code);
      }
    }

    state_ = State::COMPLETE;
    publish_status("complete");
    publish_coverage_progress();
  }

  void publish_status(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  void publish_coverage_progress()
  {
    if (!latest_path_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    std::string fid = latest_path_->header.frame_id.empty() ? "map" : latest_path_->header.frame_id;

    visualization_msgs::msg::Marker covered;
    covered.header.frame_id = fid;
    covered.header.stamp = now();
    covered.ns = "covered";
    covered.id = 0;
    covered.type = visualization_msgs::msg::Marker::CUBE_LIST;
    covered.action = visualization_msgs::msg::Marker::ADD;
    covered.scale.x = 0.25;
    covered.scale.y = 0.25;
    covered.scale.z = 0.02;
    covered.color.a = 0.8;
    covered.color.g = 1.0;

    visualization_msgs::msg::Marker remaining;
    remaining.header.frame_id = fid;
    remaining.header.stamp = now();
    remaining.ns = "remaining";
    remaining.id = 1;
    remaining.type = visualization_msgs::msg::Marker::CUBE_LIST;
    remaining.action = visualization_msgs::msg::Marker::ADD;
    remaining.scale.x = 0.25;
    remaining.scale.y = 0.25;
    remaining.scale.z = 0.02;
    remaining.color.a = 0.4;
    remaining.color.r = 0.5;
    remaining.color.g = 0.5;
    remaining.color.b = 0.5;

    int progress_idx = (state_ == State::EXECUTING) ? current_waypoint_ : latest_path_->poses.size();

    for (size_t i = 0; i < latest_path_->poses.size(); ++i) {
      geometry_msgs::msg::Point p;
      p.x = latest_path_->poses[i].pose.position.x;
      p.y = latest_path_->poses[i].pose.position.y;
      p.z = 0.0;

      if (static_cast<int>(i) < progress_idx) {
        covered.points.push_back(p);
      } else {
        remaining.points.push_back(p);
      }
    }

    markers.markers.push_back(covered);
    markers.markers.push_back(remaining);
    progress_pub_->publish(markers);
  }

  std::string state_string() const
  {
    switch (state_) {
      case State::IDLE:       return "idle";
      case State::READY:      return "ready";
      case State::EXECUTING:  return "executing";
      case State::COMPLETE:   return "complete";
    }
    return "unknown";
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr replan_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub_;

  nav_msgs::msg::Path::SharedPtr latest_path_;
  State state_ = State::IDLE;
  int current_waypoint_ = 0;
  bool auto_start_ = false;
};

}  // namespace vacuum_coverage

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vacuum_coverage::CoverageExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
