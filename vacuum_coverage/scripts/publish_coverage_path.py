#!/usr/bin/env python3
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from std_srvs.srv import Trigger


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class CoveragePathPublisher(Node):
    def __init__(self):
        super().__init__('coverage_path_publisher')
        self.declare_parameter('yaml_path', 'coverage_path.yaml')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/coverage_plan')
        yaml_path = self.get_parameter('yaml_path').value
        frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value

        self.computed_path = self.load_path(yaml_path, frame_id)
        if self.computed_path is None:
            self.get_logger().error(f'Failed to load {yaml_path}')
            return

        self.path_pub = self.create_publisher(Path, topic, 1)
        self._pub_count = 0
        self._pub_timer = self.create_timer(1.0, self.publish_callback)

        self.follow_client = ActionClient(
            self, FollowPath, '/controller_server/follow_path')
        self.srv = self.create_service(
            Trigger, '/coverage_follow_path', self.follow_path_callback)

        self.get_logger().info(
            f'Loaded {len(self.computed_path.poses)} poses — '
            f'call /coverage_follow_path to start')

    def load_path(self, yaml_path, frame_id):
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        pts = data.get('coverage_path')
        if not pts:
            return None
        path = Path()
        path.header.frame_id = frame_id
        n = len(pts)
        for i, pt in enumerate(pts):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(pt['x'])
            pose.pose.position.y = float(pt['y'])
            if i < n - 1:
                dx = pts[i + 1]['x'] - pt['x']
                dy = pts[i + 1]['y'] - pt['y']
                yaw = math.atan2(dy, dx)
            else:
                yaw = last_yaw
            last_yaw = yaw
            q = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)
        return path

    def publish_callback(self):
        self.computed_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.computed_path)
        self._pub_count += 1
        if self._pub_count >= 3:
            self.get_logger().info('Published path to ' +
                                   self.get_parameter('topic').value)
            self.destroy_timer(self._pub_timer)
            del self._pub_timer

    def follow_path_callback(self, request, response):
        if self.computed_path is None:
            response.success = False
            response.message = 'No path loaded'
            return response
        if not self.follow_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = 'FollowPath action server not available'
            return response
        goal = FollowPath.Goal()
        goal.path = self.computed_path
        send_future = self.follow_client.send_goal_async(goal)
        send_future.add_done_callback(self.follow_goal_response_callback)
        response.success = True
        response.message = 'FollowPath goal sent'
        self.get_logger().info('Sent FollowPath goal to controller_server')
        return response

    def follow_goal_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('FollowPath goal was rejected')
        else:
            self.get_logger().info('FollowPath goal accepted — robot is navigating')


def main():
    rclpy.init()
    node = CoveragePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
