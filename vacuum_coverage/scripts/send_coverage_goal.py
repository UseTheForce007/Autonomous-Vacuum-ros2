#!/usr/bin/env python3

import os

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

import yaml

from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import (
    Coordinate, Coordinates,
    HeadlandMode, SwathMode, RowSwathMode, RouteMode, PathMode,
)
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger


class CoveragePathClient(Node):

    def __init__(self):
        super().__init__('coverage_path_client')

        self.declare_parameter('room_polygons_path', '')
        room_polygons_path = self.get_parameter('room_polygons_path').value
        self.declare_parameter('map_file_path', '')
        map_file_path = self.get_parameter('map_file_path').value
        self.declare_parameter('obstacle_buffer_cells', 0)
        obstacle_buffer = self.get_parameter('obstacle_buffer_cells').value
        self.declare_parameter('min_obstacle_area', 0.03)
        min_obstacle_area = self.get_parameter('min_obstacle_area').value

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(Path, '~/computed_coverage_path', qos)
        self.viz_pub = self.create_publisher(
            MarkerArray, '~/polygon_viz', qos)

        self.timer = self.create_timer(1.0, self.republish_all)

        self.srv = self.create_service(Trigger, '~/follow_path', self.follow_path_callback)

        self.coverage_client = ActionClient(
            self, ComputeCoveragePath, '/compute_coverage_path')
        self.follow_client = ActionClient(
            self, FollowPath, '/controller_server/follow_path')

        self.computed_path = None
        self.polygon_points = []
        self.obstacle_rings = []
        self.obstacle_viz_points = []

        polygons = self.read_room_polygons(room_polygons_path)
        if not polygons:
            return

        if map_file_path:
            rings = self.load_obstacle_rings(
                map_file_path, obstacle_buffer, min_obstacle_area)
            if rings:
                polygons.extend(rings)

        self.publish_markers()

        self.get_logger().info('Waiting for compute_coverage_path action server...')
        if not self.coverage_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                'Coverage server not available within timeout. '
                'Is coverage_server running?')
            return

        self.send_coverage_goal(polygons)

    def read_room_polygons(self, path):
        if not path:
            self.get_logger().error('room_polygons_path parameter is empty!')
            return None

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to read {path}: {e}')
            return None

        rooms = data.get('rooms', [])
        if not rooms:
            self.get_logger().error('No rooms found in YAML file')
            return None

        coords_list = []
        for room_entry in rooms:
            for room_name, points in room_entry.items():
                coords = Coordinates()
                for pt in points:
                    c = Coordinate()
                    c.axis1 = float(pt['x'])
                    c.axis2 = float(pt['y'])
                    coords.coordinates.append(c)

                if coords.coordinates and (
                    coords.coordinates[0].axis1 != coords.coordinates[-1].axis1 or
                    coords.coordinates[0].axis2 != coords.coordinates[-1].axis2
                ):
                    coords.coordinates.append(coords.coordinates[0])

                coords_list.append(coords)
                self.get_logger().info(
                    f'Loaded {room_name}: {len(coords.coordinates)} vertices')

                pts = []
                for c in coords.coordinates:
                    p = Point()
                    p.x = c.axis1
                    p.y = c.axis2
                    p.z = 0.0
                    pts.append(p)
                self.polygon_points = pts

        return coords_list

    def load_obstacle_rings(
        self, map_file_path, buffer_cells=0, min_area_m2=0.02
    ):
        try:
            with open(map_file_path, 'r') as f:
                meta = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to read {map_file_path}: {e}')
            return []

        resolution = meta['resolution']
        origin_x, origin_y = meta['origin'][0], meta['origin'][1]

        pgm_rel = meta['image']
        pgm_path = os.path.join(os.path.dirname(map_file_path), pgm_rel)

        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f'Failed to load {pgm_path}')
            return []

        h, w = img.shape

        # Occupied pixels: value == 0 (black walls/obstacles)
        _, occupied = cv2.threshold(img, 1, 255, cv2.THRESH_BINARY_INV)

        # Close small gaps in walls
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        occupied = cv2.morphologyEx(occupied, cv2.MORPH_CLOSE, close_kernel)

        if buffer_cells > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT, (buffer_cells * 2 + 1, buffer_cells * 2 + 1))
            occupied = cv2.dilate(occupied, kernel)

        contours, hierarchy = cv2.findContours(
            occupied, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        min_area_px = min_area_m2 / (resolution * resolution)
        rings = []
        for i, contour in enumerate(contours):
            area_px = cv2.contourArea(contour)
            if area_px < min_area_px:
                continue

            # Compute hierarchy depth
            depth = 0
            parent = hierarchy[0][i][3]
            while parent != -1:
                depth += 1
                parent = hierarchy[0][parent][3]

            # Keep only internal walls at depth 2
            # (depth 0 = outer walls, depth 1 = free space, depth 2 = internal obstacles)
            if depth != 2:
                continue

            arclen = cv2.arcLength(contour, True)
            epsilon = 0.02 * arclen
            simplified = cv2.approxPolyDP(contour, epsilon, True)
            if len(simplified) < 3:
                continue

            coords = Coordinates()
            viz_pts = []
            for pt in simplified:
                px, py = pt[0][0], pt[0][1]
                wx = origin_x + (px * resolution)
                wy = origin_y + ((h - py) * resolution)

                c = Coordinate()
                c.axis1 = float(wx)
                c.axis2 = float(wy)
                coords.coordinates.append(c)

                p = Point()
                p.x = float(wx)
                p.y = float(wy)
                p.z = 0.0
                viz_pts.append(p)

            if coords.coordinates[0].axis1 != coords.coordinates[-1].axis1 or \
               coords.coordinates[0].axis2 != coords.coordinates[-1].axis2:
                coords.coordinates.append(coords.coordinates[0])
                viz_pts.append(viz_pts[0])

            rings.append(coords)
            self.obstacle_viz_points.append(viz_pts)
            self.get_logger().info(
                f'  Obstacle #{i}: area={area_px * resolution * resolution:.2f}m\u00b2, '
                f'{len(coords.coordinates)} vertices')

        self.get_logger().info(
            f'Loaded {len(rings)} internal obstacle rings from {pgm_path}')
        return rings

    def send_coverage_goal(self, polygons):
        goal_msg = ComputeCoveragePath.Goal()
        goal_msg.generate_headland = True
        goal_msg.generate_route = True
        goal_msg.generate_path = True
        goal_msg.use_gml_file = False
        goal_msg.polygons = polygons
        goal_msg.frame_id = 'map'

        goal_msg.headland_mode = HeadlandMode()
        goal_msg.headland_mode.mode = 'UNKNOWN'
        goal_msg.headland_mode.width = 0.25

        goal_msg.swath_mode = SwathMode()
        goal_msg.swath_mode.objective = 'LENGTH'
        goal_msg.swath_mode.mode = 'SET_ANGLE'
        goal_msg.swath_mode.best_angle = 0.0
        goal_msg.swath_mode.step_angle = 0.017453

        goal_msg.row_swath_mode = RowSwathMode()
        goal_msg.row_swath_mode.mode = 'UNKNOWN'
        goal_msg.row_swath_mode.skip_ids = []
        goal_msg.row_swath_mode.offset = 0.0

        goal_msg.route_mode = RouteMode()
        goal_msg.route_mode.mode = 'BOUSTROPHEDON'
        goal_msg.route_mode.spiral_n = 4
        goal_msg.route_mode.custom_order = []

        goal_msg.path_mode = PathMode()
        goal_msg.path_mode.mode = 'DUBIN'
        goal_msg.path_mode.continuity_mode = 'CONTINUOUS'
        goal_msg.path_mode.turn_point_distance = 0.1

        self.get_logger().info('Sending compute_coverage_path goal...')
        future = self.coverage_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Coverage goal was rejected!')
            return

        self.get_logger().info('Coverage goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.computed_path = result.nav_path
            self.path_pub.publish(self.computed_path)
            self.publish_markers()
            self.get_logger().info(
                f'Coverage path computed: {len(self.computed_path.poses)} poses, '
                f'planning time: {result.planning_time.nanosec / 1e9:.2f}s')
        else:
            self.get_logger().error(
                f'Coverage failed with error code: {result.error_code}')

    def publish_markers(self):
        markers = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        if self.polygon_points:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now_msg
            m.ns = 'room_polygon'
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.points = self.polygon_points + [self.polygon_points[0]]
            markers.markers.append(m)

        for i, pts in enumerate(self.obstacle_viz_points):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now_msg
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.04
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.points = pts
            markers.markers.append(m)

        if markers.markers:
            self.viz_pub.publish(markers)

    def republish_all(self):
        if self.computed_path is not None:
            self.path_pub.publish(self.computed_path)
        self.publish_markers()

    def follow_path_callback(self, request, response):
        if self.computed_path is None:
            response.success = False
            response.message = 'No coverage path computed yet'
            return response

        if not self.follow_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = 'FollowPath action server not available'
            return response

        goal = FollowPath.Goal()
        goal.path = self.computed_path

        self.get_logger().info('Sending FollowPath goal to controller_server...')
        send_future = self.follow_client.send_goal_async(goal)
        send_future.add_done_callback(self.follow_goal_response_callback)

        response.success = True
        response.message = 'FollowPath goal sent to controller'
        return response

    def follow_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('FollowPath goal was rejected by controller')
            return
        self.get_logger().info('FollowPath goal accepted, robot is navigating')


def main():
    rclpy.init()
    node = CoveragePathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
