import os

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_vacuum = 'vacuum_coverage'

    prefix = get_package_prefix(pkg_vacuum)
    extract_script = os.path.join(prefix, 'lib', pkg_vacuum, 'extract_poly.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'room_polygons_path', default_value='',
            description='Path to room_polygons.yaml'),
        DeclareLaunchArgument(
            'run_extract', default_value='false',
            description='Run extract_poly.py before launching coverage'),
        DeclareLaunchArgument(
            'map_yaml', default_value='',
            description='Path to map YAML (used if run_extract:=true)'),
        DeclareLaunchArgument(
            'map_pgm', default_value='',
            description='Path to map PGM (used if run_extract:=true)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'robot_width', default_value='0.35',
            description='Robot width in meters'),
        DeclareLaunchArgument(
            'operation_width', default_value='0.35',
            description='Operation/implement width in meters'),
        DeclareLaunchArgument(
            'min_turning_radius', default_value='0.18',
            description='Minimum turning radius in meters'),
        DeclareLaunchArgument(
            'map_file_path', default_value='',
            description='Path to map YAML for obstacle extraction (empty = no obstacles)'),

        LogInfo(
            condition=IfCondition(LaunchConfiguration('run_extract')),
            msg=['Extracting polygons from map...']),

        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('run_extract')),
            cmd=[
                'python3', extract_script,
                LaunchConfiguration('map_yaml'),
                LaunchConfiguration('map_pgm'),
            ],
            name='extract_poly',
            output='screen',
        ),

        Node(
            package='opennav_coverage',
            executable='opennav_coverage',
            name='coverage_server',
            output='screen',
            parameters=[{
                'coordinates_in_cartesian_frame': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_width': LaunchConfiguration('robot_width'),
                'operation_width': LaunchConfiguration('operation_width'),
                'min_turning_radius': LaunchConfiguration('min_turning_radius'),
            }],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_coverage',
            output='screen',
            parameters=[{
                'node_names': ['coverage_server'],
                'autostart': True,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        Node(
            package=pkg_vacuum,
            executable='send_coverage_goal.py',
            name='coverage_path_client',
            output='screen',
            parameters=[{
                'room_polygons_path': LaunchConfiguration('room_polygons_path'),
                'map_file_path': LaunchConfiguration('map_file_path'),
            }],
        ),
    ])
