import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_coverage = get_package_share_directory('vacuum_coverage')
    pkg_nav = get_package_share_directory('vacuum_nav')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='True')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='True')
    use_respawn = LaunchConfiguration('use_respawn', default='False')
    map_file = LaunchConfiguration('map', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='Whether to run SLAM'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Whether to use composed bringup'),
        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Whether to respawn if a node crashes'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Full path to map yaml file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'nav2_bringup.launch.py')),
            launch_arguments={
                'slam': slam,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'map': map_file,
            }.items(),
        ),

        Node(
            package='vacuum_coverage',
            executable='coverage_executor_node',
            name='coverage_executor',
            output='screen',
            parameters=[{
                'waypoints_topic': '/planner_server/coverage_path',
                'replan_service': '/planner_server/replan',
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
