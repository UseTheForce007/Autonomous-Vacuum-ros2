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
    slam = LaunchConfiguration('slam', default='False')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='True')
    use_respawn = LaunchConfiguration('use_respawn', default='False')

    cell_size = LaunchConfiguration('cell_size', default='0.3')
    occupancy_threshold = LaunchConfiguration('occupancy_threshold', default='50')
    free_threshold_ratio = LaunchConfiguration('free_threshold_ratio', default='0.9')
    start_x = LaunchConfiguration('start_x', default='0.0')
    start_y = LaunchConfiguration('start_y', default='0.0')

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
            'cell_size', default_value='0.3',
            description='Decomposed cell side length for coverage graph'),
        DeclareLaunchArgument(
            'occupancy_threshold', default_value='50',
            description='Max occupancy value to consider a cell free'),
        DeclareLaunchArgument(
            'free_threshold_ratio', default_value='0.9',
            description='Min fraction of free sub-cells for a block to be free'),
        DeclareLaunchArgument(
            'start_x', default_value='0.0',
            description='Coverage start X in world coordinates'),
        DeclareLaunchArgument(
            'start_y', default_value='0.0',
            description='Coverage start Y in world coordinates'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'nav2_bringup.launch.py')),
            launch_arguments={
                'slam': slam,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
            }.items(),
        ),

        Node(
            package='vacuum_coverage',
            executable='map_to_graph_node',
            name='map_to_graph',
            output='screen',
            parameters=[{
                'cell_size': cell_size,
                'occupancy_threshold': occupancy_threshold,
                'free_threshold_ratio': free_threshold_ratio,
                'map_topic': '/map',
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='vacuum_coverage',
            executable='stc_planner_node',
            name='stc_planner',
            output='screen',
            parameters=[{
                'cell_size': cell_size,
                'occupancy_threshold': occupancy_threshold,
                'free_threshold_ratio': free_threshold_ratio,
                'map_topic': '/map',
                'start_x': start_x,
                'start_y': start_y,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='vacuum_coverage',
            executable='coverage_executor_node',
            name='coverage_executor',
            output='screen',
            parameters=[{
                'waypoints_topic': '/stc_planner/coverage_path',
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
