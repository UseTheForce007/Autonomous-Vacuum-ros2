from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vacuum_coverage',
            executable='stc_planner_node',
            name='stc_planner',
            output='screen',
            parameters=[{
                'cell_size': 0.3,
                'occupancy_threshold': 50,
                'free_threshold_ratio': 0.9,
                'map_topic': '/map',
                'start_x': 0.0,
                'start_y': 0.0,
            }],
        )
    ])
