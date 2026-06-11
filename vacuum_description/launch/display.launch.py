import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    robot_description_path = os.path.join(
        get_package_share_directory('vacuum_description'),
        'urdf', 'vacuum.urdf.xacro'
    )

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', robot_description_path
    ])

    rviz_config = os.path.join(
        get_package_share_directory('vacuum_description'),
        'config', 'vacuum.rviz'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
