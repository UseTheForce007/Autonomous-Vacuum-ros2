import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world', default_value='simple_room',
        description='World: simple_room, multi_room, static_obstacles, dynamic_obstacles'
    )

    robot_description_path = os.path.join(
        get_package_share_directory('vacuum_description'),
        'urdf', 'vacuum.urdf.xacro'
    )

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', robot_description_path
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f'config_file:={os.path.join(get_package_share_directory("vacuum_gazebo"), "config", "bridge.yaml")}'],
        output='screen'
    )

    def launch_actions(context):
        world_name = context.launch_configurations['world']
        world_path = os.path.join(
            get_package_share_directory('vacuum_gazebo'),
            'worlds', f'{world_name}.world'
        )

        gz_sim = ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '1', world_path],
            output='screen'
        )

        spawn_x = '5' if world_name == 'simple_room' else '8'

        spawn_robot = ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-topic', '/robot_description',
                 '-entity', 'vacuum',
                 '-x', spawn_x, '-y', '0', '-z', '0.05'],
            output='screen'
        )

        actions = [gz_sim, spawn_robot]

        if world_name == 'dynamic_obstacles':
            move_obstacle = ExecuteProcess(
                cmd=['gz', 'topic', '-t', '/model/moving_obstacle/cmd_vel',
                     '-m', 'gz.msgs.Twist',
                     '-p', 'linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}',
                     '--rate', '2'],
                output='screen'
            )
            actions.append(move_obstacle)

        return actions

    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_actions),
        robot_state_publisher,
        bridge
    ])
