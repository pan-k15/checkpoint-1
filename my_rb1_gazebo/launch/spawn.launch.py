import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --------------------------
    # Package Directories
    # --------------------------
    description_pkg = get_package_share_directory("my_rb1_description")
    urdf_file = os.path.join(description_pkg, "urdf", "my_rb1_robot.urdf")

    # --------------------------
    # Launch Arguments for spawn position
    # --------------------------
    declare_spawn_x = DeclareLaunchArgument("x", default_value="0.0",
                                            description="Spawn X position")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0",
                                            description="Spawn Y position")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.5",
                                            description="Spawn Z position")

    # --------------------------
    # Robot State Publisher
    # --------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            )
        }]
    )

    # --------------------------
    # Spawn Robot (with small delay)
    # --------------------------
    spawn_robot = TimerAction(
        period=2.0,  # wait 2 seconds to ensure Gazebo is ready
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='my_robot_spawn',
            output='screen',
            arguments=[
                '-name', 'rb1',
                '-allow_renaming', 'true',
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
            ]
        )]
    )

    # --------------------------
    # Bridge Topics (ROS ↔ Gazebo Sim)
    # --------------------------
    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            # Clock (sim time)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Velocity command (ROS sends → Gazebo receives)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

            # TF (ROS publishes → Gazebo listens)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # Odometry (Gazebo publishes → ROS listens)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # Laser scan (Gazebo publishes → ROS listens)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ]
    )

    # --------------------------
    # Launch Description
    # --------------------------
    return LaunchDescription([
        # Launch arguments
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,

        # Start Robot State Publisher
        robot_state_publisher_node,

        # Spawn Robot
        spawn_robot,

        # Bridge Gazebo Sim <-> ROS 2 topics
        parameter_bridge_node,
    ])
