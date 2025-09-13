from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to URDF
    urdf_file = os.path.join(
        get_package_share_directory('my_rb1_description'),
        'urdf',
        'my_rb1_robot.urdf'
    )

    # Load URDF contents
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Path to RViz config
    rviz_config_file = os.path.join(
        get_package_share_directory('my_rb1_description'),
        'rviz',
        'robot.rviz'
    )

    return LaunchDescription([
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        # RViz2 with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  # 👈 load robot.rviz
        ),
    ])
