import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r xmate.sdf'}.items(),
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/rotor1@std_msgs/msg/Float64@gz.msgs.Double',
                   '/rotor2@std_msgs/msg/Float64@gz.msgs.Double',
                   '/rotor3@std_msgs/msg/Float64@gz.msgs.Double',
                   '/rotor4@std_msgs/msg/Float64@gz.msgs.Double',
                   '/rotor5@std_msgs/msg/Float64@gz.msgs.Double',
                   '/rotor6@std_msgs/msg/Float64@gz.msgs.Double',
                   '/rotor7@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    return LaunchDescription([
        gz_sim,
        bridge
    ])
