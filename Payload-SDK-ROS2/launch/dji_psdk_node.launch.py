from launch import LaunchDescription
from launch_ros.actions import Node,DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
            package="dji_psdk_ros2",
            executable="dji_psdk_ros2",
            name="dji_psdk_node"
        )]
    )