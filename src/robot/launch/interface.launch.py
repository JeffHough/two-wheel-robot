from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot',
            executable='ros_interface',
            namespace="",
            name='my_node',
            shell=True,
        ),
        Node(
            package='robot',
            executable='serial_write.py',
            shell=True
        ),
        Node(
            package='robot',
            executable='state_holder.py',
            shell=True
        ),
    ])
