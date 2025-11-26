from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_cpp', # <--- 改成 C++ 包名
            executable='simulator_node', # <--- 改成 C++ 可执行文件名
            name='my_robot_sim_cpp',
            output='screen',
            parameters=[{'initial_battery_level': 50.0}]
        ),
        Node(
            package='robot_cpp',
            executable='console_node',
            name='my_console_cpp',
            output='screen',
        ),
    ])