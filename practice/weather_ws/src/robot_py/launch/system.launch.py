from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot_py',
            executable='simulator',
            name='my_robot_sim',# 重命名节点
            output='screen',
            parameters=[{'initial_battery_level':50.0}] # <--- 重点：传递参数
        ),
        Node(
            package='robot_py',
            executable='console',
            name='my_console',
            output='screen',
        ),
    ])