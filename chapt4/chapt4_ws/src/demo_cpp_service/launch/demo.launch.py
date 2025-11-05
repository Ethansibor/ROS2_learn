import launch
import launch_ros

def generate_launch_description():
    # 因为launch工具在运行Python格式的启动脚本时，会在文件中搜索名称为generate_launch_description的函数

    # 创建参数声明action，用于解析launch命令后的参数
    action_declare_arg_max_speed = launch.actions.DeclareLaunchArgument('launch_max_speed',default_value='2.0')

    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_service', #功能包名字
        executable='turtle_control',#可执行文件名字
        # 使用launch中参数launch_max_speed值替换节点中的max_speed参数值
        parameters=[{'max_speed':launch.substitutions.LaunchConfiguration('launch_max_speed',default='2.0')}],
        output='screen',
        # output参数用于指定日志输出的位置，screen表示屏幕，log表示日志，both表示前两者同时输出。
    )
    action_node_patrol_client = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='patrol_client',
        output='log',
    )
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        output = 'both',
    )
    # 合成启动描述并返回
    launch_description = launch.LaunchDescription([
        action_node_turtle_control,
        action_node_patrol_client,
        action_node_turtlesim_node,
        action_declare_arg_max_speed
    ])
    return launch_description
    '''调用launch.LaunchDescription创建启动描述对
    象launch_description,并将其返回。launch工具在拿到启动描述对象后,会根据其内容完成
    启动。
    '''