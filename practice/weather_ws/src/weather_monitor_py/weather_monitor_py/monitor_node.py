import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from interfaces.msg import Weather
from interfaces.srv import SetThreshold

class WeatherMonitor(Node):

    def __init__(self):
        super().__init__('weather_monitor_py')

        # 1. 声明参数
        self.declare_parameter('temperature_threshold',20)

        # 2. 创建订阅者
        self.subscription = self.create_subscription(
            Weather,
            '/weather_forecast',
            self.listener_callback,
            10
        )

        # 3. 创建服务端
        self.srv = self.create_service(
            SetThreshold,
            'set_weather_threshold',
            self.set_threshold_callback
        )
        self.get_logger().info('天气监控节点已启动，默认阈值：20')

    def listener_callback(self,msg):
        current_threshold = self.get_parameter('temperature_threshold').value

        if msg.temperature < current_threshold:
            self.get_logger().warn(
                f"低温警报！{msg.city}:{msg.temperature}°C (阈值：{current_threshold}°C)"
            )

    def set_threshold_callback(self,request,response):
        # 服务回调：设置新参数
        new_threshold = request.threshold

        # 1.
        # 设置参数
        param_to_set = rclpy.parameter.Parameter(
            'temperature_threshold',
            rclpy.Parameter.Type.INTEGER,
            new_threshold
        )
        # 2.
        self.set_parameters([param_to_set])

        # self.set_parameters([rclpy.Parameter('temperature_threshold',
        #     rclpy.Parameter.Type.INTEGER,
        #     new_threshold)])

        response.success = True
        self.get_logger().info(f"温度阈值已更新为: {new_threshold}°C")

        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = WeatherMonitor()
    rclpy.spin(node)
    rclpy.shutdown()
