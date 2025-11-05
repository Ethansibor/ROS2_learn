import rclpy
from rclpy.node import Node
from interfaces.msg import Weather

class WeatherSubscriber(Node):
    def __init__(self):
        super().__init__('weather_subscriber_py')
        self.subscription_ = self.create_subscription(Weather,'weather_forecast',self.listener_callback,10)

    def listener_callback(self,msg):
        self.get_logger().info(f'收到天气: {msg.city}, {msg.temperature}度, {msg.description}')

def main(args=None):
    rclpy.init(args=args)
    node = WeatherSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()