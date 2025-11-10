import rclpy
from rclpy.node import Node
from interfaces.srv import SetThreshold
import sys

class ThresholdClient(Node):
    def __init__(self):
        super().__init__('threshold_client_py')
        # 创建客户端
        self.client = self.create_client(SetThreshold,'set_weather_threshold')

        # 等待服务可用