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
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，正在等待...')

        self.req = SetThreshold.Request()

    def send_request(self,threshold):
        self.req.threshold = threshold
        self.future = self.client.call_async(self.req)
        # “请发送这个服务请求，但不要卡在这里等它回来”。call_async 会立即返回一个 Future对象（一个“未来的承诺”），
        # 然后程序继续往下执行
        self.get_logger().info(f'发送请求：设置阈值为{threshold}°C')

def main(args=None):
    rclpy.init(args=args)
    client_node = ThresholdClient()
    
    try:
        threshold_value = int(sys.argv[1])
    except (IndexError,ValueError):
        threshold_value = 15

    client_node.send_request(threshold_value)

    # 等待服务响应
    while rclpy.ok():
        rclpy.spin_once(client_node)
        if client_node.future.done():
            try:
                response = client_node.future.result()
                if response.success:
                    client_node.get_logger().info('服务调用成功：阈值已设置')
                else:
                    client_node.get_logger().error('服务调用失败')
            except Exception as e:
                client_node.get_logger().error(f'服务调用异常:{e}')
            break

    client_node.destroy_node()
    rclpy.shutdown()

