import rclpy
from rclpy.node import Node
from interfaces.msg import RobotStatus
from interfaces.srv import ChargeRobot

class ControlConsole(Node):
    def __init__(self):
        super().__init__('control_console_py')

        #
        self.subscription_ = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.topic_callback,
            10
        )

        self.client = self.create_client(ChargeRobot,'charge_robot')

        self.get_logger().info("控制台就绪，正在监控机器人...")

    def topic_callback(self,msg):
        self.get_logger().info(f'状态: [{msg.status}] 电量: {msg.battery_level:.1f}%')

        #
        if msg.battery_level < 25.0 and msg.status != "Charging":
            self.get_logger().warn("电量过低! 尝试请求充电...")
            self.call_charge_service()

    def call_charge_service(self):
        #
        if not self.client.service_is_ready():
            self.get_logger().error("充电桩(服务)未上线!")
            return
        
        #
        request = ChargeRobot.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.charge_response_callback)

    def charge_response_callback(self,future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"充电成功: {response.message}")
            else:
                self.get_logger().error("充电失败!")
        except Exception as e:
            self.get_logger().error(f"服务调用异常: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlConsole()
    rclpy.spin(node)
    rclpy.shutdown()