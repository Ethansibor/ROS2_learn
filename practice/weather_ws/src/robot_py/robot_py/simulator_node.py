import rclpy
from rclpy.node import Node
from interfaces.msg import RobotStatus
from interfaces.srv import ChargeRobot

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator_py')

        # 1. 声明参数 (默认电量 100.0)
        self.declare_parameter('initial_battery_level',100.0)

        # 获取初始参数值
        self.battery_level = self.get_parameter('initial_battery_level').value
        self.status = 'Idle'

        # 创建发布者 (发布机器人状态)
        self.publisher_ = self.create_publisher(RobotStatus,'robot_status',10)

        # 创建服务端 (提供充电服务)
        self.service_ = self.create_service(ChargeRobot,'charge_robot',self.charge_callback)

        # 创建定时器 (2Hz = 0.5s) 模拟消耗电量
        self.timer_ = self.create_timer(0.5,self.timer_callback)

        self.get_logger().info(f'机器人启动! 初始电量: {self.battery_level}%')

    def timer_callback(self):
        if self.status == "Charging":
            if self.battery_level == 100:
                self.status = "Idle"
        elif self.status != "Dead": # 只有没死的时候才耗电
            # 正常耗电逻辑
            self.battery_level -= 0.5
            self.status = "Moving" if self.battery_level > 0 else "Dead"
        
        # 低电量逻辑
        if self.battery_level <= 0.0:
            self.battery_level = 0.0
            self.status = "Dead"
        elif self.battery_level < 20.0 and self.status != "Charging":
            self.status = "Low Battery"

        # 发布状态
        msg = RobotStatus()
        msg.battery_level = float(self.battery_level)
        msg.status = self.status
        self.publisher_.publish(msg)

    def charge_callback(self,request,response):
        # 收到充电请求，瞬间充满 (模拟)
        self.get_logger().info("收到充电请求...")
        self.battery_level = 100.0
        self.status = "Charging"

        response.success = True
        response.message = "机器人电量已充满!"
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    rclpy.spin(node)
    rclpy.shutdown()
