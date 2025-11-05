import rclpy
from rclpy.node import Node
from interfaces.msg import Weather

class WeatherPublisher(Node):
    def __init__(self):
        super().__init__('weather_publisher_py')
        self.publishers_ = self.create_publisher(Weather,'weather_forecast',10) 
        #'weather_forecast' (相对名称) '/weather_forecast' (绝对名称)
        self.timer = self.create_timer(1.0,self.timer_callback)

        self.forecasts = [
            {'city': '北京', 'temp': 15, 'desc': '晴'},
            {'city': '上海', 'temp': 18, 'desc': '多云'},
            {'city': '广州', 'temp': 22, 'desc': '小雨'}
        ]

        self.i = 0

    def timer_callback(self):
        msg = Weather()
        msg.city = self.forecasts[self.i]['city']
        msg.temperature = self.forecasts[self.i]['temp']
        msg.description = self.forecasts[self.i]['desc']
        self.publishers_.publish(msg) 
        self.get_logger().info(f'发布天气: {msg.city}, {msg.temperature}度, {msg.description}')
        self.i = (self.i + 1) % len(self.forecasts)

def main(args=None):
    rclpy.init(args=args)
    node = WeatherPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


