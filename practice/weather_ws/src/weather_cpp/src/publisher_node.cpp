#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/weather.hpp" //包含自定义消息的头文件
//使用时间单位字面量，在代码中用s和ms表示时间
using namespace std::chrono_literals;

class WeatherPublisher : public rclcpp:Node {
public:
    WeatherPublisher() : Node("weather_publisher_cpp") {
        publisher_ = this->create_publisher<interfaces::msg::Weather>("weather_forecast",10)
        timer_ = this->create_wall_timer(1s,std::bind(&WeatherPublisher::timer_callback,this));

        // 模拟的天气数据
        forecasts_ = {
            {"北京", 15, "晴"},
            {"上海", 18, "多云"},
            {"广州", 22, "小雨"}
        };
    }
}