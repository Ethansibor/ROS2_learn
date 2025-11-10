#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/weather.hpp" // 包含自定义消息的头文件

using std::placeholders::_1;

class WeatherSubscriber : public rclcpp::Node {
public:
    WeatherSubscriber() : Node("weather_subscriber_cpp") {
        subscription_ = this->create_subscription<interfaces::msg::Weather>(
            "weather_forecast",10,
            std::bind(&WeatherSubscriber::topic_callback,this,_1));
    }
private:
    void topic_callback(const interfaces::msg::Weather::SharedPtr msg) const{ //常量方法
        RCLCPP_INFO(this->get_logger(), "收到天气: '%s', %d度, '%s'",
            msg->city.c_str(), 
            msg->temperature, 
            msg->description.c_str()
        );        

    }
    rclcpp::Subscription<interfaces::msg::Weather>::SharedPtr subscription_;


};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WeatherSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}