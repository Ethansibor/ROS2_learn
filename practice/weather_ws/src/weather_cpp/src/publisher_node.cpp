#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/weather.hpp" //包含自定义消息的头文件
//使用时间单位字面量，在代码中用s和ms表示时间
using namespace std::chrono_literals;

class WeatherPublisher : public rclcpp::Node {
public:
    WeatherPublisher() : Node("weather_publisher_cpp") {
        publisher_ = this->create_publisher<interfaces::msg::Weather>("weather_forecast",10);
        timer_ = this->create_wall_timer(1s,std::bind(&WeatherPublisher::timer_callback,this));

        // 模拟的天气数据
        forecasts_ = {
            {"北京", 15, "晴"},
            {"上海", 18, "多云"},
            {"广州", 22, "小雨"}
        };
    }
private:
    void timer_callback() {
        auto msg = interfaces::msg::Weather();
        const auto & forecasts = forecasts_[i_];

        msg.city = std::get<0>(forecasts);
        msg.temperature = std::get<1>(forecasts);
        msg.description = std::get<2>(forecasts);

        publisher_ -> publish(msg);
        RCLCPP_INFO(this->get_logger(),"发布天气: '%s', %d度, '%s'",msg.city.c_str(),msg.temperature,msg.description.c_str());
        i_ = (i_ + 1) % forecasts_.size();
    }
    rclcpp::Publisher<interfaces::msg::Weather>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::tuple<std::string,int8_t,std::string>> forecasts_;
    size_t i_{0};
};

int main(int argc,char* argv[]) {
    rclcpp::init(argc,argv);
    auto node = std::make_shared<WeatherPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
