#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/weather.hpp"
#include "interfaces/srv/set_threshold.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp" //用于参数回调的返回值

#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

class WeatherMonitor : public rclcpp::Node{
public:
    WeatherMonitor() : Node("weather_monitor_cpp"){
        // 1.声明参数并获取默认值
        this->declare_parameter<int>("temperature_threshold",20);
        current_threshold_ = this->get_parameter("temperature_threshold").as_int();

        // 2.创建订阅者
        subscription_ = this->create_subscription<interfaces::msg::Weather>("/weather_forecast",10,std::bind(&WeatherMonitor::listener_callback,this,_1));
        
        // 3.创建自定义服务（用于友好API）
        service_ = this->create_service<interfaces::srv::SetThreshold>("/set_weather_threshold",std::bind(&WeatherMonitor::set_threshold_callback,this,_1,_2));

        // 4.注册参数更新回调（用于响应所有参数变化）
        param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&WeatherMonitor::parameter_callback,this,_1));

        RCLCPP_INFO(this->get_logger(),"天气监控节点已启动，默认阈值：%d°C",current_threshold_);
    }

private:
    rclcpp::Subscription<interfaces::msg::Weather>::SharedPtr subscription_;
    rclcpp::Service<interfaces::srv::SetThreshold>::SharedPtr service_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    int current_threshold_; //缓存参数值，用于高效的话题回调

    //话题回调（订阅者）
    void listener_callback(const interfaces::msg::Weather::SharedPtr msg) const {
        if(msg->temperature < current_threshold_) {
            RCLCPP_WARN(this->get_logger(),"低温警报！%s:%d°C (阈值: %d°C)",msg->city.c_str(),msg->temperature,current_threshold_);
        }
    }

    void set_threshold_callback(const std::shared_ptr<interfaces::srv::SetThreshold::Request> request,std::shared_ptr<interfaces::srv::SetThreshold::Response> response)
    {
        // 这是“内部设置参数”的方式 (流程 C)
        // 我们创建 rclcpp::Parameter 对象
        auto param_to_set = rclcpp::Parameter("temperature_threshold",request->threshold);

        // 调用 set_parameters
        this->set_parameter({param_to_set});

        response->success = true;
        // 注意：因为我们注册了参数回调，set_parameters 会触发 parameter_callback
        // 所以我们不需要在这里打印日志或更新 current_threshold_
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true; //默认接受

        for(const auto &param : parameters) {
            if(param.get_name() == "temperature_threshold"){
                //更新我们缓存的成员变量
                current_threshold_ = param.as_int();
                RCLCPP_INFO(this->get_logger(),"温度阈值已更新为: %d°C", current_threshold_);
            }
        }
        return result;
    }

};

int main(int argc,char *argv[]) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<WeatherMonitor>());
    rclcpp::shutdown();
    return 0;
}