#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/robot_status.hpp"
#include "interfaces/srv/charge_robot.hpp"

using std::placeholders::_1;

class ControlConsole : public rclcpp::Node{
public:
    ControlConsole() : Node("control_console_cpp"){
        //1.订阅状态
        subscription_ = this->create_subscription<interfaces::msg::RobotStatus>(
            "robot_status",10,std::bind(&ControlConsole::topic_callback,this,_1));
        //2.创建客户端
        client_ = this->create_client<interfaces::srv::ChargeRobot>("charge_robot");

        RCLCPP_INFO(this->get_logger(),"控制台就绪...");

    }
private:
    void topic_callback(const interfaces::msg::RobotStatus::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "状态: [%s] 电量: %.1f%%", 
                msg->status.c_str(), msg->battery_level);

            // 自动充电逻辑
            if (msg->battery_level < 25.0 && msg->status != "Charging") {
                // 防止重复发送请求 (简单的防抖动)
                if (!is_charging_requested_) {
                    RCLCPP_WARN(this->get_logger(), "电量过低! 尝试请求充电...");
                    call_charge_service();
                }
            } else {
                is_charging_requested_ = false; // 重置标志位
            }
        }

    void call_charge_service() {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "充电桩未上线!");
            return;
        }

        auto request = std::make_shared<interfaces::srv::ChargeRobot::Request>();
        
        // 标记已发送请求
        is_charging_requested_ = true;

        // 异步发送请求
        // 注意：我们在回调里不能使用 spin，所以必须使用异步回调来处理结果
        auto future_result = client_->async_send_request(request, 
            [this](rclcpp::Client<interfaces::srv::ChargeRobot>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "充电成功: %s", response->message.c_str());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "充电失败");
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "服务调用异常: %s", e.what());
                }
            });
    }

    rclcpp::Subscription<interfaces::msg::RobotStatus>::SharedPtr subscription_;
    rclcpp::Client<interfaces::srv::ChargeRobot>::SharedPtr client_;
    bool is_charging_requested_{false};

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlConsole>());
    rclcpp::shutdown();
    return 0;
}