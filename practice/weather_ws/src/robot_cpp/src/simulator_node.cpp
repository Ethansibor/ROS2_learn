#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/robot_status.hpp"
#include "interfaces/srv/charge_robot.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class RobotSimulator : public rclcpp::Node{
public:
    RobotSimulator():Node("robot_simulator_cpp"){
        // 参数
        this->declare_parameter("initial_battery_level",100.0);
        battery_level_ = this->get_parameter("initial_battery_level").as_double();
        status_ = "Idle";
        // 发布者
        publisher_ = this->create_publisher<interfaces::msg::RobotStatus>("robot_status",10);

        // 服务端
        service_ = this->create_service<interfaces::srv::ChargeRobot>("charge_robot",std::bind(&RobotSimulator::charge_callback,this,_1,_2));

        // 定时器
        timer_ = this->create_wall_timer(500ms,std::bind(&RobotSimulator::timer_callback,this));

        RCLCPP_INFO(this->get_logger(),"机器人启动! 初始电量: %.1f%%", battery_level_);
    }



private:
    float battery_level_;
    std::string status_;
    rclcpp::Publisher<interfaces::msg::RobotStatus>::SharedPtr publisher_;
    rclcpp::Service<interfaces::srv::ChargeRobot>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    void charge_callback(const std::shared_ptr<interfaces::srv::ChargeRobot::Request> request,std::shared_ptr<interfaces::srv::ChargeRobot::Response>response){
        RCLCPP_INFO(this->get_logger(),"收到充电请求...");
        battery_level_ = 100.0;
        status_ = "Charging";

        response->success = "true";
        response->message = "机器人电量已充满!";
        // (request 参数在这里没有用到，但为了函数签名匹配必须保留)
    }
    void timer_callback(){
        if (status_ == "Charging") {
            if (battery_level_ >= 100.0) { // 浮点数比较推荐用 >=
                status_ = "Idle";
            }
        } 
        else if (status_ != "Dead") {
            battery_level_ -= 0.5;
            if (battery_level_ > 0) {
                status_ = "Moving";
            } 
            else {
                status_ = "Dead";
                battery_level_ = 0.0;
            }
        }
        // 额外的低电量状态标记 (非充电状态下)
        if (battery_level_ < 20.0 && status_ != "Charging" && status_ != "Dead") {
            status_ = "Low Battery";
        }

        //发布消息
        auto msg = interfaces::msg::RobotStatus();
        msg.battery_level = battery_level_;
        msg.status = status_;
        publisher_->publish(msg);
    }

};

int main(int argc,char * argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobotSimulator>();
    rclcpp::spin(node);
    return 0;
}