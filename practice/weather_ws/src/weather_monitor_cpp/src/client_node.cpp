#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/set_threshold.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc,char **argv) {
    rclcpp::init(argc,argv);

    // 我们只需要一个最小化的节点来进行客户端调用
    auto node = rclcpp::Node::make_shared("threshold_client_cpp");

    // 1.创建客户端
    auto client = node->create_client<interfaces::srv::SetThreshold>("set_weather_threshold");

    // 2.等待服务上线
    while(!client->wait_for_service(1s)){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(node->get_logger(),"客户端中断，退出...");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(),"服务不可用，正在等待...");
    }

    // 3.构造请求
    auto request = std::make_shared<interfaces::srv::SetThreshold::Request>();
    request->threshold = 10; // 我们将新阈值硬编码为 10

    // 4.异步发送并等待
    auto future = client->async_send_request(request);

    RCLCPP_INFO(node->get_logger(),"发送请求：设置阈值为10");

    if(rclcpp::spin_until_future_complete(node,future)== rclcpp::FutureReturnCode::SUCCESS){
        auto response = future.get();
        if(response->success){
            RCLCPP_INFO(node->get_logger(),"服务调用成功：阈值已设置");
        }
        else{
            RCLCPP_INFO(node->get_logger(),"服务调用失败");
        } 
    }
    else{
        RCLCPP_ERROR(node->get_logger(),"服务调用异常");
    }

    rclcpp::shutdown();
    return 0;

}
