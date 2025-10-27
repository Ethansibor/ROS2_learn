#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono> //引入时间相关头文件

using namespace std::chrono_literals; //使用时间单位的字面量
using Patrol = chapt4_interfaces::srv::Patrol;
class PatrolClient : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Client<Patrol>::SharedPtr patrol_client_;
    public:
        PatrolClient() : Node("patrol_client")
        {
            patrol_client_ = this->create_client<Patrol>("patrol");
            timer_ = this->create_wall_timer(10s,std::bind(&PatrolClient::timer_callback,this));
            srand(time(NULL)); //初始化随即种子，使用当前时间做为种子

        }
        void timer_callback()
        {
            // TODO 生成随即目标点，请求服务端
            // 1. 等待服务端上线
            while (!patrol_client_ -> wait_for_service(std::chrono::seconds(1)))//服务端无效进入（如果第一次判断的时候服务端就有效那岂不是不判断客户端就直接往下执行了？）
            {
                //等待时检测rclcpp的状态
                if(!rclcpp::ok())//客户端不正常直接return
                {
                    RCLCPP_ERROR(this->get_logger(),"等待服务的过程中被打断...");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),"等待服务端上线中");//客户端正常继续等待
            }
            // 2. 构造请求的对象
            auto request = std::make_shared<Patrol::Request>();
            request->target_x = rand() % 15;
            request->target_y = rand() % 15;
            RCLCPP_INFO(this->get_logger(),"请求巡逻： (%f,%f)",request->target_x,request->target_y);
            // 3.发送异步请求，然后等待返回，返回时调用回调函数
            patrol_client_->async_send_request(request,[&](rclcpp::Client<Patrol>::SharedFuture result_future)->void{
                auto response = result_future.get();
                if(response->result == Patrol::Response::SUCCESS)
                {
                    RCLCPP_INFO(this->get_logger(),"目标点处理成功");
                }
                else if(response->result == Patrol::Response::FALL)
                {
                    RCLCPP_INFO(this->get_logger(),"目标点处理失败");
                }
            });
            
        }
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PatrolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
