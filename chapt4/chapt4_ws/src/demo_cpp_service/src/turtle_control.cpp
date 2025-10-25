#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
// 1. 添加服务头文件并创建别名
#include "chapt4_interfaces/srv/patrol.hpp"
using Patrol = chapt4_interfaces::srv::Patrol;

class TurtleController : public rclcpp::Node
{
    private:
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
        double target_x_{1.0}; //目标位置x,设置默认值1.0
        double target_y_{1.0}; //目标位置y,设置默认值1.0
        double k_{1.0};// 比例系数，控制输出=误差x比例系数
        double max_speed_{3.0};//最大线速度，设置默认值3.0

        // 2. 添加Patrol类型服务共享指针 patrol_server_ 为成员变量
        rclcpp::Service<Patrol>::SharedPtr patrol_server_;
    
    public:
        TurtleController() : Node("turtle_controller")
        {
            velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);  
            pose_subscription_  = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&TurtleController::on_pose_received_,this,std::placeholders::_1));

            // 3. 创建服务
            patrol_server_ = this->create_service<Patrol>("patrol",
                [&](const std::shared_ptr<Patrol::Request> request,
                std::shared_ptr<Patrol::Response> response) -> void {
                    // 判断巡逻点是否在模拟器边界内
                    if ((0 <request->target_x && request->target_x < 12.0f)
                        && (0 < request->target_y && request->target_y <12.0f))
                    {
                        target_x_ = request->target_x;
                        target_y_ = request->target_y;
                        response->result = Patrol::Response::SUCCESS;
                    }
                    else
                    {
                        response->result = Patrol::Response::FALL;
                    }
                });            
        }

    private:
    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose)
    {
        // TODO: 收到位置计算误差，发布速度指令
        auto message = geometry_msgs::msg::Twist();
        //1.记录当前位置
        double current_x = pose->x;
        double current_y = pose->y;
        RCLCPP_INFO(this->get_logger(),"当前位置：(x=%f,y=%f)",current_x,current_y);

        //2.计算与目标之间的距离，以及与当前海龟朝向的角度差
        double distance = 
            std::sqrt((target_x_ - current_x)*(target_x_ - current_x)+(target_y_ - current_y)*(target_y_ - current_y));
        double angle =
            std::atan2(target_y_ - current_y,target_x_ - current_x) - pose->theta;
        //从海龟​​当前位置​​指向​​目标位置​​的向量，与​​世界坐标系X轴正方向​​之间的夹角
        //pose->theta表示的是海龟​​当前身体朝向​​与世界坐标系X轴正方向的夹角

        //3.控制策略：距离大于0.1继续运动，角度差大于0.2则原地旋转，否则直行
        if(distance > 0.1)
        {
            if(fabs(angle)>0.2)
            {
                message.angular.z = fabs(angle);
            }
            else
            {
                message.linear.x = k_ * distance;
            }
        }
        
        //4.限制最大值并发布消息
        if (message.linear.x > max_speed_)
        {
            message.linear.x = max_speed_;
        }
        velocity_publisher_->publish(message);

    }
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

