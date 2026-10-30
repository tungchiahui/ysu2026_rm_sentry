#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float32.hpp"
#include <rclcpp/timer.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include "behavior_manager/struct_typedef.h"

using namespace std::chrono_literals;

// 自旋速度，仅供测试，实际速度应该由bt树决定
static constexpr fp32 SPIN_SPEED = 0.5f;


class Chassis_Spin_Node: public rclcpp::Node
{
    public:
        Chassis_Spin_Node():Node("chassis_spin_server")
        {
            RCLCPP_INFO(this->get_logger(),"底盘自旋服务启动！");
        
            cmd_spin_pub_ = this->create_publisher<example_interfaces::msg::Float32>("cmd_spin",10);
            timer1_ = this->create_wall_timer(20ms,std::bind(&Chassis_Spin_Node::timer1_callback,this));
        }
    private:
        void timer1_callback()
        {
            start_spin(SPIN_SPEED);
        }

        void start_spin(fp32 spin_speed)
        {
            example_interfaces::msg::Float32 msg;
            msg.data = spin_speed;
            cmd_spin_pub_->publish(msg);
        }

        // 暂时为无效代码，后续再根据bt树判断是否需要停止
        void stop_spin()
        {
            example_interfaces::msg::Float32 msg;
            msg.data = 0.0f;
            cmd_spin_pub_->publish(msg);
        }

        rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr cmd_spin_pub_;
        rclcpp::TimerBase::SharedPtr timer1_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Chassis_Spin_Node>());

  rclcpp::shutdown();
  return 0;
}