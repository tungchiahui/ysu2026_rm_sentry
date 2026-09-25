#include "joy_bringup/joy_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/joy__struct.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class Joy_Node: public rclcpp::Node
{
  public:
    Joy_Node():Node("joy_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(),"joy2cmdvel节点已启动!");
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Joy_Node::joy_sub_callback,this,std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer1_ = this->create_wall_timer(10ms, std::bind(&Joy_Node::timer1_callback,this));
    }
  private:
    void joy_sub_callback(const sensor_msgs::msg::Joy &msg)
    {

        /*
        vx > 0  → 向前
        vx < 0  → 向后

        vy > 0  → 向左
        vy < 0  → 向右

        wz > 0  → 逆时针旋转（从车顶往下看）
        wz < 0  → 顺时针旋转
        */

        // cmd_vel_field.vx = msg.axes[1]; //左摇杆朝前是1.0
        // cmd_vel_field.vy = msg.axes[0]; //左摇杆朝左是1.0
        // cmd_vel_field.wz = msg.axes[3]; //右摇杆朝左是1.0

        cmd_vel_field.vx = - msg.axes[0]; //左摇杆朝右是1.0
        cmd_vel_field.vy =   msg.axes[1]; //左摇杆朝前是1.0
        cmd_vel_field.wz =   msg.axes[3]; //右摇杆朝左是1.0
    }

    void timer1_callback()
    {
      auto msg = geometry_msgs::msg::Twist();

      msg.linear.x = cmd_vel_field.vx;
      msg.linear.y = cmd_vel_field.vy;
      msg.linear.z = 0.0f;

      msg.angular.x = 0.0f;
      msg.angular.y = 0.0f;
      msg.angular.z = cmd_vel_field.wz;

      cmd_vel_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer1_;

    struct
    {
      fp32 vx;
      fp32 vy;
      fp32 wz;
    }cmd_vel_field;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Joy_Node>());

  rclcpp::shutdown();
  return 0;
}