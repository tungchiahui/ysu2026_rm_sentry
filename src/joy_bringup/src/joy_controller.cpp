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
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class Joy_Node: public rclcpp::Node
{
  public:
    Joy_Node():Node("joy_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(),"joy2cmdvel节点已启动!");
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Joy_Node::joy_sub_callback,this,std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint", 10);
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

        cmd_vel_field.vx = msg.axes[1]; //左摇杆朝前是1.0
        cmd_vel_field.vy = msg.axes[0]; //左摇杆朝左是1.0
        cmd_gimbal_joint_field.yaw = msg.axes[3] * M_PI; //右摇杆朝左是1.0 * M_PI
        cmd_gimbal_joint_field.pitch = msg.axes[4] * 1.0; //右摇杆朝前是1.0 * 1.0 （因为pitch有范围限制，而范围设置为1rad，所以这里乘以1.0）

    }

    void timer1_callback()
    {
      auto now = this->now();

      auto cmd_vel_msg = geometry_msgs::msg::Twist();
      auto joint_state_msg = sensor_msgs::msg::JointState();

      cmd_vel_msg.linear.x = cmd_vel_field.vx;
      cmd_vel_msg.linear.y = cmd_vel_field.vy;
      cmd_vel_msg.linear.z = 0.0f;

      cmd_vel_msg.angular.x = 0.0f;
      cmd_vel_msg.angular.y = 0.0f;
      cmd_vel_msg.angular.z = 0.0f;

      joint_state_msg.header.stamp = now;
      joint_state_msg.header.frame_id = "gimbal_pitch_odom_joint"; //貌似没啥用

      joint_state_msg.name.push_back("gimbal_pitch_joint");
      joint_state_msg.name.push_back("gimbal_yaw_joint");

      joint_state_msg.position.push_back(cmd_gimbal_joint_field.pitch);
      joint_state_msg.position.push_back(cmd_gimbal_joint_field.yaw);

      joint_state_pub_->publish(joint_state_msg);

      cmd_vel_pub_->publish(cmd_vel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer1_;

    struct
    {
        fp32 vx{0.0f};
        fp32 vy{0.0f};
    } cmd_vel_field;

    struct
    {
        fp32 yaw{0.0f};
        fp32 pitch{0.0f};
    } cmd_gimbal_joint_field;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Joy_Node>());

  rclcpp::shutdown();
  return 0;
}