// 云台扫描节点：用定时器积分生成绝对位置角度，首次有效反馈用于初始化目标位置。
// yaw 保留多圈角度；启用 pitch 扫描时在软限位内往返，角度单位为 rad，速度单位为 rad/s。
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "behavior_manager/struct_typedef.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

// pitch 软限位；yaw 支持多圈，不限制位置角度
static constexpr fp32 PITCH_MIN = -0.46f;
static constexpr fp32 PITCH_MAX =  0.46f;

// 软件速度上限，不代表机械极限
static constexpr fp32 MAX_YAW_SPEED = 10.0f;
static constexpr fp32 MAX_PITCH_SPEED = 1.0f;

// 云台扫描速度 单位：rad/s
// 当前 pitch 速度为零，保持其初始位置；配置为非零扫描速度时才会进行俯仰往返。
static constexpr fp32 YAW_SCAN_SPEED = 1.0f;
static constexpr fp32 PITCH_SCAN_SPEED = 0.0f;

using namespace std::chrono_literals;

// 按关节名读取角度，不要求 pitch、yaw 的排列顺序
// 按名称匹配两个目标关节，避免真实反馈与仿真反馈的关节排列不同而读错角度。
static bool read_gimbal_position(
    const sensor_msgs::msg::JointState &msg,
    fp64 &pitch_position,
    fp64 &yaw_position)
{
    if (msg.name.size() != msg.position.size() ||
        msg.position.size() < 2)
    {
        return false;
    }

    bool found_pitch = false;
    bool found_yaw = false;

    for (std::size_t i = 0; i < msg.name.size(); ++i)
    {
        if (msg.name[i] == "gimbal_pitch_joint")
        {
            // 同一个目标关节不能重复出现
            if (found_pitch)
            {
                return false;
            }

            pitch_position = msg.position[i];
            found_pitch = true;
        }
        else if (msg.name[i] == "gimbal_yaw_joint")
        {
            if (found_yaw)
            {
                return false;
            }

            yaw_position = msg.position[i];
            found_yaw = true;
        }
    }

    return found_pitch && found_yaw &&
           std::isfinite(pitch_position) &&
           std::isfinite(yaw_position);
}


class Gimbal_Scan_Node: public rclcpp::Node
{
  public:
    Gimbal_Scan_Node():Node("gimbal_scan_server")
    {
        RCLCPP_INFO(this->get_logger(),"云台扫描节点已启动，等待云台反馈!");

        joint_state_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "serial/gimbal_joint_state",
                10,
                std::bind(&Gimbal_Scan_Node::joint_state_sub_callback,this,std::placeholders::_1)
            );

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint",10);

        // 在创建定时器之前初始化时间
        last_update_ = this->now();

        timer1_ = this->create_wall_timer(10ms,std::bind(&Gimbal_Scan_Node::timer1_callback,this));
    }

  private:

    // 仅清零内部积分速度，保留已有目标角度；目标有效时，定时器仍会发布保持位置的命令。
    void stop_input()
    {
        cmd_gimbal_joint_field.yaw_vel = 0.0f;
        cmd_gimbal_joint_field.pitch_vel = 0.0f;
    }

    // 订阅编码器角度
    void joint_state_sub_callback(const sensor_msgs::msg::JointState &msg)
    {
        fp64 pitch_position = 0.0;
        fp64 yaw_position = 0.0;

        if (!read_gimbal_position(msg, pitch_position, yaw_position))
        {
            return;
        }

        // 只在第一次有效反馈时初始化目标角度
        // 后续反馈不覆盖正在积分的目标角度
        if (!received_joint_state_)
        {
            // 初始角度超出软限位时等待处理，避免直接跳到限位位置
            // yaw 保留多圈值，但必须能够表示为当前使用的 fp32 类型
            // std::numeric_limits<fp32>::max() 是获取 fp32 类型的最大表示值
            if (pitch_position < PITCH_MIN || pitch_position > PITCH_MAX ||
                std::abs(yaw_position) > std::numeric_limits<fp32>::max())
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "初始云台角度超出允许范围，等待有效反馈!");
                return;
            }

            cmd_gimbal_joint_field.pitch = pitch_position;
            cmd_gimbal_joint_field.yaw = yaw_position;

            // 先用固定速度演示，后面再改成扫描任务参数
            cmd_gimbal_joint_field.yaw_vel = YAW_SCAN_SPEED;
            cmd_gimbal_joint_field.pitch_vel =
                cmd_gimbal_joint_field.pitch >= PITCH_MAX ? // 上限处反转初始扫描方向
                -PITCH_SCAN_SPEED : PITCH_SCAN_SPEED;

            last_update_ = this->now();

            RCLCPP_INFO(this->get_logger(),"开始扫描，pitch=%.3f，yaw=%.3f",cmd_gimbal_joint_field.pitch,cmd_gimbal_joint_field.yaw);
        }

        last_joint_state_time_ = std::chrono::steady_clock::now();
        received_joint_state_ = true;
    }

    void timer1_callback()
    {
        auto now = this->now();

        // 积分使用节点的 ROS 时间；暂停、回跳或异常跨度由后面的 dt 条件处理。
        fp32 dt = (now - last_update_).seconds();
        last_update_ = now;

        // 尚未得到有效反馈时，等待初始化
        if (!received_joint_state_)
        {
            return;
        }

        // 反馈新鲜度使用单调时钟，不受 ROS 时间暂停或回跳影响。
        std::chrono::steady_clock::time_point steady_now = std::chrono::steady_clock::now();

        bool joint_state_fresh =
            received_joint_state_ &&
            (steady_now - last_joint_state_time_ <=
                JOINT_STATE_TIMEOUT);

        if (!joint_state_fresh)
        {
            // 反馈超时停止推进目标，但不直接停止发布仍有效的目标位置。
            stop_input();

            if (!scan_fault_)
            {
                scan_fault_ = true;

                RCLCPP_WARN(this->get_logger(),"云台反馈超时，停止推进扫描目标!");
            }
        }

        // 非有限目标或速度不参与积分，也不发送给下位机
        if (!std::isfinite(cmd_gimbal_joint_field.yaw) ||
            !std::isfinite(cmd_gimbal_joint_field.pitch) ||
            !std::isfinite(cmd_gimbal_joint_field.yaw_vel) ||
            !std::isfinite(cmd_gimbal_joint_field.pitch_vel))
        {
            stop_input();
            scan_fault_ = true;
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "云台目标或速度无效，停止发布扫描命令!");
            return;
        }

        // 角度积分
        // 暂停时 dt 为零；时间回跳或异常跨越时跳过本次积分
        // 故障后保持停止，避免边界换向重新启动 pitch
        if (!scan_fault_ && dt > 0.0 && dt <= 0.1)
        {
            cmd_gimbal_joint_field.yaw_vel = std::clamp(
                cmd_gimbal_joint_field.yaw_vel,
                -MAX_YAW_SPEED,
                MAX_YAW_SPEED
            );

            cmd_gimbal_joint_field.pitch_vel = std::clamp(
                cmd_gimbal_joint_field.pitch_vel,
                -MAX_PITCH_SPEED,
                MAX_PITCH_SPEED
            );

            // 直接累积多圈绝对 yaw，不折返或归一化到 [-pi, pi]。
            cmd_gimbal_joint_field.yaw +=
                cmd_gimbal_joint_field.yaw_vel * dt;

            cmd_gimbal_joint_field.pitch +=
                cmd_gimbal_joint_field.pitch_vel * dt;

            // pitch 到达软限位后往返，yaw 继续同向转圈
            if (cmd_gimbal_joint_field.pitch_vel > 0.0f &&
                cmd_gimbal_joint_field.pitch >= PITCH_MAX)
            {
                cmd_gimbal_joint_field.pitch = PITCH_MAX;
                cmd_gimbal_joint_field.pitch_vel =
                    -cmd_gimbal_joint_field.pitch_vel;
            }
            else if (cmd_gimbal_joint_field.pitch_vel < 0.0f &&
                     cmd_gimbal_joint_field.pitch <= PITCH_MIN)
            {
                cmd_gimbal_joint_field.pitch = PITCH_MIN;
                cmd_gimbal_joint_field.pitch_vel =
                    -cmd_gimbal_joint_field.pitch_vel;
            }
        }

        if (!std::isfinite(cmd_gimbal_joint_field.yaw) ||
            !std::isfinite(cmd_gimbal_joint_field.pitch))
        {
            stop_input();
            scan_fault_ = true;
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "云台积分目标无效，停止发布扫描命令!");
            return;
        }

        // 在发布前执行最终 pitch 软限幅
        cmd_gimbal_joint_field.pitch = std::clamp(
            cmd_gimbal_joint_field.pitch,
            PITCH_MIN,
            PITCH_MAX
        );

        // 对外发送绝对位置目标；yaw_vel、pitch_vel 只用于本节点积分，不作为速度字段发送。
        auto joint_state_msg = sensor_msgs::msg::JointState();

        joint_state_msg.header.stamp = now;
        joint_state_msg.header.frame_id = "gimbal_pitch_odom_joint";

        joint_state_msg.name.push_back("gimbal_pitch_joint");
        joint_state_msg.name.push_back("gimbal_yaw_joint");

        joint_state_msg.position.push_back(cmd_gimbal_joint_field.pitch);

        joint_state_msg.position.push_back(cmd_gimbal_joint_field.yaw);

        joint_state_pub_->publish(joint_state_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    rclcpp::TimerBase::SharedPtr timer1_;

    // 目标角度、角速度结构
    struct
    {
        fp32 yaw{0.0f};
        fp32 pitch{0.0f};
        fp32 yaw_vel{0.0f};
        fp32 pitch_vel{0.0f};
    } cmd_gimbal_joint_field;

    rclcpp::Time last_update_;

    // 首次有效反馈后才允许推进目标；scan_fault_ 为锁存故障，反馈恢复不会自动清除它。
    bool received_joint_state_{false};
    bool scan_fault_{false};

    std::chrono::steady_clock::time_point last_joint_state_time_{std::chrono::steady_clock::now()};

    const std::chrono::milliseconds JOINT_STATE_TIMEOUT = 200ms;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);

    rclcpp::spin(std::make_shared<Gimbal_Scan_Node>());

    rclcpp::shutdown();
    return 0;
}