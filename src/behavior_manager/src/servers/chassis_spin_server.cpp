// 自旋能力服务端：接收 ChassisSpin 目标，在工作线程中持续发布附加自旋角速度。
// 单次任务保持运行直到取消、线程停止或异常；零速度目标也不会使任务自然完成。
#include "behavior_manager/action/detail/chassis_spin__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float32.hpp"
#include <rclcpp/timer.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <algorithm>
#include <thread>
#include <stop_token>
#include <mutex>
#include <exception>
#include "behavior_manager/struct_typedef.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behavior_manager/action/chassis_spin.hpp"

using namespace std::chrono_literals;

// 软件速度上限，单位 rad/s
static constexpr fp32 MAX_SPIN_SPEED = 3.0f;


class Chassis_Spin_Node: public rclcpp::Node
{
    public:
        Chassis_Spin_Node():Node("chassis_spin_server")
        {
            RCLCPP_INFO(this->get_logger(),"底盘自旋服务启动！");
        
            // cmd_spin 的单位为 rad/s，由下游 fake_vel_transform 与导航角速度合成最终 cmd_vel。
            cmd_spin_pub_ = this->create_publisher<example_interfaces::msg::Float32>("cmd_spin",10);
            chassis_spin_action_server_ =
            rclcpp_action::create_server<behavior_manager::action::ChassisSpin>(
                this,
                "chassis_spin",
                std::bind(
                    &Chassis_Spin_Node::handle_goal_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                ),
                std::bind(
                    &Chassis_Spin_Node::handle_cancel_callback,
                    this,
                    std::placeholders::_1
                ),
                std::bind(
                    &Chassis_Spin_Node::handle_accepted_callback,
                    this,
                    std::placeholders::_1
                )
            );

        }

        // 先请求线程退出并等待结束，保证节点成员销毁前，线程不再访问 this。
        // request_stop() 是本地线程停止请求，与 ROS Action 的取消请求不同。
        ~Chassis_Spin_Node() override
        {
            if (execute_thread_.joinable())
            {
                execute_thread_.request_stop();
                execute_thread_.join();
            }
        }
    private:
        // 收到目标请求时只做接收判定；持续发布与反馈放到工作线程中执行。
        rclcpp_action::GoalResponse handle_goal_callback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const behavior_manager::action::ChassisSpin::Goal> goal)
        {
            (void)uuid;
            std::lock_guard<std::mutex> lock(task_mutex_);

            // 一次只执行一个目标，不抢占已有任务；已有任务时发送零速度新目标也会被拒绝。
            if (goal_handle_)
            {
                RCLCPP_WARN(this->get_logger(),"已有自旋任务，拒绝新目标!");
                return rclcpp_action::GoalResponse::REJECT;
            }

            // 拒绝 NaN、无穷大等无效参数
            if (!std::isfinite(goal->spin_speed))
            {
                RCLCPP_WARN(this->get_logger(),"目标速度包含无效数值!");
                return rclcpp_action::GoalResponse::REJECT;
            }

            RCLCPP_INFO(this->get_logger(),"目标参数合法：速度=%.2f rad/s",goal->spin_speed);

            // 有限但超限的速度仍接受，稍后在 accepted 回调中统一 clamp。
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // 收到取消请求
        rclcpp_action::CancelResponse handle_cancel_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<behavior_manager::action::ChassisSpin>> goal_handle)
        {
            std::lock_guard<std::mutex> lock(task_mutex_);

            // 只允许取消当前正在执行的任务
            if (!goal_handle_ || goal_handle != goal_handle_)
            {
                // 拒绝不属于当前任务的取消请求。
                return rclcpp_action::CancelResponse::REJECT;
            }

            RCLCPP_INFO(this->get_logger(),"接受自旋任务取消请求!");

            // 这里只接受取消请求；执行线程看到 is_canceling() 后发布零速度并结束 Action。
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // 目标被接受后调用
        void handle_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<behavior_manager::action::ChassisSpin>> goal_handle)
        {
            // 用独立作用域限定锁的持有时间，只保护任务状态的更新，离开此块即解锁。
            {
                std::lock_guard<std::mutex> lock(task_mutex_);

                // 保存这一次任务的句柄
                goal_handle_ = goal_handle;

                // 通过任务句柄，读取客户端发来的 Goal 参数
                const auto goal = goal_handle_->get_goal();

                // 将请求速度限幅，再保存到类成员中
                spin_speed_ = std::clamp(goal->spin_speed,-MAX_SPIN_SPEED,MAX_SPIN_SPEED);

                // ROS 时间用于反馈 elapsed_time，不作为任务自动结束的条件。
                start_time_ = this->now();

                RCLCPP_INFO(this->get_logger(),"请求速度=%.2f,执行速度=%.2f",goal->spin_speed,spin_speed_);
            }

            try
            {
                // 在 task_mutex_ 已释放后回收上次线程，避免持锁 join 阻塞线程的收尾加锁。
                if (execute_thread_.joinable())
                {
                    execute_thread_.join();
                }

                // 按值保存本次句柄，避免工作线程把之后的新任务句柄当成自己的任务。
                // accepted 回调启动线程后返回，ROS 执行器可以继续接收取消等请求。
                execute_thread_ = std::jthread(
                    [this, goal_handle](std::stop_token stop_token)
                    {
                        execute_callback(stop_token, goal_handle);
                    }
                );
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),"工作线程启动失败：%s",e.what());

                // Goal 已经接受，需要向客户端报告执行失败
                try
                {
                    if (rclcpp::ok() && goal_handle->is_active())
                    {
                        auto result = std::make_shared<behavior_manager::action::ChassisSpin::Result>();

                        result->success = false;
                        result->message = "工作线程启动失败";

                        goal_handle->abort(result);
                    }
                }
                catch (const std::exception &finish_error)
                {
                    RCLCPP_ERROR(this->get_logger(),"线程启动失败后的任务收尾异常：%s",finish_error.what());
                }

                // 确认任务已结束，再清理共享状态
                if (!rclcpp::ok() || !goal_handle->is_active())
                {
                    std::lock_guard<std::mutex> lock(task_mutex_);

                    if (goal_handle_ == goal_handle)
                    {
                        goal_handle_.reset();
                        spin_speed_ = 0.0f;
                    }
                }
            }
        }

        // 单次 Goal 的参数固定，线程使用自己的参数副本；新速度需要另一次目标请求。
        void execute_callback(std::stop_token stop_token,std::shared_ptr<rclcpp_action::ServerGoalHandle<behavior_manager::action::ChassisSpin>> goal_handle)
        {
            fp32 spin_speed = 0.0f;
            rclcpp::Time start_time;

            // 短暂加锁，把任务参数复制为线程自己的局部变量
            {
                std::lock_guard<std::mutex> lock(task_mutex_);

                spin_speed = spin_speed_;
                start_time = start_time_;
            }

            // 此时已经解锁，后面的循环不一直占用任务锁
            try
            {
                auto feedback = std::make_shared<behavior_manager::action::ChassisSpin::Feedback>();

                while (rclcpp::ok() && !stop_token.stop_requested())
                {
                    // ROS Action 取消：退出执行循环，进入下面的收尾
                    if (goal_handle->is_canceling())
                    {
                        break;
                    }

                    start_spin(spin_speed);

                    // 反馈的是限幅后的指令速度，不是底盘实测转速。
                    feedback->commanded_speed = spin_speed;
                    feedback->elapsed_time = static_cast<fp32>((this->now() - start_time).seconds());

                    goal_handle->publish_feedback(feedback);

                    // 约 50Hz 持续发布指令与反馈，实际周期还包含本轮处理耗时。
                    std::this_thread::sleep_for(20ms);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),"自旋执行异常：%s",e.what());
            }

            // 取消、线程停止或执行异常都走统一收尾：先发布零速度，再报告 Action 终态。
            // ROS 已关闭时跳过通信，不能依靠此处保证节点关闭后仍发送出停止指令。
            if (rclcpp::ok())
            {
                try
                {
                    stop_spin();

                    auto result = std::make_shared<behavior_manager::action::ChassisSpin::Result>();

                    // 持续任务没有自然成功出口，取消报告 CANCELED，其他退出报告 ABORTED。
                    // success=false 不表示取消被拒绝，取消结果应结合 Action 终态判断。
                    result->success = false;

                    if (goal_handle->is_canceling())
                    {
                        result->message = "自旋任务已取消，已发送零速度";
                        goal_handle->canceled(result);
                    }
                    else
                    {
                        result->message = "自旋执行线程停止或发生异常";
                        goal_handle->abort(result);
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(),"自旋任务收尾异常：%s",e.what());
                }
            }

            const bool task_finished = !rclcpp::ok() || !goal_handle->is_active();

            if (task_finished)
            {
                // 确认 Action 已进入终态，或 ROS 已关闭后，才释放当前任务的忙状态。
                std::lock_guard<std::mutex> lock(task_mutex_);

                if (goal_handle_ == goal_handle)
                {
                    goal_handle_.reset();
                    spin_speed_ = 0.0f;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),"未能结束当前 Goal,保留忙状态,请检查收尾错误!");
            }
        }

        // 只发布附加自旋速度请求，最终底盘速度由下游合成；发布不代表实车已达到该速度。
        void start_spin(fp32 spin_speed)
        {
            example_interfaces::msg::Float32 msg;
            msg.data = spin_speed;
            cmd_spin_pub_->publish(msg);
        }

        // 发布零附加自旋角速度，由执行线程在取消或异常收尾时调用。
        void stop_spin()
        {
            example_interfaces::msg::Float32 msg;
            msg.data = 0.0f;
            cmd_spin_pub_->publish(msg);
        }

        rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr cmd_spin_pub_;
        rclcpp_action::Server<behavior_manager::action::ChassisSpin>::SharedPtr chassis_spin_action_server_;
        std::jthread execute_thread_;
        // 保护回调与工作线程共同访问的 goal_handle_、spin_speed_ 和 start_time_。
        std::mutex task_mutex_;

        //当前任务，之后通过它返回结果
        std::shared_ptr<rclcpp_action::ServerGoalHandle<behavior_manager::action::ChassisSpin>> goal_handle_;
        //限幅后真正要使用的速度
        fp32 spin_speed_{0.0f};
        // 用于计算反馈中的 elapsed_time
        rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Chassis_Spin_Node>());

  rclcpp::shutdown();
  return 0;
}