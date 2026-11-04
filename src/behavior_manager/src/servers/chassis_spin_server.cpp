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

        ~Chassis_Spin_Node() override
        {
            if (execute_thread_.joinable())
            {
                execute_thread_.request_stop();
                execute_thread_.join();
            }
        }
    private:
        // 收到目标请求
        rclcpp_action::GoalResponse handle_goal_callback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const behavior_manager::action::ChassisSpin::Goal> goal)
        {
            (void)uuid;
            std::lock_guard<std::mutex> lock(task_mutex_);

            // 已有任务时，拒绝新任务，避免覆盖当前句柄
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

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // 收到取消请求
        rclcpp_action::CancelResponse handle_cancel_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<behavior_manager::action::ChassisSpin>> goal_handle)
        {
            std::lock_guard<std::mutex> lock(task_mutex_);

            // 只允许取消当前正在执行的任务
            if (!goal_handle_ || goal_handle != goal_handle_)
            {
                // 执行不取消
                return rclcpp_action::CancelResponse::REJECT;
            }

            RCLCPP_INFO(this->get_logger(),"接受自旋任务取消请求!");

            // 执行取消
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // 目标被接受后调用
        void handle_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<behavior_manager::action::ChassisSpin>> goal_handle)
        {
            // 这个函数体是为了加锁
            {
                std::lock_guard<std::mutex> lock(task_mutex_);

                // 保存这一次任务的句柄
                goal_handle_ = goal_handle;

                // 通过任务句柄，读取客户端发来的 Goal 参数
                const auto goal = goal_handle_->get_goal();

                // 将请求速度限幅，再保存到类成员中
                spin_speed_ = std::clamp(goal->spin_speed,-MAX_SPIN_SPEED,MAX_SPIN_SPEED);

                //记录现在的时间
                start_time_ = this->now();

                RCLCPP_INFO(this->get_logger(),"请求速度=%.2f,执行速度=%.2f",goal->spin_speed,spin_speed_);
            }

            try
            {
                // 回收上一次已结束的工作线程
                if (execute_thread_.joinable())
                {
                    execute_thread_.join();
                }

                // 创建本次任务的工作线程
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

                    feedback->commanded_speed = spin_speed;
                    feedback->elapsed_time = static_cast<fp32>((this->now() - start_time).seconds());

                    goal_handle->publish_feedback(feedback);

                    std::this_thread::sleep_for(20ms);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),"自旋执行异常：%s",e.what());
            }

            // 循环退出后统一收尾
            // ROS 通信仍有效时，发送停止请求并结束 Action
            if (rclcpp::ok())
            {
                try
                {
                    stop_spin();

                    auto result = std::make_shared<behavior_manager::action::ChassisSpin::Result>();

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
                // 收尾完成后，才释放当前任务
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
        rclcpp_action::Server<behavior_manager::action::ChassisSpin>::SharedPtr chassis_spin_action_server_;
        std::jthread execute_thread_;
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