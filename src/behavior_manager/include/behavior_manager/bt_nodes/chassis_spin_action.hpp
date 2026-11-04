#pragma once

#include <memory>
#include <string>
#include <chrono>
#include <cmath>
#include <future>
#include <exception>
#include <atomic>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behavior_manager/action/chassis_spin.hpp"

using namespace std::chrono_literals;

class ChassisSpinAction : public BT::StatefulActionNode
{
public:
    ChassisSpinAction(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
        node_(node)
    {
        chassis_spin_action_client_ = rclcpp_action::create_client<behavior_manager::action::ChassisSpin>(node_,"chassis_spin");
    }

    // 定义这个 BT 节点可以从 XML 接收的参数
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<float>("spin_speed", "自旋速度，单位 rad/s")
        };
    }

    // 入口启动时，等待服务端被发现
    bool waitForServer(std::chrono::seconds timeout)
    {
        return chassis_spin_action_client_->wait_for_action_server(timeout);
    }

    // 停止树后，入口用它判断是否可以退出
    bool isTaskFinished() const
    {
        return !cancel_state_ || cancel_state_->finished.load();
    }

private:
    // 每次发送目标，创建一份独立的取消状态
    struct CancelState
    {
        std::atomic_bool requested{false};  // BT 是否要求取消
        std::atomic_bool sent{false};       // 是否已经发送取消请求
        std::atomic_bool finished{false};  // 服务端是否已拒绝目标或返回最终结果
    };

    std::shared_ptr<CancelState> cancel_state_;

    static void request_cancel(
        rclcpp_action::Client<behavior_manager::action::ChassisSpin>::SharedPtr client,
        rclcpp_action::ClientGoalHandle<behavior_manager::action::ChassisSpin>::SharedPtr goal_handle,
        std::shared_ptr<CancelState> state,
        rclcpp::Logger logger)
    {
        // 被拒绝的目标没有句柄；同一个目标只发送一次取消请求
        if (!goal_handle || state->sent.exchange(true))
        {
            return;
        }

        try
        {
            client->async_cancel_goal(goal_handle);

            RCLCPP_INFO(logger, "已发送自旋任务取消请求");
        }
        catch (const std::exception &e)
        {
            state->sent.store(false);

            RCLCPP_ERROR(
                logger, "发送自旋取消请求失败：%s", e.what());
        }
    }


    BT::NodeStatus onStart() override
    {
        const auto spin_speed = getInput<float>("spin_speed");

        if (!spin_speed || !std::isfinite(spin_speed.value()))
        {
            RCLCPP_ERROR(node_->get_logger(), "自旋速度缺失或数值无效");
            return BT::NodeStatus::FAILURE;
        }

        if (!chassis_spin_action_client_->action_server_is_ready())
        {
            RCLCPP_WARN(node_->get_logger(), "尚未发现自旋 Action 服务");
            return BT::NodeStatus::FAILURE;
        }

        // 清理上一次执行保存的状态
        goal_handle_.reset();
        goal_future_ = {};
        result_future_ = {};

        behavior_manager::action::ChassisSpin::Goal goal;
        goal.spin_speed = spin_speed.value();

        // 本次任务使用自己的取消状态
        cancel_state_ = std::make_shared<CancelState>();

        try
        {
            rclcpp_action::Client<behavior_manager::action::ChassisSpin>::SendGoalOptions options;

            // 按值保存本次任务需要的对象
            auto state = cancel_state_;
            auto client = chassis_spin_action_client_;
            auto logger = node_->get_logger();

            options.goal_response_callback =
                [state, client, logger](
                    rclcpp_action::ClientGoalHandle<behavior_manager::action::ChassisSpin>::SharedPtr goal_handle)
                {
                    // 被拒绝的目标，没有后续执行任务
                    if (!goal_handle)
                    {
                        state->finished.store(true);
                        return;
                    }

                    // BT 已经停止，但服务端现在才接受目标
                    if (state->requested.load())
                    {
                        ChassisSpinAction::request_cancel(
                            client,
                            goal_handle,
                            state,
                            logger);
                    }
                };

            options.result_callback =
                [state, logger](
                    const rclcpp_action::ClientGoalHandle<
                        behavior_manager::action::ChassisSpin>::WrappedResult &result)
                {
                    state->finished.store(true);

                    if (result.code == rclcpp_action::ResultCode::CANCELED)
                    {
                        RCLCPP_INFO(logger, "服务端确认自旋任务已取消");
                    }
                    else
                    {
                        RCLCPP_INFO(
                            logger,
                            "自旋任务已结束，结果状态码=%d",
                            static_cast<int>(result.code));
                    }
                };

            goal_future_ = chassis_spin_action_client_->async_send_goal(goal, options);
        }
        catch (const std::exception &e)
        {
            cancel_state_->finished.store(true);

            RCLCPP_ERROR(node_->get_logger(), "发送自旋目标失败：%s", e.what());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(),"已发送自旋目标：%.2f rad/s,等待服务端答复",goal.spin_speed);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        try
        {
            // 尚未取得任务句柄，先检查服务端是否答复
            if (!goal_handle_)
            {
                if (goal_future_.wait_for(std::chrono::milliseconds(0))
                    != std::future_status::ready)
                {
                    return BT::NodeStatus::RUNNING;
                }

                goal_handle_ = goal_future_.get();

                // 空句柄表示服务端拒绝了目标
                if (!goal_handle_)
                {
                    RCLCPP_WARN(node_->get_logger(), "自旋目标被拒绝");
                    return BT::NodeStatus::FAILURE;
                }

                RCLCPP_INFO(node_->get_logger(), "自旋目标已被接受");

                // 开始等待这一次任务的最终结果
                result_future_ =
                    chassis_spin_action_client_->async_get_result(
                        goal_handle_);
            }

            // 已接受但尚未结束，保持 RUNNING
            // 只检查答复到了没有，不会阻塞
            if (result_future_.wait_for(0ms) != std::future_status::ready)
            {
                return BT::NodeStatus::RUNNING;
            }

            const auto result = result_future_.get();

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED
                && result.result
                && result.result->success)
            {
                return BT::NodeStatus::SUCCESS;
            }

            RCLCPP_WARN(node_->get_logger(),"自旋任务结束，未成功完成，结果状态码=%d",static_cast<int>(result.code));

            return BT::NodeStatus::FAILURE;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "检查自旋任务失败：%s", e.what());

            // 客户端检查异常时，也尝试取消可能仍在执行的任务
            onHalted();

            return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() override
    {
        if (!cancel_state_)
        {
            return;
        }

        // 先记录取消意图，保证晚到的目标答复也能处理
        cancel_state_->requested.store(true);

        // 尚未收到答复，交给目标答复回调处理
        if (!goal_future_.valid() || goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        {
            return;
        }

        try
        {
            // 答复已到，即使 onRunning 尚未取出句柄，也能取消
            const auto goal_handle = goal_future_.get();

            request_cancel(
                chassis_spin_action_client_,
                goal_handle,
                cancel_state_,
                node_->get_logger());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "停止 BT 自旋节点时发生异常：%s",
                e.what());
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<behavior_manager::action::ChassisSpin>::SharedPtr chassis_spin_action_client_;

    // 等待服务端回答“接受还是拒绝”
    std::shared_future<rclcpp_action::ClientGoalHandle<behavior_manager::action::ChassisSpin>::SharedPtr> goal_future_;

    // 服务端接受目标后，通过它操作这一次任务
    rclcpp_action::ClientGoalHandle<behavior_manager::action::ChassisSpin>::SharedPtr goal_handle_;

    // 等待这一次任务最终结束
    std::shared_future<rclcpp_action::ClientGoalHandle<behavior_manager::action::ChassisSpin>::WrappedResult> result_future_;
};
