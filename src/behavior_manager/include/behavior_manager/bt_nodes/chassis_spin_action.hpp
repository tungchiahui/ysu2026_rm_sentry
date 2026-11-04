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

// BT 自旋叶子：通过 ROS Action 把启动、运行检查和停止转为对远端自旋服务的请求。
// 使用异步请求与 future，不在 tick 中阻塞等待；答复回调需要入口持续处理 ROS 事件。
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

    // 供入口判断本次请求是否无需继续等待：未发起、发送异常、被拒绝或已收到任意终态。
    // 这是通信收尾状态，不等同于任务成功，也不是底盘实测速度为零的确认。
    bool isTaskFinished() const
    {
        return !cancel_state_ || cancel_state_->finished.load();
    }

private:
    // 每次目标各有一份状态，旧回调持有旧状态，避免晚到的答复影响下一次任务。
    // 原子标记允许 ROS 回调与停止逻辑并发访问，但不代表整个 BT 节点可以任意并发 tick。
    struct CancelState
    {
        std::atomic_bool requested{false};  // BT 已提出取消意图，目标尚未接受时也要保留
        std::atomic_bool sent{false};       // 取消请求已发送，不代表服务端已接受或完成取消
        std::atomic_bool finished{false};  // 发送异常、目标被拒绝或收到任意终态后置为 true
    };

    std::shared_ptr<CancelState> cancel_state_;

    // 只取消指定句柄对应的任务，不阻塞等待答复；入口另外等待最终结果。
    // 使用显式传入的对象，不访问 this，便于晚到的目标答复继续完成取消处理。
    static void request_cancel(
        rclcpp_action::Client<behavior_manager::action::ChassisSpin>::SharedPtr client,
        rclcpp_action::ClientGoalHandle<behavior_manager::action::ChassisSpin>::SharedPtr goal_handle,
        std::shared_ptr<CancelState> state,
        rclcpp::Logger logger)
    {
        // 空句柄无需取消；exchange(true) 返回旧值，只有首次调用取得发送权。
        // 目标答复回调与 onHalted() 都可能到达这里，此标记用于避免重复发送。
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
            // 发送异常时释放发送标记，允许后续调用再尝试；这里没有自动重试循环。
            state->sent.store(false);

            RCLCPP_ERROR(
                logger, "发送自旋取消请求失败：%s", e.what());
        }
    }


    // 从 IDLE 开始执行时调用：读取参数、发送目标，返回 RUNNING 后交给后续 tick 检查。
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

        // 清理上一次执行保存的本地查询状态；旧回调仍持有各自捕获的 CancelState。
        goal_handle_.reset();
        goal_future_ = {};
        result_future_ = {};

        // 本地检查参数有效性；速度限幅由服务端统一执行。
        behavior_manager::action::ChassisSpin::Goal goal;
        goal.spin_speed = spin_speed.value();

        // 本次任务使用自己的取消状态
        cancel_state_ = std::make_shared<CancelState>();

        try
        {
            rclcpp_action::Client<behavior_manager::action::ChassisSpin>::SendGoalOptions options;

            // 按值捕获本次任务需要的对象，不捕获 this，保证晚到答复使用对应任务的状态。
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

            // 即使 BT 已经停止 tick，入口继续处理 ROS 回调时仍能收到终态并结束收尾等待。
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

    // RUNNING 期间轮询两阶段答复：先等待接受/拒绝，再等待任务最终结果。
    BT::NodeStatus onRunning() override
    {
        try
        {
            // 尚未取得任务句柄，先检查服务端是否答复
            if (!goal_handle_)
            {
                // 零时长仅探测是否就绪；确认 ready 后再 get()，避免阻塞整棵行为树。
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

                // 接受目标不等于执行结束；取得同一任务的最终结果 future 供后续 tick 查询。
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

            // 当前服务端持续执行直到取消或异常，没有自然成功出口；执行期间一直 RUNNING。
            // 通用成功判断同时检查 Action 终态和 Result 中的业务成功字段。
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED
                && result.result
                && result.result->success)
            {
                return BT::NodeStatus::SUCCESS;
            }

            // 运行中观察到外部取消或中止，映射为 FAILURE；BT 主动 halt 后靠结果回调收尾。
            RCLCPP_WARN(node_->get_logger(),"自旋任务结束，未成功完成，结果状态码=%d",static_cast<int>(result.code));

            return BT::NodeStatus::FAILURE;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "检查自旋任务失败：%s", e.what());

            // 返回 FAILURE 不会自动再调用 onHalted()，先尝试取消可能仍在执行的任务。
            onHalted();

            return BT::NodeStatus::FAILURE;
        }
    }

    // BT 停止本叶子时提出取消意图；此函数返回不代表远端任务已停止。
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

    // 目标请求的答复：接受时得到客户端任务句柄，拒绝时得到空句柄。
    std::shared_future<rclcpp_action::ClientGoalHandle<behavior_manager::action::ChassisSpin>::SharedPtr> goal_future_;

    // 服务端接受目标后，通过它操作这一次任务
    rclcpp_action::ClientGoalHandle<behavior_manager::action::ChassisSpin>::SharedPtr goal_handle_;

    // 执行结果的答复：包含 SUCCEEDED/CANCELED/ABORTED 等终态以及 Result 数据。
    std::shared_future<rclcpp_action::ClientGoalHandle<behavior_manager::action::ChassisSpin>::WrappedResult> result_future_;
};
