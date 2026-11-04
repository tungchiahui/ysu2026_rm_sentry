#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"
#include "behavior_manager/bt_nodes/chassis_spin_action.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("behavior_manager");

    // 注册 XML 标签和对应的 C++ 类，同时传入 ROS 节点
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ChassisSpinAction>("ChassisSpin",node);

    BT::Tree tree;

    try
    {
        // 查找本包安装后的 share 目录
        const auto package_share = ament_index_cpp::get_package_share_directory("behavior_manager");

        // 不传参数时，默认加载本包安装的测试树
        const auto bt_xml_file = node->declare_parameter<std::string>("bt_xml_file",package_share + "/behavior_trees/spin_test.xml");

        RCLCPP_INFO(node->get_logger(),"加载行为树文件：%s",bt_xml_file.c_str());

        tree = factory.createTreeFromFile(bt_xml_file);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(),"加载行为树失败：%s",e.what());

        rclcpp::shutdown();
        return 1;
    }

    // 查找整棵树中的自旋叶子，不要求它位于根节点
    std::vector<ChassisSpinAction *> spin_bt_nodes;

    for (const auto &subtree : tree.subtrees)
    {
        for (const auto &bt_node : subtree->nodes)
        {
            auto *spin_bt_node =
                dynamic_cast<ChassisSpinAction *>(bt_node.get());

            if (spin_bt_node)
            {
                spin_bt_nodes.push_back(spin_bt_node);
            }
        }
    }

    // 执行树之前，等待这些节点对应的服务端
    for (auto *spin_bt_node : spin_bt_nodes)
    {
        if (!spin_bt_node->waitForServer(5s))
        {
            RCLCPP_ERROR(node->get_logger(), "未找到自旋 Action 服务");
            rclcpp::shutdown();
            return 1;
        }
    }

    // 退出收尾时，检查所有自旋任务是否结束
    const auto all_tasks_finished = [&spin_bt_nodes]()
    {
        for (const auto *spin_bt_node : spin_bt_nodes)
        {
            if (!spin_bt_node->isTaskFinished())
            {
                return false;
            }
        }

        return true;
    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto status = BT::NodeStatus::IDLE;

    // 执行多久，由树本身决定
    while (rclcpp::ok())
    {
        executor.spin_all(5ms);

        status = tree.tickOnce();

        if (status != BT::NodeStatus::RUNNING)
        {
            break;
        }

        std::this_thread::sleep_for(20ms);
    }

    // ROS 通信仍然有效时，停止树并触发 onHalted()
    RCLCPP_INFO(node->get_logger(), "行为树执行结束，开始任务收尾");
    tree.haltTree();

    // 停止 tick 后，继续处理取消答复和最终结果
    const auto finish_deadline = std::chrono::steady_clock::now() + 3s;

    while (rclcpp::ok()
        && !all_tasks_finished()
        && std::chrono::steady_clock::now() < finish_deadline)
    {
        executor.spin_all(5ms);
        std::this_thread::sleep_for(20ms);
    }

    const bool finished = all_tasks_finished();

    if (!finished)
    {
        RCLCPP_ERROR(node->get_logger(),"等待任务结束超时，尚未确认自旋已停止");
    }

    rclcpp::shutdown();

    return finished && status != BT::NodeStatus::FAILURE ? 0 : 1;
}