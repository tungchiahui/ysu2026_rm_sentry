// 行为树入口：加载 XML、注册叶子节点、驱动 tick，并等待远端任务完成退出收尾。
// ROS 用户回调与 BT tick 在本线程中按序执行，自旋的持续发布由服务端工作线程负责。
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
        // 查找本包安装后的 share 目录，与当前终端的工作目录无关。
        const auto package_share = ament_index_cpp::get_package_share_directory("behavior_manager");

        // 不传参数时加载安装目录中的测试树；bt_xml_file 可指定其他 XML 路径。
        // 若安装文件是副本，源码 XML 修改后需重新安装，也可用参数直接加载源码文件。
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

    // 查找所有子树中的自旋叶子，允许根节点是 Parallel、Sequence 等控制节点。
    // 指针指向 tree 拥有的节点，只在本棵树存活期间使用，不在这里释放节点。
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

    // 执行树之前等待服务端被发现；5s 是通信发现上限，与自旋持续时间无关。
    for (auto *spin_bt_node : spin_bt_nodes)
    {
        if (!spin_bt_node->waitForServer(5s))
        {
            RCLCPP_ERROR(node->get_logger(), "未找到自旋 Action 服务");
            rclcpp::shutdown();
            return 1;
        }
    }

    // 当前只检查自旋叶子的通信收尾；未发起任务的节点也视为无需等待。
    // finished 表示请求已结束或无需继续等待，不表示任务成功或实车速度已经为零。
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
        // spin_all 在 5ms 预算内反复收集就绪事件，及时消费 Action 反馈与各类答复。
        // 先处理 ROS 回调，再让 BT 检查 future；5ms 是处理预算，不是固定休眠时间。
        executor.spin_all(5ms);

        status = tree.tickOnce();

        if (status != BT::NodeStatus::RUNNING)
        {
            break;
        }

        // 控制 tick 的轮询节奏；加上回调与 tick 耗时后，并非严格的 50Hz。
        std::this_thread::sleep_for(20ms);
    }

    // 停止仍在运行的叶子，触发 onHalted()；控制节点也可能已在返回前停止了子节点。
    // 取消通信需要 ROS 上下文仍然有效；默认 Ctrl+C 关闭 ROS 后无法继续此通信流程。
    RCLCPP_INFO(node->get_logger(), "行为树执行结束，开始任务收尾");
    tree.haltTree();

    // 树结束或 haltTree() 返回，不代表远端 Action 已结束；停止 tick 后仍需处理 ROS 回调。
    // 使用单调时钟限制收尾等待，避免 ROS 时间暂停或回跳影响这个 3s 通信等待上限。
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