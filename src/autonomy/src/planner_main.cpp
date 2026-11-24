#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>

#include "check_start_ready.h"
#include "check_scan_gate.h"
#include "check_pass_gate.h"

int planner_main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_planner");

    BT::BehaviorTreeFactory factory;

    // 1. Register both custom nodes
    factory.registerNodeType<CheckStartReady>("CheckStartReady", node);
    // factory.registerNodeType<CheckScanGate>("CheckScanGate", node);
    // factory.registerNodeType<CheckPassGate>("CheckPassGate");
    // factory.registerNodeType<RunSlalom>("RunSlalom");

    // 2. Create the tree from the XML file
    auto tree = factory.createTreeFromFile("./my_tree.xml");

    // 3. Tick the tree in a loop (e.g., every 10ms / 100Hz)
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);  // deliver incoming messages (e.g. depth) to subscriptions

        BT::NodeStatus status = tree.tickOnce();

        // Check if the whole tree has finished (SUCCESS or FAILURE)
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();
    return 0;
}