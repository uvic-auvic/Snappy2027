#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>

#include "check_submerged.h"
#include "check_surfaced.h"
// #include "check_scan_gate.h"
// #include "check_pass_gate.h"
// #include "run_slalom.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_planner");

    BT::BehaviorTreeFactory factory;

    // 1. Register both custom nodes
    factory.registerNodeType<CheckSubmerged>("CheckSubmerged", node);
    factory.registerNodeType<CheckSurfaced>("CheckSurfaced", node);
    // factory.registerNodeType<CheckScanGate>("CheckScanGate", node);
    // factory.registerNodeType<CheckPassGate>("CheckPassGate");
    // factory.registerNodeType<RunSlalom>("RunSlalom");

    // 2. Create the tree from the XML file
    node->declare_parameter("tree_file", std::string(""));
    auto tree = factory.createTreeFromFile(node->get_parameter("tree_file").as_string());

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