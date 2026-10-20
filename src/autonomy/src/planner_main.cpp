#include "behaviortree_cpp/behavior_tree.h"
#include <thread>
#include <chrono>

// (Include or define your CheckStartReady and ScanGate classes here)
#include "check_start_ready.h"
#include "check_scan_gate.h"
#include "check_pass_gate.h"

int planner_main()
{
    BT::BehaviorTreeFactory factory;

    // 1. Register both custom nodes
    factory.registerNodeType<CheckStartReady>("CheckStartReady");
    factory.registerNodeType<ScanGate>("ScanGate");
    factory.registerNodeType<PassGate>("PassGate");

    // 2. Create the tree from the XML file
    auto tree = factory.createTreeFromFile("./my_tree.xml");

    // 3. Tick the tree in a loop (e.g., every 10ms / 100Hz)
    while (true)
    {
        BT::NodeStatus status = tree.tickOnce();

        // Check if the whole tree has finished (SUCCESS or FAILURE)
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}