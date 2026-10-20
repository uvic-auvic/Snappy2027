#pragma once
#include "behaviortree_cpp/behavior_tree.h"

class CheckStartReady : public BT::StatefulActionNode
{
public:
    CheckStartReady(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    // Called once when first ticked
    BT::NodeStatus onStart() override
    {
        return BT::NodeStatus::RUNNING;
    }

    // Called on every subsequent tick while RUNNING
    BT::NodeStatus onRunning() override
    {
        if (isStartReady()) // isStartReady will check that we actually want to start and that the sub has submerged
        {
            return BT::NodeStatus::SUCCESS; // Breaks out of the loop, proceeds in tree
        }

        // Keep staying in this state on the next tick
        return BT::NodeStatus::RUNNING;
    }

    void onHalt() override
    {
        // Clean up if interrupted
    }
};