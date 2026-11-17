#pragma once
#include "behaviortree_cpp/behavior_tree.h"

class RunSlalom : public BT::StatefulActionNode
{
public:
    RunSlalom(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    // Called once when first ticked (Kick off ...)
    BT::NodeStatus onStart() override
    {
        // Example: ...();
        return BT::NodeStatus::RUNNING;
    }

    // Called on every subsequent tick while RUNNING
    BT::NodeStatus onRunning() override
    {
        // Check if the slalom has been successfully ran
        if (isSlalomRan())
        {
            return BT::NodeStatus::SUCCESS; // Pass complete, move to next node in tree
        }

        // Optional: Handle failure/timeout cases if passing takes too long
        if (hasRunTimedOut()) // if (hasSlalomRunTimedOut())
        {
            return BT::NodeStatus::FAILURE; 
        }

        // Keep staying in this state on the next tick
        return BT::NodeStatus::RUNNING;
    }

    // Called if the tree interrupts this node (e.g., a higher-priority fallback triggers)
    void onHalt() override
    {
        // Clean up: ...
        // ...();
    }
};