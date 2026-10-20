#pragma once
#include "behaviortree_cpp/behavior_tree.h"

class CheckScanGate : public BT::StatefulActionNode
{
public:
    CheckScanGate(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    // Called once when first ticked (Kick off the scan process)
    BT::NodeStatus onStart() override
    {
        // Example: startCameraScan();
        return BT::NodeStatus::RUNNING;
    }

    // Called on every subsequent tick while RUNNING
    BT::NodeStatus onRunning() override
    {
        // Check if the gate has been successfully scanned
        if (isGateScanned())
        {
            return BT::NodeStatus::SUCCESS; // Scan complete, move to next node in tree
        }

        // Optional: Handle failure/timeout cases if scanning takes too long
        if (hasScanTimedOut())
        {
            return BT::NodeStatus::FAILURE; 
        }

        // Keep staying in this state on the next tick
        return BT::NodeStatus::RUNNING;
    }

    // Called if the tree interrupts this node (e.g., a higher-priority fallback triggers)
    void onHalt() override
    {
        // Clean up: stop the camera or reset scanning hardware
        // stopCameraScan();
    }
};