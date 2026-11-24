#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class CheckStartReady : public BT::StatefulActionNode
{
public:
    CheckStartReady(const std::string& name, const BT::NodeConfig& config,
                    rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config)
    {
        // Listen to the depth sensor (published by drivers/src/pressure_sensor.cpp).
        // Every time a new reading arrives, save it in depth_.
        depth_sub_ = node->create_subscription<std_msgs::msg::Float32>(
            "/depth_data", 10,
            [this](const std_msgs::msg::Float32& msg)
            {
                depth_ = msg.data;
                has_depth_ = true;
            });
    }

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

private:
    // The sub counts as submerged once it is at least this deep (metres).
    // The controller holds the sub at 1.0 m (launch/config/controller_params.yaml).
    static constexpr double kSubmergedDepth = 0.85;

    bool isStartReady()
    {
        // No reading from the depth sensor yet -> not ready.
        if (!has_depth_)
        {
            return false;
        }

        // Depth is positive going down, so "deep enough" means depth_ >= kSubmergedDepth.
        return depth_ >= kSubmergedDepth;
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
    double depth_ = 0.0;
    bool has_depth_ = false;
};
