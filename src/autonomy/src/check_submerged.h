#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "snappy_interfaces/msg/task.hpp"

class CheckSubmerged : public BT::StatefulActionNode
{
public:
    CheckSubmerged(const std::string& name, const BT::NodeConfig& config,
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

        task_pub_ = node->create_publisher<snappy_interfaces::msg::Task>(
            "/planner/task", rclcpp::QoS(1).transient_local());
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    // Called once when first ticked
    BT::NodeStatus onStart() override
    {
        // Command the dive: hold x/y at 0, go to kTargetDepth, level orientation.
        snappy_interfaces::msg::Task task;
        task.seq   = 0;
        task.x     = 0.0;
        task.y     = 0.0;
        task.z     = kTargetDepth;   // depth is positive going down
        task.roll  = 0.0;
        task.pitch = 0.0;
        task.yaw   = 0.0;            // radians
        task_pub_->publish(task);

        return BT::NodeStatus::RUNNING;
    }

    // Called on every subsequent tick while RUNNING
    BT::NodeStatus onRunning() override
    {
        if (isSubmerged()) // true once the depth sensor says we are at or below kSubmergedDepth
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
    static constexpr double kTargetDepth = 1.0;

    bool isSubmerged()
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
    rclcpp::Publisher<snappy_interfaces::msg::Task>::SharedPtr task_pub_;

    double depth_ = 0.0;
    bool has_depth_ = false;
};
