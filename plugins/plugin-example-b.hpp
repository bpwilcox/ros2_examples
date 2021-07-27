//----------------------------------------------------------------------------
//  Copyright (C) 2021, iRobot Corp.
//  This material contains trade secrets and confidential information
//  of Evolution Robotics, Inc.  Any use, reproduction, disclosure or
//  dissemination is strictly prohibited without the explicit written
//  permission of Evolution Robotics, Inc.  All rights reserved.
//----------------------------------------------------------------------------

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/msg/int16.hpp>
#include "plugin-interface-example.hpp"

namespace ros2_examples
{

class PluginExampleB : public PluginInterfaceExample
{
public:
    PluginExampleB();

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent_node) override;

    void activate() override;

    void deactivate() override;

    void cleanup() override;

    void publish() override;

private:
    rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::Int16>::SharedPtr m_pub;
    uint16_t m_count;
};

}
