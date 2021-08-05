//----------------------------------------------------------------------------
//  Copyright (C) 2021, iRobot Corp.
//  This material contains trade secrets and confidential information
//  of Evolution Robotics, Inc.  Any use, reproduction, disclosure or
//  dissemination is strictly prohibited without the explicit written
//  permission of Evolution Robotics, Inc.  All rights reserved.
//----------------------------------------------------------------------------

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace ros2_examples
{

// A plugin base class that provides virtual functions to align with lifecycle transitions
class PluginInterfaceLifecycle
{
public:
    virtual ~PluginInterfaceLifecycle() {}

    // Plugin base class constructors cannot have parameters, so a node pointer is provided
    // during the configure transition
    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & node) = 0;

    virtual void activate() = 0;

    virtual void deactivate() = 0;

    virtual void cleanup() = 0;

    virtual void publish() = 0;

    std::string get_name() { return m_plugin_name; }

protected:
    std::string m_plugin_name;
};

}
