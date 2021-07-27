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

class PluginInterfaceExample
{
public:
    PluginInterfaceExample();

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & node);

    virtual void activate();

    virtual void deactivate();

    virtual void cleanup();

    virtual void publish();

protected:
    std::string m_plugin_name;
    rclcpp_lifecycle::LifecycleNode::WeakPtr m_node;
};

}
