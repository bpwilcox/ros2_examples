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
#include <example_interfaces/msg/string.hpp>
#include "plugin-lifecycle.hpp"

namespace ros2_examples
{

class PluginLifecycleA : public PluginLifecycle<example_interfaces::msg::String>
{
public:
    PluginLifecycleA()
    : PluginLifecycle<example_interfaces::msg::String>()
    {
        m_plugin_name = "Plugin Example A";
        m_topic_name = "topic_A";
    }

    void publish() override
    {
        example_interfaces::msg::String msg;
        msg.data = m_plugin_name + std::string(" says hello");
        if (m_pub) {
            m_pub->publish(msg);
        }
    }
};

class PluginLifecycleB : public PluginLifecycle<example_interfaces::msg::Int16>
{
public:
    PluginLifecycleB()
    : PluginLifecycle<example_interfaces::msg::Int16>(), m_count(0)
    {
        m_plugin_name = "Plugin Example B";
        m_topic_name = "topic_B";
    }

    void publish() override
    {
        example_interfaces::msg::Int16 msg;
        msg.data = m_count;;
        if (m_pub) {
            m_pub->publish(msg);
            m_count++;
            if (m_count > 100) {
                m_count = 0;
            }
        }
    }

private:
    uint16_t m_count;
};

}
