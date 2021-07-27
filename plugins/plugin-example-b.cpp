//----------------------------------------------------------------------------
//  Copyright (C) 2021, iRobot Corp.
//  This material contains trade secrets and confidential information
//  of Evolution Robotics, Inc.  Any use, reproduction, disclosure or
//  dissemination is strictly prohibited without the explicit written
//  permission of Evolution Robotics, Inc.  All rights reserved.
//----------------------------------------------------------------------------

#include "plugin-example-b.hpp"

using namespace ros2_examples;

PluginExampleB::PluginExampleB ()
: m_count(0)
{
    m_plugin_name = "Plugin Example B";
}

void PluginExampleB::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent_node)
{
    auto node = parent_node.lock();
    if (node) {
        m_pub = node->create_publisher<example_interfaces::msg::Int16>("topic_B", 10);
    }

    RCLCPP_INFO(rclcpp::get_logger(m_plugin_name.c_str()), "%s is configured", m_plugin_name.c_str());
}

void PluginExampleB::activate()
{
    if (m_pub) {
        m_pub->on_activate();
    }

    RCLCPP_INFO(rclcpp::get_logger(m_plugin_name.c_str()), "%s is activated", m_plugin_name.c_str());
}

void PluginExampleB::deactivate()
{
    if (m_pub) {
        m_pub->on_deactivate();
    }

    RCLCPP_INFO(rclcpp::get_logger(m_plugin_name.c_str()), "%s is deactivated", m_plugin_name.c_str());
}

void PluginExampleB::cleanup()
{
    m_pub.reset();

    RCLCPP_INFO(rclcpp::get_logger(m_plugin_name.c_str()), "%s is unconfigured", m_plugin_name.c_str());
}

void PluginExampleB::publish()
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

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_examples::PluginExampleB, ros2_examples::PluginInterfaceExample)
