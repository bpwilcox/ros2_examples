//----------------------------------------------------------------------------
//  Copyright (C) 2021, iRobot Corp.
//  This material contains trade secrets and confidential information
//  of Evolution Robotics, Inc.  Any use, reproduction, disclosure or
//  dissemination is strictly prohibited without the explicit written
//  permission of Evolution Robotics, Inc.  All rights reserved.
//----------------------------------------------------------------------------

#include "plugin-example-a.hpp"

using namespace ros2_examples;

PluginExampleA::PluginExampleA ()
{
    m_plugin_name = "Plugin Example A";
}

void PluginExampleA::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent_node)
{
    auto node = parent_node.lock();
    if (node) {
        m_pub = node->create_publisher<example_interfaces::msg::String>("topic_A", 10);
    }

    RCLCPP_INFO(rclcpp::get_logger(m_plugin_name.c_str()), "%s is configured", m_plugin_name.c_str());
}

void PluginExampleA::activate()
{
    if (m_pub) {
        m_pub->on_activate();
    }

    RCLCPP_INFO(rclcpp::get_logger(m_plugin_name.c_str()), "%s is activated", m_plugin_name.c_str());
}

void PluginExampleA::deactivate()
{
    if (m_pub) {
        m_pub->on_deactivate();
    }

    RCLCPP_INFO(rclcpp::get_logger(m_plugin_name.c_str()), "%s is deactivated", m_plugin_name.c_str());
}

void PluginExampleA::cleanup()
{
    m_pub.reset();

    RCLCPP_INFO(rclcpp::get_logger(m_plugin_name.c_str()), "%s is unconfigured", m_plugin_name.c_str());
}

void PluginExampleA::publish()
{
    example_interfaces::msg::String msg;
    msg.data = m_plugin_name + std::string(" says hello");
    if (m_pub) {
        m_pub->publish(msg);
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_examples::PluginExampleA, ros2_examples::PluginInterfaceExample)
