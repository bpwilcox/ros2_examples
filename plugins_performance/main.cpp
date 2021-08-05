//----------------------------------------------------------------------------
//  Copyright (C) 2021, iRobot Corp.
//  This material contains trade secrets and confidential information
//  of Evolution Robotics, Inc.  Any use, reproduction, disclosure or
//  dissemination is strictly prohibited without the explicit written
//  permission of Evolution Robotics, Inc.  All rights reserved.
//----------------------------------------------------------------------------

#include <string>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <pluginlib/class_loader.hpp>
#include "pluginlib/class_list_macros.hpp"
#include "plugin-interface-lifecycle.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;

// This lifecycle node example demonstrates how to load and use plugins using pluginlib
class LifecyclePluginsNode : public rclcpp_lifecycle::LifecycleNode 
{
public:
    LifecyclePluginsNode(std::vector<std::string> default_plugin_names)
    : rclcpp_lifecycle::LifecycleNode("lifecycle_plugin_example")
    {
        // Declare default plugins in the constructor. They can be changed during runtime if not read-only.
        declare_parameter("plugin_names", default_plugin_names);
    }

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
    {
        auto node = shared_from_this();
        
        // The node may be re-configured with different plugins by grabbing new parameter values off the node 
        get_parameter("plugin_names", m_plugin_names);

        // Here we create and store unique pointers to each loaded instance of the plugins
        for (size_t i = 0; i < m_plugin_names.size(); i++)
        {
            m_plugins.push_back(m_plugin_loader.createUniqueInstance(m_plugin_names[i]));
        }

        // These plugins require a node during the configure step in order to register ROS2 node entities such as publishers
        for (auto & plugin : m_plugins) {
            plugin->configure(node);
            RCLCPP_INFO(this->get_logger(), "Plugin '%s' is configured", plugin->get_name().c_str());
        }

        // We create a timer to publish from the plugins. We cancel it immediately so that it won't call
        // until the node is active
        m_timer = create_wall_timer(1s, std::bind(&LifecyclePluginsNode::publish_plugins, this));
        m_timer->cancel();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        // Here we activate the plugins which in turn activates any publishers
        for (auto & plugin : m_plugins) {
            plugin->activate();
            RCLCPP_INFO(this->get_logger(), "Plugin '%s' is activated", plugin->get_name().c_str());
        }

        // Start the timer
        m_timer->reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        // Deactivate the plugins which will deactivate any publishers
        for (auto & plugin : m_plugins) {
            plugin->deactivate();
            RCLCPP_INFO(this->get_logger(), "Plugin '%s' is deactivated", plugin->get_name().c_str());
        }

        // Stop the timer
        m_timer->cancel();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
    {
        // Cleanup the plugins to reset and deallocate any memory resources
        for (auto & plugin : m_plugins) {
            plugin->cleanup();
            RCLCPP_INFO(this->get_logger(), "Plugin '%s' is unconfigured", plugin->get_name().c_str());
        }

        // Remove plugins 
        m_plugins.clear();

        // Remove timer
        m_timer.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
    {
        // Only allow shutdown transition to succeed if already in the unconfigured state.
        if (previous_state.id() != State::PRIMARY_STATE_UNCONFIGURED) {
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "Shutting down");
        return CallbackReturn::SUCCESS;
    }

    void publish_plugins()
    {
        // Called every loop of the timer, publishes from every loaded plugin
        for (auto & plugin : m_plugins) {
            plugin->publish();
        }
    }

    rclcpp::TimerBase::SharedPtr m_timer;

    pluginlib::ClassLoader<ros2_examples::PluginInterfaceLifecycle> m_plugin_loader {"plugins", "ros2_examples::PluginInterfaceLifecycle"};
    std::vector<pluginlib::UniquePtr<ros2_examples::PluginInterfaceLifecycle>> m_plugins;
    std::vector<std::string> m_plugin_names;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::vector<std::string> plugins {"ros2_examples/PluginLifecycleA", "ros2_examples/PluginLifecycleB"};
  auto node = std::make_shared<LifecyclePluginsNode>(plugins);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
