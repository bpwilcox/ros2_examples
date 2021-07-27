//----------------------------------------------------------------------------
//  Copyright (C) 2021, iRobot Corp.
//  This material contains trade secrets and confidential information
//  of Evolution Robotics, Inc.  Any use, reproduction, disclosure or
//  dissemination is strictly prohibited without the explicit written
//  permission of Evolution Robotics, Inc.  All rights reserved.
//----------------------------------------------------------------------------

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <pluginlib/class_loader.hpp>
#include "pluginlib/class_list_macros.hpp"
#include "plugin-interface-example.hpp"

using namespace std::chrono_literals;

class LifecyclePluginsExample : public rclcpp_lifecycle::LifecycleNode 
{
public:
    LifecyclePluginsExample(std::vector<std::string> plugin_names)
    : rclcpp_lifecycle::LifecycleNode("lifecycle_plugin_example"), m_plugin_names(plugin_names)
    {}

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
    {
        auto node = shared_from_this();

        for (size_t i = 0; i < m_plugin_names.size(); i++)
        {
            m_plugins.push_back(m_plugin_loader.createUniqueInstance(m_plugin_names[i]));
        }

        for (auto & plugin : m_plugins) {
            plugin->configure(node);
        }

        m_timer = create_wall_timer(1s, std::bind(&LifecyclePluginsExample::publish_plugins, this));
        m_timer->cancel();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        for (auto & plugin : m_plugins) {
            plugin->activate();
        }

        m_timer->reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        for (auto & plugin : m_plugins) {
            plugin->deactivate();
        }

        m_timer->cancel();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
    {
        for (auto & plugin : m_plugins) {
            plugin->cleanup();
        }

        m_plugins.clear();
        m_timer.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
    {
        return CallbackReturn::SUCCESS;
    }

    void publish_plugins()
    {
        for (auto & plugin : m_plugins) {
            plugin->publish();
        }
    }

    rclcpp::TimerBase::SharedPtr m_timer;

    pluginlib::ClassLoader<ros2_examples::PluginInterfaceExample> m_plugin_loader {"plugins", "ros2_examples::PluginInterfaceExample"};
    std::vector<pluginlib::UniquePtr<ros2_examples::PluginInterfaceExample>> m_plugins;
    std::vector<std::string> m_plugin_names;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::vector<std::string> plugins {"ros2_examples/PluginExampleA", "ros2_examples/PluginExampleB"};
  auto node = std::make_shared<LifecyclePluginsExample>(plugins);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
