//----------------------------------------------------------------------------
//  Copyright (C) 2021, iRobot Corp.
//  This material contains trade secrets and confidential information
//  of Evolution Robotics, Inc.  Any use, reproduction, disclosure or
//  dissemination is strictly prohibited without the explicit written
//  permission of Evolution Robotics, Inc.  All rights reserved.
//----------------------------------------------------------------------------

#include <chrono>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <performance_test/ros2/resource_usage_logger.hpp>
#include <pluginlib/class_loader.hpp>
#include "pluginlib/class_list_macros.hpp"
#include "plugin-interface-example.hpp"
#include "plugin-example-a.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class LifecyclePlugin : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecyclePlugin(int num)
    : rclcpp_lifecycle::LifecycleNode("lifecycle_plugin" + std::to_string(num))
    {}

    std::string get_name() { return std::string("Lifecycle Plugin Node");}

    void publish()
    {
        example_interfaces::msg::String msg;
        msg.data = std::string(" Lifecycle Plugin Node says hello");
        if (m_pub) {
            m_pub->publish(msg);
        }
    }

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
    {
        m_pub = create_publisher<example_interfaces::msg::String>("topic_lfecycle", 10);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        m_pub->on_activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        m_pub->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
    {
        m_pub.reset();
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr m_pub;
};

// This lifecycle node example demonstrates how to load and use plugins using pluginlib
class LifecyclePluginsExample : public rclcpp_lifecycle::LifecycleNode 
{
public:
    LifecyclePluginsExample(std::vector<std::string> default_plugin_names)
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

        auto start = high_resolution_clock::now();
        // Here we create and store unique pointers to each loaded instance of the plugins
        // for (size_t i = 0; i < m_plugin_names.size(); i++)
        // {
        //     m_plugins.push_back(m_plugin_loader.createUniqueInstance(m_plugin_names[i]));
        // }

        // for (size_t i = 0; i < m_plugin_names.size(); i++)
        // {
        //     m_plugins_n.push_back(std::make_unique<ros2_examples::PluginExampleA>());
        // }

        for (size_t i = 0; i < m_plugin_names.size(); i++)
        {
            m_plugins_l.push_back(std::make_unique<LifecyclePlugin>(i));
        }

        auto stop = high_resolution_clock::now();

        auto duration = duration_cast<microseconds>(stop - start);
        std::cout << "Time taken by loading plugins: " << duration.count() << " microseconds" << std::endl;

        // These plugins require a node during the configure step in order to register ROS2 node entities such as publishers
        for (auto & plugin : m_plugins_l) {
            // plugin->configure(node);
            plugin->configure();
            RCLCPP_INFO(this->get_logger(), "Plugin '%s' is configured", plugin->get_name().c_str());
        }

        // We create a timer to publish from the plugins. We cancel it immediately so that it won't call
        // until the node is active
        m_timer = create_wall_timer(1s, std::bind(&LifecyclePluginsExample::publish_plugins, this));
        m_timer->cancel();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        // Here we activate the plugins which in turn activates any publishers
        for (auto & plugin : m_plugins_l) {
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
        for (auto & plugin : m_plugins_l) {
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
        for (auto & plugin : m_plugins_l) {
            plugin->cleanup();
            RCLCPP_INFO(this->get_logger(), "Plugin '%s' is unconfigured", plugin->get_name().c_str());
        }

        // Remove plugins
        // std::vector<pluginlib::UniquePtr<ros2_examples::PluginInterfaceExample>>().swap(m_plugins);
        // m_plugins.clear();
        // m_plugins = std::vector<pluginlib::UniquePtr<ros2_examples::PluginInterfaceExample>>();
        // m_plugins_n = std::vector<std::unique_ptr<ros2_examples::PluginInterfaceExample>>();
        m_plugins_l = std::vector<std::unique_ptr<LifecyclePlugin>>();

        // Here we create and store unique pointers to each loaded instance of the plugins
        // for (size_t i = 0; i < m_plugin_names.size(); i++)
        // {
        //     if (m_plugin_loader.isClassLoaded(m_plugin_names[i])) {
        //         RCLCPP_INFO(this->get_logger(), "Unloading library for '%s'", m_plugin_names[i].c_str());
        //         m_plugin_loader.unloadLibraryForClass(m_plugin_names[i]);
        //     }
        // }

        // m_plugin_names = std::vector<std::string>();

        // Remove timer
        m_timer.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down");
        return CallbackReturn::SUCCESS;
    }

    void publish_plugins()
    {
        // Called every loop of the timer, publishes from every loaded plugin
        for (auto & plugin : m_plugins_l) {
            plugin->publish();
        }
    }

    rclcpp::TimerBase::SharedPtr m_timer;

    pluginlib::ClassLoader<ros2_examples::PluginInterfaceExample> m_plugin_loader {"plugins", "ros2_examples::PluginInterfaceExample"};
    std::vector<pluginlib::UniquePtr<ros2_examples::PluginInterfaceExample>> m_plugins;
    std::vector<std::unique_ptr<ros2_examples::PluginInterfaceExample>> m_plugins_n;
    std::vector<std::unique_ptr<LifecyclePlugin>> m_plugins_l;

    std::vector<std::string> m_plugin_names;
};

void spin_test_node(rclcpp_lifecycle::LifecycleNode::SharedPtr node, int transition_duration = 5, int num_plugins = 1)
{
    static std::vector <uint8_t> transitions {
      Transition::TRANSITION_CONFIGURE,
      Transition::TRANSITION_ACTIVATE,
      Transition::TRANSITION_DEACTIVATE,
      Transition::TRANSITION_CLEANUP,
      Transition::TRANSITION_UNCONFIGURED_SHUTDOWN,
    };

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    executor->add_node(node->get_node_base_interface());
    std::thread thread([=](){
        executor->spin();
    });

    thread.detach();

    std::vector<std::string> vec (num_plugins, "ros2_examples/PluginExampleA");
    node->set_parameter(rclcpp::Parameter("plugin_names", vec));

    for (int i = 0; i < transitions.size(); i++) {
        std::this_thread::sleep_for(std::chrono::seconds(transition_duration));
        node->trigger_transition(transitions[i]);
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));

    executor->cancel();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string dir_name = "plugins_log";
    std::string make_dir = "mkdir -p " + dir_name;
    const auto ret = system(make_dir.c_str());
    static_cast<void>(ret);
    std::string resources_output_path = dir_name + "/resources.txt";

    performance_test::ResourceUsageLogger ru_logger(resources_output_path);
    ru_logger.start(std::chrono::milliseconds(250));

    std::vector<std::string> plugins {"ros2_examples/PluginExampleA", "ros2_examples/PluginExampleB"};
    auto node = std::make_shared<LifecyclePluginsExample>(plugins);

    spin_test_node(node, 5, 10);
    ru_logger.stop();

    rclcpp::shutdown();
    return 0;
}
