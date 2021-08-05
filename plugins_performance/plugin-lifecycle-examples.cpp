#include "plugin-lifecycle-examples.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_examples::PluginLifecycleA, ros2_examples::PluginInterfaceLifecycle)
PLUGINLIB_EXPORT_CLASS(ros2_examples::PluginLifecycleB, ros2_examples::PluginInterfaceLifecycle)
