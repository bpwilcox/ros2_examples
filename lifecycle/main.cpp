//----------------------------------------------------------------------------
//  Copyright (C) 2021, iRobot Corp.
//  This material contains trade secrets and confidential information
//  of Evolution Robotics, Inc.  Any use, reproduction, disclosure or
//  dissemination is strictly prohibited without the explicit written
//  permission of Evolution Robotics, Inc.  All rights reserved.
//----------------------------------------------------------------------------

#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

using lifecycle_msgs::msg::Transition;

class LifecycleCallbackTest : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleCallbackTest()
    : rclcpp_lifecycle::LifecycleNode("lifecycle_callback_test_node")
    {
        auto cb = [this](const std::vector<rclcpp::Parameter> & parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto & parameter : parameters) {
                if (parameter.get_name() == "force_failure_id") {
                    force_failure_id_ = parameter.get_value<uint8_t>();
                }
            }

            return result;
        };

        param_handle_ = add_on_set_parameters_callback(cb);

        declare_parameter("force_failure_id", force_failure_id_);
    }

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
    {
        if (force_failure_id_ == Transition::TRANSITION_CONFIGURE) {
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        if (force_failure_id_ == Transition::TRANSITION_ACTIVATE) {
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        if (force_failure_id_ == Transition::TRANSITION_DEACTIVATE) {
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
    {
        if (force_failure_id_ == Transition::TRANSITION_CLEANUP) {
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State & state) override
    {
        if (force_failure_id_ == Transition::TRANSITION_ON_ERROR_FAILURE) {
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
    {
        if (force_failure_id_ == Transition::TRANSITION_UNCONFIGURED_SHUTDOWN) {
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
    }

    uint8_t force_failure_id_ = 0;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handle_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleCallbackTest>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
