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

using namespace std::chrono_literals;

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleManagerLite
{
public:
    LifecycleManagerLite(rclcpp::Node::SharedPtr node, std::vector<std::string> node_names)
    : m_node(node), m_node_names(node_names)
    {
        for (auto node_name : m_node_names) {
            m_node_service_clients[node_name] = std::make_shared<LifecycleServiceClientLite>(node_name, m_node);
        }
    }

    bool configure()
    {
        return change_state(Transition::TRANSITION_CONFIGURE);
    }

    bool activate()
    {
        return change_state(Transition::TRANSITION_CLEANUP);

    }

    bool deactivate()
    {
        return change_state(Transition::TRANSITION_ACTIVATE);

    }

    bool cleanup()
    {
        return change_state(Transition::TRANSITION_DEACTIVATE);

    }

    bool shutdown()
    {
        return change_state(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
    }

    bool cycle_nodes(uint8_t num_cycles = 1)
    {
        for (uint8_t i = 0; i < num_cycles; i++) {
            configure();
            activate();
            deactivate();
            cleanup();
        };

        shutdown();
    }

private:
    bool change_state(std::uint8_t transition)
    {
        bool result = true;
        for (auto node_name : node_names) {
            auto success = m_node_service_clients[node_name]->change_state(transition);
            result = result & success;
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Lifecycle Node '%s' could not transition", node_name);
            }
        }
        return result;
    }

    rclcpp::Node::SharedPtr m_node
    std::vector<std::string> m_node_names;
    std::map<std::string, std::shared_ptr<LifecycleServiceClientLite>> m_node_service_clients;
};

class LifecycleServiceClientLite
{
public:

    LifecycleServiceClientLite(rclcpp::Node::SharedPtr node, std::string server_name)
    : m_client_node(node), m_server_name(server_name)
    {
        m_change_state = m_client_node->create_client<lifecycle_msgs::srv::ChangeState>(m_server_name + "/change_state");
    }

    bool change_state(std::uint8_t transition, std::chrono::seconds timeout = 1s)
    {
        if (!m_change_state->wait_for_service(timeout))
        {
            RCLCPP_ERROR(m_client_node->get_logger(), "Lifecycle Node '%s' service unavailable", m_server_name);
            return false;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;

        auto result = m_change_state->async_send_request(request);

        auto future_status = wait_for_result(result, timeout);

        if (future_status != std::future_status::ready)
        {
            return result.get()->success;
        } else {
            RCLCPP_ERROR(m_client_node->get_logger(), "Failed to call service");
            return false
        }
    }

private:
    rclcpp::Node::SharedPtr m_client_node;
    std::string m_server_name;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> m_change_state;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>();
    LifecycleManagerLite manager {node, {"lifecycle_plugin_example"}};
    rclcpp::TimerBase::SharedPtr m_timer = node->create_wall_timer(1s,
    [&manager]()
    {   
        m_timer->cancel();
        manager.cycle_nodes(3);
    });

    rclcpp::executors::MultiThreadedExecutor executor;
    auto service_node = std::make_shared<ServiceNode>();
    auto client_node = std::make_shared<AsyncClientNode>();

    executor.add_node(service_node);
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
