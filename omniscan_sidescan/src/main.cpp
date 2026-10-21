// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/sonar_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <csignal>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}  // namespace

int main(int argc, char ** argv)
{
  const auto previous_int = std::signal(SIGINT, request_stop);
  const auto previous_term = std::signal(SIGTERM, request_stop);
  int result = 0;
  try {
    // Complete lifecycle shutdown while the ROS context is still valid. Signal
    // handlers only set a flag; ROS work stays on the executor thread.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
#ifdef OMNISCAN_ALL_CHANNELS
    auto node = std::make_shared<omniscan_sidescan::SonarAllNode>();
#else
    auto node = std::make_shared<omniscan_sidescan::SonarNode>();
#endif
    // Existing ros2 run/launch entry points remain immediately usable.
    // Components start unconfigured for an external lifecycle manager.
    if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
      node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      result = 1;
    } else {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      while (rclcpp::ok() && !stop_requested &&
        node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
      {
        executor.spin_once(std::chrono::milliseconds(50));
      }
      executor.remove_node(node->get_node_base_interface());
      if (rclcpp::ok() && node->get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
      {
        node->shutdown();
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("omniscan_sidescan"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  std::signal(SIGINT, previous_int);
  std::signal(SIGTERM, previous_term);
  return result;
}
