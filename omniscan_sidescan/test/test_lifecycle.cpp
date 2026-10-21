// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/sonar_node.hpp"

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <limits>

class Lifecycle : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(Lifecycle, StartsUnconfiguredAndParametersArePurelyValidated)
{
  auto node = std::make_shared<omniscan_sidescan::SonarNode>();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  int commits = 0;
  auto observer = node->add_post_set_parameters_callback([&commits](const auto &) {++commits;});
  auto veto = node->add_on_set_parameters_callback([](const auto &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.reason = "test veto";
        return result;
    });
  EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter("length_mm", 7000)}).successful);
  EXPECT_EQ(node->get_parameter("length_mm").as_int(), 5000);
  EXPECT_EQ(commits, 0);
  node->remove_on_set_parameters_callback(veto.get());
  EXPECT_TRUE(node->set_parameters_atomically({rclcpp::Parameter("length_mm", 7000)}).successful);
  EXPECT_EQ(commits, 1);
  EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter("frame_id", " ")}).successful);
  EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter("num_results",
      1201)}).successful);
  EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter("port_port", 1234)}).successful);
  EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter("read_timeout",
      std::numeric_limits<double>::quiet_NaN())}).successful);
  EXPECT_EQ(commits, 1);
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  // Configuration creates ROS resources; only activation starts TCP workers.
  EXPECT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node->shutdown();
}

TEST_F(Lifecycle, InvalidInitialSettingsFailConfiguration)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("reconnect_initial_delay", 2.0),
      rclcpp::Parameter("reconnect_max_delay", 1.0)});
  auto node = std::make_shared<omniscan_sidescan::SonarAllNode>(options);
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node->shutdown();
}

TEST_F(Lifecycle, NonfiniteInitialTimeoutCannotActivateHardware)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("read_timeout",
      std::numeric_limits<double>::quiet_NaN())});
  try {
    auto node = std::make_shared<omniscan_sidescan::SonarNode>(options);
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    node->shutdown();
  } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
    // Some ROS distributions reject NaN during declaration itself.
  }
}
