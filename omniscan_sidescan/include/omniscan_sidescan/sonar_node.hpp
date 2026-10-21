// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#pragma once

#include "omniscan_sidescan/channel.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <omniscan_sidescan_msgs/msg/side_scan_sonar.hpp>
#include <omniscan_sidescan_msgs/msg/side_scan_sonar_all.hpp>
#include <omniscan_sidescan_msgs/msg/side_scan_sonar_all_raw.hpp>
#include <omniscan_sidescan_msgs/msg/side_scan_sonar_raw.hpp>
#include <omniscan_sidescan_msgs/msg/sonar_profile.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <array>

namespace omniscan_sidescan
{
class SonarNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SonarNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SonarNode() override;

protected:
  SonarNode(bool all_channels, const rclcpp::NodeOptions & options);
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  using Snapshots = std::vector<ChannelSnapshot>;
  using HeadPublisher = rclcpp_lifecycle::LifecyclePublisher<omniscan_sidescan_msgs::msg::SonarProfile>;
  void declare_parameters();
  Params parameters(const std::vector<rclcpp::Parameter> & overrides = {}) const;
  rcl_interfaces::msg::SetParametersResult validate(
    const std::vector<rclcpp::Parameter> & values) const;
  void committed();
  void stop();
  void set_publishers_active(bool active);
  void release_resources();
  void profiles();
  void diagnostics();
  bool fresh(const ChannelSnapshot & snapshot, Deadline now) const;
  std::string combined_problem(const Snapshots & snapshots, Deadline now) const;
  Snapshots snapshots() const;

  const size_t channel_count_;
  const std::array<std::string, 4> names_{"port", "starboard", "bow", "stern"};
  std::vector<std::unique_ptr<Channel>> channels_;
  std::array<std::string, 4> frames_;
  std::string frame_id_;
  Params desired_;
  uint64_t generation_{};
  double read_timeout_{0.5}, max_age_{0.5}, max_skew_{0.05};
  std::array<uint64_t, 4> last_profile_{}, last_combined_{};
  std::atomic_bool active_{false};
  mutable std::mutex mutex_;
  std::vector<HeadPublisher::SharedPtr> head_publishers_;
  rclcpp_lifecycle::LifecyclePublisher<omniscan_sidescan_msgs::msg::SideScanSonarRaw>::SharedPtr
    raw_two_;
  rclcpp_lifecycle::LifecyclePublisher<omniscan_sidescan_msgs::msg::SideScanSonar>::SharedPtr
    scaled_two_;
  rclcpp_lifecycle::LifecyclePublisher<omniscan_sidescan_msgs::msg::SideScanSonarAllRaw>::SharedPtr
    raw_all_;
  rclcpp_lifecycle::LifecyclePublisher<omniscan_sidescan_msgs::msg::SideScanSonarAll>::SharedPtr
    scaled_all_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validation_callback_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr committed_callback_;
};

class SonarAllNode : public SonarNode
{
public:
  explicit SonarAllNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SonarNode(true, options) {}
};
}  // namespace omniscan_sidescan
