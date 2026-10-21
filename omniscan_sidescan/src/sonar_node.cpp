// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/sonar_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace omniscan_sidescan
{
namespace
{
using Descriptor = rcl_interfaces::msg::ParameterDescriptor;
using Status = diagnostic_msgs::msg::DiagnosticStatus;

Descriptor integer_descriptor(
  const std::string & description, int64_t low, int64_t high,
  bool ro = false)
{
  Descriptor d;
  d.description = description;
  d.read_only = ro;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = low;
  range.to_value = high;
  range.step = 1;
  d.integer_range.push_back(range);
  return d;
}

Descriptor float_descriptor(
  const std::string & description, double low, double high,
  bool ro = false)
{
  Descriptor d;
  d.description = description;
  d.read_only = ro;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = low;
  range.to_value = high;
  d.floating_point_range.push_back(range);
  return d;
}

Descriptor text_descriptor(const std::string & description, bool ro = false)
{
  Descriptor d;
  d.description = description;
  d.read_only = ro;
  return d;
}

bool blank(const std::string & value)
{
  return value.find_first_not_of(" \t\r\n") == std::string::npos;
}

template<typename Message>
void common_fields(Message & message, const Profile & p)
{
  message.start_mm = p.start_mm;
  message.length_mm = p.length_mm;
  message.num_results = p.num_results;
  message.sos_dmps = p.sos_dmps;
  message.channel_number = p.channel_number;
  message.vehicle_heading_deg = p.vehicle_heading_deg;
}

// The legacy messages flatten the same metadata under each head's prefix.
#define COPY_HEAD_FIELD(head, field) message.head ## _ ## field = p.field
#define COPY_HEAD(head) \
  COPY_HEAD_FIELD(head, ping_number); \
  COPY_HEAD_FIELD(head, timestamp_ms); \
  COPY_HEAD_FIELD(head, ping_hz); \
  COPY_HEAD_FIELD(head, gain_index); \
  COPY_HEAD_FIELD(head, pulse_duration_sec); \
  COPY_HEAD_FIELD(head, analog_gain); \
  COPY_HEAD_FIELD(head, max_pwr_db); \
  COPY_HEAD_FIELD(head, min_pwr_db); \
  COPY_HEAD_FIELD(head, transducer_heading_deg); \
  if constexpr (requires {message.head ## _pwr_data;}) { \
    message.head ## _pwr_data.assign(p.raw.begin(), p.raw.end()); \
  } else {message.head ## _scaled_data.assign(p.scaled.begin(), p.scaled.end());}

template<typename Message>
void head_fields(Message & message, const Profile & p, size_t index)
{
  switch (index) {
    case 0: {COPY_HEAD(port); break;}
    case 1: {COPY_HEAD(starboard); break;}
    case 2:
      if constexpr (requires {message.bow_ping_number;}) {COPY_HEAD(bow);}
      break;
    case 3:
      if constexpr (requires {message.stern_ping_number;}) {COPY_HEAD(stern);}
      break;
  }
}
#undef COPY_HEAD
#undef COPY_HEAD_FIELD

template<typename Message>
Message combined_message(
  const std::vector<ChannelSnapshot> & snapshots, const std::string & frame, int64_t stamp)
{
  Message message;
  message.header.frame_id = frame;
  message.header.stamp = rclcpp::Time(stamp);
  common_fields(message, snapshots.front().sample->profile);
  for (size_t i = 0; i < snapshots.size(); ++i) {
    head_fields(message, snapshots[i].sample->profile, i);
  }
  return message;
}

template<typename T>
void key(Status & status, const std::string & name, const T & value)
{
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = name;
  std::ostringstream stream;
  stream << value;
  entry.value = stream.str();
  status.values.push_back(std::move(entry));
}

void parameter_keys(Status & status, const std::string & prefix, const Params & p)
{
  key(status, prefix + "speed_of_sound_mm", p.speed_of_sound_mm);
  key(status, prefix + "start_mm", p.start_mm);
  key(status, prefix + "length_mm", p.length_mm);
  key(status, prefix + "msec_per_ping", p.msec_per_ping);
  key(status, prefix + "pulse_len_percent", p.pulse_len_percent);
  key(status, prefix + "filter_duration_percent", p.filter_duration_percent);
  key(status, prefix + "gain_index", p.gain_index);
  key(status, prefix + "num_results", p.num_results);
}
}  // namespace

SonarNode::SonarNode(const rclcpp::NodeOptions & options)
: SonarNode(false, options) {}

SonarNode::SonarNode(bool all_channels, const rclcpp::NodeOptions & options)
: LifecycleNode(all_channels ? "omniscanall" : "omniscan", options),
  channel_count_(all_channels ? 4 : 2)
{
  declare_parameters();
  validation_callback_ = add_on_set_parameters_callback(
    [this](const auto & values) {return validate(values);});
  committed_callback_ = add_post_set_parameters_callback([this](const auto &) {committed();});
  desired_ = parameters();
  committed();
}

SonarNode::~SonarNode()
{
  // An active component can be unloaded independently of its container.
  if (rclcpp::ok(get_node_base_interface()->get_context()) &&
    get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    try {
      shutdown();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "lifecycle shutdown: %s", error.what());
    }
  }
  stop();
}

void SonarNode::declare_parameters()
{
  declare_parameter("sensor_number", int64_t{450},
      integer_descriptor("sensor topic number", 0, 65535, true));
  declare_parameter("frame_id", "sonar", text_descriptor("combined rig frame"));
  declare_parameter("speed_of_sound_mm", int64_t{1482000},
      integer_descriptor("sound speed in mm/s", 1482000, 1500000));
  declare_parameter("start_mm", int64_t{0}, integer_descriptor("start of profile in mm", 0, 5000));
  declare_parameter("length_mm", int64_t{5000},
      integer_descriptor("profile length in mm", 1, 300000));
  declare_parameter("msec_per_ping", int64_t{0},
      integer_descriptor("minimum ping period; zero is fastest", 0, 10));
  declare_parameter("pulse_len_percent", 0.002,
      float_descriptor("pulse duration fraction", 0.002, 0.004));
  declare_parameter("filter_duration_percent", 0.0015,
      float_descriptor("filter duration fraction", 0.0015, 0.0016));
  declare_parameter("gain_index", int64_t{-1},
      integer_descriptor("-1 automatic, otherwise manual gain", -1, 7));
  declare_parameter("num_results", int64_t{600},
      integer_descriptor("samples per profile", 200, 1200));
  declare_parameter("read_timeout", 0.5,
      float_descriptor("profile deadline in seconds", 0.001, 60));
  declare_parameter("connect_timeout", 5.0,
      float_descriptor("TCP connect deadline in seconds", 0.001, 60, true));
  declare_parameter("configuration_timeout", 2.0,
      float_descriptor("profile confirmation deadline in seconds", 0.001, 60, true));
  declare_parameter("reconnect_initial_delay", 0.25,
      float_descriptor("initial retry delay in seconds", 0.001, 60, true));
  declare_parameter("reconnect_max_delay", 10.0,
      float_descriptor("maximum retry delay in seconds", 0.001, 60, true));
  declare_parameter("max_profile_age", 0.5,
      float_descriptor("maximum host receive age in seconds", 0.001, 60));
  declare_parameter("max_profile_skew", 0.05,
      float_descriptor("maximum combined host receive skew in seconds", 0.001, 60));
  declare_parameter("max_consecutive_failures", int64_t{10},
      integer_descriptor("failures before reconnect; zero disables", 0, 100000, true));
  const std::array<std::string, 4> addresses{"192.168.2.94", "192.168.2.95", "192.168.2.93",
    "192.168.2.92"};
  for (size_t i = 0; i < channel_count_; ++i) {
    declare_parameter(names_[i] + "_ip_address", addresses[i],
        text_descriptor("device address", true));
    declare_parameter(names_[i] + "_port", int64_t{51200},
        integer_descriptor("device TCP port", 1, 65535, true));
    declare_parameter(names_[i] + "_frame_id", "sonar_" + names_[i],
        text_descriptor("physical head frame", true));
  }
}

Params SonarNode::parameters(const std::vector<rclcpp::Parameter> & overrides) const
{
  const auto get = [this, &overrides](const std::string & name) {
      const auto match = [&name](const auto & p) {return p.get_name() == name;};
      const auto found = std::find_if(overrides.begin(), overrides.end(), match);
      return found == overrides.end() ? get_parameter(name) : *found;
    };
  Params p;
  p.speed_of_sound_mm = get("speed_of_sound_mm").as_int();
  p.start_mm = get("start_mm").as_int();
  p.length_mm = get("length_mm").as_int();
  p.msec_per_ping = get("msec_per_ping").as_int();
  p.pulse_len_percent = get("pulse_len_percent").as_double();
  p.filter_duration_percent = get("filter_duration_percent").as_double();
  p.gain_index = get("gain_index").as_int();
  p.num_results = get("num_results").as_int();
  p.validate();
  return p;
}

rcl_interfaces::msg::SetParametersResult SonarNode::validate(
  const std::vector<rclcpp::Parameter> & values) const
{
  rcl_interfaces::msg::SetParametersResult result;
  try {
    parameters(values);
    const auto proposed = [this, &values](const std::string & name) {
        for (const auto & value : values) {
          if (value.get_name() == name) {
            return value;
          }
        }
        return get_parameter(name);
      };
    if (blank(proposed("frame_id").as_string())) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    for (const auto * name : {"read_timeout", "connect_timeout", "configuration_timeout",
        "reconnect_initial_delay", "reconnect_max_delay", "max_profile_age", "max_profile_skew"})
    {
      const auto duration = proposed(name).as_double();
      if (!std::isfinite(duration) || duration < 0.001 || duration > 60) {
        throw std::invalid_argument(std::string(name) + " must be finite and in [0.001, 60]");
      }
    }
    result.successful = true;
  } catch (const std::exception & error) {
    result.reason = error.what();
  }
  return result;
}

void SonarNode::committed()
{
  auto params = parameters();
  const auto frame = get_parameter("frame_id").as_string();
  const auto timeout = get_parameter("read_timeout").as_double();
  const auto age = get_parameter("max_profile_age").as_double();
  const auto skew = get_parameter("max_profile_skew").as_double();
  std::lock_guard lock(mutex_);
  if (desired_ != params) {++generation_;}
  desired_ = params;
  frame_id_ = frame;
  read_timeout_ = timeout;
  max_age_ = age;
  max_skew_ = skew;
  for (auto & channel : channels_) {
    channel->configure(desired_, generation_, read_timeout_);
  }
}

SonarNode::CallbackReturn SonarNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    const auto validation = validate({});
    if (!validation.successful) {throw std::invalid_argument(validation.reason);}
    const auto initial = get_parameter("reconnect_initial_delay").as_double();
    const auto maximum = get_parameter("reconnect_max_delay").as_double();
    if (initial > maximum) {
      throw std::invalid_argument("reconnect_initial_delay exceeds reconnect_max_delay");
    }
    const auto prefix = "omniscan" + std::to_string(get_parameter("sensor_number").as_int());
    // Read ROS parameters before acquiring our state mutex. Parameter commit
    // callbacks take the ROS parameter mutex first, so reversing that order
    // would deadlock with a concurrent programmatic parameter update.
    std::vector<ChannelOptions> channel_options(channel_count_);
    std::array<std::string, 4> frames;
    for (size_t i = 0; i < channel_count_; ++i) {
      auto address = get_parameter(names_[i] + "_ip_address").as_string();
      frames[i] = get_parameter(names_[i] + "_frame_id").as_string();
      if (blank(address) || blank(frames[i])) {
        throw std::invalid_argument("device address and frame must not be empty");
      }
      auto & options = channel_options[i];
      const auto first = address.find_first_not_of(" \t\r\n");
      const auto last = address.find_last_not_of(" \t\r\n");
      options.connection.address = address.substr(first, last - first + 1);
      options.connection.port = static_cast<uint16_t>(get_parameter(names_[i] + "_port").as_int());
      options.connection.connect_timeout = get_parameter("connect_timeout").as_double();
      options.connection.configuration_timeout = get_parameter("configuration_timeout").as_double();
      options.max_consecutive_failures =
        static_cast<uint64_t>(get_parameter("max_consecutive_failures").as_int());
      options.reconnect_initial_delay = initial;
      options.reconnect_max_delay = maximum;
    }
    const auto qos = rclcpp::SensorDataQoS();
    auto clock = get_clock();
    auto logger = get_logger();
    std::lock_guard lock(mutex_);
    frames_ = std::move(frames);
    if (channel_count_ == 4) {
      raw_all_ = create_publisher<omniscan_sidescan_msgs::msg::SideScanSonarAllRaw>(prefix +
          "/range_raw", qos);
      scaled_all_ = create_publisher<omniscan_sidescan_msgs::msg::SideScanSonarAll>(prefix +
          "/range", qos);
    } else {
      raw_two_ = create_publisher<omniscan_sidescan_msgs::msg::SideScanSonarRaw>(prefix +
          "/range_raw", qos);
      scaled_two_ = create_publisher<omniscan_sidescan_msgs::msg::SideScanSonar>(prefix + "/range",
          qos);
    }
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",
        rclcpp::QoS(10));
    for (size_t i = 0; i < channel_count_; ++i) {
      head_publishers_.push_back(create_publisher<omniscan_sidescan_msgs::msg::SonarProfile>(
          prefix + "/" + names_[i] + "/profile", qos));
      auto log = [logger, name = names_[i]](const std::string & message, bool error) {
          if (error) {
            RCLCPP_WARN(logger, "%s sonar: %s", name.c_str(), message.c_str());
          } else {
            RCLCPP_INFO(logger, "%s sonar: %s", name.c_str(), message.c_str());
          }
        };
      channels_.push_back(std::make_unique<Channel>(channel_options[i], desired_, generation_,
          read_timeout_,
        [clock] {return clock->now().nanoseconds();}, [this] {profiles();}, std::move(log)));
    }
    last_profile_.fill(0);
    last_combined_.fill(0);
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {diagnostics();});
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "configure: %s", error.what());
    release_resources();
    return CallbackReturn::FAILURE;
  }
}

void SonarNode::set_publishers_active(bool active)
{
  const auto set = [active](auto & publisher) {
      if (publisher) {if (active) {publisher->on_activate();} else {publisher->on_deactivate();}}
    };
  for (auto & publisher : head_publishers_) {
    set(publisher);
  }
  set(raw_two_); set(scaled_two_); set(raw_all_); set(scaled_all_); set(diagnostics_pub_);
}

SonarNode::CallbackReturn SonarNode::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    {
      std::lock_guard lock(mutex_);
      set_publishers_active(true);
      active_.store(true);
    }
    for (auto & channel : channels_) {
      channel->start();
    }
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "activate: %s", error.what());
    stop();
    return CallbackReturn::FAILURE;
  }
}

void SonarNode::stop()
{
  active_.store(false);
  // Joining without the node mutex lets a worker finish an in-flight notify.
  for (auto & channel : channels_) {
    channel->stop();
  }
  for (auto & channel : channels_) {
    channel->join();
  }
  std::lock_guard lock(mutex_);
  set_publishers_active(false);
}

SonarNode::CallbackReturn SonarNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop();
  return CallbackReturn::SUCCESS;
}

void SonarNode::release_resources()
{
  std::lock_guard lock(mutex_);
  diagnostics_timer_.reset();
  channels_.clear();
  head_publishers_.clear();
  raw_two_.reset(); scaled_two_.reset(); raw_all_.reset(); scaled_all_.reset();
  diagnostics_pub_.reset();
}

SonarNode::CallbackReturn SonarNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop();
  release_resources();
  return CallbackReturn::SUCCESS;
}

SonarNode::CallbackReturn SonarNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop();
  release_resources();
  return CallbackReturn::SUCCESS;
}

SonarNode::CallbackReturn SonarNode::on_error(const rclcpp_lifecycle::State &)
{
  stop();
  release_resources();
  return CallbackReturn::SUCCESS;
}

SonarNode::Snapshots SonarNode::snapshots() const
{
  Snapshots result;
  result.reserve(channels_.size());
  for (const auto & channel : channels_) {
    result.push_back(channel->snapshot());
  }
  return result;
}

bool SonarNode::fresh(const ChannelSnapshot & s, Deadline now) const
{
  if (s.state != "streaming" || !s.sample || s.desired_generation != generation_ ||
    s.applied_generation != static_cast<int64_t>(generation_) ||
    s.sample->generation != generation_)
  {return false;}
  const auto age = std::chrono::duration<double>(now - s.sample->profile.received).count();
  return age >= 0 && age <= max_age_;
}

std::string SonarNode::combined_problem(const Snapshots & views, Deadline now) const
{
  Deadline oldest = Deadline::max(), newest = Deadline::min();
  for (size_t i = 0; i < views.size(); ++i) {
    if (!fresh(views[i], now)) {return names_[i] + " has no fresh confirmed profile";}
    const auto & p = views[i].sample->profile;
    const auto & first = views.front().sample->profile;
    if (p.start_mm != first.start_mm || p.length_mm != first.length_mm ||
      p.num_results != first.num_results || p.sos_dmps != first.sos_dmps)
    {return "inconsistent profile geometry";}
    oldest = std::min(oldest, p.received);
    newest = std::max(newest, p.received);
  }
  if (std::chrono::duration<double>(newest - oldest).count() > max_skew_) {
    return "host receive skew exceeds maximum";
  }
  return "";
}

void SonarNode::profiles()
{
  if (!active_.load()) {return;}
  std::lock_guard lock(mutex_);
  if (!active_.load() || !rclcpp::ok(get_node_base_interface()->get_context())) {return;}
  try {
    auto views = snapshots();
    const auto now = SteadyClock::now();
    for (size_t i = 0; i < views.size(); ++i) {
      if (!fresh(views[i], now) || views[i].sample->sequence == last_profile_[i]) {continue;}
      const auto & sample = *views[i].sample;
      const auto & p = sample.profile;
      omniscan_sidescan_msgs::msg::SonarProfile message;
      message.header.frame_id = frames_[i];
      message.header.stamp = rclcpp::Time(p.timestamp_ns);
      message.configuration_generation = sample.generation;
      common_fields(message, p);
      message.ping_number = p.ping_number;
      message.timestamp_ms = p.timestamp_ms;
      message.ping_hz = p.ping_hz;
      message.gain_index = p.gain_index;
      message.pulse_duration_sec = p.pulse_duration_sec;
      message.analog_gain = p.analog_gain;
      message.max_pwr_db = p.max_pwr_db;
      message.min_pwr_db = p.min_pwr_db;
      message.transducer_heading_deg = p.transducer_heading_deg;
      message.pwr_data.assign(p.raw.begin(), p.raw.end());
      message.scaled_data.assign(p.scaled.begin(), p.scaled.end());
      head_publishers_[i]->publish(message);
      last_profile_[i] = sample.sequence;
    }
    if (!combined_problem(views, now).empty()) {return;}
    for (size_t i = 0; i < views.size(); ++i) {
      if (views[i].sample->sequence == last_combined_[i]) {return;}
    }
    const auto oldest = std::min_element(views.begin(), views.end(),
        [](const auto & a, const auto & b) {
          return a.sample->profile.received < b.sample->profile.received;
      });
    const auto stamp = oldest->sample->profile.timestamp_ns;
    if (channel_count_ == 4) {
      raw_all_->publish(combined_message<omniscan_sidescan_msgs::msg::SideScanSonarAllRaw>(views,
          frame_id_, stamp));
      scaled_all_->publish(combined_message<omniscan_sidescan_msgs::msg::SideScanSonarAll>(views,
          frame_id_, stamp));
    } else {
      raw_two_->publish(combined_message<omniscan_sidescan_msgs::msg::SideScanSonarRaw>(views,
          frame_id_, stamp));
      scaled_two_->publish(combined_message<omniscan_sidescan_msgs::msg::SideScanSonar>(views,
          frame_id_, stamp));
    }
    for (size_t i = 0; i < views.size(); ++i) {
      last_combined_[i] = views[i].sample->sequence;
    }
  } catch (const rclcpp::exceptions::RCLError & error) {
    if (rclcpp::ok(get_node_base_interface()->get_context())) {
      RCLCPP_ERROR(get_logger(), "publish: %s", error.what());
    }
  }
}

void SonarNode::diagnostics()
{
  if (!active_.load()) {return;}
  std::lock_guard lock(mutex_);
  if (!active_.load()) {return;}
  const auto views = snapshots();
  const auto now = SteadyClock::now();
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = get_clock()->now();
  for (size_t i = 0; i < views.size(); ++i) {
    const auto & s = views[i];
    Status status;
    status.name = std::string(get_fully_qualified_name()) + "/" + names_[i];
    status.hardware_id = names_[i];
    status.level = fresh(s, now) ? Status::OK : Status::ERROR;
    status.message = status.level ==
      Status::OK ? "streaming" : s.state + ": " +
      (s.error.empty() ? "no fresh confirmed profile" : s.error);
    key(status, "state", s.state);
    key(status, "desired_generation", s.desired_generation);
    key(status, "applied_generation", s.applied_generation);
    key(status, "connection_attempts", s.connection_attempts);
    key(status, "connections", s.connections);
    key(status, "profiles_received", s.profiles_received);
    key(status, "duplicate_profiles", s.duplicate_profiles);
    key(status, "parser_errors", s.parser_errors);
    key(status, "confirmation",
        "geometry, sound speed and manual gain checked in profiles; ping period, pulse and filter fractions are commanded only");
    parameter_keys(status, "desired.", desired_);
    if (s.applied_params) {parameter_keys(status, "commanded.", *s.applied_params);}
    if (s.sample) {
      const auto & p = s.sample->profile;
      key(status, "profile_age_sec", std::chrono::duration<double>(now - p.received).count());
      key(status, "reported.start_mm", p.start_mm);
      key(status, "reported.length_mm", p.length_mm);
      key(status, "reported.num_results", p.num_results);
      key(status, "reported.sos_dmps", p.sos_dmps);
      key(status, "reported.gain_index", p.gain_index);
    }
    message.status.push_back(std::move(status));
  }
  Status combined;
  combined.name = std::string(get_fully_qualified_name()) + "/combined";
  combined.hardware_id = "sonar_rig";
  combined.message = combined_problem(views, now);
  combined.level = combined.message.empty() ? Status::OK : Status::WARN;
  if (combined.message.empty()) {
    combined.message = "fresh consistent profiles within host receive skew";
  }
  message.status.push_back(std::move(combined));
  diagnostics_pub_->publish(message);
}
}  // namespace omniscan_sidescan

RCLCPP_COMPONENTS_REGISTER_NODE(omniscan_sidescan::SonarNode)
RCLCPP_COMPONENTS_REGISTER_NODE(omniscan_sidescan::SonarAllNode)
