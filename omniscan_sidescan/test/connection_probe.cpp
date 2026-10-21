// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/connection.hpp"

#include <iostream>

namespace os = omniscan_sidescan;

// A stdin gate lets the independent Python peer finish writing AND half-close
// before read_profile drains TCP, making the final-profile/EOF race repeatable.
int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  try {
    std::atomic_bool stop{false};
    os::ConnectionOptions options;
    options.address = "127.0.0.1";
    options.port = static_cast<uint16_t>(std::stoul(argv[1]));
    options.connect_timeout = 0.5;
    options.configuration_timeout = 2;
    os::Params params;
    params.num_results = 200;
    os::Connection connection(options, stop, [] {return 123456789LL;});
    connection.initialize(params);
    connection.read_profile(0.5);
    std::cout << "READY" << std::endl;
    char gate;
    std::cin.get(gate);
    try {
      const auto profile = connection.read_profile(0.5);
      std::cout << "PROFILE " << profile.ping_number << std::endl;
    } catch (const os::ConfigurationError &) {
      std::cout << "CONFIGURATION_ERROR" << std::endl;
    } catch (const os::DataError &) {
      std::cout << "DATA_ERROR" << std::endl;
    } catch (const os::ConnectionError &) {
      std::cout << "CONNECTION_ERROR" << std::endl;
    }
    try {
      connection.read_profile(0.5);
      return 3;
    } catch (const os::ConnectionError &) {
      std::cout << "CLOSED" << std::endl;
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
