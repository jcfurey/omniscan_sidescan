cmake_minimum_required(VERSION 3.16.3)
project(omniscan_sidescan)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

ament_auto_add_library(omniscan_protocol SHARED
  src/protocol.cpp src/connection.cpp src/channel.cpp)
target_link_libraries(omniscan_protocol Threads::Threads)
ament_auto_add_library(omniscan_components SHARED src/sonar_node.cpp)
target_link_libraries(omniscan_components omniscan_protocol)
rclcpp_components_register_nodes(omniscan_components
  "omniscan_sidescan::SonarNode" "omniscan_sidescan::SonarAllNode")

ament_auto_add_executable(sidescansonar_node src/main.cpp)
target_link_libraries(sidescansonar_node omniscan_components)
ament_auto_add_executable(sidescansonarall_node src/main.cpp)
target_compile_definitions(sidescansonarall_node PRIVATE OMNISCAN_ALL_CHANNELS)
target_link_libraries(sidescansonarall_node omniscan_components)
foreach(target omniscan_protocol omniscan_components sidescansonar_node sidescansonarall_node)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/omniscan450_simulator
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/sidescansonar_node DESTINATION lib/${PROJECT_NAME}
  RENAME sidescansonar_node_python)
install(PROGRAMS scripts/sidescansonarall_node DESTINATION lib/${PROJECT_NAME}
  RENAME sidescansonarall_node_python)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol omniscan_protocol)
  ament_add_gtest(test_lifecycle test/test_lifecycle.cpp)
  target_link_libraries(test_lifecycle omniscan_components)
  ament_uncrustify()
  ament_add_pytest_test(${PROJECT_NAME}_pytest test TIMEOUT 120)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
