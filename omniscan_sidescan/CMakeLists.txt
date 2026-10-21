cmake_minimum_required(VERSION 3.16.3)
project(omniscan_sidescan)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Threads REQUIRED)
find_package(c-ares 1.34 REQUIRED)
ament_export_dependencies(c-ares)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

ament_auto_add_library(omniscan_protocol SHARED
  src/protocol.cpp src/connection.cpp src/channel.cpp src/resolver.cpp src/publication.cpp)
target_link_libraries(omniscan_protocol Threads::Threads c-ares::cares)
ament_auto_add_library(omniscan_runtime SHARED src/sonar_node.cpp)
target_link_libraries(omniscan_runtime omniscan_protocol)
# Keep plugin registration out of libraries linked by ordinary executables.
add_library(omniscan_components SHARED src/components.cpp)
target_link_libraries(omniscan_components omniscan_runtime)
install(TARGETS omniscan_components LIBRARY DESTINATION lib)
# GNU unique symbols pin plugins until process exit, after class_loader's
# registration storage may be destroyed. Keep this plugin genuinely unloadable.
target_compile_options(omniscan_components PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fno-gnu-unique>)
rclcpp_components_register_nodes(omniscan_components
  "omniscan_sidescan::SonarNode" "omniscan_sidescan::SonarAllNode")

ament_auto_add_executable(sidescansonar_node NO_TARGET_LINK_LIBRARIES src/main.cpp)
target_link_libraries(sidescansonar_node omniscan_runtime)
ament_auto_add_executable(sidescansonarall_node NO_TARGET_LINK_LIBRARIES src/main.cpp)
target_compile_definitions(sidescansonarall_node PRIVATE OMNISCAN_ALL_CHANNELS)
target_link_libraries(sidescansonarall_node omniscan_runtime)
foreach(target omniscan_protocol omniscan_runtime omniscan_components
    sidescansonar_node sidescansonarall_node)
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
  target_link_libraries(test_lifecycle omniscan_runtime)
  ament_add_gtest(test_transport test/test_transport.cpp)
  target_link_libraries(test_transport omniscan_protocol)
  ament_add_gtest(test_plugin_lifetime test/test_plugin_lifetime.cpp)
  target_link_libraries(test_plugin_lifetime
    class_loader::class_loader rclcpp_components::component ${CMAKE_DL_LIBS})
  target_compile_definitions(test_plugin_lifetime PRIVATE
    OMNISCAN_PLUGIN_PATH="$<TARGET_FILE:omniscan_components>")
  add_dependencies(test_plugin_lifetime omniscan_components)
  add_executable(channel_stress test/channel_stress.cpp)
  target_link_libraries(channel_stress PRIVATE omniscan_protocol)
  target_compile_options(channel_stress PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(connection_probe test/connection_probe.cpp)
  target_link_libraries(connection_probe PRIVATE omniscan_protocol)
  target_compile_options(connection_probe PRIVATE -Wall -Wextra -Wpedantic)
  ament_uncrustify()
  add_library(publication_stall SHARED test/publication_stall.cpp)
  target_link_libraries(publication_stall PRIVATE rclcpp::rclcpp ${CMAKE_DL_LIBS})
  ament_add_pytest_test(${PROJECT_NAME}_pytest test TIMEOUT 180
    ENV "OMNISCAN_CHANNEL_STRESS_EXECUTABLE=$<TARGET_FILE:channel_stress>"
        "OMNISCAN_CONNECTION_PROBE_EXECUTABLE=$<TARGET_FILE:connection_probe>"
        "OMNISCAN_PUBLICATION_STALL_LIBRARY=$<TARGET_FILE:publication_stall>")
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
