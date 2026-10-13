cmake_minimum_required(VERSION 3.16.3)
project(omniscan_sidescan)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/omniscan450_simulator
  scripts/sidescansonar_node
  scripts/sidescansonarall_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_add_pytest_test(${PROJECT_NAME}_pytest test TIMEOUT 60)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
