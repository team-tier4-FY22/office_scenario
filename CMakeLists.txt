cmake_minimum_required(VERSION 3.8)
project(office_scenario)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(OFFICE_SCENARIO_SRC
  src/office_scenario_core.cpp)

set(OFFICE_SCENARIO_HEADERS
  include/office_scenario/office_scenario_core.hpp)

ament_auto_add_executable(${PROJECT_NAME}
  src/office_scenario_node.cpp
  ${OFFICE_SCENARIO_SRC}
  ${OFFICE_SCENARIO_HEADERS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
