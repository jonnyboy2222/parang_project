cmake_minimum_required(VERSION 3.8)
project(autodrive_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- core library (ROS-agnostic logic) ---
add_library(safety_filter_core src/safety_filter_core.cpp)
target_include_directories(safety_filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(safety_filter_core)

# --- executables ---
add_executable(fake_lidar src/fake_lidar_node.cpp)
ament_target_dependencies(fake_lidar rclcpp std_msgs)

add_executable(safety_filter src/safety_filter_node.cpp)
target_link_libraries(safety_filter safety_filter_core)
ament_target_dependencies(safety_filter rclcpp std_msgs)

add_executable(controller src/controller_node.cpp)
ament_target_dependencies(controller rclcpp std_msgs)

# --- install ---
install(TARGETS
  fake_lidar
  safety_filter
  controller
  safety_filter_core
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
