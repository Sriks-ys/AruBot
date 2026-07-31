cmake_minimum_required(VERSION 3.8)
project(miscellaneous)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(tf_odometry_relay src/tf_odometry_relay.cpp)

ament_target_dependencies(
  tf_odometry_relay
  rclcpp
  tf2_msgs
)

install(
  TARGETS
    tf_odometry_relay
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
