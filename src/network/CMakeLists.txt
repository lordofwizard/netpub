cmake_minimum_required(VERSION 3.8)
project(network)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()



add_executable(network_status_publisher src/network_status_publisher.cpp)
# Add dependencies to the executable
ament_target_dependencies(network_status_publisher rclcpp std_msgs)
install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
network_status_publisher 
  DESTINATION lib/${PROJECT_NAME})

  
#Required for dependencies and other folder
# install(
#   DIRECTORY 
#   scripts
#   DESTINATION share/${PROJECT_NAME}
# )
ament_package()

