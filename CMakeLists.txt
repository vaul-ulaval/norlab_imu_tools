cmake_minimum_required(VERSION 3.8)
project(norlab_imu_tools)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
        ${norlab_imu_tools_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/imu_bias_observer.cpp
  src/imu_and_wheel_odom_node.cpp
  src/imu_bias_compensator_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN norlab_imu_tools::imuAndWheelOdomNode
  EXECUTABLE imu_and_wheel_odom_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN norlab_imu_tools::imuBiasCompensatorNode
  EXECUTABLE imu_bias_compensator_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN norlab_imu_tools::imuBiasObserverNode
  EXECUTABLE imu_bias_observer_node
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
