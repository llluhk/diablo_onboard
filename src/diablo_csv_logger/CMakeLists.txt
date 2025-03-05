cmake_minimum_required(VERSION 3.5)
project(diablo_csv_logger)

# Default to C++14 for Foxy
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ception_msgs REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(message_filters REQUIRED)

# Add executable
add_executable(diablo_csv_logger src/diablo_csv_logger.cpp)

# Link dependencies
ament_target_dependencies( 
  diablo_csv_logger
  rclcpp
  sensor_msgs
  motion_msgs
  ception_msgs
  message_filters
  )        
  

# Install
install(TARGETS
  diablo_csv_logger
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()