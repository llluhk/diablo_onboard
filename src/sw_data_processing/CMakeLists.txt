cmake_minimum_required(VERSION 3.5)
project(sw_data_processing)

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ception_msgs REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Boost REQUIRED)  # 添加 Boost

# 生成可执行文件
add_executable(sw_data_processing_node src/sw_data_processing.cpp)

# 链接库
ament_target_dependencies(sw_data_processing_node
  rclcpp
  sensor_msgs
  motion_msgs
  ception_msgs
  std_msgs
  message_filters
  custom_msgs
)

# 链接 Boost
target_link_libraries(sw_data_processing_node Boost::boost)

# 安装可执行文件
install(TARGETS
  sw_data_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

# 测试依赖
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


