cmake_minimum_required(VERSION 3.8)
project(my_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_first_node src/my_first_node.cpp)

ament_target_dependencies(my_first_node rclcpp std_msgs)

install(TARGETS
  my_first_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
