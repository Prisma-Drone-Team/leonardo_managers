cmake_minimum_required(VERSION 3.5)
project(gcs_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)


include_directories(
  include
)

add_executable(gcs_manager src/gcs_manager.cpp)
ament_target_dependencies(gcs_manager rclcpp std_msgs tf2 tf2_ros geometry_msgs nav2_msgs)

target_link_libraries(gcs_manager
  jsoncpp )

install(TARGETS
  gcs_manager
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
