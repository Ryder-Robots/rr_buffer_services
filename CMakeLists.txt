cmake_minimum_required(VERSION 3.8)
project(rr_buffer_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(rr_common_plugins REQUIRED)
find_package(rr_common_base REQUIRED)
find_package(pluginlib REQUIRED)

# Project library binaries
add_library(${PROJECT_NAME}_lib
  src/rr_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp rr_interfaces std_msgs sensor_msgs builtin_interfaces rr_common_base pluginlib)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp )

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)  

# Example of installing an executable
install(TARGETS 
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Build and link test using same library ---
if(BUILD_TESTING)
  add_subdirectory(test)
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}_test test/rr_controller_test.cpp)
  # target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_lib)
  # ament_target_dependencies(${PROJECT_NAME}_test rclcpp rr_interfaces sensor_msgs)
endif()

ament_package()
