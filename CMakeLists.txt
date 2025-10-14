cmake_minimum_required(VERSION 3.8)
project(rr_buffer_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rr_interfaces REQUIRED)

# Project library binaries
add_library(${PROJECT_NAME}_lib
  src/rr_image_subscriber.cpp
  src/rr_subscriber_factory.cpp
  src/rr_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp rr_interfaces std_msgs sensor_msgs builtin_interfaces)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


add_executable(${PROJECT_NAME}_node
  src/rrbuffer.cpp )

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)  

# Example of installing an executable
install(TARGETS 
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(rclcpp REQUIRED)
#   find_package(sensor_msgs REQUIRED)
#   find_package(builtin_interfaces REQUIRED)
#   find_package(rr_interfaces REQUIRED)

#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
#   ament_add_gtest(${PROJECT_NAME}_controller_test 
#     test/rr_controller_test.cpp
#     src/rr_image_subscriber.cpp 
#     src/rr_subscriber_factory.cpp 
#     src/rr_controller.cpp
#   )
#   target_include_directories(${PROJECT_NAME}_controller_test PUBLIC
#     ${rclcpp_INCLUDE_DIRS}
#     ${sensor_msgs_INCLUDE_DIRS}
#     ${builtin_interfaces_INCLUDE_DIRS}
#     ${rr_interfaces_INCLUDE_DIRS}
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
#   )
#   target_link_libraries(${PROJECT_NAME}_controller_test rclcpp rr_interfaces std_msgs sensor_msgs builtin_interfaces)
# endif()
