# find_package(rtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(${PROJECT_NAME}_controller_test
    ${CMAKE_SOURCE_DIR}/src/rr_controller.cpp
    rr_controller_test.cpp
)

target_link_libraries(${PROJECT_NAME}_controller_test ${PROJECT_NAME}_lib)
