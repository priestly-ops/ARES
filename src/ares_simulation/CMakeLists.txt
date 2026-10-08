cmake_minimum_required(VERSION 3.10)

project(ares_simulation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(week6_clock_boundary src/week6_clock_boundary.cpp)
target_compile_features(week6_clock_boundary PUBLIC cxx_std_17)
target_include_directories(week6_clock_boundary PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(week6_clock_boundary PRIVATE
  rclcpp::rclcpp
  rosgraph_msgs::rosgraph_msgs
  std_msgs::std_msgs
)

install(TARGETS week6_clock_boundary
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    worlds
    models
    launch
    config
    maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_week6_clock_boundary
    test/test_week6_clock_boundary.cpp
  )
  target_compile_features(test_week6_clock_boundary PUBLIC cxx_std_17)
  target_include_directories(test_week6_clock_boundary PUBLIC include)
  target_link_libraries(test_week6_clock_boundary
    rosgraph_msgs::rosgraph_msgs
  )
endif()

ament_package()
