cmake_minimum_required(VERSION 3.8)
project(ares_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    ares_localization/cmd_vel_adapter.py
    ares_localization/waypoint_mission.py
    ares_localization/initial_pose_sync.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
