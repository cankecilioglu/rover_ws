cmake_minimum_required(VERSION 3.8)
project(rover_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/gps_covariance_relay.py
  scripts/ground_truth_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()