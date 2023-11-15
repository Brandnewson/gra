cmake_minimum_required(VERSION 3.0.2)
project(gra_base)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation controller_manager hardware_interface controller_interface geometry_msgs pluginlib tf2_ros tf2_geometry_msgs sensor_msgs)
find_package(Eigen3 REQUIRED)

# add_message_files(
#     FILES
#     MotorAdaptiveState.msg
#     MotorParameters.msg
#     MotorSetpoint.msg
#     MotorState.msg
#     BaseAdaptiveState.msg
#     BaseParameters.msg
#     BaseSetpoint.msg
#     BaseState.msg
# )

generate_messages(
    DEPENDENCIES
    std_msgs
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(cirkit_unit03_hw src/cirkit_unit03_hw.cpp src/ixis_imcs01_driver/ixis_imcs01_driver.cpp)

target_link_libraries(cirkit_unit03_hw
  ${catkin_LIBRARIES}
)


install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.perspective"
)

install (DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install (DIRECTORY description/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description
)
