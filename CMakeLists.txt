cmake_minimum_required(VERSION 2.8.3)

project(force_modulated_ds)

# Add support for c++11
add_definitions(-std=c++11)

# Find required packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  gaussian_process_regression
  motion_force_control
  foot_interfaces
  # sgaussians
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/forceModulatedTask_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation dynamic_reconfigure gaussian_process_regression motion_force_control foot_interfaces
  # DEPENDS Eigen
 )

include_directories(include ${catkin_INCLUDE_DIRS})

# Generate executable
add_executable(ForceModulatedTask src/forceModulatedTask_node.cpp src/ForceModulatedTask.cpp src/ForceModulatedDs.cpp)
target_link_libraries(ForceModulatedTask ${catkin_LIBRARIES} passive_ds_force_controller)
add_dependencies(ForceModulatedTask ${PROJECT_NAME}_gencpp)