cmake_minimum_required(VERSION 3.10.2)
project(ocs2_mobile_manipulator_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  log
  log_setup
  system
  filesystem
)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(AMENT_DEPENDENCIES
  tf2
  tf2_ros
  urdf
  kdl_parser
  rclcpp
  robot_state_publisher
  ocs2_ros_interfaces
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_self_collision_visualization
  ocs2_mobile_manipulator
  pinocchio
  hpp-fcl
)

###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# MPC node
add_executable(mobile_manipulator_mpc_node
  src/MobileManipulatorMpcNode.cpp
)
ament_target_dependencies(mobile_manipulator_mpc_node
  ${AMENT_DEPENDENCIES}
)
target_compile_options(mobile_manipulator_mpc_node PUBLIC ${FLAGS})
target_include_directories(mobile_manipulator_mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# DistanceVisualization node
add_executable(mobile_manipulator_distance_visualization
  src/MobileManipulatorDistanceVisualization.cpp
)
ament_target_dependencies(mobile_manipulator_distance_visualization
  ${AMENT_DEPENDENCIES}
)
target_compile_options(mobile_manipulator_distance_visualization PUBLIC ${FLAGS})

# Dummy node
add_executable(mobile_manipulator_dummy_mrt_node
  src/MobileManipulatorDummyMRT.cpp
  src/MobileManipulatorDummyVisualization.cpp
)
ament_target_dependencies(mobile_manipulator_dummy_mrt_node
  ${AMENT_DEPENDENCIES}
)
target_compile_options(mobile_manipulator_dummy_mrt_node PUBLIC ${FLAGS})
target_include_directories(mobile_manipulator_dummy_mrt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Target node
add_executable(mobile_manipulator_target
  src/MobileManipulatorTarget.cpp
)
ament_target_dependencies(mobile_manipulator_target
  ${AMENT_DEPENDENCIES}
)
target_compile_options(mobile_manipulator_target PUBLIC ${FLAGS})
target_include_directories(mobile_manipulator_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS mobile_manipulator_mpc_node mobile_manipulator_dummy_mrt_node
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS 
    mobile_manipulator_mpc_node 
    mobile_manipulator_distance_visualization
    mobile_manipulator_dummy_mrt_node 
    mobile_manipulator_target
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
# install(DIRECTORY launch rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
