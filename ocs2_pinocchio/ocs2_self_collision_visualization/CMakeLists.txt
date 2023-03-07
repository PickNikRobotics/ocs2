cmake_minimum_required(VERSION 3.10.2)
project(ocs2_self_collision_visualization)

set(AMENT_DEPENDENCIES
  ocs2_core
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_ros_interfaces
  visualization_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)

find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  log
  log_setup
  system
  filesystem
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

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME}
  src/GeometryInterfaceVisualization.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${pinocchio_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${hpp-fcl_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling")
    add_clang_tooling(
      TARGETS ${PROJECT_NAME}
      SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
      CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
      CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(ocs2_core)
ament_export_dependencies(ocs2_robotic_tools)
ament_export_dependencies(ocs2_pinocchio_interface)
ament_export_dependencies(ocs2_self_collision)
ament_export_dependencies(ocs2_ros_interfaces)

ament_package()
