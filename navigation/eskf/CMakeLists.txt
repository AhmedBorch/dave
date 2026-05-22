cmake_minimum_required(VERSION 3.8)
project(eskf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vortex_utils REQUIRED)

if(NOT DEFINED EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

# Include Directories
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

# Define Library/Component Name
set(LIB_NAME "${PROJECT_NAME}_component")

# 1. Build the Shared Component Library
add_library(${LIB_NAME} SHARED
  src/eskf.cpp
  src/eskf_ros.cpp
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_eigen
  vortex_utils
)

# 2. Compile a Standalone Executable Wrapper (For easy running/launching)
# add_executable(${PROJECT_NAME}_node src/eskf_ros.cpp)
#target_compile_definitions(${PROJECT_NAME}_node PRIVATE "COMPONENT_EMBEDDED")

# Link the executable directly to your component library
#target_link_libraries(${PROJECT_NAME}_node PRIVATE ${LIB_NAME})

#ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
#  rclcpp
#  rclcpp_components
#)

# 3. Register the component plugin so containers can see it
rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "ESKFNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# Install rules
install(TARGETS ${LIB_NAME} ${PROJECT_NAME}_node
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_export_targets(export_${LIB_NAME})
ament_package()
