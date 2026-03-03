# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 3.5)
project(ydlidar_ros2_driver C CXX)

##################ros2#############################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
  -Wno-unused-parameter
  -Wno-unused-variable
)
endif()

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

############## YDLIDAR SDK START#####################################
# Build the lidar node only when the external SDK is available.
find_package(ydlidar_sdk QUIET)
if(ydlidar_sdk_FOUND)
  message(STATUS "Found ydlidar_sdk: building ${PROJECT_NAME}_node")
else()
  message(WARNING "ydlidar_sdk not found: ${PROJECT_NAME}_node will not be built")
endif()
############## YDLIDAR SDK END#####################################

#Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
)

if(ydlidar_sdk_FOUND)
  include_directories(${YDLIDAR_SDK_INCLUDE_DIRS})
  # link library directories
  link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})
endif()

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
if(ydlidar_sdk_FOUND)
  add_executable(${PROJECT_NAME}_node
      src/${PROJECT_NAME}_node.cpp)
  #---------------------------------------------------------------------------------------
  # link libraries
  #--------------------------------------------------------------------------------------
  ament_target_dependencies(${PROJECT_NAME}_node
      "rclcpp"
      "sensor_msgs"
      "visualization_msgs"
      "geometry_msgs"
      "std_srvs"
      )

  target_link_libraries(${PROJECT_NAME}_node
      ${YDLIDAR_SDK_LIBRARIES})
endif()

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_client
    src/${PROJECT_NAME}_client.cpp)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_client
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
    )

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
if(ydlidar_sdk_FOUND)
  install(TARGETS
      ${PROJECT_NAME}_node ${PROJECT_NAME}_client
      DESTINATION lib/${PROJECT_NAME})
else()
  install(TARGETS
      ${PROJECT_NAME}_client
      DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch params startup config
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

