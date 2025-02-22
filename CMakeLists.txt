cmake_minimum_required(VERSION 3.5)
project(msp_offboard_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(msp_msgs REQUIRED)
find_package(msp_controller REQUIRED)
find_package(Boost REQUIRED thread)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(glog REQUIRED)

add_library(TrajectoryGenerator      SHARED src/lib/RapidTrajectoryGenerator/RapidTrajectoryGenerator.cpp 
                                            src/lib/RapidTrajectoryGenerator/SingleAxisTrajectory.cpp
											src/lib/SegmentTrajectoryGenerator/SegmentedTrajectoryPlanner.cpp
											src/lib/SegmentTrajectoryGenerator/MSPRapidTrajectoryGenerator.cpp
											)

ament_target_dependencies(TrajectoryGenerator Boost rclcpp)
target_include_directories(TrajectoryGenerator PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS TrajectoryGenerator
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	#INCLUDES DESTINATION include
)

add_executable(msp_offboard_controller src/msp_offboard_controller.cpp                                      )

ament_target_dependencies(msp_offboard_controller msp_controller rclcpp px4_msgs msp_msgs tf2 tf2_ros glog )

target_link_libraries(msp_offboard_controller TrajectoryGenerator glog )

#ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime px4_msgs msp_msgs  )

ament_export_include_directories(include)
ament_export_libraries(TrajectoryGenerator)


install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
#install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS
  msp_offboard_controller 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
