cmake_minimum_required(VERSION 3.8)
project(ids_peak)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
if (NOT TARGET ids_peak)
    find_package (ids_peak REQUIRED
        HINTS 
            ../../../../../../../lib/
    )
endif ()

if (NOT TARGET ids_peak_ipl)
    find_package (ids_peak_ipl REQUIRED
        HINTS
            ../../../../../../../lib/
    )
endif()

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ids_peak REQUIRED)
find_package(ids_peak_ipl REQUIRED)

find_package(camera_info_manager REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

find_package(image_transport REQUIRED)

find_package(ament_lint_auto)

find_package(ament_cmake REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
  camera_info_manager
)

find_package(image_transport REQUIRED)

find_package(rosidl_default_generators REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)


#find_path(HEADERS NAMES peak.hpp HINTS ${AMENT_PREFIX_PATH})

 ###########
## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${IDS_PEAK_INCLUDE_DIR}
  ${IDS_PEAK_IPL_INCLUDE_DIR}
)
include_directories({SENSOR_MSGS_INCLUDE_DIR})
link_directories({SENSOR_MSGS_LIBRARIES})



add_executable(${PROJECT_NAME}_cam_node src/${PROJECT_NAME}_cam_node.cpp 
 src/${PROJECT_NAME}_cam.cpp
  )
add_executable(${PROJECT_NAME}_cam_node_2 src/ids_peak_cam_node_2.cpp 
 src/ids_peak_cam_2.cpp
  )
add_executable(${PROJECT_NAME}_cam_node_3 src/ids_peak_cam_node_3.cpp 
 src/ids_peak_cam_3.cpp 
  )
add_executable(${PROJECT_NAME}_cam_node_4 src/ids_peak_cam_node_4.cpp 
 src/ids_peak_cam_4.cpp
  )

target_include_directories(${PROJECT_NAME}_cam_node PUBLIC include)

target_include_directories(${PROJECT_NAME}_cam_node_2 PUBLIC include)

target_include_directories(${PROJECT_NAME}_cam_node_3 PUBLIC include)

target_include_directories(${PROJECT_NAME}_cam_node_4 PUBLIC include)

#target_include_directories(${PROJECT_NAME}_node PUBLIC $<BUILD_INTERFACE:${HEADERS}>)

#target_include_directories(${PROJECT_NAME}_node_2 PUBLIC $<BUILD_INTERFACE:${HEADERS}>)

#target_include_directories(${PROJECT_NAME}_node_3 PUBLIC $<BUILD_INTERFACE:${HEADERS}>)

#target_include_directories(${PROJECT_NAME}_node_4 PUBLIC $<BUILD_INTERFACE:${HEADERS}>)

target_link_libraries(${PROJECT_NAME}_cam_node
  ${IDS_PEAK_LIBRARIES}
  ${IDS_PEAK_IPL_LIBRARIES}
  ids_peak
  ids_peak_ipl
  image_transport::image_transport
)

target_link_libraries(${PROJECT_NAME}_cam_node_2
  ${IDS_PEAK_LIBRARIES}
  ${IDS_PEAK_IPL_LIBRARIES}
  ids_peak
  ids_peak_ipl
  image_transport::image_transport
)


target_link_libraries(${PROJECT_NAME}_cam_node_3
  ${IDS_PEAK_LIBRARIES}
  ${IDS_PEAK_IPL_LIBRARIES}
  ids_peak
  ids_peak_ipl
  image_transport::image_transport
)

target_link_libraries(${PROJECT_NAME}_cam_node_4
  ${IDS_PEAK_LIBRARIES}
  ${IDS_PEAK_IPL_LIBRARIES}
  ids_peak
  ids_peak_ipl
  image_transport::image_transport
)

ament_export_libraries(${PROJECT_NAME}_cam_node
	 ${} ${OpenCV_LIBS} ids_peak ids_peak_ipl image_transport cv_bridge camera_info_manager sensor_msgs -pthread 
)

ament_export_libraries(${PROJECT_NAME}_cam_node_2
	 ${catkin_LIBRARIES} ${OpenCV_LIBS} ids_peak ids_peak_ipl  image_transport cv_bridge camera_info_manager sensor_msgs -pthread 
)

ament_export_libraries(${PROJECT_NAME}_cam_node_3
	 ${catkin_LIBRARIES} ${OpenCV_LIBS} ids_peak ids_peak_ipl image_transport cv_bridge camera_info_manager  sensor_msgs -pthread 
)

ament_export_libraries(${PROJECT_NAME}_cam_node_4
	 ${catkin_LIBRARIES} ${OpenCV_LIBS} ids_peak ids_peak_ipl image_transport cv_bridge camera_info_manager sensor_msgs -pthread 
)

ament_target_dependencies(ids_peak_cam_node ids_peak ids_peak_ipl image_transport cv_bridge camera_info_manager sensor_msgs)

ament_target_dependencies(ids_peak_cam_node_2 ids_peak ids_peak_ipl image_transport cv_bridge camera_info_manager sensor_msgs)

ament_target_dependencies(ids_peak_cam_node_3 ids_peak ids_peak_ipl image_transport cv_bridge camera_info_manager sensor_msgs)

ament_target_dependencies(ids_peak_cam_node_4 ids_peak ids_peak_ipl image_transport cv_bridge camera_info_manager sensor_msgs)

install(TARGETS 
ids_peak_cam_node
DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS 
ids_peak_cam_node_2
DESTINATION lib/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies()

ament_auto_package()

