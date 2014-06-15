cmake_minimum_required(VERSION 2.8.3)
project(openni2_camera)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge image_transport camera_info_manager)

find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})

#include libopenni2
pkg_check_modules(PC_OPENNI2 REQUIRED libopenni2)


catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport camera_info_manager
  DEPENDS system_lib libopenni2
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PC_OPENNI2_LIBRARY_DIRS})

add_executable(openni2_camera_node src/openni2_camera.cpp)

target_link_libraries(openni2_camera_node ${catkin_LIBRARIES} OpenNI2 ${OpenCV_LIBRARIES})

install(TARGETS openni2_camera_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


