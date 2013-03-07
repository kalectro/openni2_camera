cmake_minimum_required(VERSION 2.8.3)
project(openni2_camera)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge image_transport)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(openni2_camera_node src/openni2_camera.cpp)

target_link_libraries(openni2_camera_node ${catkin_LIBRARIES} OpenNI2)

install(TARGETS openni2_camera_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


