cmake_minimum_required(VERSION 2.8.3)
project(occam_ros)
find_package(catkin
  REQUIRED COMPONENTS
    image_transport
    roscpp
    std_msgs
    sensor_msgs
        camera_info_manager)


set(ROS_BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_LIBUSB libusb-1.0)
include_directories(${PC_LIBUSB_INCLUDE_DIRS})
link_directories(${PC_LIBUSB_LIBDIR})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES occam
  CATKIN_DEPENDS
    image_transport
    roscpp
    std_msgs
    sensor_msgs
        camera_info_manager
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wno-format-extra-args")
set(INDIGOSDK_PATH "indigosdk")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INDIGOSDK_PATH}/include
)
add_subdirectory(${INDIGOSDK_PATH} indigosdk)

add_executable(occam_driver_node nodes/occam_driver_node.cpp)
target_link_libraries(occam_driver_node occam ${catkin_LIBRARIES})

add_executable(rectify nodes/rectify.cpp)
target_link_libraries(rectify occam ${catkin_LIBRARIES})

add_executable(cloud nodes/cloud.cpp)
target_link_libraries(cloud occam ${catkin_LIBRARIES})

add_executable(stitch nodes/stitch.cpp)
target_link_libraries(stitch occam ${catkin_LIBRARIES})

add_executable(imgproc nodes/imgproc.cpp)
target_link_libraries(imgproc occam ${catkin_LIBRARIES})

install(TARGETS occam_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
