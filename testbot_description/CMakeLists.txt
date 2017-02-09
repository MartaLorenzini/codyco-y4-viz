cmake_minimum_required(VERSION 2.8.3)
project(testbot_description)

find_package(catkin REQUIRED COMPONENTS
catkin
urdf
std_msgs
geometry_msgs
sensor_msgs
message_generation
roscpp
rospy tf )
include_directories(${catkin_INCLUDE_DIRS})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

find_package(iDynTree REQUIRED)
find_package(IPOPT REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${IPOPT_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${IDYNTREE_INCLUDE_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package (Matio REQUIRED)
include_directories(${MATIO_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_BINARY_DIR}/data)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/data/URDFdir.h.in" "${CMAKE_CURRENT_BINARY_DIR}/data/URDFdir.h" @ONLY)

include_directories(include)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
  )

catkin_package(
    CATKIN_DEPENDS message_runtime
  )

## Declare a C++ executable
add_executable(joint_state_publisher src/joint_state_publisher.cpp)
add_executable(wrench_stamped_publisher src/wrench_stamped_publisher.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
add_executable(tf_simpleTransform_broadcaster src/tf_simpleTransform_broadcaster.cpp)
add_executable(tf_phantomTransform_broadcaster src/tf_phantomTransform_broadcaster.cpp)
add_executable(icub_state_publisher src/icub_state_publisher.cpp)
add_executable(icub_wrenchLeft_publisher src/icub_wrenchLeft_publisher.cpp)
add_executable(icub_wrenchRight_publisher src/icub_wrenchRight_publisher.cpp)

# add_dependencies
add_dependencies(joint_state_publisher testbot_description_generate_messages_cpp)
add_dependencies(wrench_stamped_publisher testbot_description_generate_messages_cpp)
add_dependencies(tf_broadcaster testbot_description_generate_messages_cpp)
add_dependencies(tf_simpleTransform_broadcaster testbot_description_generate_messages_cpp)
add_dependencies(tf_phantomTransform_broadcaster testbot_description_generate_messages_cpp)
add_dependencies(icub_state_publisher testbot_description_generate_messages_cpp)
add_dependencies(icub_wrenchLeft_publisher testbot_description_generate_messages_cpp)
add_dependencies(icub_wrenchRight_publisher testbot_description_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${MATIO_LIBRARIES} ${IPOPT_LIBRARIES})
target_link_libraries(wrench_stamped_publisher ${catkin_LIBRARIES})
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
target_link_libraries(tf_simpleTransform_broadcaster ${catkin_LIBRARIES})
target_link_libraries(tf_phantomTransform_broadcaster ${catkin_LIBRARIES})
target_link_libraries(icub_state_publisher ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${MATIO_LIBRARIES} ${IPOPT_LIBRARIES})
target_link_libraries(icub_wrenchLeft_publisher ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${MATIO_LIBRARIES} ${IPOPT_LIBRARIES})
target_link_libraries(icub_wrenchRight_publisher ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${MATIO_LIBRARIES} ${IPOPT_LIBRARIES})
