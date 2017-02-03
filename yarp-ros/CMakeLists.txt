cmake_minimum_required(VERSION 2.8.9)
find_package(YARP REQUIRED)
include_directories(${YARP_INCLUDE_DIRS})

add_executable(yarp_wrench_stamped_publisher src/yarp_wrench_stamped_publisher.cpp)
add_executable(yarp_icub_state_publisher src/yarp_icub_state_publisher.cpp)
add_executable(listener src/listener.cpp)

target_link_libraries(yarp_wrench_stamped_publisher ${YARP_LIBRARIES})
target_link_libraries(yarp_icub_state_publisher ${YARP_LIBRARIES})
target_link_libraries(listener ${YARP_LIBRARIES})
