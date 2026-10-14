cmake_minimum_required(VERSION 3.5)
project(main_control_flight_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

add_subdirectory(lib/main-control-flight)

## ROS-dependent targets
#list(APPEND package_INCLUDE_DIRS )
#list(APPEND package_LIBRARIES )
list(APPEND package_ROS_DEPENDS geometry_msgs sensor_msgs diagnostic_msgs) # mavros_msgs)
list(APPEND package_DEPENDS Eigen3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach (dependency ${package_ROS_DEPENDS})
    find_package(${dependency} REQUIRED)
endforeach ()
find_package(Eigen3 REQUIRED NO_MODULE)

## Fixed-wing Control
add_library(fixedwing_control_ros2 src/FixedwingControlRos2.cpp)
target_include_directories(fixedwing_control_ros2 PUBLIC include)
target_link_libraries(fixedwing_control_ros2 PUBLIC fixedwing_control) # ROS-independent libraries
ament_target_dependencies(fixedwing_control_ros2
                          PUBLIC ament_cmake rclcpp
                          ${package_ROS_DEPENDS}) #mavros_msgs) # ROS-dependent dependencies

add_executable(fixedwing_control_ros2_test tests/fixedwing_control_ros2_test.cpp)
target_link_libraries(fixedwing_control_ros2_test PRIVATE fixedwing_control_ros2)
list(APPEND ros2_TARGETS fixedwing_control_ros2_test)

add_executable(fixedwing_control_node2 src/fixedwing_control_node2.cpp)
target_link_libraries(fixedwing_control_node2 PRIVATE fixedwing_control_ros2)
list(APPEND ros2_TARGETS fixedwing_control_node2)

## ROS2 publisher/subscriber test
#add_executable(subscriber_member_function tests/ros2/minimal_subscriber.cpp)
#ament_target_dependencies(subscriber_member_function rclcpp std_msgs)
#list(APPEND ros2_TARGETS subscriber_member_function)
#
#add_executable(publisher_member_function tests/ros2/minimal_publisher.cpp)
#ament_target_dependencies(publisher_member_function rclcpp std_msgs)
#list(APPEND ros2_TARGETS publisher_member_function)

# Need to install all targets to be runnable from the command line afterwards
install(TARGETS ${ros2_TARGETS}
        DESTINATION lib/${PROJECT_NAME})
# Ament package (ROS2) needs to be called last
ament_package()
