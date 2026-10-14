cmake_minimum_required(VERSION 3.5)
project(visualization_flight_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

## ROS-dependent targets
#list(APPEND package_INCLUDE_DIRS )
#list(APPEND package_LIBRARIES )
list(APPEND package_ROS_DEPENDS
     tf2 tf2_ros
     geometry_msgs sensor_msgs visualization_msgs
     mavros_msgs
     )
list(APPEND package_DEPENDS Eigen3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach (dependency ${package_ROS_DEPENDS})
    find_package(${dependency} REQUIRED)
endforeach ()
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp 0.5.2 REQUIRED)

add_library(visualizer_flight_ros2
            src/VisualizerFlightRos2Base.cpp
            src/VisualizerFlightRos2.cpp)
target_include_directories(visualizer_flight_ros2 PUBLIC include/ lib/visualization-common-ros2/include/)
target_link_libraries(visualizer_flight_ros2 PUBLIC Eigen3::Eigen yaml-cpp)
ament_target_dependencies(visualizer_flight_ros2
                          PUBLIC ament_cmake rclcpp
                          ${package_ROS_DEPENDS}) # ROS-dependent dependencies

add_executable(visualization_flight_node2 src/visualizer_flight_node2.cpp)
target_link_libraries(visualization_flight_node2 PRIVATE visualizer_flight_ros2)
list(APPEND ros2_TARGETS visualization_flight_node2)

install(DIRECTORY resources
        DESTINATION share/${PROJECT_NAME})

install(TARGETS ${ros2_TARGETS}
        DESTINATION lib/${PROJECT_NAME}) # Need to install all targets to be runnable from the command line afterwards
ament_package() # Ament package (ROS2) needs to be called last
