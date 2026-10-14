cmake_minimum_required(VERSION 3.5)
project(simulation_flight_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (NOT WITH_CASADI)
    set(WITH_CASADI ON)
endif ()
message("simulation_flight_ros WITH_CASADI: ${WITH_CASADI}")

add_subdirectory(lib/simulation-flight)

## ROS-dependent targets
#list(APPEND package_INCLUDE_DIRS )
#list(APPEND package_LIBRARIES )
list(APPEND package_ROS_DEPENDS geometry_msgs sensor_msgs) # mavros_msgs)
list(APPEND package_DEPENDS Eigen3 Boost::headers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach (dependency ${package_ROS_DEPENDS})
    find_package(${dependency} REQUIRED)
endforeach ()
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(flight_simulator_ros2 src/FlightSimulatorRos2.cpp)
target_include_directories(flight_simulator_ros2 PUBLIC include/)
target_link_libraries(flight_simulator_ros2 PUBLIC flight_simulator) # ROS-independent libraries
ament_target_dependencies(flight_simulator_ros2
                          PUBLIC ament_cmake rclcpp
                          ${package_ROS_DEPENDS}) #mavros_msgs) # ROS-dependent dependencies

add_executable(flight_simulator_ros2_test tests/flight_simulator_ros2_test.cpp)
target_link_libraries(flight_simulator_ros2_test PRIVATE flight_simulator_ros2)
list(APPEND ros2_TARGETS flight_simulator_ros2_test)

add_executable(flight_simulator_node2 src/flight_simulator_node2.cpp)
target_link_libraries(flight_simulator_node2 PRIVATE flight_simulator_ros2)
list(APPEND ros2_TARGETS flight_simulator_node2)

# Need to install all targets to be runnable from the command line afterwards
install(TARGETS ${ros2_TARGETS}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY lib/simulation-flight/lib/model-flight/config
        DESTINATION share/${PROJECT_NAME})

# Ament package (ROS2) needs to be called last
ament_package()
