cmake_minimum_required(VERSION 3.5)
project(awe_control)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(Eigen3 REQUIRED NO_MODULE)

if (NOT TARGET awe_control)
    add_library(awe_control
                src/AweControlTools.cpp
                src/AWEEightController.cpp
                src/AweCircleController.cpp
                src/AweTurnController.cpp
                src/AweHeliControl.cpp
#                src/SpeedController.cpp
#                src/VelocityController.cpp
    )
    target_include_directories(awe_control PUBLIC include/)
    target_link_libraries(awe_control PUBLIC Eigen3::Eigen)
endif ()
