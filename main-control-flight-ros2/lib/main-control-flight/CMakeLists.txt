cmake_minimum_required(VERSION 3.5)
project(main_control_flight)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(Eigen3 REQUIRED NO_MODULE)

add_subdirectory(lib/main-control-common)
add_subdirectory(lib/awe-control)

## Fixed-wing Control
add_library(fixedwing_control src/fixedwing/FixedwingControl.cpp)
target_include_directories(fixedwing_control PUBLIC include/)
target_link_libraries(fixedwing_control PUBLIC Eigen3::Eigen main_control_common awe_control)

add_executable(fixedwing_control_test tests/fixedwing_control_test.cpp)
target_link_libraries(fixedwing_control_test PRIVATE fixedwing_control)
