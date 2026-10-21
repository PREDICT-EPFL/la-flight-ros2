cmake_minimum_required(VERSION 3.5)
project(simulation_flight)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (NOT WITH_CASADI)
    set(WITH_CASADI OFF)
endif ()
message("simulation_flight WITH_CASADI: ${WITH_CASADI}")

add_subdirectory(lib/simulation-common)
add_subdirectory(lib/model-flight)

find_package(Eigen3 REQUIRED NO_MODULE) # This ensures Eigen3 is installed on the system. The target to link to is 'Eigen3::Eigen' afterwards.

add_library(flight_simulator src/FlightSimulator.cpp)
target_include_directories(flight_simulator PUBLIC include/)
target_link_libraries(flight_simulator PUBLIC simulation_common flight_model)

add_executable(flight_sim_test tests/flight_sim_test.cpp)
target_link_libraries(flight_sim_test PRIVATE flight_simulator)

if (WITH_CASADI)
    find_package(Casadi REQUIRED)
    target_link_libraries(flight_simulator PUBLIC casadi::casadi)
    target_compile_definitions(flight_simulator PUBLIC WITH_CASADI=1)

    target_compile_definitions(flight_sim_test PUBLIC WITH_CASADI=1)
endif ()
