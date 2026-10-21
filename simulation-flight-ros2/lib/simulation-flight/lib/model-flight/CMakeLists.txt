cmake_minimum_required(VERSION 3.5)
project(model_flight)

if (NOT TARGET flight_model)

    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_BUILD_TYPE Release)

    if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif ()

    if (NOT WITH_CASADI)
        set(WITH_CASADI ON)
    endif ()
    message("model_flight WITH_CASADI: ${WITH_CASADI}")

    add_subdirectory(lib/model-common)

    find_package(Eigen3 REQUIRED NO_MODULE) # This ensures Eigen3 is installed on the system. The target to link to is 'Eigen3::Eigen' afterwards.
    find_package(yaml-cpp 0.5.2 REQUIRED)

    ### Build targets
    add_library(flight_model INTERFACE)
    target_include_directories(flight_model INTERFACE include/)
    target_link_libraries(flight_model INTERFACE model_common Eigen3::Eigen yaml-cpp)

    if (${WITH_CASADI})
        find_package(Casadi REQUIRED) # This ensures Casadi is installed on the system. The target to link to is 'casadi' afterwards.
        target_include_directories(flight_model INTERFACE include_casadi/)
        target_link_libraries(flight_model INTERFACE casadi::casadi)

        # Build additional executable quaternion_test
        add_executable(quaternion_test tests_casadi/quaternion_test.cpp)
        target_link_libraries(quaternion_test PRIVATE flight_model)

        # Build additional executable tether_test
        add_executable(tether_test tests_casadi/tether_test.cpp)
        target_link_libraries(tether_test PRIVATE flight_model)

        #    # Optionally build trim_test with polympc-ros
        #    find_package(catkin REQUIRED COMPONENTS polympc-ros)
        #    add_executable(trim_test src/trim_test.cpp)
        #    target_include_directories(trim_test
        #                               PRIVATE ${catkin_INCLUDE_DIRS})
        #    target_link_libraries(trim_test
        #                          PRIVATE flight_model_eigen flight_model_casadi yaml-cpp ${catkin_LIBRARIES})
    endif ()

    # Build executable dynamics_test_eigen
    add_executable(flight_dynamics_test tests/flight_dynamics_test.cpp)
    target_link_libraries(flight_dynamics_test PRIVATE flight_model)
    if (${WITH_CASADI})
        target_compile_definitions(flight_dynamics_test PRIVATE WITH_CASADI=1)
    endif ()

endif ()
