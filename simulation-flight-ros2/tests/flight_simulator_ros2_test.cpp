#include "simulation_flight_ros2/FlightSimulatorRos2.hpp"
//#include "model_flight/FixedWingDynamicsEigen.hpp"
//#include "model_flight/FixedWingDynamicsCasadi.hpp"
#include "model_flight/AweDynamicsCasadi.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const double air_density_ground = 1.1589; // Standard atmosphere at 468 meters

    simulation_ros2::flight::FlightSimulatorMavros2 flightSimRos2("flight_sim_ros_test");
    flightSimRos2.p_atmosphere.airDensity_ground = air_density_ground; // Static param, before aircraft setup

//    using Aircraft = flight_model::fixed_wing::AugFixedWing;
//    flightSimRos2.setup_eigen_aircraft<Aircraft>(flightSimRos2.get_rosparam_model_params_filepath());
//    flightSimRos2.p_sim_opts.actuator_dynamics = true;

    using Aircraft = flight_model::awe_casadi::MultiKiteMapped<2>;
    flightSimRos2.setup_casadi_aircraft<Aircraft>(flightSimRos2.get_rosparam_model_params_filepath());

//    flightSimRos2.set_partial_state((Eigen::VectorXd(2) << 10, 20).finished(), 6);
//    flightSimRos2.set_partial_control(Eigen::VectorXd(4) << 0, 0, 0.66, 0);

    const double final_time = 0.1;
    flightSimRos2.run(final_time);

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
