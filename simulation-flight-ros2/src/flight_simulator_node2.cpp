#include "simulation_flight_ros2/FlightSimulatorRos2.hpp"
//#include "model_flight/FixedWingDynamicsEigen.hpp"
//#include "model_flight/FixedWingDynamicsCasadi.hpp"
#include "model_flight/AweDynamicsCasadi.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const double air_density_ground = 1.1589; // Standard atmosphere at 468 meters

//    simulation_ros2::flight::FlightSimulatorRos2 flightSimRos2("flight_simulator_node");
//    flightSimRos2.p_atmosphere.airDensity_ground = air_density_ground; // Static param, before aircraft setup
//
//    using Aircraft = flight_model::fixed_wing::AugFixedWing;
//    flightSimRos2.setup_eigen_aircraft<Aircraft>(flightSimRos2.get_rosparam_model_params_filepath());
//    flightSimRos2.p_sim_opts.actuator_dynamics = true;

//    using Aircraft = flight_model::fixed_wing_casadi::AugKite;
//    class AircraftWithWind : public Aircraft
//    {
//    public:
//        explicit AircraftWithWind(const std::string &filename,
//                                  const flight_model::StateRepresentation &stateRepresentation = flight_model::StateRepresentation::AttQuat)
//                : Aircraft(filename, stateRepresentation,
//                           {/* Static params */},
//                           {/* Dynamic params */ "v_wind_N", "v_wind_E", "v_wind_D"}) {}
//
//        ~AircraftWithWind() = default;
//    };
//    class KiteWithTether : public Aircraft
//    {
//    public:
//        explicit KiteWithTether(const std::string &filename,
//                                const flight_model::StateRepresentation &stateRepresentation = flight_model::StateRepresentation::AttQuat)
//                : Aircraft(filename, stateRepresentation,
//                           {/* Static params */},
//                           {/* Dynamic params */ "v_wind_N", "v_wind_E", "v_wind_D"},
//                           {/* Opt Params */},
//                           true) {}
//
//        ~KiteWithTether() = default;
//    };
//    flightSimRos2.setup_casadi_aircraft<AircraftWithWind>(flightSimRos2.get_rosparam_model_params_filepath());
//    flightSimRos2.p_sim_opts.actuator_dynamics = true;

    simulation_ros2::flight::FlightSimulator2KitesRos2 flightSimRos2("flight_simulator_node");
    using Aircraft = flight_model::awe_casadi::MultiKiteMapped<2>;
    // simulation_ros2::flight::FlightSimulator3KitesRos2 flightSimRos2("flight_simulator_node");
    // using Aircraft = flight_model::awe_casadi::MultiKiteMapped<3>;

    flightSimRos2.p_atmosphere.airDensity_ground = air_density_ground; // Static param, before aircraft setup
    flightSimRos2.setup_casadi_aircraft<Aircraft>(flightSimRos2.get_rosparam_model_params_filepath());
    flightSimRos2.p_sim_opts.actuator_dynamics = false;
    flightSimRos2.p_sim_opts.tether = true;

    Eigen::VectorXd init_state = flightSimRos2.get_rosparam_init_state();
    flightSimRos2.set_partial_state(init_state);

    Eigen::VectorXd init_control = flightSimRos2.get_rosparam_init_control();
    flightSimRos2.set_partial_control(init_control);

    flightSimRos2.run();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
