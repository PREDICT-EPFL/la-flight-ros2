#include "simulation_flight/FlightSimulator.hpp"
#include "model_flight/FixedWingDynamicsEigen.hpp"

#ifdef WITH_CASADI
#include "model_flight/FixedWingDynamicsCasadi.hpp"
#include "model_flight/AweDynamicsCasadi.hpp"
#endif

int main(int argc, char **argv)
{
    const std::string config_dir = "/home/johannes/flight_models/";

    std::string model_params_file;
    model_params_file = "eg4_xflr-Pvw-YR"; /// debug only ///

    if (model_params_file.at(0) == '/'); // Full path given, don't modify
    else { model_params_file = config_dir + model_params_file; }
    if (model_params_file.find(".yaml") == std::string::npos) { model_params_file.append(".yaml"); } // Add .yaml ending

    auto x_fixedWing = (Eigen::VectorXd(13) << 12, 0.05, -0.05, 0.05, -0.05, 0.05, 0, 0, -100, 1, 0, 0, 0).finished();
    auto u_fixedWing = (Eigen::VectorXd(4) << 0, 0, 0, 0).finished();

    auto x_2Kites = (Eigen::VectorXd(26) <<
                                         12, 0, 0, 0, 0, 0, 0, 0, -100, 1, 0, 0, 0,
                                         12, 0, 0, 0, 0, 0, 0, 0, -100, 1, 0, 0, 0).finished();

    // Test casadi sim engine
    {
#ifdef WITH_CASADI
        std::cout << "\n****** Test Casadi Sim Engine ******\n";
        using namespace flight_model;

        using Simulator = simulation::flight::FlightSimulator;
        Simulator flightSim1;
        flightSim1.set_rate(200);
        flightSim1.p_sim_opts.actuator_dynamics = false;

        using Model = flight_model::awe_casadi::MultiKiteMapped<2>;
        auto x0 = x_2Kites;


        flightSim1.p_atmosphere.airDensity_ground = 1.1589; // Standard atmosphere at 468 meters
        flightSim1.setup_casadi_aircraft<Model>(model_params_file);
        flightSim1.set_sim_speed(1.0);

//        std::cout << "x0: " << x0.transpose() << "\n";
//        flightSim1.set_partial_state(x0);
        flightSim1.run(3.0);
#endif
    }

    // Test Eigen sim engine
    {
        std::cout << "\n****** Test Eigen Sim Engine ******\n";
        using namespace flight_model;

        //    using Simulator = simulation::flight::FlightSimulator;
        using Simulator = simulation::flight::TouchdownSimulator;

        Simulator flightSim2;
        flightSim2.set_rate(200);
        flightSim2.p_sim_opts.actuator_dynamics = true;

        using Model = fixed_wing::FixedWing;
        auto x0 = x_fixedWing;
        auto u = u_fixedWing;

        flightSim2.p_atmosphere.airDensity_ground = 1.1589; // Standard atmosphere at 468 meters
        flightSim2.setup_eigen_aircraft<Model>(model_params_file);
        flightSim2.p_sim_opts.atmosphere = false;
        flightSim2.set_sim_speed(1.0);

        flightSim2.set_partial_state(x0);
        flightSim2.run(3.0);

        // Run in custom loop, e.g. with closed-loop control
        std::cout << "\n****** Test Custom Loop ******\n";
        flightSim2.reset_sim_time();
        flightSim2.set_partial_state(x0);

        double t = -1;
        Eigen::VectorXd x = x0;
        for (int i = 0; i < 1000; i++)
        {
            // Controller
            flightSim2.set_control(u);

            flightSim2.run_once();
            t = flightSim2.get_sim_time();
            x = flightSim2.get_state();
        }
        std::cout << "Result: time = " << std::setprecision(4) << t << ", " << "state: " << x.transpose() << "\n";
    }
    return EXIT_SUCCESS;
}
