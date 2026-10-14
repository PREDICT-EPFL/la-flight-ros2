#include "model_flight/FixedWingDynamicsEigen.hpp"

#ifdef WITH_CASADI
#include "model_flight/FixedWingDynamicsCasadi.hpp"
#include "model_flight/AweDynamicsCasadi.hpp"
#endif

int main()
{
    const std::string config_dir = "/home/johannes/flight_models/";

    using namespace flight_model;

    const std::vector<double> x0_lon_test = {12, 0.01, 0.02, -0.01};
    const std::vector<double> u0_lon_test = {0.1, 0.15};
    const std::string param_file = config_dir + "eg4_xflr-Pvw-YR.yaml";

    std::cout << "LonFixedWing\n";
    {
        using Model = fixed_wing::LonFixedWing;
        Model model(StateRepresentation::LongitudinalFlightPath);
        model.load_params_from_yaml(param_file);

        Model::State x0(x0_lon_test.data()); // = model.get_default_initial_state();
        Model::Input u(u0_lon_test.data()); // = {0.1, 0.1};
        std::cout << "x0: " << x0.transpose() << "\n"
                  << "u: " << u.transpose() << "\n";

        Model::Param p = {};
        Model::DynOut y;
        Model::State x_dot = model.dynamics(x0, u, p, y);
        std::cout << "x_dot: " << x_dot.transpose() << "\n";
        std::cout << "\n";
    }
    std::cout << "LonFixedWingWithPos\n";
    {
        using Model = fixed_wing::LonFixedWingWithPos;
        Model model(StateRepresentation::LongitudinalFlightPath);
        model.load_params_from_yaml(param_file);

        const std::vector<double> x0_lonWithPos_test = {12, 0.01, 0.02, 0.01, 0, -100};
        Model::State x0(x0_lonWithPos_test.data()); // = model.get_default_initial_state();
        Model::Input u(u0_lon_test.data()); // = {0.1, 0.1};
        std::cout << "x0: " << x0.transpose() << "\n"
                  << "u: " << u.transpose() << "\n";

        Model::Param p = {};
        Model::DynOut y;
        Model::State x_dot = model.dynamics(x0, u, p, y);
        std::cout << "x_dot: " << x_dot.transpose() << "\n";
        std::cout << "\n";
    }

    using namespace model_common::quatmath;
    const double roll = 30.0 * M_PI / 180.0;
    const double pitch = 10.0 * M_PI / 180.0;
    const double yaw = 60.0 * M_PI / 180.0;
    const Eigen::Quaternion<double> q = T1quat<double>(roll) * T2quat<double>(pitch) * T3quat<double>(yaw);
    const std::vector<double> x0_fixedWing = {12, 0.1, 0.2, 0.01, 0.02, 0.03,
                                              0, 0, 0, q.w(), q.x(), q.y(), q.z()};
    const std::vector<double> u0_fixedWing = {0.8, 0.03, 0.02, 0.01};

    std::cout << "FixedWing\n";
    {
        using Model = fixed_wing::FixedWing;
        Model model;
        model.load_params_from_yaml(param_file);

        Model::State x0(x0_fixedWing.data()); //= model.get_default_initial_state();
        Model::Input u(u0_fixedWing.data()); // = {0.1, 0.1, 0.1, 0.01};
        std::cout << "x0: " << x0.transpose() << "\n"
                  << "u: " << u.transpose() << "\n";

        Model::Param p = (Model::Param() << 0, 0, 0).finished();
        Model::DynOut z;
        Model::State x_dot = model.dynamics<double>(x0, u, p, z);
        std::cout << "x_dot: " << x_dot.transpose() << "\n";
        std::cout << "\n";
    }
    std::cout << "AugFixedWing\n";
    {
        using Model = fixed_wing::AugFixedWing;
        Model model;
        model.load_params_from_yaml(param_file);

        const std::vector<double> x0_fixedWingAug = {12, 0.1, 0.2, 0.01, 0.02, 0.03,
                                                     0, 0, 0, 0.99, 0.01, 0.01, 0.01,
                                                     0.1, 0.03, 0.02, 0.01};
        Model::State x0(x0_fixedWingAug.data());
        Model::Input u(u0_fixedWing.data());
        std::cout << "x0: " << x0.transpose() << "\n"
                  << "u: " << u.transpose() << "\n";

        Model::Param p = (Model::Param() << 0, 0, 0).finished();
        Model::DynOut z;
        Model::State x_dot = model.dynamics<double>(x0, u, p, z);
        std::cout << "x_dot: " << x_dot.transpose() << "\n";
        std::cout << "\n";
    }


#ifdef WITH_CASADI
    std::cout << "WITH_CASADI\n";
    {
        using namespace casadi;
        using namespace model_common::quatmath_casadi;

        const std::vector<double> x0_lon_test = {12, 0.01, 0.02, -0.01};
        const std::vector<double> u0_lon_test = {0.1, 0.15};
        const std::string param_file = config_dir + "eg4_xflr-Pvw-YR.yaml";

        std::cout << "LonKite\n";
        {
            /* Static parameters: Set {name, value} pair once that will be constant afterwards */
            // use default      const std::map<std::string, double> staticParams = {{"air_density", 1.225}};

            /* Dynamic parameters: Set parameter names that will be modifiable before running an optimization  */
            const std::vector<std::string> dynParamNames = {"v_wind_hor", "v_wind_ver"};

            fixed_wing_casadi::LonKite model(param_file,
                                             flight_model::StateRepresentation::LongitudinalFlightPath,
                                             std::map<std::string, double>(), dynParamNames);
            using namespace casadi;
            Function dyn_func = model.getNumericDynamics(true, true);

            DM x0(x0_lon_test);
            DM u(u0_lon_test);
            DM d = DM::vertcat({0, 0});
            std::cout << "x0: " << x0 << "\n"
                      << "u: " << u << "\n"
                      << "d: " << d << "\n";

            DM x_dot = dyn_func(DMVector({x0, u, d}))[0];
            std::cout << "x_dot: " << x_dot << "\n";

//        Function deb_func = model.getNumericOutput("debugSX", true, true);
//        DM debug = deb_func(DMVector({x0, u, d}))[0];
//        std::cout << "debugSX: " << debug << "\n";
            std::cout << "\n";
        }
        std::cout << "LonFixedWingWithPos\n";
        {
            /* Static parameters: Set {name, value} pair once that will be constant afterwards */
            // use default      const std::map<std::string, double> staticParams = {{"air_density", 1.225}};

            /* Dynamic parameters: Set parameter names that will be modifiable before running an optimization  */
            const std::vector<std::string> dynParamNames = {"v_wind_hor", "v_wind_ver"};

            fixed_wing_casadi::LonKiteWithPos model(param_file,
                                                    flight_model::StateRepresentation::LongitudinalFlightPath,
                                                    std::map<std::string, double>(), dynParamNames);
            using namespace casadi;
            Function dyn_func = model.getNumericDynamics(true, true);

            const std::vector<double> x0_lonWithPos_test = {12, 0.01, 0.02, 0.01, 0, -100};
            DM x0(x0_lonWithPos_test);
            DM u(u0_lon_test);
            DM d = DM::vertcat({0, 0});
            std::cout << "x0: " << x0 << "\n"
                      << "u: " << u << "\n"
                      << "d: " << d << "\n";

            DM x_dot = dyn_func(DMVector({x0, u, d}))[0];
            std::cout << "x_dot: " << x_dot << "\n";
            std::cout << "\n";
        }

        const double roll = 30.0 * M_PI / 180.0;
        const double pitch = 10.0 * M_PI / 180.0;
        const double yaw = 60.0 * M_PI / 180.0;
        const std::vector<double> q = quat_multiply<DM>(quat_multiply(T1quat(roll), T2quat(pitch)),
                                                        T3quat(yaw)).nonzeros();
        const std::vector<double> x0_fixedWing = {12, 0.1, 0.2, 0.01, 0.02, 0.03,
                                                  0, 0, 0, q[0], q[1], q[2], q[3]};
        const std::vector<double> u0_fixedWing = {0.8, 0.03, 0.02, 0.01};

        std::cout << "Kite\n";
        {
            /* Static parameters: Set {name, value} pair once that will be constant afterwards */
            // use default      const std::map<std::string, double> staticParams = {{"air_density", 1.225}};

            /* Dynamic parameters: Set parameter names that will be modifiable before running an optimization  */
            const std::vector<std::string> dynParamNames = {"v_wind_N", "v_wind_E"};

            fixed_wing_casadi::Kite model(param_file,
                                          flight_model::StateRepresentation::AttQuat,
                                          std::map<std::string, double>(), dynParamNames);
            using namespace casadi;
            Function dyn_func = model.getNumericDynamics(true, true);

            DM x0(x0_fixedWing); //model.getDefaultInitialState();
            DM u(u0_fixedWing);
            DM d = DM::vertcat({0, 0});
            std::cout << "x0: " << x0 << "\n"
                      << "u: " << u << "\n"
                      << "d: " << d << "\n";

            DM x_dot = dyn_func(DMVector({x0, u, d}))[0];
            std::cout << "x_dot: " << x_dot << "\n";

            Function deb_func = model.getNumericMapping("debugSX", true, true);
            DM debug = deb_func(DMVector({x0, u, d}))[0];
            std::cout << "debugSX: " << debug << "\n";
            std::cout << "\n";
        }
        std::cout << "AugKite\n";
        {
            /* Static parameters: Set {name, value} pair once that will be constant afterwards */
            // use default      const std::map<std::string, double> staticParams = {{"air_density", 1.225}};

            /* Dynamic parameters: Set parameter names that will be modifiable before running an optimization  */
            const std::vector<std::string> dynParamNames = {"v_wind_N", "v_wind_E"};

            fixed_wing_casadi::AugKite model(param_file,
                                             flight_model::StateRepresentation::AttQuat,
                                             std::map<std::string, double>(), dynParamNames);
            using namespace casadi;
            Function dyn_func = model.getNumericDynamics(true, true);

            const std::vector<double> x0_fixedWingAug = {12, 0.1, 0.2, 0.01, 0.02, 0.03,
                                                         0, 0, 0, 0.99, 0.01, 0.01, 0.01,
                                                         0.1, 0.03, 0.02, 0.01};
            DM x0(x0_fixedWingAug);
            DM u(u0_fixedWing);
            DM d = DM::vertcat({0, 0});
            std::cout << "x0: " << x0 << "\n"
                      << "u: " << u << "\n"
                      << "d: " << d << "\n";

            DM x_dot = dyn_func(DMVector({x0, u, d}))[0];
            std::cout << "x_dot: " << x_dot << "\n";
            std::cout << "\n";
        }
        std::cout << "FixedWing\n";
        {
            /* Static parameters: Set {name, value} pair once that will be constant afterwards */
            // use default      const std::map<std::string, double> staticParams = {{"air_density", 1.225}};

            /* Dynamic parameters: Set parameter names that will be modifiable before running an optimization  */
            const std::vector<std::string> dynParamNames = {"v_wind_N", "v_wind_E"};

            fixed_wing_casadi::FixedWing model(param_file,
                                               flight_model::StateRepresentation::AttQuat,
                                               std::map<std::string, double>(), dynParamNames);

            using namespace casadi;
            Function dyn_func = model.getNumericDynamics(true, true);

            DM x0_fixedWing_DM(x0_fixedWing);
            DM x0(DM::vertcat({x0_fixedWing_DM(Slice(0, 6)), x0_fixedWing_DM(Slice(9, 13))}));
            DM u(u0_fixedWing);
            DM d = DM::vertcat({0, 0});
            std::cout << "x0: " << x0 << "\n"
                      << "u: " << u << "\n"
                      << "d: " << d << "\n";

            DM x_dot = dyn_func(DMVector({x0, u, d}))[0];
            std::cout << "x_dot: " << x_dot << "\n";
            std::cout << "\n";
        }

        std::cout << "MultiKite\n";
        {
            /* Static parameters: Set {name, value} pair once that will be constant afterwards */
            // use default      const std::map<std::string, double> staticParams = {{"air_density", 1.225}};

//            /* Dynamic parameters: Set parameter names that will be modifiable before running an optimization  */
//            const std::vector<std::string> dynParamNames = {"v_wind_N", "v_wind_E", "v_wind_D"};
//
//            fixed_wing_casadi::MultiKite model(param_file,
//                                               flight_model::StateRepresentation::AttQuat,
//                                               std::map<std::string, double>(), dynParamNames);

            flight_model::awe_casadi::MultiKite model(2, param_file);

            using namespace casadi;
            Function dyn_func = model.getNumericDynamics(true, true);
            Function dyn_out_func = model.getNumericMapping("dyn_out", true, true);
            Function Va0_func = model.getNumericMapping("k0_Va", true, true);
            Function Va1_func = model.getNumericMapping("k1_Va", true, true);

            DM x0_fixedWing_DM(x0_fixedWing);
            DM u(u0_fixedWing);

            DM x0 = DM::repmat(x0_fixedWing_DM, 2, 1);
            u = DM::repmat(u, 2, 1);

            DM d = DM::vertcat({0, 0, 0});
            std::cout << "x0: " << x0 << "\n"
                      << "u: " << u << "\n"
                      << "d: " << d << "\n";

            DM x_dot = dyn_func(DMVector({x0, u, d}))[0];
            std::cout << "x_dot: " << x_dot << "\n";

            DM dyn_out = dyn_out_func(DMVector({x0, u, d}))[0];
            DM Va0 = Va0_func(DMVector({x0, u, d}))[0];
            DM Va1 = Va1_func(DMVector({x0, u, d}))[0];
            std::cout << "dyn_out: " << dyn_out << ",      Va0: " << Va0 << ", Va1: " << Va1 << "\n";

            std::cout << "\n";
        }
    }
#endif

    return EXIT_SUCCESS;
}
