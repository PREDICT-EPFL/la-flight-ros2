#ifndef SRC_FLIGHTSIMULATOR_HPP
#define SRC_FLIGHTSIMULATOR_HPP

#include <Eigen/Core>

#include "simulation_common/SimulatorBase.hpp"
#include "simulation_common/sim_engine/SimEngineEigen.hpp"

#include "simulation_common/sim_engine/SimEngineEigen.hpp"
#include "simulation_common/sim_engine/OdeIntPlant.hpp"

#ifdef WITH_CASADI
#include "casadi/casadi.hpp"
#include "simulation_common/sim_engine/SimEngineCasadi.hpp"
#endif

#include "atmosphere.hpp"

namespace simulation {
namespace flight {

class FlightSimulator : public virtual SimulatorBase
{
public:
    template<typename EigenVehicle>
    void setup_eigen_aircraft(const std::string &model_params_file)
    {
        std::cout << "[Simulator]: Eigen sim engine (Boost ODE integration)\n";
        std::cout << "[Simulator]: Using model parameter file: " << model_params_file << "\n";

        /** Eigen model **/
        /* Copy pointer after setting up temporary instance */
        using EigenPlant = odeint_model::OdeIntPlant<EigenVehicle>;
        const auto sim_engine_ptr = std::make_shared<engine::EigenSimEngine<EigenPlant>>(get_sim_dt());
        if (!model_params_file.empty()) { sim_engine_ptr->plant.load_params_from_yaml(model_params_file); }
//        sim_engine_ptr->plant.set_static_params(staticParams.vec());
        m_sim_engine = sim_engine_ptr;

        /** Initialization **/
        m_state = sim_engine_ptr->plant.get_default_initial_state();
        m_control = m_control_in = Eigen::VectorXd::Zero(EigenVehicle::nu);
        m_params = m_vW_ned; // #TODO
        m_sim_engine->initialize(m_state, m_control, m_params);
        m_output = m_sim_engine->get_output();

        const EigenVehicle vehicle;
        p_control_lb = vehicle.u_lbound;
        p_control_ub = vehicle.u_ubound;

        for (auto &dryden : m_drydenTurbulenceModel) { dryden.set_wingspan(sim_engine_ptr->plant.turb_ref_length); }
    }
#ifdef WITH_CASADI
    template<typename CasadiVehicle>
    void setup_casadi_aircraft(const std::string &model_params_file = "")
    {
        std::cout << "[Simulator]: Casadi sim engine (CVODES ODE integration)\n";
        std::cout << "[Simulator]: Using model parameter file: " << model_params_file << "\n";

        /** Casadi model **/
        /* Construct kite dynamics for forward integration (for initial guess) */
        auto dynamics_model = CasadiVehicle(model_params_file);

        const casadi::Function dynamics_func = dynamics_model.getNumericDynamics(true, true);
        const casadi::Function output_func = dynamics_model.getNumericOutput(true, false);
        const casadi::Function dynOut_func = dynamics_model.getNumericMapping("dyn_out", true, false);

        m_sim_engine = std::make_shared<engine_casadi::SimEngineCasadi>(dynamics_func, get_sim_dt(),
                                                                        output_func, dynOut_func);

        /** Initialization **/
        const int n_aug = (p_sim_opts.actuator_dynamics ? dynamics_model.getNumericDynamics().size1_in(1) : 0);
        m_state = eigen_casadi_conversion::toEigenX(dynamics_model.getDefaultInitialState());
        m_control = m_control_in = Eigen::VectorXd::Zero(dynamics_model.getSymbolicControl().size1());
        m_params = Eigen::VectorXd::Zero(dynamics_model.getSymbolicConstParameters().size1());
        m_sim_engine->initialize(m_state, m_control, m_params);
        m_output = m_sim_engine->get_output();

        p_control_lb = eigen_casadi_conversion::toEigenX(dynamics_model.u_lbound);
        p_control_ub = eigen_casadi_conversion::toEigenX(dynamics_model.u_ubound);
    }
#endif

    void setup_atmosphere(double mean_windFrom_deg, double mean_wind_speed)
    {
        p_atmosphere.mean_windFrom_deg = mean_windFrom_deg;
        p_atmosphere.mean_wind_speed = mean_wind_speed;

        p_atmosphere.mean_vW = {mean_wind_speed * -cos(mean_windFrom_deg * M_PI / 180.0),
                                mean_wind_speed * -sin(mean_windFrom_deg * M_PI / 180.0),
                                0};
        m_vW_ned = p_atmosphere.mean_vW; //.template block<2, 1>(0, 0);

        if (p_sim_opts.atmosphere)
        {
            m_windShearModel.setup(p_atmosphere.mean_wind_speed);
            for (auto &dryden : m_drydenTurbulenceModel) { dryden.setup(get_sim_dt(), p_atmosphere.mean_wind_speed); }
            m_discreteGustModel.setup(get_sim_dt(), p_atmosphere.mean_windFrom_deg, p_atmosphere.mean_wind_speed);
        }
    }

    void set_actuator_rate(double actuator_rate)
    {
        p_sim_opts.actuator_dt = floor(1.0 / actuator_rate * 1e4) / 1e4; // Round to 0.1 ms
        if (m_VERBOSE >= PrintLevel1) std::cout << "p_sim_opts.actuator_dt: " << p_sim_opts.actuator_dt << "\n";
    }
    void set_partial_state(const Eigen::VectorXd &state, int start_index = 0)
    {
        if (state.size() > 0)
        {
            m_state.segment(start_index, state.size()) = state;
            m_sim_engine->set_state(m_state);
            //std::cout << "initial state: " << m_sim_engine->get_state() << "\n";
        }
    }
    void set_partial_control(const Eigen::VectorXd &control, int start_index = 0)
    {
        if (control.size() > 0)
        {
            Eigen::VectorXd full_control = m_control;
            full_control.segment(start_index, control.size()) = control;

            // Don't write to m_control directly because FlightSimulator uses an additional control_in layer
//            SimulatorBase::set_control(full_control);
            set_control(full_control);
//            m_control_in = full_control;
        }
    }
    void set_control(const Eigen::VectorXd &control) // Need to hide set_control from base class
    {
        // Need to hide set_control from base class because FlightSimulator uses an additional control_in layer
//        SimulatorBase::set_control(control);
        m_control_in = control;
    }

    struct Atmosphere
    {
        double airDensity_ground{1.15};
        double mean_windFrom_deg{170.0};
        double mean_wind_speed{1};
        Eigen::Vector3d mean_vW{0, 0, 0};
    } p_atmosphere{};
    struct FlightSimOptions
    {
        bool actuator_dynamics{false};
        double actuator_dt{1.0 / 50.0};
        bool atmosphere{false};
        bool tether{false};
    } p_sim_opts{};

protected:
    /** Execution **/
    void init_impl() override;
    bool simulate_once_impl() override;

    Eigen::VectorXd m_control_in; // Buffer for incoming control, need to shadow set_control and write to control_in
    Eigen::VectorXd p_control_lb, p_control_ub; // Physical control limits

    /* Actuator update */
    double m_sim_time_actuator_update{0};
    void update_control()
    {
        const double sim_time_diff = m_sim_time - m_sim_time_actuator_update;

        if (sim_time_diff >= p_sim_opts.actuator_dt) // actuator_dt is already corrected by simulation speed factor
        {
            m_sim_time_actuator_update = m_sim_time;

            /* Only update servo commands at some rate (e.g. 50 Hz) */
            m_control = p_control_lb.cwiseMax(m_control_in.cwiseMin(p_control_ub));// Clip control to physical limits
            if (m_VERBOSE >= PrintLevel2)
                std::cout << "update control: " << m_control_in.transpose() << "->" << m_control.transpose() << "\n";
        }
    }

    /* Dynamic Wind */
    Eigen::Vector3d m_vW_ned{0, 0, 0};
    atmosphere_models::WindShearModel m_windShearModel;
    std::array<atmosphere_models::turbulence_models::psd::DrydenTurbulenceModel, 3> m_drydenTurbulenceModel;
    atmosphere_models::turbulence_models::discrete::DiscreteGustModel m_discreteGustModel;

    /* Dynamics output */
    double m_Va_pitot{10}, m_Va{10}, m_alpha{0}, m_beta{0};
    Eigen::Quaternion<double> m_q_nb{1, 0, 0, 0};
    Eigen::Vector3d m_nongravForce{0, 0, 0};
    Eigen::Vector3d m_tethForce{0, 0, 0};

    static void print_hit_ground()
    {
        std::cout << "\n"
                  << "----------------------------------\n"
                     "Hit the ground. End of simulation.\n"
                  << "----------------------------------\n";
    }
};

class TouchdownSimulator : public FlightSimulator
{
public:

protected:
    bool simulate_once_impl() override
    {
        update_control();

        /* Update wind */
        if (p_sim_opts.atmosphere)
        {
            const double height = -m_state(8);
            /* Wind shear */
            const double windspeed_scaler = (p_atmosphere.mean_wind_speed > 0) ?
                                            m_windShearModel.get_windspeed(height) / p_atmosphere.mean_wind_speed : 0;
            /* Turbulence */
            Eigen::Vector3d n_vTurb(0, 0, 0);
            Eigen::Vector3d b_wTurb(0, 0, 0);
            m_drydenTurbulenceModel[0].run(height, m_Va);
            m_drydenTurbulenceModel[0].get_turbulence_ned(m_q_nb, m_alpha, m_beta, n_vTurb, b_wTurb);
            /* Gusts */
            Eigen::Vector3d n_vGust(0, 0, 0);
            m_discreteGustModel.getGustVelNED(n_vGust);

            /* Total wind velocity (NED) */
            m_vW_ned = windspeed_scaler * p_atmosphere.mean_vW + n_vTurb + n_vGust;

            /* Set new wind velocity to simulation engine */
            m_params = m_vW_ned;
            m_sim_engine->set_params(m_params);
            /* Set wind angular rates to state */
            m_state.segment(3, 3) += b_wTurb;
        }

        /* Run simulation engine */
        m_output = m_sim_engine->get_output(); // Could apply a sensor model to m_output here

        m_Va_pitot = m_output(0);
        m_Va = m_output(1);
        m_alpha = m_output(2);
        m_beta = m_output(3);
        m_nongravForce = m_output.segment(4, 3);
        if (p_sim_opts.tether) { m_tethForce = m_output.segment(7, 3); }
//        std::cout << "output: " << m_output.transpose() << "\n";
//        m_output(Slice(10, m_output.size1())) *= 180.0 / M_PI;
//        const std::vector<double> debug = m_output(Slice(10, m_output.size1())).nonzeros();
//        std::cout << "debug: " << debug << "\n";

        m_sim_engine->simulate_one_step(m_state, m_control);
        m_state = m_sim_engine->get_state();

        // Could use an additional measured_state variable here equal to [sensor model](m_state).

        bool hit_ground = (-m_state(8) < 0.0 &&                       // Below ground
                           m_state.segment(0, 3).norm() > 1.0);   // Speed > 1 m/s
        if (hit_ground) print_hit_ground();
        return !hit_ground;
    }
private:

};

} // namespace flight
} // namespace simulation

#endif //SRC_FLIGHTSIMULATOR_HPP
