#include "simulation_flight/FlightSimulator.hpp"

using namespace simulation::flight;

void FlightSimulator::init_impl()
{
    std::cout << "\n[Simulator]: Initializing simulation ...\n";

    m_Va_pitot = m_Va = m_state.segment(0, 3).norm();
    m_q_nb.coeffs() << m_state.segment(10, 3), m_state(9);

    std::cout << "[Simulator]: Actuator dynamics " << (p_sim_opts.actuator_dynamics ? "ON" : "OFF") << "\n";
    std::cout << "[Simulator]: Actuator rate: ";
    if (p_sim_opts.actuator_dt > 0) std::cout << 1.0 / p_sim_opts.actuator_dt << " Hz\n";
    else std::cout << "Inf\n";

    /** Setup Wind Models **/
    std::cout << "[Simulator]: Atmosphere " << (p_sim_opts.atmosphere ? "ON" : "OFF") << "\n";
    if (p_sim_opts.atmosphere)
    {
        setup_atmosphere(p_atmosphere.mean_windFrom_deg ,p_atmosphere.mean_wind_speed);
    }

    std::cout << std::defaultfloat << std::setprecision(4)
              << "[Simulator]: Initial state  : [" << m_state.transpose() << "]\n"
              << "[Simulator]: Initial control: [" << m_control.transpose() << "]\n"
              << "[Simulator]: Initial params : [" << m_params.transpose() << "]\n"
              << "[Simulator]: Control bounds : [" << p_control_lb.transpose() << "] < u < ["
              << p_control_ub.transpose() << "]\n";

    /* Evaluate model once to fill memory */
    m_dyn_out = m_sim_engine->get_dyn_out(m_control);
}
bool FlightSimulator::simulate_once_impl()
{
    const unsigned N_aircraft = std::floor(m_state.rows() / 13);

    update_control();

    /* Update wind */
    if (p_sim_opts.atmosphere)
    {
        for (unsigned k = 0; k < N_aircraft; k++)
        {
            const unsigned nx0 = k * 13;
            const unsigned nz0 = k * 10;
            const double height = -m_state(k * 13 + 8); // HaG Height above ground

            /* Wind shear */
            const double windspeed_scaler = (p_atmosphere.mean_wind_speed > 0) ?
                                            (m_windShearModel.get_windspeed(height) / p_atmosphere.mean_wind_speed) : 0;
            /* Turbulence (use data from previous simulation step) */
            const double Va = m_dyn_out(nz0 + 1);
            const double alpha = m_dyn_out(nz0 + 2);
            const double beta = m_dyn_out(nz0 + 3);
            Eigen::Quaterniond q_nb;
            q_nb.coeffs() << m_state.segment<3>(nx0 + 10), m_state(nx0 + 9);

            Eigen::Vector3d n_vTurb(0, 0, 0);
            Eigen::Vector3d b_wTurb(0, 0, 0);
            m_drydenTurbulenceModel[k].run(height, Va);
            m_drydenTurbulenceModel[k].get_turbulence_ned(q_nb, alpha, beta, n_vTurb, b_wTurb);
            /* Gusts */
            Eigen::Vector3d n_vGust(0, 0, 0);
            m_discreteGustModel.getGustVelNED(n_vGust);

            /* Total wind velocity (NED) */
            if (k == 0) m_vW_ned = windspeed_scaler * p_atmosphere.mean_vW + n_vTurb + n_vGust;

            /* Set new wind velocity to simulation engine */
            m_params.segment<3>(k * 3) = windspeed_scaler * p_atmosphere.mean_vW + n_vTurb + n_vGust;

            /* Set wind angular rates to state */
            m_state.segment<3>(k * 13 + 3) += b_wTurb;
        }
        m_sim_engine->set_params(m_params);
    }

    /* Run simulation engine */
//    std::cout << "ctrl: " << m_control.transpose() << "\n";
    m_dyn_out = m_sim_engine->get_dyn_out(m_control);

    m_q_nb.coeffs() << m_state.segment(10, 3), m_state(9);
    m_Va_pitot = m_dyn_out(0);
    m_Va = m_dyn_out(1);
    m_alpha = m_dyn_out(2);
    m_beta = m_dyn_out(3);
    m_nongravForce = m_dyn_out.segment<3>(4);
    if (p_sim_opts.tether) { m_tethForce = m_dyn_out.segment<3>(7); }
//        std::cout << "dynout: " << m_dyn_out.transpose() << "\n";
//        //m_dyn_out(Slice(10, m_dyn_out.size1())) *= 180.0 / M_PI;
        // Eigen::VectorXd debug = m_dyn_out.bottomRows(3*12);
        // std::cout << "debug: " << debug.transpose() << "\n";
    const Eigen::Vector3d teth_control{m_control_in(4 + 0), m_control_in(4 + 1), m_control_in(6 + 4 + 0)};
    if (teth_control.cwiseAbs().sum() > 0)
    {
        const Eigen::Vector3d tethlens = m_state.segment<3>(N_aircraft * 13 + 6);
        std::cout << std::setprecision(5) << "tethlen: (lat) " << tethlens(0) << " (lon) " << tethlens(1) << " (main) " << tethlens(2) << "\n";
    }

    m_sim_engine->simulate_one_step(m_state, m_control);
    m_state = m_sim_engine->get_state();
    m_output = m_sim_engine->get_output(); // Could apply a sensor model to m_output here

    // Could use an additional measured_state variable here equal to [sensor model](m_state).

    const double height = -m_state(8);
    bool hit_ground = (true && height < 0.0);
    if (hit_ground) print_hit_ground();
    return !hit_ground;
}