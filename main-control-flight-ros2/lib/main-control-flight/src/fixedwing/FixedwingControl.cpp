#include "main_control_flight/fixedwing/FixedwingControl.hpp"

#define COLLECTIVE0 0.6
#define LIFT_TO_ELEV_CMD 0.1
#define TETHMAIN_MAX_VEL 8.0

using namespace control::flight::fixedwing;

/** Protected members ============================================================================ **/
void FixedwingControl::init_impl()
{
    std::cout << "[Fixed-wing Control]\n";
}
void FixedwingControl::test_impl()
{
    /* Populate states with meaningful values */
    const TestState testState{};

    for (auto &state: states)
    {
        state.pos = testState.posNED;
        state.pos_avail = true;

        state.q_nb = testState.q_ned_b;

        state.vel_body = testState.vel_bodyFRD;
        state.vel_ned = state.q_nb * state.vel_body;
        state.timestamp = SteadyClock::now();
        state.vel_avail = true;

        state.rates = testState.ratesFRD;
        state.acc = testState.accFRD;

        state.windEst = testState.windEstNE;
        state.windEst_avail = true;
    }

    manual = testState.manual;

    run_once();
}

void FixedwingControl::compute_control_impl()
{
    commands = {};
    command_valid = false;

    for (auto &state: states) { state.update_euler_angles(); }

    // Activate main control?
    main_control_wasActive = main_control_isActive;
    if (true)
    {
        // Variant 1: Toggle active/inactive
        bool manual_toggle = activeParser.wants_active_tap<bool>(manual.activation_button, ActiveParser::EQUAL, 1);
        if (manual_toggle) { main_control_isActive = !main_control_isActive; }
    }
    else
    {
        // Variant 2: Keep pressed = active
        bool manual_toggle = activeParser.wants_active_hold<bool>(manual.activation_button, ActiveParser::EQUAL, 1);
        main_control_isActive = manual_toggle;
    }

    if (main_control_isActive)
    {
        if (!main_control_wasActive)
        {
            /* Transition to ACTIVE */
            printModeChange("\nMAIN CONTROL ACTIVE ---------------------------------------\n");

            if (m_mission_commander.getMission() != MissionMode::NOMISSION)
            {
                m_mission_commander.startMission();
            }
        }
        // Do every time while activated
        chooseFlightControlMode();
        control_flight_high_level();
        control_flight_low_level();
    }
    else
    {
        if (main_control_wasActive)
        {
            /* Transition to INACTIVE */
            printModeChange("\nMAIN CONTROL INACTIVE ---------------------------------------\n");
            std::cout << "(manual passthrough)\n";

            if (m_mission_commander.isActive()) { m_mission_commander.endMission(); }
//            changeFlightControlMode(FixedwingControlMode::MANUAL);
        }
        // Run controller anyway but don't publish the commands
        control_flight_high_level();
        control_flight_low_level();
        command_valid = false;
    }

    // Prepare command to be published
    prepareCommand();

    // Loop modules
    m_control_mode.spin_once();
}

void FixedwingControl::chooseFlightControlMode()
{
    if (m_mission_commander.isActive())
    {
        executeMission();
    }
//    else
//    {
//        /* If defined, use parameter wish */
//        changeFlightControlMode(p.flightControlMode);
//    }
}
void FixedwingControl::changeFlightControlMode(const FixedwingControlMode::ControlMode &flightControlModeToSet,
                                               bool inhibitParamUpdate)
{
    if (flightControlModeToSet == m_control_mode()) return; // Only proceed if mode is new

//    if (!pos_isAvail)
//    {
//        if (fc_isArmed)
//        {
//            /* If FC is armed and position is not avail.
//             * Reject flight modes which need position/velocity data and degrade them to attitude mode */
//            switch (flightControlModeToSet)
//            {
//                case FlightControlMode::MPC:
//                case FlightControlMode::AWETURN:
//                case FlightControlMode::AWEEIGHT:
//                case FlightControlMode::AWECIRCLE:
//                case FlightControlMode::AWECLIMB:
//                case FlightControlMode::LOITER:
//                case FlightControlMode::ALTHOLD:
//                case FlightControlMode::SKID:
//                    /* Degradation to ATTITUDE */
//                    flightControlModeToSet = FlightControlMode::ATTITUDE;
//                    std::cout << "Position NOT AVAIL: degraded to ATTITUDE\n";
//                    break;
//                default:
//                    /* Everything is fine, do not change anything. */
//                    ;
//            }
//        }
//    }

    m_control_mode.set(flightControlModeToSet);

    /* Reset rate controller error integrals and stick listener */
//    mpc_valid = false;
//    mpc_wasActive = false;
//    set_mpcActivation(false);
//    highLevel_resetSetpoints();
//    highLevel_controlFlight();
}

void FixedwingControl::setAuxCommands(const Vector3 &tetherLatLonMain_vels)
{
        // Use aux0 channel to command LATERAL tether reel in/out velocity
        commands[1].aux0 = tetherLatLonMain_vels(0);
        // Use aux1 channel to command LONGITUDINAL tether reel in/out velocity
        commands[0].aux1 = tetherLatLonMain_vels(1);
        // Use aux1 channel to command MAIN tether reel in/out velocity
        commands[0].aux0 = tetherLatLonMain_vels(2);
}
void FixedwingControl::prepareCommand()
{
    //std::cout << "prep command: man, com: " << manual.roll << ", " << commands[0].dR << "\n";
    if (!command_valid)
    {
        /* We always publish actuator_control messages, so that we can switch into PX4's offboard mode.
         * Before offboard mode is active, we give through manual controls. Like this, there is no jump switching
         * from PX4 MANUAL mode to active offboard mode.
         */
        commands[0].aileron = manual.roll;
        commands[0].elevator = manual.pitch;
        commands[0].throttle = manual.thrust;
        commands[0].rudder = manual.yaw;
        commands[0].aux0 = 0; //manual.trigger_0;
        commands[0].aux1 = 0; //manual.trigger_1;

        commands[2] = commands[1] = commands[0];
    }

    /* Constrain thrust command for safety */
//    commands[0].throttle = constrain(commands[0].throttle, 0, 1);

    /* Ensure command is within bounds. PX4 would anyway clip it afterwards, but we will not log that,
     * so we have to ensure consistency with this commands[0]. */
    for (FixedwingCommand &command : commands)
    {
        command.throttle = constrain(command.throttle, 0, 1);
        command.elevator = constrain(command.elevator, -1, 1);
        command.rudder = constrain(command.rudder, -1, 1);
        command.aileron = constrain(command.aileron, -1, 1);
        command.aux0 = constrain(command.aux0, -1, 1);
        command.aux1 = constrain(command.aux1, -1, 1);
    }
}

/** Missions ================================================================================= **/
void FixedwingControl::executeMission()
{
    do
    {
        switch (m_mission_commander.getMission())
        {
            case MAWE:
                mission_mawe();
                break;

            default:;
        }
    } while (m_mission_commander.runAgain());
}

void FixedwingControl::mission_mawe()
{
    // Estimate tether lengths from kite and conpoint positions
    const double tethmain_len = states[3].pos.norm();

    // Disk center position
    Vector3 n_pos_avg(0, 0, 0);
    for (unsigned k = 0; k < p_control_kites; k++) { n_pos_avg += states[k].pos / p_control_kites; }
    const double dist = n_pos_avg.norm(); // -n_pos_avg(2);
    // Average rotor disk radius
    Eigen::Vector<double, 3> Rs;
    for (unsigned k = 0; k < p_control_kites; k++)
    {
        double dist = (k < p_control_kites - 1) ?
                                (states[k].pos - states[k + 1].pos).norm() :
                                (states[k].pos - states[0].pos).norm();
        Rs[k] = dist / (2.0 * std::cos((M_PI / p_control_kites) / 2.0));
    }
    const double R_mean = Rs.mean();
    const double tethlon_len = Vector2((n_pos_avg - states[3].pos).norm(), R_mean).norm();
    const double tethlat_len = 2 * R_mean;

    switch (m_mission_commander.getActiveLegId())
    {
        case 0: /* AWE Climb Mode */
            if (m_mission_commander.doUntilDone())
            {
                std::cout << "C0: climb (tethmain --> 50m, tethlon --> 20m, tethlat --> 13m)\n";
                m_awec.thr_planned = 1;
                m_awec.coll_planned = 1;
                m_awec.liftdir_planned = {-0.6, -1.0};
                m_awec.latlon_ref = {radians(10), radians(0)}; // Irrelevant for now

                changeFlightControlMode(FixedwingControlMode::MAWE_CLIMB);
                m_mission_commander.activateMarker();
            }
            // Commands
            {
                // Schedule tether reel-out
                m_awec.tethlen_vels_planned.setZero();
                if (tethlon_len < 20) { m_awec.tethlen_vels_planned(1) = 1; } // LON tethers
                else if (tethmain_len < 50) { m_awec.tethlen_vels_planned(2) =  2.0 / TETHMAIN_MAX_VEL; } // MAIN tether
//                else if (tethlat_len < 15) { m_awec.tethlen_vels_planned(0) = 1; } // LAT tethers
                //        else if (tethmain_len < 65) { m_awec.tethlen_vels_planned(2) = 1; } // MAIN tether

//                if (tethlat_len > 15) { m_mission_commander.reportTaskDone(); }
                if (tethmain_len > 50) { m_mission_commander.reportTaskDone(); }
            }
            break;
        case 1: /* Prepare reel-out */
            if (m_mission_commander.doUntilDone())
            {
                std::cout << "C1: prep gen (lat = 45 deg, 10s)\n";
                m_awec.thr_planned = 0;
                m_awec.coll_planned = 0.5;
                m_awec.latlon_ref = {radians(55), radians(0)};
                m_awec.tethlen_vels_planned.setZero();
                changeFlightControlMode(FixedwingControlMode::MAWE_LATLON);
                m_mission_commander.activateMarker();
            }
            // Commands
            {
                if (tethlat_len < 15) { m_awec.tethlen_vels_planned(0) = 1; } // LAT tethers

                if (tethlat_len > 15)
                {
                    m_awec.tethlen_vels_planned.setZero();
                    if (m_mission_commander.getLegAge() > 17.0)
                    {
                        m_mission_commander.reportTaskDone();
                    }
                }
            }
            break;
        case 2: /* Pumping cycle: reel-out */
            if (m_mission_commander.doUntilDone())
            {
                std::cout << "C2: gen (tethmain --> 120m)\n";
                // COMMAND collective, main tether vel > 0, until l1 reached
                m_awec.coll_planned = 0.7;
                m_awec.tethlen_vels_planned(2) = 2.0 / TETHMAIN_MAX_VEL;
                m_mission_commander.activateMarker();
            }
            // Monitor task done condition here, because MAWE_LATLON mode is too generic
            {
                const double tethmain_len = states[3].pos.norm();
                if (tethmain_len > 120) { m_mission_commander.reportTaskDone(); }
                break;
            }
        case 3: /* Prepare reel-in */
            if (m_mission_commander.doFor_Seconds(10))
            {
                std::cout << "C3: prep degen (lat = 5deg, lon=-45, 10s)\n";
                m_awec.coll_planned = -0.3;
                m_awec.latlon_ref = {radians(40), radians(0)};
                m_awec.tethlen_vels_planned.setZero();
                m_mission_commander.activateMarker();
            }
            break;
        case 4: /* Reel-in */
            if (m_mission_commander.doUntilDone())
            {
                std::cout << "C4: degen (tethmain --> 50m)\n";
                // COMMAND main tether vel < 0, until l0 reached
                m_awec.coll_planned = -0.7;
                m_awec.tethlen_vels_planned(2) = -8.0 / TETHMAIN_MAX_VEL;
                m_mission_commander.activateMarker();
            }
            // Monitor task done condition here, because MAWE_LATLON mode is too generic
            {
                if (tethmain_len < 50) { m_mission_commander.reportTaskDone(); }
            }
            break;
        case 5: /* Prep Descent */
            if (m_mission_commander.doFor_Seconds(10))
            {
                std::cout << "C5: Prep descent (10s)\n";
                // COMMAND main tether vel < 0, until l0 reached
                m_awec.coll_planned = 0.5;
                m_awec.tethlen_vels_planned.setZero();
                m_mission_commander.activateMarker();
            }
            break;
        case 6: /* Descent */
            if (m_mission_commander.doUntilDone())
            {
                std::cout << "C6: descent (tethmain --> 0.5m)\n";
                m_awec.thr_planned = 0.1;
                m_awec.coll_planned = 0.3;
                m_awec.liftdir_planned = {-0.6, -0.7};
//                m_awec.latlon_ref = {radians(10), radians(0)}; // Irrelevant for now

                changeFlightControlMode(FixedwingControlMode::MAWE_CLIMB);

                m_mission_commander.activateMarker();
            }
            // Commands
            {
                // Schedule tether reel-in
                m_awec.tethlen_vels_planned.setZero();
                if (tethlat_len > 5) { m_awec.tethlen_vels_planned(0) = -1; } // LAT tether
                else if (tethmain_len > 3) { m_awec.tethlen_vels_planned(2) = -2.0 / TETHMAIN_MAX_VEL; } // MAIN tether
                else if (tethmain_len > 1) { m_awec.tethlen_vels_planned(2) = -1.0 / TETHMAIN_MAX_VEL; } // MAIN tether
                else if (tethlon_len > 6) { m_awec.tethlen_vels_planned(1) = -1; }

//                if (tethmain_len < 0.5) { m_mission_commander.reportTaskDone(); }
                if (tethlon_len < 6) { m_mission_commander.reportTaskDone(); }
            }
            break;
        case 7: /* Landing */
            if (m_mission_commander.doFor_Seconds(1))
            {
                std::cout << "C7: landing hover (1s)\n";
                m_awec.thr_planned = 0.5;
                m_awec.coll_planned = 0.3;
                m_awec.tethlen_vels_planned.setZero();
                m_mission_commander.activateMarker();
            }
            break;
        case 8: /* Mission End */
            if (m_mission_commander.endMission())
            {
                m_mission_commander.activateMarker();
            }
            break;
        default:
            /* Inactive */
            ;
    }
}

/** Private use ============================================================================== **/
void FixedwingControl::control_flight_complete()
{
    command_valid = false;
}

void FixedwingControl::control_flight_high_level()
{
    heading_ref_valid = false;
    pos_ref_valid = false;

    velNED_ref = {0, 0, 0};
    velNED_ref_valid = false;

    n_acc_ref = {0, 0, -9.81};
    n_acc_ref_valid = false;

    rates_ref = {0, 0, 0};
    rates_ref_valid = false;

    /** High-level references **/
    const double centered_manual_thrust = (manual_avail ?
                                          (2.0 * (manual.thrust - 0.5)) : 0); // Deflection from center

    const double rate_gain = radians(60);
    //std::cout << "manual (" << (manual_avail ? "AVAIL" : "NOT AVAIL") << "): " << manual << "\n";

    /** Position reference **/
    if (m_control_mode() == FixedwingControlMode::MAWE_LATLON)
    {
        if (m_control_mode.has_changed())
        {
            printModeChange("MAWE LATLON");
        }

        /* Parameter triggers */
        const bool p_up = (m_param_parser_up.wants_active_tap(manual.param_up_down, ActiveParser::GREATER, 0.5));
        const bool p_dn = (m_param_parser_dn.wants_active_tap(manual.param_up_down, ActiveParser::SMALLER, -0.5));
        const bool p_right = (m_param_parser_right.wants_active_tap(manual.param_right_left, ActiveParser::GREATER, 0.5));
        const bool p_left = (m_param_parser_left.wants_active_tap(manual.param_right_left, ActiveParser::SMALLER, -0.5));

        const double latlon_step = radians(10);
        if (p_up || p_dn)
        {
            int param_manip{0};
            if (p_up) { param_manip = 1; }
            if (p_dn) { param_manip = -1; }
            m_awec.latlon_ref(0) += param_manip * latlon_step;
            if (std::abs(m_awec.latlon_ref(0)) < 1e-8) { m_awec.latlon_ref(0) = 0; }
        }
        if (p_right || p_left)
        {
            int param_manip{0};
            if (p_right) { param_manip = 1; }
            if (p_left) { param_manip = -1; }
            m_awec.latlon_ref(1) += param_manip * latlon_step;
            if (std::abs(m_awec.latlon_ref(1)) < 1e-8) { m_awec.latlon_ref(1) = 0; }
        }

        m_awec.latlon_ref_valid = true;

        // Winddome frame (w)
        m_awec.wind_from = radians(160);
        m_awec.update_winddome(m_awec.wind_from);

        // Local NED frame on winddome (l)
        std::vector<Vector3> posns;
        Vector3 n_pos_avg(0, 0, 0);
        for (unsigned k = 0; k < p_control_kites; k++) { n_pos_avg += states[k].pos / p_control_kites; }

        const Vector2 w_latlon = awe_control3::AweHeliController::get_local_latlon(m_awec.q_wn * n_pos_avg);
//        std::cout << "w_latlon: " << degrees(w_latlon).transpose() << "\n";
        const Quaternion q_lw = T2quat(-w_latlon(0)) * T1quat(w_latlon(1));
//        const Quaternion q_ln = q_lw * m_awec.q_wn;

        const double l_brg_to_ref = awe_control3::AweHeliController::get_direct_bearing(w_latlon, m_awec.latlon_ref);
        const double dist_on_sphere = awe_control3::AweHeliController::get_surf_distance(w_latlon, m_awec.latlon_ref, n_pos_avg.norm());
        const double anglediff_on_sphere = awe_control3::AweHeliController::get_surf_distance_angle(w_latlon, m_awec.latlon_ref);

        // Average rotor disk radius
        Eigen::VectorXd Rs(p_control_kites), dists(p_control_kites);
        for (unsigned k = 0; k < p_control_kites; k++)
        {
            double dist = (k < p_control_kites - 1) ?
                                    (states[k].pos - states[k + 1].pos).norm() :
                                    (states[k].pos - states[0].pos).norm();
            dists[k] = dist;
            Rs[k] = dist / (2.0 * std::cos((M_PI / p_control_kites) / 2.0));
        }
        const double R_mean = Rs.mean();
        const double dist_mean = dists.mean();

        const double angleErr_lim{radians(15)};
        const double scaler = constrain(2 - (R_mean - 5) / (15 - 5), 1, 2); // Scale between factor 1 and 2
//        const double scaler = constrain(2 - (dist_mean - 5) / (15 - 5), 1, 1.5); // Scale between factor 1 and 2
        const double liftMag_max = 0.5 * scaler;
//        std::cout << "scaler: " << scaler << "\n"; // Scale between factor 1 and 2

        // #TODO: need to ensure traction has priority, otherwise not controllable
        const double liftMag_per_angleErr{liftMag_max / angleErr_lim};
        const double angleErr_clipped = constrain(anglediff_on_sphere, -angleErr_lim, angleErr_lim);
        const double liftMag = liftMag_per_angleErr * angleErr_clipped;

        const Vector3 l_liftdir_cmd = liftMag * Vector3(cos(l_brg_to_ref), sin(l_brg_to_ref), 0);

        if (p_up || p_dn || p_right || p_left)
        {
            std::cout << "latlon_ref " << degrees(m_awec.latlon_ref).transpose() << "\n";
//            std::cout << "brg_to_ref: " << degrees(l_brg_to_ref) << ", " << dist_on_sphere << ", " << degrees(anglediff_on_sphere) << "\n";
        }

        // Convert to lift direction in disk frame
        Vector3 w_liftdir_cmd = q_lw.inverse() * l_liftdir_cmd;
        const Quaternion q_dw = awe_control3::AweHeliController::get_q_dn(m_awec.q_wn * n_pos_avg);
        const Vector3 d_liftdir_cmd = q_dw * w_liftdir_cmd;
//        std::cout << "liftdir_cmd: (l) " << l_liftdir_cmd.transpose() << " (w)" << w_liftdir_cmd.transpose() << " (d) " << d_liftdir_cmd.transpose() << "\n";

        // Lift
        const double collective = COLLECTIVE0 + m_awec.coll_planned + -manual.thrust;

        m_awec.liftdir_ref = d_liftdir_cmd.segment<2>(0);
        m_awec.liftdir_ref_valid = true;

        // Propagate liftdir
        Eigen::VectorXd normlift_tilt_cmds = control_multi_lift(m_awec.p_liftdir_trim + m_awec.liftdir_ref);
        Eigen::VectorXd normlift_cmds = control_centrifugal_sanity(normlift_tilt_cmds);
        normlift_cmds.array() += collective;
        const Vector3 elev_cmds = -LIFT_TO_ELEV_CMD * normlift_cmds; // Negative elevator command generates more lift

        // Control each kite's throttle
        const double throttle = manual.trigger_1;

        // Tether lengths
        Vector3 tetherLatLonMain_vels = m_awec.tethlen_vels_planned;
        if (manual.button_2) { tetherLatLonMain_vels(0) += -manual.yaw; } // While triangle button pressed: LAT
        else if (manual.button_3) { tetherLatLonMain_vels(1) += -manual.yaw; } // While rectangle button pressed: LON
        else if (manual.button_0) { tetherLatLonMain_vels(2) +=  -manual.yaw; } // While X button is pressed: MAIN

        for (unsigned k = 0; k < p_control_kites; k++)
        {
            commands[k].elevator = elev_cmds(k);
            commands[k].throttle = throttle;
        }
        setAuxCommands(tetherLatLonMain_vels);

//        if (anglediff_on_sphere < radians(5)) { m_mission_commander.reportTaskDone(); }
        command_valid = true;
    }
    else if (m_control_mode() == FixedwingControlMode::MAWE_CLIMB)
    {
        if (m_control_mode.has_changed())
        {
            printModeChange("MAWE CLIMB");
        }

        // Lift
        const double collective = COLLECTIVE0 + m_awec.coll_planned + -manual.thrust;

        m_awec.liftdir_ref = m_awec.liftdir_planned + Eigen::Vector2d(manual.pitch, -manual.roll);
        m_awec.liftdir_ref_valid = true;

        // Propagate liftdir
        Eigen::VectorXd normlift_tilt_cmds = control_multi_lift(m_awec.p_liftdir_trim + m_awec.liftdir_ref);
        Eigen::VectorXd normlift_cmds = control_centrifugal_sanity(normlift_tilt_cmds);
        normlift_cmds.array() += collective;
        const Vector3 elev_cmds = -LIFT_TO_ELEV_CMD * normlift_cmds; // Negative elevator command generates more lift

        // Set climb throttle and elevator
        const double throttle = m_awec.thr_planned + manual.trigger_1;

        // Tether lengths
        Vector3 tetherLatLonMain_vels = m_awec.tethlen_vels_planned;
        if (manual.button_2) { tetherLatLonMain_vels(0) += -manual.yaw; } // While triangle button pressed: LAT
        else if (manual.button_3) { tetherLatLonMain_vels(1) += -manual.yaw; } // While rectangle button pressed: LON
        else if (manual.button_0) { tetherLatLonMain_vels(2) +=  -manual.yaw; } // While X button is pressed: MAIN

        for (unsigned k = 0; k < p_control_kites; k++)
        {
            commands[k].elevator = elev_cmds(k);
            commands[k].throttle = throttle;
        }
        setAuxCommands(tetherLatLonMain_vels);

        command_valid = true;
    }
    else if (m_control_mode() == FixedwingControlMode::MAWE_DESCENT)
    {
        if (m_control_mode.has_changed())
        {
            printModeChange("MAWE DESCENT: NOT IMPLEMENTED! Equivalent to MAWE CLIMB");
        }
    }
    else if (m_control_mode() == FixedwingControlMode::MAWE_MANUAL)
    {
        if (m_control_mode.has_changed())
        {
            printModeChange("MAWE MANUAL");
        }

        /* Parameter triggers */
        const bool p_up = (m_param_parser_up.wants_active_tap(manual.param_up_down, ActiveParser::GREATER, 0.5));
        const bool p_dn = (m_param_parser_dn.wants_active_tap(manual.param_up_down, ActiveParser::SMALLER, -0.5));
        const bool p_right = (m_param_parser_right.wants_active_tap(manual.param_right_left, ActiveParser::GREATER, 0.5));
        const bool p_left = (m_param_parser_left.wants_active_tap(manual.param_right_left, ActiveParser::SMALLER, -0.5));

        constexpr double normlift_step = 0.05;
        if (p_up || p_dn)
        {
            int param_manip{0};
            if (p_up) { param_manip = 1; }
            if (p_dn) { param_manip = -1; }
            m_awec.p_liftdir_trim(0) += param_manip * normlift_step;
        }
        if (p_right || p_left)
        {
            int param_manip{0};
            if (p_right) { param_manip = 1; }
            if (p_left) { param_manip = -1; }
            m_awec.p_liftdir_trim(1) += param_manip * normlift_step;
        }
        if (p_up || p_dn || p_right || p_left)
        {
            std::cout << "d_liftdir_trim: " << m_awec.p_liftdir_trim.transpose() << " ("
                      << (int) degrees(std::atan2(m_awec.p_liftdir_trim(1), m_awec.p_liftdir_trim(0)))
                      << " @ " << m_awec.p_liftdir_trim.norm() << ")\n";
        }

        // Lift
        const double collective = COLLECTIVE0 + -manual.thrust;

        m_awec.liftdir_ref = Eigen::Vector2d(manual.pitch, -manual.roll);
        m_awec.liftdir_ref_valid = true;

        // Propagate liftdir
        Eigen::VectorXd normlift_tilt_cmds = control_multi_lift(m_awec.p_liftdir_trim + m_awec.liftdir_ref);
        Eigen::VectorXd normlift_cmds = control_centrifugal_sanity(normlift_tilt_cmds);
        normlift_cmds.array() += collective;
        const Vector3 elev_cmds = -LIFT_TO_ELEV_CMD * normlift_cmds; // Negative elevator command generates more lift

        // Control each kite's throttle
        const double throttle = manual.trigger_1;

        // Tether lengths
        Vector3 tetherLatLonMain_vels{0, 0, 0};
        if (manual.button_2) { tetherLatLonMain_vels(0) = -manual.yaw; } // While triangle button pressed: LAT
        else if (manual.button_3) { tetherLatLonMain_vels(1) = -manual.yaw; } // While rectangle button pressed: LON
        else if (manual.button_0) { tetherLatLonMain_vels(2) = -manual.yaw; } // While X button is pressed: MAIN

        for (unsigned k = 0; k < p_control_kites; k++)
        {
            commands[k].elevator = elev_cmds(k);
            commands[k].throttle = throttle;
        }
        setAuxCommands(tetherLatLonMain_vels);

        command_valid = true;
    }
    else if (m_control_mode() == FixedwingControlMode::POSITION)
    {
        if (m_control_mode.has_changed()) { printModeChange("POSITION"); }

        if (std::abs(manual.pitch) > 0.1)
        {
            pos_ref(0) += 0.05 * manual.pitch;
            posvel_ref_needs_update = true;
        }
        if (std::abs(manual.roll) > 0.1)
        {
            pos_ref(1) += 0.05 * -manual.roll;
            posvel_ref_needs_update = true;
        }
        if (std::abs(centered_manual_thrust) > 0.2)
        {
            pos_ref(2) += -0.05 * centered_manual_thrust;
            posvel_ref_needs_update = true;
            m_diag.pos_ref = pos_ref;
        }
        pos_ref_valid = true;

        /* Constraint position ref to room limits */
        pos_ref = constrain(pos_ref, {-2, -2, -2}, {2, 2, -0.5});

        if (std::abs(manual.yaw) > 0.1)
        {
            heading_ref += radians(2) * -manual.yaw;
            if (heading_ref < 0) { heading_ref += radians(360); }
            if (heading_ref > radians(360)) { heading_ref -= radians(360); }
            posvel_ref_needs_update = true;
        }
        heading_ref_valid = true;

        if (posvel_ref_needs_update)
        {
            int defaultPrecision = std::cout.precision();
            std::cout << std::fixed << std::setprecision(2)
                      << "Pos ref - North " << pos_ref(0) << ", East " << pos_ref(1) << ", Height " << -pos_ref(2)
                      << ", Hdg " << degrees(heading_ref)
                      << std::defaultfloat << std::setprecision(defaultPrecision) << "\n";
        }
    }
    else if (m_control_mode() == FixedwingControlMode::VELOCITY)
    {
        if (m_control_mode.has_changed()) { printModeChange("VELOCITY"); }

        Eigen::Vector3d hdg_velNED_ref{};
        hdg_velNED_ref(0) = 5.0 * manual.pitch;
        hdg_velNED_ref(1) = 5.0 * -manual.roll;
        hdg_velNED_ref(2) = 5.0 * -centered_manual_thrust;

        velNED_ref = hdg_velNED_ref; // vel ref in NED
//        velNED_ref = T3quat(-att(2)) * hdg_velNED_ref; vel ref projected in heading frame (front defined by HDG)
        velNED_ref_valid = true;

        rates_ref(2) = rate_gain * -manual.yaw;

//        if (std::abs(manual.yaw) > 0.1)
//        {
//            heading_ref += radians(2) * -manual.yaw;
//            if (heading_ref < 0) { heading_ref += radians(360); }
//            if (heading_ref > radians(360)) { heading_ref -= radians(360); }
//        }
//        heading_ref_valid = true;

//        if (velNED_ref.norm() > 0.0)
//        {
//            int defaultPrecision = std::cout.precision();
//            std::cout << std::fixed << std::setprecision(2)
//                      << "Vel ref - North " << velNED_ref(0) << ", East " << velNED_ref(1) << ", Dn " << -velNED_ref(2)
//                      << std::defaultfloat << std::setprecision(defaultPrecision) << "\n";
//        }
    }
    else if (m_control_mode() == FixedwingControlMode::ATTITUDE)
    {
        if (m_control_mode.has_changed()) { printModeChange("ATTITUDE"); }

        Eigen::Vector3d hdg_acc_ref{};
        hdg_acc_ref(0) = 3.0 * manual.pitch;
        hdg_acc_ref(1) = 3.0 * -manual.roll;
        hdg_acc_ref(2) = -9.81 + 9.81 * -centered_manual_thrust;

        n_acc_ref = T3quat(-states[0].att(2)) * hdg_acc_ref;
        n_acc_ref_valid = true;

        rates_ref(2) = rate_gain * -manual.yaw;

    }
    else if (m_control_mode() == FixedwingControlMode::RATES)
    {
        if (m_control_mode.has_changed()) { printModeChange("RATES"); }

        rates_ref(0) = rate_gain * -manual.roll;
        rates_ref(1) = rate_gain * -manual.pitch;
        rates_ref(2) = rate_gain * -manual.yaw;
        rates_ref_valid = true;
    }
    else if (m_control_mode() == FixedwingControlMode::MANUAL)
    {
        commands[0].aileron = manual.roll;
        commands[0].elevator = manual.pitch;
        commands[0].throttle = manual.thrust;
        commands[0].rudder = manual.yaw;
        command_valid = true;
    }
}
void FixedwingControl::control_flight_low_level()
{
    if (m_mpc_interface.evaluate_mpc_active(main_control_isActive))
    {
        commands[0] = m_mpc_interface.command;
        command_valid = true;
    }

    if (command_valid) { return; } // Have already valid command

//    bool pos_ref_valid_ = pos_ref_valid;
    bool velNED_ref_valid_ = velNED_ref_valid;
    bool n_acc_ref_valid_ = n_acc_ref_valid;
    bool rates_ref_valid_ = rates_ref_valid;

    //std::cout << "LL ";

//    /** Control position with velocity **/
//    if (pos_ref_valid_)
//    {
//        //std::cout << "-> POS ";
//        const double pos_err_lim = 2.0;
//        const Vector3 pos_errcon = constrain(pos_ref - states[0].pos, -pos_err_lim, pos_err_lim);
//
//        velNED_ref = pos_errcon / 1.3;
//        velNED_ref_valid_ = true;
//    }

    /** Control velocity with acceleration **/
    double vert_vel_err{0};
    if (velNED_ref_valid_)
    {
        //std::cout << "-> VEL ";
//        const double vel_err_lim = 2.0;
//        const Vector3 n_vg_errcon = constrain(velNED_ref - states[0].vel_ned, -vel_err_lim, vel_err_lim);
//
//        const Vector3 n_acc_ref_ = Vector3(0, 0, -9.81) + n_vg_errcon / 0.8;
//
//        vert_vel_err = n_vg_errcon(2);
//
//        /** Control acceleration with force pointing (attitude) **/
//        const double pointing_lim = radians(30.0);
        const Vector3 n_acc_refcon = {0, 0,
                                      -9.81}; // PositionAttitudeControl::get_acc_constrained_ref(n_acc_ref_, pointing_lim);

        n_acc_ref = n_acc_refcon;
        n_acc_ref_valid_ = true;
    }

    /** Control pointing and heading with angular velocity **/
    if (n_acc_ref_valid_)
    {
        //std::cout << "-> ACC ";
        const Vector3 pointing_ref = n_acc_ref.normalized();
        const Vector3 rates_refcon = {0, 0, 0}; // PositionAttitudeControl::get_rates_ref(pointing_ref, state[0].q_nb,
        //                            heading_ref, att(2),
        //                            rates_ref(2));
        rates_ref = rates_refcon;
        rates_ref_valid_ = true;

        /** Control linear acceleration with throttle **/
//        std::cout << "n_acc_ref: " << n_acc_ref.transpose() << ", acc: "
//                  << state[0].acc.transpose() << ", velerr: " << vert_vel_err << "\n";
        const double dF = 0; // m_posatt_control.get_thrust(n_acc_ref.norm(), state[0].acc.norm(), vert_vel_err);
        commands[0].throttle = dF; // Throttle [0 ... 1]
    }
    else
    {
        commands[0].throttle = manual.thrust;
    }

    if (!rates_ref_valid && !rates_ref_valid_)
    {
        const Vector3 att_ref = {
                radians(20) * -manual.roll,
                radians(20) * -manual.pitch,
                0};
//        std::cout << "BACKUP ATT (REF " << degrees(att_ref).transpose() << "deg)\n";

        rates_ref = (att_ref - states[0].att) / 0.4;
        rates_ref(2) = radians(60) * -manual.yaw;

        commands[0].throttle = manual.thrust;
        rates_ref_valid_ = true;
    }

    /** Control angular velocity with actuators **/
    if (rates_ref_valid_)
    {
        //std::cout << "-> RATES (" << degrees(rates_ref).transpose() << "deg/s) ";
        const Vector3 dRPY = {0, 0, 0}; //PositionAttitudeControl::get_angular_control(rates_ref, state[0].rates);
        commands[0].aileron = dRPY(0) / radians(15.0); // Normalized [-1 ... 1]
        commands[0].elevator = dRPY(1) / radians(15.0); // Normalized [-1 ... 1]
        commands[0].rudder = dRPY(2);

        command_valid = true;
    }

//    std::cout << "\n";

//    std::cout << "acc_spc: " << n_acc_refcon.transpose() << ", "
//              //                  << "pointing_sp: " << pointing_sp.transpose() << ", "
//              << "rates_spc: " << rates_refcon.transpose() << ", "
//              << "dRP: " << dRPY(0) << " " << dRPY(1) << "\n";

//        std::cout << "Gam_sp: " << Gamma_sp.transpose().array() << ", "
//                  << "Gam: " << Gamma0.transpose() << ", "
//                  << "dR: " << degrees(dR) << "\n";

}

Eigen::VectorXd FixedwingControl::control_centrifugal_sanity(const Eigen::VectorXd &normlift_tilt_cmds)
{
    Vector3 n_pos_avg(0, 0, 0);
    for (unsigned k = 0; k < p_control_kites; k++) { n_pos_avg += states[k].pos / p_control_kites; }

    Eigen::VectorXd lf_centri(p_control_kites);
//    std::vector<Vector3> psns_ICR(p_control_kites);
//    Eigen::VectorXd ICR_offs(p_control_kites);
    Eigen::VectorXd speeds(p_control_kites);
    Eigen::VectorXd speeds2(p_control_kites);

    for (unsigned k = 0; k < p_control_kites; k++)
    {
        // Centrifugal load factor sanity
        lf_centri(k) = states[k].rates.cross(states[k].vel_body).norm() / 9.81;
        speeds(k) = states[k].vel_body.norm();
        speeds2(k) = states[k].vel_body.squaredNorm();

        // ICR sanity
//        const Vector3 n_vel = states[k].q_nb * states[k].vel_body;
//        const Vector3 n_omg = states[k].q_nb * states[k].rates;
//
//        Vector3 r = n_omg.cross(n_vel) / n_omg.squaredNorm();
//        if (n_omg.norm() < 1e-8) r.setConstant(Eigen::Infinity);
//
//        const Vector3 posICR = states[k].pos + r;
//        psns_ICR[k] = posICR;
//        ICR_offs(k) = (n_pos_avg - posICR).norm();
    }

//    std::vector<double> lf_centri_ordered(p_control_kites);
//    for (unsigned k = 0; k < p_control_kites; k++) { lf_centri_ordered[k] = lf_centri(k); }
//    std::sort(lf_centri_ordered.begin(), lf_centri_ordered.end(), std::greater<double>());
    const double LFc_cond = std::fmin(lf_centri.maxCoeff() / lf_centri.minCoeff(), 10);

    m_diag.lf_centri(0) = lf_centri(0);
    m_diag.lf_centri(1) = lf_centri(1);
    m_diag.lf_centri(2) = lf_centri.maxCoeff() - lf_centri.minCoeff();

    m_diag.lf_centri_cond(0) = LFc_cond;
    m_diag.lf_centri_cond(1) = lf_centri.maxCoeff() - lf_centri.minCoeff();

//    m_diag.ICR_offs = ICR_offs;
//    std::cout << "LF_centri: (" << lf_centri.transpose() << ") g, LF cond: " << LFc_cond << ", ICR offs: (" << ICR_offs.transpose() <<  ") m\n";

    // Activate directional control between minimum speed of 12m/s and operational speed of 15m/s, of slowest kite
    const double avg_speed = speeds.mean();

    m_diag.speed(0) = speeds2(0);
    m_diag.speed(1) = speeds2(1);
    m_diag.speed(2) = speeds2.maxCoeff() - speeds2.minCoeff();

    const double activation = awe_control3::AweHeliController::activation(avg_speed, 15, 20);
//    std::cout << "avg_speed: " << avg_speed << ", act: " << activation << "\n";
    return activation * normlift_tilt_cmds;
}

Eigen::VectorXd FixedwingControl::control_multi_lift(const Eigen::Vector2d &d_lift_dir)
{
    // Transform lift direction to account for nonlinearities (rotate about disk z axis)
    const Eigen::Vector3d d_lift_rotated = T3quat(-m_awec.p_liftdir_angular_trim) *
                                           Eigen::Vector3d(d_lift_dir(0), d_lift_dir(1), 0);
//    std::cout << "d_lift_dir: " << d_lift_dir.transpose() << ", rotated: " << d_lift_rotated.transpose() << "\n";

    // Modulate liftdir command in disk frame
    std::vector<Eigen::Vector3d> n_posns;
    for (unsigned k = 0; k < p_control_kites; k++) { n_posns.push_back(states[k].pos); }

    const Eigen::VectorXd normalized_lifts = m_awec.get_normlifts(n_posns, d_lift_rotated.segment<2>(0));
    return normalized_lifts;
}
