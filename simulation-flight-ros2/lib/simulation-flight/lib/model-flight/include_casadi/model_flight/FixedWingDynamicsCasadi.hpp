#ifndef SRC_FIXEDWINGDYNAMICSCASADI_HPP
#define SRC_FIXEDWINGDYNAMICSCASADI_HPP

#include "model_common/common_utils.hpp"
#include "model_common/DynamicsBaseCasadi.hpp"
#include "model_common/QuaternionMathCasadi.hpp"
#include "model_flight/StateRepresentation.hpp"

namespace flight_model {
namespace fixed_wing_casadi {

using namespace model_common;
using namespace casadi;

constexpr bool FIX_V0 = false; // In all models: Normalize rates with V0 instead of Va
constexpr double FIXED_V0 = 12;

casadi::SX getThrust(const casadi::SX &throttle, const casadi::SX &Va)
{
    /* Thrust manifold coefficients */
    const double a = 17.6582;
    const double b = -0.6554;
    const double ts = -0.0266;

    /* Max. RPM(Va) */
    const double RPM_max0 = 6300.0;
    const double RPM_max_per_Va = 900.0 / 22.0;
    SX RPM_max = RPM_max0 + RPM_max_per_Va * Va;
    SX RPM = throttle * RPM_max;
    SX RPM_scaled = RPM / 8000.0;

    const double thrust_offset = 0.0124;
    SX thrust_solius_12x6 = a * (RPM_scaled - ts) * (RPM_scaled - ts) + b * RPM_scaled * Va - thrust_offset;
    SX thrust = thrust_solius_12x6 * 6.3739 / 10.82;

    return thrust;
}

/** Longitudinal 4-state fixed-wing model -------------------------------------------------------------------------- **/
class LonKite : public model_common::casadi_model::DynamicsBase
{
public:
    explicit LonKite(const std::string &filename,
                     const StateRepresentation &stateRepresentation = LongitudinalEulerAoa,
                     const std::map<std::string, double> &static_params = {},
                     const std::vector<std::string> &dyn_params = {},
                     const std::vector<std::string> &opt_params = {},
                     bool teth_on = false) :
            DynamicsBase(filename, static_params, dyn_params, opt_params),
            state_representation(stateRepresentation)
    {
        /** ============================================================================================================ **/
        /** Model parameters **/
        SX model_params;
        std::map<std::string, double> default_params;

        SX dummy_no_params = SX::sym("dummy_no_params");
        model_params = SX::vertcat({model_params, dummy_no_params});
        default_params.insert({"dummy_no_params", 0});

        /** --------------------- **/
        /** Environment & wind    **/
        /** --------------------- **/
        const double g = 9.806;   // gravitational acceleration [m/s2] [WGS84]
        SX rho = SX::sym("air_density");
        SX vW_hor = SX::sym("v_wind_hor");
        SX vW_ver = SX::sym("v_wind_ver");

        default_params.insert({"air_density", 1.225});
        default_params.insert({"v_wind_hor", 0.5});
        default_params.insert({"v_wind_ver", 0.07});
        model_params = SX::vertcat({model_params, rho, vW_hor, vW_ver});

        /** --------------------- **/
        /** Geometry              **/
        /** --------------------- **/
        SX b = SX::sym("b");
        SX c = SX::sym("c");
        SX AR = SX::sym("AR");
        SX S = SX::sym("S");
        model_params = SX::vertcat({model_params, b, c, AR, S});

        SX IMU_ANG = SX::sym("imu_ang");
        default_params.insert({"imu_ang", 0.0});
        model_params = SX::vertcat({model_params, IMU_ANG});

        /** --------------------------- **/
        /** Mass and inertia            **/
        /** --------------------------- **/
        SX mass = SX::sym("mass");
        SX Iyy = SX::sym("Iyy");
        model_params = SX::vertcat({model_params, mass, Iyy});

        /** ------------------------------- **/
        /** Aerodynamic effects             **/
        /** ------------------------------- **/
        /* Drag */
        SX e_oswald = SX::sym("e_oswald");
        SX CD0 = SX::sym("CD0");

        /* From angle of attack */
        SX CL0 = SX::sym("CL0");
        SX CLa = SX::sym("CLa");

        SX Cm0 = SX::sym("Cm0");
        SX Cma = SX::sym("Cma");

        /* From pitch rate */
        SX CLq = SX::sym("CLq");
        SX Cmq = SX::sym("Cmq");

        model_params = SX::vertcat({model_params, e_oswald, CD0, CL0, CLa, Cm0, Cma,
                                    CLq, Cmq});

        /** ------------------------------ **/
        /** Aerodynamic effects of control **/
        /** ------------------------------ **/
        /* From elevator deflection dE */
        SX CLde = SX::sym("CLde");
        SX Cmde = SX::sym("Cmde");

        model_params = SX::vertcat({model_params, CLde, Cmde});

        /** ------------------------------ **/
        /**      Propulsion parameters     **/
        /** ------------------------------ **/
        SX b_thrust_ang = SX::sym("thrust_angle");
        //SX CLa_thrust = SX::sym("CLa_thr");

        default_params.insert({"thrust_angle", 0.0 * M_PI / 180.0});
        default_params.insert({"CLa_thr", 0.0});
        model_params = SX::vertcat({model_params, b_thrust_ang});//, CLa_thrust});

        /** ============================================================================================================ **/
        /** Start of model **/
        SX debugSX;

        /** State variables **/
        SX vx = SX::sym("vx");
        SX vz = SX::sym("vz");
        SX wy = SX::sym("wy"); // pitch rate
        SX pitch = SX::sym("pitch");

        SX alpha = SX::sym("alpha");
        SX Va = SX::sym("Va");

        SX gamma = SX::sym("gamma");

        /** Control variables **/
        SX dF = SX::sym("dF");     // Throttle [-]
        SX dE = SX::sym("dE");     // Elevator deflection [positive causing negative pitch movement (nose down)] [rad]

        if (state_representation == LongitudinalUW)
        {
            Va = norm_2(SX::vertcat({vx, vz}));
            alpha = atan(vz / vx);
            gamma = pitch - alpha;
        }

        /** ---------------------------------------------------------- **/
        /** Thrust Forces and Moments in body frame **/
        /** ---------------------------------------------------------- **/
//    SX throttle = F_thr0 / 6.3739;
        SX thrust = getThrust(dF, Va);
        SX b_F_thrust = thrust * SX::vertcat({cos(b_thrust_ang), 0, sin(b_thrust_ang)});

        SX b_r_thrust = SX::vertcat({0.25, 0, 0});
        SX b_M_thrust = SX::cross(b_r_thrust, b_F_thrust);
        SX M_thrust = b_M_thrust(1);

        /** ---------------------------------------------------------- **/
        /** Aerodynamic Forces and Moments in aerodynamic (wind) frame **/
        /** ---------------------------------------------------------- **/
        SX V0 = (FIX_V0) ? FIXED_V0 : Va;

        SX dyn_press = 0.5 * rho * Va * Va;

        /** Forces in x, y, z directions: -Drag, Side force, -Lift **/
        SX CL = CL0 + CLa * alpha + CLq * c / (2.0 * V0) * wy + CLde * dE;
        SX CD = CD0 + CL * CL / (M_PI * e_oswald * AR);

        SX LIFT = dyn_press * S * CL;
        SX DRAG = dyn_press * S * CD;

        /** Moments about x, y, z axes: L, M, N **/
        SX M = dyn_press * S * c * (Cm0 + Cma * alpha + c / (2.0 * V0) * Cmq * wy + Cmde * dE);

        /** Dynamics **/
        SX Va_dot, alpha_dot, wy_dot, pitch_dot, gamma_dot, vx_dot, vz_dot;

        switch (stateRepresentation)
        {
            case LongitudinalFlightPath:
            {
                Va_dot = -DRAG / mass + cos(alpha + b_thrust_ang) * thrust / mass - g * sin(gamma);
                gamma_dot = (LIFT / mass + sin(alpha + b_thrust_ang) * thrust / mass - g * cos(gamma)) / Va;
                alpha_dot = wy - gamma_dot;
                wy_dot = (M + M_thrust) / Iyy;
                break;
            }
            case LongitudinalEulerAoa:
            {
                Va_dot = -DRAG / mass + cos(alpha + b_thrust_ang) * thrust / mass - g * sin(pitch - alpha);
                alpha_dot =
                        (-LIFT / mass - sin(alpha + b_thrust_ang) * thrust / mass + g * cos(pitch - alpha) + wy * Va) /
                        Va;
                wy_dot = (M + M_thrust) / Iyy;
                pitch_dot = wy;
                break;
            }
            case LongitudinalUW:
            default:
            {
                SX X = -cos(alpha) * DRAG + sin(alpha) * LIFT;
                SX Z = -cos(alpha) * LIFT - sin(alpha) * DRAG;
                vx_dot = (X + b_F_thrust(0)) / mass - g * sin(pitch) - wy * vz;
                vz_dot = (Z + b_F_thrust(2)) / mass + g * cos(pitch) + wy * vx;
                wy_dot = (M + M_thrust) / Iyy;
                pitch_dot = wy;
                break;
            }
        }

        /** End of model **/
        /** ============================================================================================================ **/

        /** Categorize parameters **/
        sort_model_parameters(model_params, opt_params, dyn_params, default_params, static_params, filename);

        /** State, Dynamics, Control **/
        SX state, dynamics, control, sim_output;
        switch (stateRepresentation)
        {
            case LongitudinalFlightPath:
            {
                state = SX::vertcat({Va, alpha, wy, gamma});
                dynamics = SX::vertcat({Va_dot, alpha_dot, wy_dot, gamma_dot});

                x_trim_ubound = DM::vertcat({15, 20 * M_PI / 180, 5 * M_PI / 180, 20 * M_PI / 180});
                x_trim_lbound = DM::vertcat({10, -20 * M_PI / 180, 05 * M_PI / 180, -20 * M_PI / 180});

                sim_output = SX::vertcat({vx, vz, pitch});
                break;
            }
            case LongitudinalEulerAoa:
            {
                state = SX::vertcat({Va, alpha, wy, pitch});
                dynamics = SX::vertcat({Va_dot, alpha_dot, wy_dot, pitch_dot});

                x_trim_ubound = DM::vertcat({15, 20 * M_PI / 180, 5 * M_PI / 180, 20 * M_PI / 180});
                x_trim_lbound = DM::vertcat({10, -20 * M_PI / 180, 05 * M_PI / 180, -20 * M_PI / 180});

                sim_output = SX::vertcat({vx, vz, gamma});
                break;
            }
            case LongitudinalUW:
            default:
            {
                state = SX::vertcat({vx, vz, wy, pitch});
                dynamics = SX::vertcat({vx_dot, vz_dot, wy_dot, pitch_dot});

                x_trim_ubound = DM::vertcat({15, 20 * M_PI / 180, 5 * M_PI / 180, 20 * M_PI / 180});
                x_trim_lbound = DM::vertcat({10, -20 * M_PI / 180, 05 * M_PI / 180, -20 * M_PI / 180});

                sim_output = SX::vertcat({Va, alpha, gamma});
                break;
            }
        }
        control = SX::vertcat({dE, dF});

        u_trim_ubound = DM::vertcat({30 * M_PI / 180, 0}); // Elevator deflection, static thrust
        u_trim_lbound = DM::vertcat({-30 * M_PI / 180, 0});

        u_ubound = DM::vertcat({21.0 * M_PI / 180.0, 1});
        u_lbound = DM::vertcat({-21.0 * M_PI / 180.0, 0});

        x_default = DM::vertcat({12, -0.05, -0.05, -0.05}); // {Va/vx} {aoa/vz} {wy}, {pitch/gamma}

        this->State = state;
        this->Control = control;
        this->StateDot = dynamics;

        /** Output mappings *
        state_c(6) = msg->transforms[0].translation.x;
        state_c(7) = msg->transforms[0].translation.y;
        state_c(8) = msg->transforms[0].translation.z;*/
        SX jacobian_f_x = SX::jacobian(dynamics, state);
        SX jacobian_f_u = SX::jacobian(dynamics, control);

        //debugSX = dynamics;
        sim_output = SX::vertcat({sim_output, debugSX});

        output_mappings.insert({
                                {"sim_output",   sim_output},
                                {"dyn_out",      sim_output},

                                {"thrust",       thrust},

                                {"jacobian_f_x", jacobian_f_x},
                                {"jacobian_f_u", jacobian_f_u},

                                {"debugSX",      debugSX}});
    }
    ~LonKite() = default;

    StateRepresentation getStateRepresentation() { return state_representation; }

protected:
    StateRepresentation state_representation{LongitudinalEulerAoa};
};

/** Longitudinal 6-state fixed-wing model (4 + 2D airplane position) ----------------------------------------------- **/
class LonKiteWithPos : public LonKite
{
public:
    explicit LonKiteWithPos(const std::string &filename,
                            const StateRepresentation &stateRepresentation = LongitudinalEulerAoa,
                            const std::map<std::string, double> &static_params = {},
                            const std::vector<std::string> &dyn_params = {},
                            const std::vector<std::string> &opt_params = {},
                            bool teth_on = false) :
            LonKite(filename, stateRepresentation, static_params, dyn_params, opt_params, teth_on)
    {
        /** ============================================================================================================ **/
        /** Model parameters **/
        SX model_params;
        std::map<std::string, double> default_params;

        /** ============================================================================================================ **/
        /** Start of model **/
        SX debugSX;

        SX posHor = SX::sym("posHor");
        SX posDown = SX::sym("posDown");

        /** Dynamics **/
        SX vHor, vDown;

        switch (stateRepresentation)
        {
            case LongitudinalFlightPath:
            {
                SX Va = this->State(0);
                SX gamma = this->State(3);
                vHor = Va * cos(gamma);
                vDown = -Va * sin(gamma);
                break;
            }
            case LongitudinalEulerAoa:
            {
                SX Va = this->State(0);
                SX gamma = this->State(3) - this->State(1); // pitch - alpha
                vHor = Va * cos(gamma);
                vDown = -Va * sin(gamma);
                break;
            }
            case LongitudinalUW:
            default:
            {
                SX vx = this->State(0);
                SX vz = this->State(1);
                SX pitch = this->State(3);
                vHor = vx * cos(pitch) + vz * sin(pitch);
                vDown = -vx * sin(pitch) + vz * cos(pitch);
                break;
            }
        }
        /** End of model **/
        /** ============================================================================================================ **/

        /** Categorize parameters **/
        sort_model_parameters(model_params, opt_params, dyn_params, default_params, static_params, filename);

        /** State, Dynamics, Control **/
        SX state, dynamics;

        state = SX::vertcat({this->State, posHor, posDown});
        dynamics = SX::vertcat({this->StateDot, vHor, vDown});

        x_trim_ubound = DM::vertcat({this->x_trim_ubound, DM::inf(), DM::inf()});
        x_trim_lbound = DM::vertcat({this->x_trim_lbound, -DM::inf(), -DM::inf()});

        x_default = DM::vertcat({this->x_default, 0, -100}); // posHor, posDown

        this->State = state;
        this->StateDot = dynamics;

//    SX sim_out = SX::vertcat({output_mappings["sim_output"], dynamics});
//    output_mappings["sim_output"] = sim_out;

        /** Output mappings **/
        //    SX dummy1, dummy2;
        //    output_mappings.insert({{"dummy1", dummy1},
        //                            {"dummy2", dummy2}});
    }
    ~LonKiteWithPos() = default;
};

/** Full 13-state kite model --------------------------------------------------------------------------------------- **/
class Kite : public model_common::casadi_model::DynamicsBase
{
public:
    explicit Kite(const std::string &filename, const StateRepresentation &stateRepresentation = AttQuat,
                  const std::map<std::string, double> &static_params = {},
                  const std::vector<std::string> &dyn_params = {},
                  const std::vector<std::string> &opt_params = {},
                  bool teth_on = false,
                  bool wind_as_control = false,
                  bool turbulence_estimation = false,
                  bool couple_rud_to_ail = false) :
            DynamicsBase(filename, static_params, dyn_params, opt_params),
            state_representation(stateRepresentation)
    {
        /** ============================================================================================================ **/
        /** Model parameters **/
        SX model_params;
        std::map<std::string, double> default_params;

        /** --------------------- **/
        /** Environment & wind    **/
        /** --------------------- **/
        const double g = 9.806;   // gravitational acceleration [m/s2] [WGS84]
        SX rho = SX::sym("air_density");
        SX vW_N = SX::sym("v_wind_N");
        SX vW_E = SX::sym("v_wind_E");
        SX vW_D = SX::sym("v_wind_D");
        SX dvW_N = SX::sym("dv_wind_N");
        SX dvW_E = SX::sym("dv_wind_E");
        SX dvW_D = SX::sym("dv_wind_D");

        default_params.insert({"air_density", 1.225});
        default_params.insert({"v_wind_N", 1});
        default_params.insert({"v_wind_E", 0.5});
        default_params.insert({"v_wind_D", 0});
        default_params.insert({"dv_wind_N", 0});
        default_params.insert({"dv_wind_E", 0});
        default_params.insert({"dv_wind_D", 0});
        model_params = SX::vertcat({model_params, rho, vW_N, vW_E, vW_D}); /* dvW_N, dvW_E are added last (see below). ...
    They will be added dynamically for each sequence, so they shall appear at the end of the param vector. */

        /** --------------------- **/
        /** Geometry              **/
        /** --------------------- **/
        SX b = SX::sym("b");
        SX c = SX::sym("c");
        SX AR = SX::sym("AR");
        SX S = SX::sym("S");
        model_params = SX::vertcat({model_params, b, c, AR, S});

        SX IMU_ANG = SX::sym("imu_ang");
        default_params.insert({"imu_ang", 0.0});
        model_params = SX::vertcat({model_params, IMU_ANG});

        /** --------------------------- **/
        /** Mass and inertia            **/
        /** --------------------------- **/
        SX mass = SX::sym("mass");
        SX Ixx = SX::sym("Ixx");
        SX Iyy = SX::sym("Iyy");
        SX Izz = SX::sym("Izz");
        SX Ixz = SX::sym("Ixz");
        model_params = SX::vertcat({model_params, mass, Ixx, Iyy, Izz, Ixz});

        /** ------------------------------- **/
        /** Aerodynamic effects             **/
        /** ------------------------------- **/
        /* Drag */
        SX CD0 = SX::sym("CD0");
        SX e_oswald = SX::sym("e_oswald");

//    SX CD0a = SX::sym("CD0a");
//    SX k_CL = SX::sym("k_CL");
//    SX CL_CD0 = SX::sym("CL_CD0");

        /* From angle of attack */
        SX CL0 = SX::sym("CL0");
        SX CLa = SX::sym("CLa");

        SX Cm0 = SX::sym("Cm0");
        SX Cma = SX::sym("Cma");

        /* From sideslip */
        SX CYb = SX::sym("CYb");

        SX Cl0 = SX::sym("Cl0");
        SX Clb = SX::sym("Clb");

        SX Cn0 = SX::sym("Cn0");
        SX Cnb = SX::sym("Cnb");

        /* From pitch rate */
        SX CLq = SX::sym("CLq");
        SX Cmq = SX::sym("Cmq");

        /* From roll rate */
        SX CYp = SX::sym("CYp");
        SX Clp = SX::sym("Clp");
        SX Cnp = SX::sym("Cnp");

        /* From yaw rate */
        SX CYr = SX::sym("CYr");
        SX Clr = SX::sym("Clr");
        SX Cnr = SX::sym("Cnr");

        model_params = SX::vertcat({model_params, CD0, e_oswald, //CD0a, k_CL, CL_CD0,
                                    CL0, CLa, Cm0, Cma, CYb, Cl0, Clb, Cn0, Cnb,
                                    CLq, Cmq, CYp, Clp, Cnp, CYr, Clr, Cnr});

        /** ------------------------------ **/
        /** Aerodynamic effects of control **/
        /** ------------------------------ **/
        /* From elevator deflection dE */
//    SX CDde = SX::sym("CDde");
        SX CLde = SX::sym("CLde");
        SX Cmde = SX::sym("Cmde");

        /* From aileron deflection dA */
        SX Clda = SX::sym("Clda");
        SX Cnda = SX::sym("Cnda");

        /* From aileron deflection dR */
        SX CYdr = SX::sym("CYdr");
        SX Cldr = SX::sym("Cldr");
        SX Cndr = SX::sym("Cndr");

        model_params = SX::vertcat({model_params, //CDde,
                                    CLde, Cmde, Clda, Cnda, CYdr, Cldr, Cndr});

        /** ------------------------------ **/
        /**      Propulsion parameters     **/
        /** ------------------------------ **/
        SX b_thrust_ang = SX::sym("thrust_angle");
        //SX CLa_thrust = SX::sym("CLa_thr");

        default_params.insert({"thrust_angle", 0.0 * M_PI / 180.0});
        default_params.insert({"CLa_thr", 0.0});
        model_params = SX::vertcat({model_params, b_thrust_ang});//, CLa_thrust});

        /** ------------------------------ **/
        /**        Tether parameters       **/
        /** ------------------------------ **/
        SX teth_length = SX::sym("teth_length");
        SX teth_Ks = SX::sym("teth_Ks");
        SX teth_Kd = SX::sym("teth_Kd");

        SX teth_rx = SX::sym("teth_rx");
        SX teth_ry = SX::sym("teth_ry");
        SX teth_rz = SX::sym("teth_rz");

        if (teth_on)
            model_params = SX::vertcat({model_params, teth_length, teth_Ks, teth_Kd, teth_rx, teth_ry, teth_rz});

        /* Add per-sequence wind params last (see comment below) */
        model_params = SX::vertcat({model_params, dvW_N, dvW_E, dvW_D});

        /** ============================================================================================================ **/
        /** Start of model **/
        SX debugSX;

        /* Aircraft Inertia Matrix */
        SX J = SX::diag(SX::vertcat({Ixx, Iyy, Izz}));
        J(0, 2) = Ixz;
        J(2, 0) = Ixz;

        /** State variables **/
        SX vx = SX::sym("vx");
        SX vy = SX::sym("vy");
        SX vz = SX::sym("vz");
        SX v = SX::vertcat({vx, vy, vz}); // Linear velocity of the CoG (body frame) [m/s]

        SX wx = SX::sym("wx"); // roll rate
        SX wy = SX::sym("wy"); // pitch rate
        SX wz = SX::sym("wz"); // yaw rate
        SX w = SX::vertcat({wx, wy, wz}); // Angular velocity (body frame) [rad/s]

        SX posN = SX::sym("posN");
        SX posE = SX::sym("posE");
        SX posD = SX::sym("posD");
        SX pos = SX::vertcat({posN, posE, posD}); // Position of the CoG (geodetic (NED) frame) [m]

        SX qw = SX::sym("qw");
        SX qx = SX::sym("qx");
        SX qy = SX::sym("qy");
        SX qz = SX::sym("qz");
        SX q = SX::vertcat(
                {qw, qx, qy, qz}); // Rotation from NED to body frame. Transforms a body vector to ned. = q_nb
        SX q_bn;

        SX roll = SX::sym("roll");
        SX pitch = SX::sym("pitch");
        SX yaw = SX::sym("yaw");
        SX Phi = SX::vertcat({roll, pitch, yaw}); // Euler angles (roll, pitch, yaw)

        switch (stateRepresentation)
        {
            case AttEuler:
            {  // Euler angles representation
                using namespace quatmath_casadi;
                q_bn = quat_multiply(T1quat(roll), quat_multiply(T2quat(pitch), T3quat(yaw)));
                q = quat_inverse(q_bn); // q_nb
                break;
            }
            case AttQuat:
            default:
                q_bn = quatmath_casadi::quat_inverse(q);
        }

        /** Control variables **/
        SX dF = SX::sym("dF");     // Throttle [-]
        SX dE = SX::sym("dE");     // Elevator deflection [positive causing negative pitch movement (nose down)] [rad]
        SX dR = SX::sym("dR");     // Rudder deflection [positive causing negative yaw movement (nose left)] [rad]
        SX dA = SX::sym("dA");     // Aileron deflection [positive causing negative roll movement (right wing up)] [rad]
        if (couple_rud_to_ail) dR = 0 * -dA;

        SX vW_N_ctrl = SX::sym("vW_N_ctrl");
        SX vW_E_ctrl = SX::sym("vW_E_ctrl");

        /** Aerodynamic (Wind) frame **/
        /* Wind and apparent velocity in body frame */
        SX n_vW = SX::vertcat({vW_N, vW_E, vW_D});
        if (turbulence_estimation) n_vW = SX::vertcat({vW_N + dvW_N, vW_E + dvW_E, dvW_D});
        if (wind_as_control) n_vW = SX::vertcat({vW_N_ctrl, vW_E_ctrl, 0});
        SX b_vW = quatmath_casadi::quat_transform(q_bn, n_vW);
        SX b_va = v - b_vW;

        /* Aerodynamic variables (Airspeed, angle of attack, side slip angle) */
        SX Va = SX::norm_2(b_va);
        SX alpha = atan(b_va(2) / b_va(0));
        SX beta = asin(b_va(1) / Va);

        /* Measured airspeed component (pitot tube orientation dependent) */
        SX r_sens = SX::vertcat({0.11, 0.22, -0.05});
        SX b_va_meas = b_va + SX::cross(r_sens, w);
        /* At fast body yawrates, the body rotation (thus pitot tube is faster than the CoG) is clearly negligible
         * under the effect of the pitot measurement direction being rotated out of the airflow (sideslip) */

        /* Pitot tube is oriented about 5 degrees above body x axes */
        SX q_sens_b = quatmath_casadi::T2quat(5.0 * M_PI / 180.0);
        SX sens_va = quatmath_casadi::quat_transform(q_sens_b, b_va_meas);
        SX Va_pitot = sens_va(0);

        /** ---------------------------------------------------------- **/
        /** Thrust Forces and Moments in body frame **/
        /** ---------------------------------------------------------- **/
        SX thrust = getThrust(dF, Va);
        SX b_F_thrust = thrust * SX::vertcat({cos(b_thrust_ang), 0, sin(b_thrust_ang)});

        SX b_r_thrust = SX::vertcat({0.25, 0, 0});
        SX b_M_thrust = SX::cross(b_r_thrust, b_F_thrust);

        /** ---------------------------------------------------------- **/
        /** Aerodynamic Forces and Moments in aerodynamic (wind) frame **/
        /** ---------------------------------------------------------- **/
        SX q_ba = quatmath_casadi::quat_multiply(quatmath_casadi::T2quat(alpha), quatmath_casadi::T3quat(-beta));

        SX V0 = (FIX_V0) ? FIXED_V0 : Va;

        SX dyn_press = 0.5 * rho * Va * Va;

        SX CL = CL0 + CLa * alpha + CLq * c / (2.0 * V0) * w(1) + CLde * dE;
        SX CD = CD0 + CL * CL / (M_PI * e_oswald * AR);

        /** Forces in x, y, z directions: -Drag, Side force, -Lift **/
        SX LIFT = dyn_press * S * CL;
        SX DRAG = dyn_press * S * CD;
        SX SF = dyn_press * S * (CYb * beta + b / (2.0 * V0) * (CYp * w(0) + CYr * w(2)) + CYdr * dR);

        SX a_F_aero = SX::vertcat({-DRAG, SF, -LIFT});

        /** Moments about x, y, z axes: L, M, N **/
        SX L = dyn_press * S * b *
               (Cl0 + Clb * beta + b / (2.0 * V0) * (Clp * w(0) + Clr * w(2)) + Clda * dA + Cldr * dR);

        SX M = dyn_press * S * c * (Cm0 + Cma * alpha + c / (2.0 * V0) * Cmq * w(1) + Cmde * dE);

        SX N = dyn_press * S * b *
               (Cn0 + Cnb * beta + b / (2.0 * V0) * (Cnp * w(0) + Cnr * w(2)) + Cnda * dA + Cndr * dR);

        SX b_M_aero = SX::vertcat({L, M, N});

        /** Aerodynamic Forces and Moments in body frame **/
        SX b_F_aero = quatmath_casadi::quat_transform(q_ba, a_F_aero);

        /** ---------------------------------------- **/
        /** Gravitation, Tether (body frame) **/
        /** ---------------------------------------- **/
        /** Gravitational acceleration **/
        SX b_g = quatmath_casadi::quat_transform(q_bn, SX::vertcat({0, 0, g}));

        /** Tether force and moment **/
        SX b_F_tether;
        if (teth_on)
        {
            SX dist = SX::norm_2(pos);
            const double tethLen = 120.0;
            const double tethDiameter = 0.3e-3;
            const double tethCrossArea = M_PI_4 * tethDiameter * tethDiameter;
            const double tethDensity = 1.15;
            const double tethMassPerMeter = tethCrossArea * 1.0 * tethDensity;
            const double tethE = 9.05e9;

            const double c_orth = 1.2;

            /* Weight_tether */
            //SX decl = SX::atan(SX::norm_2(r(Slice(0, 2))) / -r(2));
            //SX g_Wteth = 0.5 * (1 + cos(decl)) * tethLen * tethMassPerMeter * SX::vertcat({0, 0, g}); // probably invalid
            SX n_Wteth = tethLen * tethMassPerMeter * SX::vertcat({0, 0, g});
            SX b_Wteth = quatmath_casadi::quat_transform(q_bn, n_Wteth);

            /* Drag_tether */
            //SX q_ba = quatmath_casadi::quat_multiply(quatmath_casadi::T2quat(alpha), quatmath_casadi::T3quat(-beta));

            SX lat = SX::atan2(-pos(0), SX::norm_2(pos(Slice(1, 3))));
            SX lon = SX::atan2(-pos(1), -pos(2));
            SX q_ln = quatmath_casadi::quat_multiply(quatmath_casadi::T2quat(-lat), quatmath_casadi::T1quat(lon));
            SX q_nl = quatmath_casadi::quat_inverse(q_ln);
            SX q_lb = quatmath_casadi::quat_multiply(q_ln, q);

            SX l_va = quatmath_casadi::quat_transform(q_lb, b_va);
            SX l_va_proj = SX::vertcat({l_va(0), l_va(1), 0});

            SX q_bl = quatmath_casadi::quat_multiply(q_bn, q_nl);
            SX f_va_proj = quatmath_casadi::quat_transform(q_bl, l_va_proj);

            SX b_Dteth =
                    -0.125 * tethDensity * c_orth * tethDiameter * tethLen * SX::times(SX::abs(f_va_proj), f_va_proj);

            /* LonForce_tether */
            SX tethElongation = (dist - tethLen) / tethLen;
            SX g_lonFteth = -pos / dist * SX::fmax(0, (tethE * tethCrossArea) * (tethElongation + 0.005));
//            SX g_lonFteth = -pos / dist * 5;
            SX b_lonFteth = quatmath_casadi::quat_transform(q_bn, g_lonFteth);
            b_F_tether = b_lonFteth + b_Dteth + b_Wteth;
        }
        else
            b_F_tether = SX::vertcat({0, 0, 0});

        SX tether_outlet_position = SX::vertcat({teth_rx, teth_ry, teth_rz});
        SX b_M_tether = SX::cross(tether_outlet_position, b_F_tether);

        /** ----------------------------- **/
        /** Motion equations (body frame) **/
        /** ----------------------------- **/
        /** Linear motion equation **/
        SX spec_nongrav_force = (b_F_aero + b_F_thrust + b_F_tether) / mass;
        SX v_dot = spec_nongrav_force + b_g - SX::cross(w, v);

        /** Angular motion equation **/
        SX w_dot = SX::mtimes(SX::inv(J), (b_M_aero + b_M_thrust + b_M_tether - SX::cross(w, SX::mtimes(J, w))));

        /** ------------------------------------ **/
        /** Kinematic Equations (geodetic frame) **/
        /** ------------------------------------ **/
        /** Translation: Aircraft position derivative **/
        SX r_dot = quatmath_casadi::quat_transform(q, v);                            // q = q_nb, v (body frame)

        /** Rotation: Aircraft attitude derivative **/
        /* Quaternion representation */
        double lambda = -5;
        SX q_dot = 0.5 * quatmath_casadi::quat_multiply(q, SX::vertcat({0, w}))      // q = q_nb, w = omega (body frame)
                   + 0.5 * lambda * q * (SX::dot(q, q) - 1);                     // Quaternion norm stabilization term,
        // as in Gros: 'Baumgarte Stabilisation over the SO(3) Rotation Group for Control',
        // improved: lambda negative and SX::dot(q, q) instead of lambda positive and 1/SX::dot(q, q).

        /* Euler angle representation */
        SX phi = Phi(0);
        SX theta = Phi(1);
        SX J_eulKin = SX::vertcat({
                                          SX::horzcat({1, sin(phi) * tan(theta), cos(phi) * tan(theta)}),
                                          SX::horzcat({0, cos(phi), -sin(phi)}),
                                          SX::horzcat({0, sin(phi) / cos(theta), cos(phi) / cos(theta)})
                                  });
        SX Phi_dot = SX::mtimes(J_eulKin, w);

        /** End of model **/
        /** ============================================================================================================ **/

        /** Categorize parameters **/
        sort_model_parameters(model_params, opt_params, dyn_params, default_params, static_params, filename);

        /** State, Dynamics, Control **/
        SX state, dynamics, control;
        switch (stateRepresentation)
        {
            case AttEuler:
            {   // Euler angle representation
                state = SX::vertcat({v, w, pos, Phi});
                dynamics = SX::vertcat({v_dot, w_dot, r_dot, Phi_dot});
                control = SX::vertcat({dF, dE, dR, dA});

                x_default = DM::vertcat({12, 0, 0, 0, 0, 0,
                                         0, 0, -100, 0, 0, 0});

                DM x_trim_bounds = DM::vertcat({DM::horzcat({10, 15}), // vx
                                                DM::horzcat({-1, 1}),  // vy
                                                DM::horzcat({-5, 5}),  // wz

                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wx
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wy
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wz

                                                DM::horzcat({-DM::inf(), DM::inf()}), // x
                                                DM::horzcat({-DM::inf(), DM::inf()}), // y
                                                DM::horzcat({-DM::inf(), DM::inf()}), // z

                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}),    // roll
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}),    // pitch
                                                DM::horzcat({-179 * M_PI / 180, 179 * M_PI / 180}) // yaw
                                               });
                x_trim_lbound = x_trim_bounds(Slice(), 0);
                x_trim_ubound = x_trim_bounds(Slice(), 1);
                break;
            }
            case AttQuat:
            default:
            {
                state = SX::vertcat({v, w, pos, q});
                dynamics = SX::vertcat({v_dot, w_dot, r_dot, q_dot});
                control = SX::vertcat({dF, dE, dR, dA});

                x_default = DM::vertcat({12, 0, 0, 0, 0, 0,
                                         0, 0, -100, 1, 0, 0, 0});

                DM x_trim_bounds = DM::vertcat({DM::horzcat({10, 15}), // vx
                                                DM::horzcat({-1, 1}),  // vy
                                                DM::horzcat({-5, 5}),  // wz

                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wx
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wy
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wz

                                                DM::horzcat({-DM::inf(), DM::inf()}), // x
                                                DM::horzcat({-DM::inf(), DM::inf()}), // y
                                                DM::horzcat({-DM::inf(), DM::inf()}), // z

                                                DM::horzcat({-1, 1}), // qw
                                                DM::horzcat({-1, 1}), // qx
                                                DM::horzcat({-1, 1}), // qy
                                                DM::horzcat({-1, 1})  // qz
                                               });
                x_trim_lbound = x_trim_bounds(Slice(), 0);
                x_trim_ubound = x_trim_bounds(Slice(), 1);
                break;
            }
        }
        if (couple_rud_to_ail) control = SX::vertcat({dF, dE, dA}); // Overwrite definition
        if (wind_as_control) control = SX::vertcat({control, vW_N_ctrl, vW_E_ctrl}); // Append wind_ctrl

        DM u_physical_bounds = DM::vertcat({DM::horzcat({0, 1}), // Throttle
                                            DM::horzcat({-21.0 * M_PI / 180.0, 21.0 * M_PI / 180.0}), // Elevator
                                            DM::horzcat({-26.5 * M_PI / 180.0, 26.5 * M_PI / 180.0}), // Rudder
                                            DM::horzcat({-20.5 * M_PI / 180.0, 20.5 * M_PI / 180.0}) // Aileron
                                           });
        u_lbound = u_physical_bounds(Slice(), 0);
        u_ubound = u_physical_bounds(Slice(), 1);

        DM u_trim_bounds = DM::vertcat({DM::horzcat({-0, 0}),                             // Static thrust
                                        DM::horzcat({-30 * M_PI / 180, 30 * M_PI / 180}), // Elevator deflection
                                        DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}),   // Rudder deflection
                                        DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180})    // Aileron deflection
                                       });
        u_trim_lbound = u_trim_bounds(Slice(), 0);
        u_trim_ubound = u_trim_bounds(Slice(), 1);

        this->State = state;
        this->Control = control;
        this->StateDot = dynamics;
        this->Output = state;

        /** Output mappings **/
        SX spec_tether_force = b_F_tether / mass;
        SX euler_angles = quatmath_casadi::quat2euler(q);
        SX jacobian_f_x = SX::jacobian(dynamics, state);
        SX jacobian_f_u = SX::jacobian(dynamics, control);

        /* Kinematic frame */
        SX g_vg = quatmath_casadi::quat_transform(q, v);
        SX Vg = SX::norm_2(g_vg);
        SX gamma = atan(-g_vg(2) / (Vg + 1e-4));
        SX chi = atan2(g_vg(1), g_vg(0));

        /* Full output (all in one vector) NOTE THAT debugSX only participates in vector if defined
         * anywhere else than initial declaration */
        SX dyn_out = SX::vertcat({Va_pitot, Va, alpha, beta, spec_nongrav_force, spec_tether_force, debugSX});

        output_mappings.insert({{"dyn_out",            dyn_out}, // Algebraic values before state update

                                {"thrust",             thrust},

                                {"Va",                 Va},
                                {"alpha",              alpha},
                                {"beta",               beta},
                                {"Va_pitot",           Va_pitot},

                                {"Vg",                 Vg},
                                {"gamma",              gamma},
                                {"chi",                chi},

                                {"euler_angles",       euler_angles},

                                {"spec_nongrav_force", spec_nongrav_force},
                                {"spec_tether_force",  spec_tether_force},

                                {"jacobian_f_x",       jacobian_f_x},
                                {"jacobian_f_u",       jacobian_f_u},

                                {"debugSX",            debugSX}});
    }
    ~Kite() = default;

    StateRepresentation getStateRepresentation() { return state_representation; }
protected:
    StateRepresentation state_representation{AttQuat};
};

/** Full 17-state kite model (13 + 4 actuator states) -------------------------------------------------------------- **/
class AugKite : public Kite
{
public:
    explicit AugKite(const std::string &filename, const StateRepresentation &stateRepresentation = AttQuat,
                     const std::map<std::string, double> &static_params = {},
                     const std::vector<std::string> &dyn_params = {},
                     const std::vector<std::string> &opt_params = {},
                     bool teth_on = false,
                     bool wind_as_control = false,
                     bool turbulence_estimation = false) :
            Kite(filename, stateRepresentation, static_params, dyn_params, opt_params,
                 teth_on, wind_as_control, turbulence_estimation)
    {
        /** ============================================================================================================ **/
        /** Model parameters **/
        SX model_params;
        std::map<std::string, double> default_params;

        /** ------------------------------ **/
        /** Actuator dynamics              **/
        /** ------------------------------ **/
        SX TC_thr = SX::sym("TC_thr");
        SX TC_dE = SX::sym("TC_dE");
        SX TC_dR = SX::sym("TC_dR");
        SX TC_dA = SX::sym("TC_dA");
        model_params = SX::vertcat({model_params, TC_thr, TC_dE, TC_dR, TC_dA});

        /** ============================================================================================================ **/
        /** Start of model **/

        /** Actuator positions (as state) **/
        SX dF = this->Control(0);
        SX dE = this->Control(1);
        SX dR = this->Control(2);
        SX dA = this->Control(3);

        /** Control commands **/
        SX dF_cmd = SX::sym("dF_cmd");     // Command for throttle
        SX dE_cmd = SX::sym("dE_cmd");     // Command for elevator deflection
        SX dR_cmd = SX::sym("dR_cmd");     // Command for rudder deflection
        SX dA_cmd = SX::sym("dA_cmd");     // Command for aileron deflection

        /** ----------------- **/
        /** Actuator dynamics **/
        /** ----------------- **/
        // const double TC_thr = 1.0 / 1.0;      // Full thrust builds up in 1 second
        // const double TC_dE_ = 0.11 / (60.0 * M_PI / 180.0);
        // const double TC_dR_ = 0.11 / (60.0 * M_PI / 180.0);
        // const double TC_dA_ = 0.12 / (40.0 * M_PI / 180.0);

        SX dF_dot = (dF_cmd - dF) / TC_thr;
        SX dE_dot = (dE_cmd - dE) / TC_dE;
        SX dR_dot = (dR_cmd - dR) / TC_dR;
        SX dA_dot = (dA_cmd - dA) / TC_dA;

        /** End of model **/
        /** ============================================================================================================ **/

        /** Categorize parameters **/
        sort_model_parameters(model_params, opt_params, dyn_params, default_params, static_params, filename);

        /** State, Dynamics, Control **/
        SX aug_state = SX::vertcat({this->State, dF, dE, dR, dA});
        SX aug_dynamics = SX::vertcat({this->StateDot, dF_dot, dE_dot, dR_dot, dA_dot});
        SX control_cmd = SX::vertcat({dF_cmd, dE_cmd, dR_cmd, dA_cmd});

        x_default = DM::vertcat({x_default, 0, 0, 0, 0});

        this->State = aug_state;
        this->Control(Slice(0, 4)) = control_cmd;
        this->StateDot = aug_dynamics;

        /** Output mappings **/
        //    SX dummy1, dummy2;
        //    output_mappings.insert({{"dummy1", dummy1},
        //                            {"dummy2", dummy2}});
    }
    ~AugKite() = default;
};

/** Full 13-state fixed-wing --------------------------------------------------------------------------------------- **/
/* Recent version without tether */
class FixedWing : public model_common::casadi_model::DynamicsBase
{
public:
    explicit FixedWing(const std::string &filename, const StateRepresentation &stateRepresentation = AttQuat,
                       const std::map<std::string, double> &static_params = {},
                       const std::vector<std::string> &dyn_params = {},
                       const std::vector<std::string> &opt_params = {}, bool dummy_input = false) :
            DynamicsBase(), state_representation(stateRepresentation)
    {
        /** ============================================================================================================ **/
        /** Model parameters **/
        SX model_params;
        std::map<std::string, double> default_params;

        /** --------------------- **/
        /** Environment & wind    **/
        /** --------------------- **/
        const double g = 9.806;   // gravitational acceleration [m/s2] [WGS84]
        SX rho = SX::sym("air_density");
        SX vW_N = SX::sym("v_wind_N");
        SX vW_E = SX::sym("v_wind_E");
        SX vW_D = SX::sym("v_wind_D");
        SX dvW_N = SX::sym("dv_wind_N");
        SX dvW_E = SX::sym("dv_wind_E");
        SX dvW_D = SX::sym("dv_wind_D");

        default_params.insert({"air_density", 1.225});
        default_params.insert({"v_wind_N", 1});
        default_params.insert({"v_wind_E", 0.5});
        default_params.insert({"v_wind_D", 0});
        default_params.insert({"dv_wind_N", 0});
        default_params.insert({"dv_wind_E", 0});
        default_params.insert({"dv_wind_D", 0});
        model_params = SX::vertcat({model_params, rho, vW_N, vW_E, vW_D}); /* dvW_N, dvW_E are added last (see below). ...
    They will be added dynamically for each sequence, so they shall appear at the end of the param vector. */

        /** --------------------- **/
        /** Geometry              **/
        /** --------------------- **/
        SX b = SX::sym("b");
        SX c = SX::sym("c");
        SX AR = SX::sym("AR");
        SX S = SX::sym("S");
        model_params = SX::vertcat({model_params, b, c, AR, S});

        SX IMU_ANG = SX::sym("imu_ang");
        default_params.insert({"imu_ang", 0.0});
        model_params = SX::vertcat({model_params, IMU_ANG});

        /** --------------------------- **/
        /** Mass and inertia            **/
        /** --------------------------- **/
        SX mass = SX::sym("mass");
        SX Ixx = SX::sym("Ixx");
        SX Iyy = SX::sym("Iyy");
        SX Izz = SX::sym("Izz");
        SX Ixz = SX::sym("Ixz");
        model_params = SX::vertcat({model_params, mass, Ixx, Iyy, Izz, Ixz});

        /** ------------------------------- **/
        /** Aerodynamic effects             **/
        /** ------------------------------- **/
        /* Drag */
        SX CD0 = SX::sym("CD0");
        SX e_oswald = SX::sym("e_oswald");

//    SX CD0a = SX::sym("CD0a");
//    SX k_CL = SX::sym("k_CL");
//    SX CL_CD0 = SX::sym("CL_CD0");

        /* From angle of attack */
        SX CL0 = SX::sym("CL0");
        SX CLa = SX::sym("CLa");

        SX Cm0 = SX::sym("Cm0");
        SX Cma = SX::sym("Cma");

        /* From sideslip */
        SX CYb = SX::sym("CYb");

        SX Cl0 = SX::sym("Cl0");
        SX Clb = SX::sym("Clb");

        SX Cn0 = SX::sym("Cn0");
        SX Cnb = SX::sym("Cnb");

        /* From pitch rate */
        SX CLq = SX::sym("CLq");
        SX Cmq = SX::sym("Cmq");

        /* From roll rate */
        SX CYp = SX::sym("CYp");
        SX Clp = SX::sym("Clp");
        SX Cnp = SX::sym("Cnp");

        /* From yaw rate */
        SX CYr = SX::sym("CYr");
        SX Clr = SX::sym("Clr");
        SX Cnr = SX::sym("Cnr");

        model_params = SX::vertcat({model_params, CD0, e_oswald, //CD0a, k_CL, CL_CD0,
                                    CL0, CLa, Cm0, Cma, CYb, Cl0, Clb, Cn0, Cnb,
                                    CLq, Cmq, CYp, Clp, Cnp, CYr, Clr, Cnr});

        /** ------------------------------ **/
        /** Aerodynamic effects of control **/
        /** ------------------------------ **/
        /* From elevator deflection dE */
//    SX CDde = SX::sym("CDde");
        SX CLde = SX::sym("CLde");
        SX Cmde = SX::sym("Cmde");

        /* From aileron deflection dA */
        SX Clda = SX::sym("Clda");
        SX Cnda = SX::sym("Cnda");

        /* From aileron deflection dR */
        SX CYdr = SX::sym("CYdr");
        SX Cldr = SX::sym("Cldr");
        SX Cndr = SX::sym("Cndr");

        model_params = SX::vertcat({model_params, //CDde,
                                    CLde, Cmde, Clda, Cnda, CYdr, Cldr, Cndr});

        /** ------------------------------ **/
        /**      Propulsion parameters     **/
        /** ------------------------------ **/
        SX b_thrust_ang = SX::sym("thrust_angle");
        //SX CLa_thrust = SX::sym("CLa_thr");

        default_params.insert({"thrust_angle", 0.0 * M_PI / 180.0});
        default_params.insert({"CLa_thr", 0.0});
        model_params = SX::vertcat({model_params, b_thrust_ang});//, CLa_thrust});

        /* Add per-sequence wind params last (see comment below) */
        model_params = SX::vertcat({model_params, dvW_N, dvW_E, dvW_D});

        /** ============================================================================================================ **/
        /** Start of model **/
        SX debugSX;

        /* Aircraft Inertia Matrix */
        SX J = SX::diag(SX::vertcat({Ixx, Iyy, Izz}));
        J(0, 2) = Ixz;
        J(2, 0) = Ixz;

        /** State variables **/
        SX vx = SX::sym("vx");
        SX vy = SX::sym("vy");
        SX vz = SX::sym("vz");
        SX v = SX::vertcat({vx, vy, vz}); // Linear velocity of the CoG (body frame) [m/s]

        SX wx = SX::sym("wx"); // roll rate
        SX wy = SX::sym("wy"); // pitch rate
        SX wz = SX::sym("wz"); // yaw rate
        SX w = SX::vertcat({wx, wy, wz}); // Angular velocity (body frame) [rad/s]

        SX posN = SX::sym("posN");
        SX posE = SX::sym("posE");
        SX posD = SX::sym("posD");
        SX pos = SX::vertcat({posN, posE, posD}); // Position of the CoG (geodetic (NED) frame) [m]

        SX qw = SX::sym("qw");
        SX qx = SX::sym("qx");
        SX qy = SX::sym("qy");
        SX qz = SX::sym("qz");
        SX q = SX::vertcat(
                {qw, qx, qy, qz}); // Rotation from NED to body frame. Transforms a body vector to ned. = q_nb
        SX q_bn;

        SX roll = SX::sym("roll");
        SX pitch = SX::sym("pitch");
        SX yaw = SX::sym("yaw");
        SX Phi = SX::vertcat({roll, pitch, yaw}); // Euler angles (roll, pitch, yaw)

        switch (stateRepresentation)
        {
            case AttEuler:
            {  // Euler angles representation
                using namespace quatmath_casadi;
                q_bn = quat_multiply(T1quat(roll), quat_multiply(T2quat(pitch), T3quat(yaw)));
                q = quat_inverse(q_bn); // q_nb
                break;
            }
            case AttQuat:
            default:
                q_bn = quatmath_casadi::quat_inverse(q);
        }

        /** Control variables **/
        SX dF = SX::sym("dF");     // Throttle [-]
        SX dE = SX::sym("dE");     // Elevator deflection [positive causing negative pitch movement (nose down)] [rad]
        SX dR = SX::sym("dR");     // Rudder deflection [positive causing negative yaw movement (nose left)] [rad]
        SX dA = SX::sym("dA");     // Aileron deflection [positive causing negative roll movement (right wing up)] [rad]

        SX vW_N_ctrl = SX::sym("vW_N_ctrl");
        SX vW_E_ctrl = SX::sym("vW_E_ctrl");

        /** Aerodynamic (Wind) frame **/
        /* Wind and apparent velocity in body frame */
        SX n_vW = SX::vertcat({vW_N, vW_E, vW_D});
        SX b_vW = quatmath_casadi::quat_transform(q_bn, n_vW);
        SX b_va = v - b_vW;

        /* Aerodynamic variables (Airspeed, angle of attack, side slip angle) */
        SX Va = SX::norm_2(b_va);
        SX alpha = atan(b_va(2) / b_va(0));
        SX beta = asin(b_va(1) / Va);

        /* Measured airspeed component (pitot tube orientation dependent) */
        SX r_sens = SX::vertcat({0.11, 0.22, -0.05});
        SX b_va_meas = b_va + SX::cross(r_sens, w);
        /* At fast body yawrates, the body rotation (thus pitot tube is faster than the CoG) is clearly negligible
         * under the effect of the pitot measurement direction being rotated out of the airflow (sideslip) */

        /* Pitot tube is oriented about 5 degrees above body x axes */
        SX q_sens_b = quatmath_casadi::T2quat(5.0 * M_PI / 180.0);
        SX sens_va = quatmath_casadi::quat_transform(q_sens_b, b_va_meas);
        SX Va_pitot = sens_va(0);

        /** ---------------------------------------------------------- **/
        /** Thrust Forces and Moments in body frame **/
        /** ---------------------------------------------------------- **/
//    SX throttle = F_thr0 / 6.3739;
        SX thrust = getThrust(dF, Va);
        SX b_F_thrust = thrust * SX::vertcat({cos(b_thrust_ang), 0, sin(b_thrust_ang)});

        SX b_r_thrust = SX::vertcat({0.25, 0, 0});
        SX b_M_thrust = SX::cross(b_r_thrust, b_F_thrust);

        /** ---------------------------------------------------------- **/
        /** Aerodynamic Forces and Moments in aerodynamic (wind) frame **/
        /** ---------------------------------------------------------- **/
        SX q_ba = quatmath_casadi::quat_multiply(quatmath_casadi::T2quat(alpha), quatmath_casadi::T3quat(-beta));

        SX V0 = (FIX_V0) ? FIXED_V0 : Va;

        SX dyn_press = 0.5 * rho * Va * Va;

        SX CL = CL0 + CLa * alpha + CLq * c / (2.0 * V0) * w(1) + CLde * dE;
        SX CD = CD0 + CL * CL / (M_PI * e_oswald * AR);

        /** Forces in x, y, z directions: -Drag, Side force, -Lift **/
        SX LIFT = dyn_press * S * CL;
        SX DRAG = dyn_press * S * CD;
        SX SF = dyn_press * S * (CYb * beta + b / (2.0 * V0) * (CYp * w(0) + CYr * w(2)) + CYdr * dR);

        SX a_F_aero = SX::vertcat({-DRAG, SF, -LIFT});

        /** Moments about x, y, z axes: L, M, N **/
        SX L = dyn_press * S * b *
               (Cl0 + Clb * beta + b / (2.0 * V0) * (Clp * w(0) + Clr * w(2)) + Clda * dA + Cldr * dR);

        SX M = dyn_press * S * c * (Cm0 + Cma * alpha + c / (2.0 * V0) * Cmq * w(1) + Cmde * dE);

        SX N = dyn_press * S * b *
               (Cn0 + Cnb * beta + b / (2.0 * V0) * (Cnp * w(0) + Cnr * w(2)) + Cnda * dA + Cndr * dR);

        SX b_M_aero = SX::vertcat({L, M, N});

        /** Aerodynamic Forces and Moments in body frame **/
        SX b_F_aero = quatmath_casadi::quat_transform(q_ba, a_F_aero);

        /** ---------------------------------------- **/
        /** Gravitation, Tether (body frame) **/
        /** ---------------------------------------- **/
        /** Gravitational acceleration **/
        SX b_g = quatmath_casadi::quat_transform(q_bn, SX::vertcat({0, 0, g}));

        /** Tether force and moment **/
        SX b_F_tether = SX::vertcat({0, 0, 0});
        SX b_M_tether = SX::vertcat({0, 0, 0});

        /** ----------------------------- **/
        /** Motion equations (body frame) **/
        /** ----------------------------- **/
        /** Linear motion equation **/
        SX spec_nongrav_force = (b_F_aero + b_F_thrust + b_F_tether) / mass;
        SX v_dot = spec_nongrav_force + b_g - SX::cross(w, v);

        /** Angular motion equation **/
        SX w_dot = SX::mtimes(SX::inv(J), (b_M_aero + b_M_thrust + b_M_tether - SX::cross(w, SX::mtimes(J, w))));

        /** ------------------------------------ **/
        /** Kinematic Equations (geodetic frame) **/
        /** ------------------------------------ **/
        /** Translation: Aircraft position derivative **/
        SX r_dot = quatmath_casadi::quat_transform(q, v);                            // q = q_nb, v (body frame)

        /** Rotation: Aircraft attitude derivative **/
        /* Quaternion representation */
        double lambda = -5;
        SX q_dot = 0.5 * quatmath_casadi::quat_multiply(q, SX::vertcat({0, w}))      // q = q_nb, w = omega (body frame)
                   + 0.5 * lambda * q * (SX::dot(q, q) - 1);                     // Quaternion norm stabilization term,
        // as in Gros: 'Baumgarte Stabilisation over the SO(3) Rotation Group for Control',
        // improved: lambda negative and SX::dot(q, q) instead of lambda positive and 1/SX::dot(q, q).

        /* Euler angle representation */
        SX phi = Phi(0);
        SX theta = Phi(1);
        SX J_eulKin = SX::vertcat({
                                          SX::horzcat({1, sin(phi) * tan(theta), cos(phi) * tan(theta)}),
                                          SX::horzcat({0, cos(phi), -sin(phi)}),
                                          SX::horzcat({0, sin(phi) / cos(theta), cos(phi) / cos(theta)})
                                  });
        SX Phi_dot = SX::mtimes(J_eulKin, w);

        /** End of model **/
        /** ============================================================================================================ **/

        /** Categorize parameters **/
        sort_model_parameters(model_params, opt_params, dyn_params, default_params, static_params, filename);

        /** State, Dynamics, Control **/
        SX state, dynamics, control;
        switch (stateRepresentation)
        {
            case AttEuler:
            {   // Euler angle representation
                state = SX::vertcat({v, w, Phi});
                dynamics = SX::vertcat({v_dot, w_dot, Phi_dot});
                control = SX::vertcat({dF, dE, dR, dA});

                x_default = DM::vertcat({12, 0, 0,
                                         0, 0, 0,
                                         0, 0, 0});

                DM x_trim_bounds = DM::vertcat({DM::horzcat({10, 15}), // vx
                                                DM::horzcat({-1, 1}),  // vy
                                                DM::horzcat({-5, 5}),  // wz

                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wx
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wy
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wz

                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}),    // roll
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}),    // pitch
                                                DM::horzcat({-179 * M_PI / 180, 179 * M_PI / 180}) // yaw
                                               });
                x_trim_lbound = x_trim_bounds(Slice(), 0);
                x_trim_ubound = x_trim_bounds(Slice(), 1);
                break;
            }
            case AttQuat:
            default:
            {
                state = SX::vertcat({v, w, q});
                dynamics = SX::vertcat({v_dot, w_dot, q_dot});
                control = SX::vertcat({dF, dE, dR, dA});

                x_default = DM::vertcat({12, 0, 0,
                                         0, 0, 0,
                                         1, 0, 0, 0});

                DM x_trim_bounds = DM::vertcat({DM::horzcat({10, 15}), // vx
                                                DM::horzcat({-1, 1}),  // vy
                                                DM::horzcat({-5, 5}),  // wz

                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wx
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wy
                                                DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}), // wz

                                                DM::horzcat({-1, 1}), // qw
                                                DM::horzcat({-1, 1}), // qx
                                                DM::horzcat({-1, 1}), // qy
                                                DM::horzcat({-1, 1})  // qz
                                               });
                x_trim_lbound = x_trim_bounds(Slice(), 0);
                x_trim_ubound = x_trim_bounds(Slice(), 1);
                break;
            }
        }

        DM u_trim_bounds = DM::vertcat({DM::horzcat({-0, 0}),                             // Static thrust
                                        DM::horzcat({-30 * M_PI / 180, 30 * M_PI / 180}), // Elevator deflection
                                        DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}),   // Rudder deflection
                                        DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180})    // Aileron deflection
                                       });
        u_trim_lbound = u_trim_bounds(Slice(), 0);
        u_trim_ubound = u_trim_bounds(Slice(), 1);

        this->State = state;
        this->Control = control;
        this->StateDot = dynamics;
        this->Output = state;

        /** Output mappings **/
        SX spec_tether_force = b_F_tether / mass;
        SX euler_angles = quatmath_casadi::quat2euler(q);
        SX jacobian_f_x = SX::jacobian(dynamics, state);
        SX jacobian_f_u = SX::jacobian(dynamics, control);

        /* Kinematic frame */
        SX g_vg = quatmath_casadi::quat_transform(q, v);
        SX Vg = SX::norm_2(g_vg);
        SX gamma = atan(-g_vg(2) / (Vg + 1e-4));
        SX chi = atan2(g_vg(1), g_vg(0));

        /* Full output (all in one vector) NOTE THAT debugSX only participates in vector if defined
         * anywhere else than initial declaration */
        SX dyn_out = SX::vertcat({Va_pitot, Va, alpha, beta, spec_nongrav_force, spec_tether_force, debugSX});

        output_mappings.insert({{"dyn_out",            dyn_out}, // Algebraic values before state update

                                {"thrust",             thrust},
                                {"Va",                 Va},
                                {"alpha",              alpha},
                                {"beta",               beta},
                                {"Va_pitot",           Va_pitot},

                                {"Vg",                 Vg},
                                {"gamma",              gamma},
                                {"chi",                chi},

                                {"euler_angles",       euler_angles},

                                {"spec_nongrav_force", spec_nongrav_force},
                                {"spec_tether_force",  spec_tether_force},

                                {"jacobian_f_x",       jacobian_f_x},
                                {"jacobian_f_u",       jacobian_f_u},

                                {"debugSX",            debugSX}});
    }
    ~FixedWing() = default;

    StateRepresentation getStateRepresentation() { return state_representation; }
protected:
    StateRepresentation state_representation{AttQuat};
};

} //namespace fixed_wing_casadi
} //namespace flight_model

#endif //SRC_FIXEDWINGDYNAMICSCASADI_HPP
