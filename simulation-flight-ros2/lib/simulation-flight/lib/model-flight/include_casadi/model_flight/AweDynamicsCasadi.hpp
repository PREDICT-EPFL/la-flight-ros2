#ifndef SRC_AWEDYNAMICSCASADI_HPP
#define SRC_AWEDYNAMICSCASADI_HPP

#include "model_common/common_utils.hpp"
#include "model_common/DynamicsBaseCasadi.hpp"
#include "model_common/QuaternionMathCasadi.hpp"
#include "model_flight/StateRepresentation.hpp"

namespace flight_model {
namespace awe_casadi {

using namespace model_common;
using namespace casadi;

constexpr bool FIX_V0 = false; // In all models: Normalize rates with V0 instead of Va
constexpr double FIXED_V0 = 12;

class Tether
{
public:
    Tether(SX length, SX diameter, SX density, SX E, SX c_orth, SX air_density = 1.225)
    {
        p.length = length;
        p.diameter = diameter;
        p.density = density;
        p.E = E;
        p.c_orth = c_orth;
        p.air_density = air_density;

        p.cross_area = M_PI_4 * p.diameter * p.diameter;
        p.massPerMeter = p.cross_area * 1.0 * p.density;
    }
    struct Params
    {
        SX length{};
        SX diameter{};
        SX density{};
        SX E{};
        SX c_orth{};
        SX air_density{};

        SX cross_area{};
        SX massPerMeter{};
    } p{};

    SX get_F_lon(const SX &pos0, const SX &pos1,

                   SX &n_lonF_at_1, SX &n_lonF_at_0)
    {
        SX debugSX;

        SX diff_pos = pos1 - pos0;
        SX diff_dist = SX::norm_2(diff_pos);

        /* Longitudinal force */
        SX elongation = (diff_dist - p.length) / (p.length + 1e-6);
        n_lonF_at_1 = -diff_pos / diff_dist * SX::fmax(0, (p.E * p.cross_area) * (elongation + 0.005));

        n_lonF_at_0 = -n_lonF_at_1;

        return debugSX;
    }

    SX get_F_weight(const SX &pos0, const SX &pos1,

                      SX &n_W_at_1, SX &n_W_at_0)
    {
        SX debugSX;

        SX diff_pos = pos1 - pos0;

        /* Weight */
        SX n_weight = p.length * p.massPerMeter * SX::vertcat({0, 0, 9.81}); // Constant

        SX decl = SX::atan2(SX::norm_2(diff_pos(Slice(0, 2))), -diff_pos(2)); // Zero means pos1 is above pos0
        SX portion_at_1 = 0.5 * (1.0 + SX::cos(decl));
        n_W_at_1 = portion_at_1 * SX(n_weight); // probably invalid
        n_W_at_0 = SX(n_weight) - n_W_at_1;

        debugSX = SX::vertcat({debugSX, p.length});
        return debugSX;
    }

    SX get_F_drag(const SX &pos0, const SX &pos1,
                  const SX &n_vel0, const SX &n_vel1, const SX &n_vW,

                  SX &n_D_at_1, SX &n_D_at_0)
    {
        SX debugSX;

        // b means tether body frame here, not kite
        SX epsi = 1e-5;
        SX b_p_offs = SX::vertcat({1, 0, 0});

        SX n_va0 = n_vel0 - n_vW;
        SX n_va1 = n_vel1 - n_vW;

        SX p_diff = pos1 - pos0;
        SX chi = SX::atan2(p_diff(1), p_diff(0));
        SX gam = SX::atan2(p_diff(2), SX::norm_2(p_diff(Slice(0, 2))));
        using namespace quatmath_casadi;
        SX q_bw = quat_multiply(T2quat(-gam), T3quat(chi)); // X axis along beam, Y horizontal

        // Body tether frame
        SX b_p0 = b_p_offs; //quat_transform(q_bw, pos0 - pos0) + b_p_offs;
        SX b_p1 = quat_transform(q_bw, pos1 - pos0) + b_p_offs;
        SX b_va0 = quat_transform(q_bw, n_va0);
        SX b_va1 = quat_transform(q_bw, n_va1);

//        debugSX = SX::vertcat({debugSX, n_va0, n_va1, b_va0, b_va1});
//
        // Remove tether-aligned velocity component
        b_va0(0) = 0;
        b_va1(0) = 0;

        // From here, (almost) like Matlab code
        SX x1 = b_p0(0);
        SX x2 = b_p1(0);
        SX v1_yz = b_va0(Slice(1, 3));
        SX v2_yz = b_va1(Slice(1, 3));

        SX m = (v2_yz - v1_yz) / (x2 - x1);
        m = make_nonzero(m, epsi);

        SX b = v1_yz - m * x1;

        SX my = m(0);
        SX by = b(0);
        SX mz = m(1);
        SX bz = b(1);
//        debugSX = SX::vertcat({debugSX, v1_yz, v2_yz, my, by, mz, bz, x1, x2, 0, 0});

        SX cnst = -p.air_density / 2.0 * p.diameter * p.c_orth;
        SX b_F_tot = cnst * SX::vertcat({0,
                                         q_antider_2d_eval(my, by, mz, bz, x1, x2),
                                         q_antider_2d_eval(mz, bz, my, by, x1, x2)});
        SX b_M_tot = cnst * SX::vertcat({0,
                                        -xq_antider_2d_eval(mz, bz, my, by, x1, x2),
                                         xq_antider_2d_eval(my, by, mz, bz, x1, x2)});

//        SX b_xc = SX::cross(b_F_tot, b_M_tot) / (SX::sq(SX::norm_2(b_F_tot)) + epsi);
//        debugSX = SX::vertcat({debugSX, q_antider(x1, my, by, mz, bz), q_antider(x2, my, by, mz, bz),
//                               xq_antider(x1, my, by, mz, bz), xq_antider(x2, my, by, mz, bz), SX::zeros(8, 1)});

        // Back to NED frame
        SX q_wb = quat_inverse(q_bw);

        SX n_F_tot = quat_transform(q_wb, b_F_tot);
        SX n_M_tot = quat_transform(q_wb, b_M_tot);
        //        SX xc = quat_transform(q_wb, b_xc - b_p_offs) + pos0;
        SX n_p_ref_M = quat_transform(q_wb, SX::zeros(3, 1) - b_p_offs) + pos0;

        debugSX = SX::vertcat({debugSX, b_F_tot, b_M_tot, n_F_tot, n_M_tot});

        // Compute equivalent forces
        SX p1 = pos0 - n_p_ref_M;
        SX p2 = pos1 - n_p_ref_M;

        SX p_diff_ = p2 - p1;
        SX M_ = n_M_tot - SX::cross(p1, n_F_tot);
        n_D_at_1 = SX::cross(M_, p_diff_) / SX::sq(SX::norm_2(p_diff_));
        n_D_at_0 = n_F_tot - n_D_at_1;

        return debugSX;
    }

    SX get_F_drag_simpler(const SX &pos0, const SX &pos1,
                          const SX &n_vel0, const SX &n_vel1, const SX &n_vW,

                          SX &n_D_at_1, SX &n_D_at_0)
    {
        SX debugSX;

        // b means tether body frame here, not kite
        SX epsi = 1e-5;

        SX n_va0 = n_vel0 - n_vW;
        SX n_va1 = n_vel1 - n_vW;

        SX p_diff = pos1 - pos0;
        SX chi = SX::atan2(p_diff(1), p_diff(0));
        SX gam = SX::atan2(p_diff(2), SX::norm_2(p_diff(Slice(0, 2))));
        using namespace quatmath_casadi;
        SX q_bw = quat_multiply(T2quat(-gam), T3quat(chi)); // X axis along beam, Y horizontal

        // Body tether frame
        SX b_va0 = quat_transform(q_bw, n_va0);
        SX b_va1 = quat_transform(q_bw, n_va1);

        // From here, (almost) like Matlab code
        SX x2 = SX::norm_2(p_diff);
        SX v1_yz = b_va0(Slice(1, 3));
        SX v2_yz = b_va1(Slice(1, 3));

        SX m = (v2_yz - v1_yz) / x2;
        m = make_nonzero(m, epsi);
        SX b = v1_yz;

        SX my = m(0);
        SX by = b(0);
        SX mz = m(1);
        SX bz = b(1);

        SX cnst = -p.air_density / 2.0 * p.diameter * p.c_orth;
        SX b_F_tot = cnst * SX::vertcat({0,
                                         q_antider_2d_eval(my, by, mz, bz, 0, x2),
                                         q_antider_2d_eval(mz, bz, my, by, 0, x2)});
        SX b_M_tot = cnst * SX::vertcat({0,
                                        -xq_antider_2d_eval(mz, bz, my, by, 0, x2),
                                         xq_antider_2d_eval(my, by, mz, bz, 0, x2)});

        // Compute equivalent forces
        SX b_D_at_1 = SX::vertcat({0, b_M_tot(2), -b_M_tot(1)}) / x2;
        SX b_D_at_0 = b_F_tot - b_D_at_1;

        SX q_wb = quat_inverse(q_bw);
        n_D_at_1 = quat_transform(q_wb, b_D_at_1);
        n_D_at_0 = quat_transform(q_wb, b_D_at_0);

        return debugSX;
    }

    SX get_forces(const SX &pos0, const SX &pos1,
                  const SX &n_vel0, const SX &n_vel1, const SX &n_vW,

                  SX &n_lonF_at_1, SX &n_lonF_at_0,
                  SX &n_W_at_1, SX &n_W_at_0,
                  SX &n_D_at_1, SX &n_D_at_0)
    {
        n_lonF_at_1 = SX::zeros(3, 1);
        n_lonF_at_0 = SX::zeros(3, 1);
        n_W_at_1 = SX::zeros(3, 1);
        n_W_at_0 = SX::zeros(3, 1);
        n_D_at_1 = SX::zeros(3, 1);
        n_D_at_0 = SX::zeros(3, 1);

        get_F_lon(pos0, pos1, n_lonF_at_1, n_lonF_at_0);
        SX debugWeight = get_F_weight(pos0, pos1, n_W_at_1, n_W_at_0);
//        SX debugDrag = get_F_drag(pos0, pos1, n_vel0, n_vel1, n_vW, n_D_at_1, n_D_at_0);
        SX debugDrag = get_F_drag_simpler(pos0, pos1, n_vel0, n_vel1, n_vW, n_D_at_1, n_D_at_0);

        return SX::vertcat({n_lonF_at_1, n_D_at_1});
    }

public:
    SX make_nonzero(const SX &x, const SX &epsi)
    {
        return x + epsi * (1.0 + x / sqrt(SX::sq(x) + SX::sq(epsi)));
    }
    SX q_antider_2d(const SX &x, const SX &m1, const SX &b1, const SX &m2, const SX &b2)
    {
        SX epsi = 1e-8;

        SX m_y = m1;
        SX b_y = b1;
        SX m_z = m2;
        SX b_z = b2;

        // Precompute common expressions
        SX my2 = SX::sq(m_y);
        SX mz2 = SX::sq(m_z);

        SX sqrt_term = SX::sqrt(
                2.0 * x * b_y * m_y + 2.0 * x * b_z * m_z + b_y * b_y + b_z * b_z + x * x * (my2 + mz2));

        SX term1_denom = SX::sq(my2 + mz2);
        SX term1 = sqrt_term * (m_y * (x * b_z * m_z * (my2 + mz2) + b_z * b_z * (2.0 * my2 - mz2) +
                                       2.0 * x * x * (my2 + mz2) * (my2 + mz2)) +
                                b_y * (my2 * m_z * (7.0 * x * m_z - 3.0 * b_z) + 3.0 * mz2 * m_z * (b_z + x * m_z) +
                                       4.0 * x * my2 * my2) +
                                b_y * b_y * (5.0 * m_y * mz2 + 2 * my2 * m_y)) / (6.0 * term1_denom);

        SX term2_denom = SX::pow(my2 + mz2, 2.5);
        SX term2 = (m_z * SX::pow(b_y * m_z - b_z * m_y, 3) *
                    SX::log(epsi + sqrt(my2 + mz2) * sqrt_term + b_y * m_y + b_z * m_z + x * my2 + x * mz2)) /
                   (2.0 * term2_denom);

        SX res = term1 + term2;
        return res;
    }
    SX q_antider_2d_eval(const SX &m1, const SX &b1, const SX &m2, const SX &b2, const SX &a, const SX &b)
    {
        return q_antider_2d(b, m1, b1, m2, b2) - q_antider_2d(a, m1, b1, m2, b2);
    }
    SX xq_antider_2d(const SX &x, const SX &m1, const SX &b1, const SX &m2, const SX &b2)
    {
        SX epsi = 1e-8;

        SX m_y = m1;
        SX b_y = b1;
        SX m_z = m2;
        SX b_z = b2;
//        std::cout << "m1: " << m_y << ", b1: " << b_y << ", m2: " << m_z << ", b2: " << b_z << "\n";

        SX my2 = SX::sq(m_y);
        SX mz2 = SX::sq(m_z);

        SX term1_denom = 8.0 * SX::pow(my2 + mz2, 3.5);
        SX term1 = -(SX::log(epsi +
                             b_y * m_y + x * my2 + b_z * m_z + x * mz2 +
                             SX::sqrt(my2 + mz2) * SX::sqrt(b_y * b_y + b_z * b_z +
                                                            2 * x * b_y * m_y + 2 * x * b_z * m_z +
                                                            x * x * (my2 + mz2))) *
                     (b_z * m_y - b_y * m_z) * (b_z * m_y - b_y * m_z) * (b_z * m_y - b_y * m_z) *
                     (-5 * b_y * m_y * m_z + b_z * (my2 - 4 * mz2))) / term1_denom;

        SX sum_my_mz2 = my2 + mz2;
        SX my4 = SX::sq(my2);
        SX mz4 = SX::sq(mz2);

        SX bx = SX::sq(b_y) + SX::sq(b_z) + 2.0 * x * (b_y * m_y + b_z * m_z) + SX::sq(x) * sum_my_mz2;

        SX by3_term = SX::pow(b_y, 3) * (-2.0 * my4 - 9.0 * my2 * mz2 + 8.0 * mz4);
        SX by2_term = SX::sq(b_y) * (2.0 * x * SX::pow(m_y, 5) + m_y * SX::pow(m_z, 3) * (-37.0 * b_z + 7.0 * x * m_z) +
                                     SX::pow(m_y, 3) * m_z * (8.0 * b_z + 9.0 * x * m_z));
        SX my_term =
                m_y * (2.0 * SX::sq(x) * b_z * m_z * SX::sq(sum_my_mz2) + 6.0 * SX::pow(x, 3) * SX::pow(sum_my_mz2, 3) +
                       SX::pow(b_z, 3) * (-13.0 * my2 * m_z + 2.0 * SX::pow(m_z, 3)) +
                       x * SX::sq(b_z) * (3.0 * my4 + my2 * mz2 - 2.0 * mz4));
        SX by_term = b_y * (2.0 * SX::sq(x) * SX::sq(sum_my_mz2) * (5.0 * my2 + 4.0 * mz2) +
                            SX::sq(b_z) * (-5.0 * my4 + 36.0 * my2 * mz2 - 4.0 * mz4) -
                            2.0 * x * b_z * m_z * (3.0 * my4 + my2 * mz2 - 2.0 * mz4));

        SX term2_denom = 24 * SX::pow(my2 + mz2, 3);
        SX term2 = sqrt(bx) * (by3_term + by2_term + my_term + by_term) / term2_denom;

        SX res = term1 + term2;
        return res;
    }
    SX xq_antider_2d_eval(const SX &m1, const SX &b1, const SX &m2, const SX &b2, const SX &a, const SX &b)
    {
        return xq_antider_2d(b, m1, b1, m2, b2) - xq_antider_2d(a, m1, b1, m2, b2);
    }

};

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
    SX thrust_solius_12x6 = a * SX::sq(RPM_scaled - ts) + b * RPM_scaled * Va - thrust_offset;
    SX thrust = thrust_solius_12x6 * 6.3739 / 10.82;

    return thrust;
}

casadi::SX activation(const casadi::SX &x, double zero = 0, double scaling_param = 10)
{
    return 2.0 / M_PI * SX::atan(scaling_param * (x - zero));
}

/** Full N*13-state N-kite model ----------------------------------------------------------------------------------- **/
class MultiKite : public model_common::casadi_model::DynamicsBase
{
public:
    explicit MultiKite(unsigned N_kts, const std::string &filename,
                       const StateRepresentation &stateRepresentation = AttQuat,
                       const std::map<std::string, double> &static_params = {},
                       const std::vector <std::string> &dyn_params = {"v_wind_N", "v_wind_E", "v_wind_D",
                                                                      "v_wind1_N", "v_wind1_E", "v_wind1_D",
                                                                      "v_wind2_N", "v_wind2_E", "v_wind2_D"},
                       const std::vector <std::string> &opt_params = {},
                       bool teth_on = true,
                       bool wind_as_control = false,
                       bool turbulence_estimation = false,
                       bool couple_rud_to_ail = false) :
            DynamicsBase(filename, static_params, dyn_params, opt_params),
            state_representation(stateRepresentation)
    {
        if (N_kts > 3) { N_kts = 3; }

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

        SX vW1_N = SX::sym("v_wind1_N");
        SX vW1_E = SX::sym("v_wind1_E");
        SX vW1_D = SX::sym("v_wind1_D");

        SX vW2_N = SX::sym("v_wind2_N");
        SX vW2_E = SX::sym("v_wind2_E");
        SX vW2_D = SX::sym("v_wind2_D");

        default_params.insert({"air_density", 1.225});
        default_params.insert({"v_wind_N", 1});
        default_params.insert({"v_wind_E", 0.5});
        default_params.insert({"v_wind_D", 0});

        default_params.insert({"v_wind1_N", 1});
        default_params.insert({"v_wind1_E", 0.5});
        default_params.insert({"v_wind1_D", 0});

        default_params.insert({"v_wind2_N", 1});
        default_params.insert({"v_wind2_E", 0.5});
        default_params.insert({"v_wind2_D", 0});
        model_params = SX::vertcat({model_params, rho, vW_N, vW_E, vW_D});
        if (N_kts >= 2) { model_params = SX::vertcat({model_params, vW1_N, vW1_E, vW1_D}); }
        if (N_kts >= 3) { model_params = SX::vertcat({model_params, vW2_N, vW2_E, vW2_D}); }

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

//        /* From aileron deflection dA */
//        SX Clda = SX::sym("Clda");
//        SX Cnda = SX::sym("Cnda");
//
//        /* From aileron deflection dR */
//        SX CYdr = SX::sym("CYdr");
//        SX Cldr = SX::sym("Cldr");
//        SX Cndr = SX::sym("Cndr");

        model_params = SX::vertcat({model_params, //CDde,
                                    CLde, Cmde,
//                                    Clda, Cnda,
//                                    CYdr, Cldr, Cndr
                                    });

        /** ------------------------------ **/
        /**      Propulsion parameters     **/
        /** ------------------------------ **/
        SX thrust_factor = SX::sym("thrust_factor");
        SX b_thrust_ang = SX::sym("thrust_angle");
        //SX CLa_thrust = SX::sym("CLa_thr");
        SX b_thrust_rx = SX::sym("b_thrust_rx");
        SX b_thrust_ry = SX::sym("b_thrust_ry");
        SX b_thrust_rz = SX::sym("b_thrust_rz");

        default_params.insert({"thrust_factor", 1.0});
        default_params.insert({"thrust_angle", 0.0 * M_PI / 180.0});
//        default_params.insert({"CLa_thr", 0.0});
        model_params = SX::vertcat({model_params,
                                    thrust_factor, b_thrust_ang,
                                    b_thrust_rx, b_thrust_ry, b_thrust_rz});//, CLa_thrust});

        /** ------------------------------ **/
        /**        Tether parameters       **/
        /** ------------------------------ **/
        SX teth_diameter = SX::sym("teth_diameter");
        SX teth_density = SX::sym("teth_density");
        SX teth_E = SX::sym("teth_E");

        SX tethlat_len0 = SX::sym("tethlat_len0");
        SX tethlon_len0 = SX::sym("tethlon_len0");
        SX tethmain_len0 = SX::sym("tethmain_len0");

        SX tethlat_rx = SX::sym("tethlat_rx");
        SX tethlat_ry = SX::sym("tethlat_ry");
        SX tethlat_rz = SX::sym("tethlat_rz");

        SX tethlon_rx = SX::sym("tethlon_rx");
        SX tethlon_ry = SX::sym("tethlon_ry");
        SX tethlon_rz = SX::sym("tethlon_rz");

        SX cpoint_mass = SX::sym("cpoint_mass");

        if (teth_on)
        {
            model_params = SX::vertcat({model_params,
                                        teth_diameter, teth_density, teth_E,
                                        tethlat_len0, tethlon_len0, tethmain_len0,
                                        tethlat_rx, tethlat_ry, tethlat_rz,
                                        tethlon_rx, tethlon_ry, tethlon_rz,
                                        cpoint_mass
                                       });
        }

        /** ------------------------------ **/
        /**        Default state params    **/
        /** ------------------------------ **/
        SX posN_init = SX::sym("posN_init");
        SX V_init = SX::sym("V_init");

        model_params = SX::vertcat({model_params,
                                    posN_init, V_init
                                   });

        /** ============================================================================================================ **/
        /** Start of model **/
        SX debugSX;

        /* Aircraft Inertia Matrix */
        SX J = SX::diag(SX::vertcat({Ixx, Iyy, Izz}));
        J(0, 2) = Ixz;
        J(2, 0) = Ixz;

        /** State variables **/
        struct KiteStateSX
        {
            unsigned m_id{0};
            KiteStateSX(unsigned kite_id = -1)
            {
                if (kite_id < 0) return; // Only one kite, no kite-specific naming
                m_id = kite_id;

                const std::string k_prefix = "k" + std::to_string(m_id) + "_";

                vx = SX::sym(k_prefix + "vx");
                vy = SX::sym(k_prefix + "vy");
                vz = SX::sym(k_prefix + "vz");
                v = SX::vertcat({vx, vy, vz}); // Linear velocity of the CoG (body frame) [m/s]

                wx = SX::sym(k_prefix + "wx"); // roll rate
                wy = SX::sym(k_prefix + "wy"); // pitch rate
                wz = SX::sym(k_prefix + "wz"); // yaw rate
                w = SX::vertcat({wx, wy, wz}); // Angular velocity (body frame) [rad/s]

                posN = SX::sym(k_prefix + "posN");
                posE = SX::sym(k_prefix + "posE");
                posD = SX::sym(k_prefix + "posD");
                pos = SX::vertcat({posN, posE, posD}); // Position of the CoG (geodetic (NED) frame) [m]

                qw = SX::sym(k_prefix + "qw");
                qx = SX::sym(k_prefix + "qx");
                qy = SX::sym(k_prefix + "qy");
                qz = SX::sym(k_prefix + "qz");
                q = SX::vertcat({qw, qx, qy, qz}); // Rotation from NED to body frame. Transforms a body vector to ned. = q_nb
                q_bn = quatmath_casadi::quat_inverse(q);

                dF = SX::sym(k_prefix + "dF");     // Throttle [-]
                dE = SX::sym(k_prefix + "dE");     // Elevator deflection [positive causing negative pitch movement (nose down)] [rad]
                dR = SX::sym(k_prefix + "dR");     // Rudder deflection [positive causing negative yaw movement (nose left)] [rad]
                dA = SX::sym(k_prefix + "dA");     // Aileron deflection [positive causing negative roll movement (right wing up)] [rad]
                dAux0 = SX::sym(k_prefix + "dAux0");
                dAux1 = SX::sym(k_prefix + "dAux1");
            }

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
            SX q_bn = quatmath_casadi::quat_inverse(q);

            SX dF = SX::sym("dF");     // Throttle [-]
            SX dE = SX::sym("dE");     // Elevator deflection [positive causing negative pitch movement (nose down)] [rad]
            SX dR = SX::sym("dR");     // Rudder deflection [positive causing negative yaw movement (nose left)] [rad]
            SX dA = SX::sym("dA");     // Aileron deflection [positive causing negative roll movement (right wing up)] [rad]
//            if (couple_rud_to_ail) dR = 0 * -dA;
            SX dAux0 = SX::sym("dAux0");
            SX dAux1 = SX::sym("dAux1");

            SX Va = SX(0);
            SX alpha = SX(0);
            SX beta = SX(0);
            SX Va_pitot = SX(0);

            SX thrust = SX(0);
            SX b_F_thrust = SX::vertcat({0, 0, 0});
            SX b_M_thrust = SX::vertcat({0, 0, 0});

            SX b_F_aero = SX::vertcat({0, 0, 0});
            SX b_M_aero = SX::vertcat({0, 0, 0});

            SX b_g = SX::vertcat({0, 0, 0});

            SX nongrav_force = SX::vertcat({0, 0, 0});
            SX b_F_tether_lon = SX::vertcat({0, 0, 0}); // Tether from connecting point to kite
            SX b_F_tether_lat = SX::vertcat({0, 0, 0}); // Tether from kite to kite
            SX b_M_tether = SX::vertcat({0, 0, 0});
            SX tether_force = SX::vertcat({0, 0, 0});

            SX v_dot = SX::vertcat({0, 0, 0});
            SX w_dot = SX::vertcat({0, 0, 0});
            SX r_dot = SX::vertcat({0, 0, 0});
            SX q_dot = SX::vertcat({0, 0, 0, 0});

            SX debugSX;
        };

        std::vector<KiteStateSX> kites;
        for (unsigned k = 0; k < N_kts; k++) { kites.emplace_back(k); }

        SX n_vel_cpoint = SX::sym("n_vel_cpoint", 3);
        SX pos_cpoint = SX::sym("pos_cpoint", 3);
        if (!teth_on)
        {
            n_vel_cpoint = SX::zeros(3, 1);
            pos_cpoint = SX::zeros(3, 1);
        }

        /* Tether length dynamics */
        SX tethlat_len = SX::sym("tethlat_len");
        SX tethlat_len_dot = SX::sym("tethlat_len_dot");
        SX tethlat_lenvel = SX::sym("tethlat_lenvel"); // Input

        SX tethlon_len = SX::sym("tethlon_len");
        SX tethlon_len_dot = SX::sym("tethlon_len_dot");
        SX tethlon_lenvel = SX::sym("tethlon_lenvel"); // Input

        SX tethmain_len = SX::sym("tethmain_len");
        SX tethmain_len_dot = SX::sym("tethmain_len_dot");
        SX tethmain_lenvel = SX::sym("tethmain_lenvel"); // Input

        /** Aerodynamic (Wind) frame **/
        /* Wind and apparent velocity in body frame */
        std::vector<SX> nvWs;
        nvWs.push_back(SX::vertcat({vW_N, vW_E, vW_D}));
        if (N_kts >= 2) { nvWs.push_back(SX::vertcat({vW1_N, vW1_E, vW1_D})); }
        if (N_kts >= 3) { nvWs.push_back(SX::vertcat({vW2_N, vW2_E, vW2_D})); }

        for (unsigned k = 0; k < kites.size(); k++)
        {
            KiteStateSX &kt = kites[k];

            SX b_vW = quatmath_casadi::quat_transform(kt.q_bn, nvWs[k]);
            SX b_va = kt.v - b_vW;

            /* Aerodynamic variables (Airspeed, angle of attack, side slip angle) */
            kt.Va = SX::norm_2(b_va);
            kt.alpha = atan(b_va(2) / b_va(0));
//            kt.beta = asin(b_va(1) / kt.Va);
            kt.beta = atan2(b_va(1), (sign(b_va(0)) * SX::sqrt(SX::sq(b_va(0)) + SX::sq(b_va(2)))));

            /* Measured airspeed component (pitot tube orientation dependent) */
            SX r_sens = SX::vertcat({0.11, 0.22, -0.05});
            SX b_va_meas = b_va + SX::cross(r_sens, kt.w);
            /* At fast body yawrates, the body rotation (thus pitot tube is faster than the CoG) is clearly negligible
             * under the effect of the pitot measurement direction being rotated out of the airflow (sideslip) */

            /* Pitot tube is oriented about 5 degrees above body x axes */
            SX q_sens_b = quatmath_casadi::T2quat(5.0 * M_PI / 180.0);
            SX sens_va = quatmath_casadi::quat_transform(q_sens_b, b_va_meas);
            SX Va_pitot = sens_va(0);

//            debugSX = SX::vertcat({debugSX, b_va});

            /** ---------------------------------------------------------- **/
            /** Thrust Forces and Moments in body frame **/
            /** ---------------------------------------------------------- **/
            SX thrust = thrust_factor * getThrust(kt.dF, kt.Va);
            kt.b_F_thrust = thrust * SX::vertcat({cos(b_thrust_ang), 0, sin(b_thrust_ang)});

            SX b_r_thrust = SX::vertcat({b_thrust_rx, b_thrust_ry, b_thrust_rz});
            kt.b_M_thrust = SX::cross(b_r_thrust, kt.b_F_thrust);

            /** ---------------------------------------------------------- **/
            /** Aerodynamic Forces and Moments in aerodynamic (wind) frame **/
            /** ---------------------------------------------------------- **/
            SX q_ba = quatmath_casadi::quat_multiply(quatmath_casadi::T2quat(kt.alpha),
                                                     quatmath_casadi::T3quat(-kt.beta));

            SX V0 = (FIX_V0) ? FIXED_V0 : kt.Va;

            SX dyn_press = 0.5 * rho * kt.Va * kt.Va;

            SX CL = CL0 + CLa * kt.alpha + CLq * c / (2.0 * V0) * kt.w(1) + CLde * kt.dE;
            SX CD = CD0 + CL * CL / (M_PI * e_oswald * AR);

            /** Forces in x, y, z directions: -Drag, Side force, -Lift **/
            SX LIFT = dyn_press * S * CL;
            SX DRAG = dyn_press * S * CD;
            SX SF = dyn_press * S * (CYb * kt.beta + b / (2.0 * V0) * (CYp * kt.w(0) + CYr * kt.w(2))); // + CYdr * kt.dR);

            SX a_F_aero = SX::vertcat({-DRAG, SF, -LIFT});

            /** Moments about x, y, z axes: L, M, N **/
            SX L = dyn_press * S * b *
                   (Cl0 + Clb * kt.beta + b / (2.0 * V0) * (Clp * kt.w(0) + Clr * kt.w(2))); // + Clda * kt.dA + Cldr * kt.dR);

            SX M = dyn_press * S * c * (Cm0 + Cma * kt.alpha + c / (2.0 * V0) * Cmq * kt.w(1) + Cmde * kt.dE);

            SX N = dyn_press * S * b *
                   (Cn0 + Cnb * kt.beta + b / (2.0 * V0) * (Cnp * kt.w(0) + Cnr * kt.w(2))); // + Cnda * kt.dA + Cndr * kt.dR);

            kt.b_M_aero = SX::vertcat({L, M, N});

            /** Aerodynamic Forces and Moments in body frame **/
            kt.b_F_aero = quatmath_casadi::quat_transform(q_ba, a_F_aero);

            /** ---------------------------------------- **/
            /** Gravitation, Tether (body frame) **/
            /** ---------------------------------------- **/
            /** Gravitational acceleration **/
            kt.b_g = quatmath_casadi::quat_transform(kt.q_bn, SX::vertcat({0, 0, g}));
        }

        SX cpoint_nongrav_force = SX::zeros(3, 1);
        SX base_F_tot = SX::zeros(3, 1);

        /* Tether attachment points */
        SX b_tethlat_attch = SX::vertcat({tethlat_rx, tethlat_ry, tethlat_rz});
        SX b_tethlon_attch = SX::vertcat({tethlon_rx, tethlon_ry, tethlon_rz});

        if (teth_on)
        {
            /** Tether material properties **/
            const double c_orth = 1.2;

            /** Lateral tether from kite0 to kite1 **/
            if (N_kts >= 2)
            {
                for (unsigned k = 0; k < kites.size(); k++)
                {
                    KiteStateSX &ktA = kites[k];
                    KiteStateSX &ktB = (k < kites.size() - 1) ? kites[k + 1] : kites[0];

                    Tether tethlat(tethlat_len, teth_diameter, teth_density, teth_E, c_orth, rho);

                    SX n_pos0 = ktA.pos + quatmath_casadi::quat_transform(ktA.q, b_tethlat_attch);
                    SX n_pos1 = ktB.pos + quatmath_casadi::quat_transform(ktB.q, b_tethlat_attch);
                    SX n_vel0 = quatmath_casadi::quat_transform(ktA.q, ktA.v);
                    SX n_vel1 = quatmath_casadi::quat_transform(ktB.q, ktB.v);

                    SX n_tethlat_F_at_k1, n_tethlat_F_at_k0,
                       n_tethlat_W_at_k1, n_tethlat_W_at_k0,
                       n_tethlat_D_at_k1, n_tethlat_D_at_k0;

                    SX nvW_A = nvWs[k];
                    SX nvW_B = (k < kites.size() - 1) ? nvWs[k + 1] : nvWs[0];
                    SX debug_tethlat = tethlat.get_forces(n_pos0, n_pos1, n_vel0, n_vel1, (nvW_A + nvW_B) / 2.0, // #todo: independent apparent velocities
                                                          n_tethlat_F_at_k1, n_tethlat_F_at_k0,
                                                          n_tethlat_W_at_k1, n_tethlat_W_at_k0,
                                                          n_tethlat_D_at_k1, n_tethlat_D_at_k0);

                    SX n_tethlat_F_tot_at_kt1 = n_tethlat_F_at_k1 + n_tethlat_W_at_k1 + n_tethlat_D_at_k1;
                    SX n_tethlat_F_tot_at_kt0 = n_tethlat_F_at_k0 + n_tethlat_W_at_k0 + n_tethlat_D_at_k0;

                    ktA.b_F_tether_lat += quatmath_casadi::quat_transform(ktA.q_bn, n_tethlat_F_tot_at_kt0);
                    ktB.b_F_tether_lat += quatmath_casadi::quat_transform(ktB.q_bn, n_tethlat_F_tot_at_kt1);
                }
                tethlat_len_dot = activation(tethlat_len, 0.00099, 50) * tethlat_lenvel; // Limits length to be non-negative}
            }

            /** Tethers from conpoint to kites **/
            for (unsigned k = 0; k < kites.size(); k++)
            {
                KiteStateSX &kt = kites[k];

                Tether tethlon(tethlon_len, teth_diameter, teth_density, teth_E, c_orth, rho);

                SX n_pos1 = kt.pos + quatmath_casadi::quat_transform(kt.q, b_tethlon_attch);
                SX n_vel1 = quatmath_casadi::quat_transform(kt.q, kt.v);

                SX n_tethlon_F_at_kt, n_tethlon_F_at_conp,
                   n_tethlon_W_at_kt, n_tethlon_W_at_conp,
                   n_tethlon_D_at_kt, n_tethlon_D_at_conp;

                SX debug_tethlon = tethlon.get_forces(pos_cpoint, n_pos1, n_vel_cpoint, n_vel1, nvWs[k], // #todo: Using vW of kite for now
                                                      n_tethlon_F_at_kt, n_tethlon_F_at_conp,
                                                      n_tethlon_W_at_kt, n_tethlon_W_at_conp,
                                                      n_tethlon_D_at_kt, n_tethlon_D_at_conp);

                SX n_tethlon_F_tot_at_kt = n_tethlon_F_at_kt + n_tethlon_W_at_kt + n_tethlon_D_at_kt;
                SX n_tethlon_F_tot_at_conp = n_tethlon_F_at_conp + n_tethlon_W_at_conp + n_tethlon_D_at_conp;

                kt.b_F_tether_lon += quatmath_casadi::quat_transform(kt.q_bn, n_tethlon_F_tot_at_kt); // Force on kite
                cpoint_nongrav_force += n_tethlon_F_tot_at_conp; // Force on cpoint
            }
            tethlon_len_dot = activation(tethlon_len, 0.00099, 50) * tethlon_lenvel; // Limits length to be non-negative

            /** Tether from ground to conpoint **/
            {
                SX pos0 = SX::vertcat({0, 0, 0});
                SX n_vel0 = SX::vertcat({0, 0, 0});
                SX pos1 = pos_cpoint;
                SX n_vel1 = n_vel_cpoint;

                Tether teth_main(tethmain_len, teth_diameter, teth_density, teth_E, c_orth, rho);

                SX n_tethm_lonF_at_conp, n_tethm_lonF_at_0;
                SX n_tethm_W_at_conp, n_tethm_W_at_0;
                SX n_tethm_D_at_conp;

                teth_main.get_F_lon(pos0, pos1,
                                    n_tethm_lonF_at_conp, n_tethm_lonF_at_0);
                teth_main.get_F_weight(pos0, pos1,
                                       n_tethm_W_at_conp, n_tethm_W_at_0);

                /* Drag - use simplified formula as origin is fixed */
                SX diff_pos = pos1 - SX::vertcat({0, 0, 0});

                SX n_vW_mean = SX::vertcat({0, 0, 0});
                for (unsigned k = 0; k < kites.size(); k++) { n_vW_mean += nvWs[k] / kites.size(); }
                SX n_va = n_vel1 - n_vW_mean; // #todo: using average of kite vWs for now
                SX diff_pos_normalized = diff_pos / (SX::norm_2(diff_pos) + 1e-3);
                SX n_va_normalized = n_va / (SX::norm_2(n_va) + 1e-3);
                SX angle = SX::acos(SX::dot(diff_pos_normalized, n_va_normalized));
                SX n_va_proj = n_va - (SX::norm_2(n_va) * SX::cos(angle) * diff_pos_normalized);

                n_tethm_D_at_conp = -0.125 * rho * c_orth * teth_diameter * tethmain_len *
                                    SX::norm_2(n_va_proj) * n_va_proj;
                cpoint_nongrav_force += n_tethm_lonF_at_conp + n_tethm_W_at_conp + n_tethm_D_at_conp;
                base_F_tot += n_tethm_lonF_at_0 + n_tethm_W_at_0;
            }
            tethmain_len_dot = activation(tethmain_len, 0.00099, 50) * 8 * tethmain_lenvel; // Limits length to be non-negative
        }
        else
        {
            for (KiteStateSX &kt: kites) { kt.b_F_tether_lat = kt.b_F_tether_lon = SX::zeros(3, 1); }

            tethlat_len = 0;
            tethlat_lenvel = 0;
            tethlat_len_dot = 0;

            tethlon_len = 0;
            tethlon_lenvel = 0;
            tethlon_len_dot = 0;

            tethmain_len = 0;
            tethmain_lenvel = 0;
            tethmain_len_dot = 0;
        }
        /* Connection point dynamics */
        SX n_vel_cpoint_dot = cpoint_nongrav_force / cpoint_mass + SX::vertcat({0, 0, g});
        SX pos_cpoint_dot = n_vel_cpoint;

        for (KiteStateSX &kt: kites)
        {
            /* Moments on kites */
            kt.b_M_tether = SX::cross(b_tethlat_attch, kt.b_F_tether_lat) +
                            SX::cross(b_tethlon_attch, kt.b_F_tether_lon);

            /** ----------------------------- **/
            /** Motion equations (body frame) **/
            /** ----------------------------- **/
            /** Linear motion equation **/
            kt.tether_force = (kt.b_F_tether_lat + kt.b_F_tether_lon);
            kt.nongrav_force = (kt.b_F_aero + kt.b_F_thrust); // Without tether!
            kt.v_dot = (kt.nongrav_force + kt.tether_force) / mass + kt.b_g - SX::cross(kt.w, kt.v);

            /** Angular motion equation **/
            kt.w_dot = SX::mtimes(SX::inv(J),
                                  (kt.b_M_aero + kt.b_M_thrust + kt.b_M_tether - SX::cross(kt.w, SX::mtimes(J, kt.w))));

            /** ------------------------------------ **/
            /** Kinematic Equations (geodetic frame) **/
            /** ------------------------------------ **/
            /** Translation: Aircraft position derivative **/
            kt.r_dot = quatmath_casadi::quat_transform(kt.q, kt.v); // q = q_nb, v (body frame)

            /** Rotation: Aircraft attitude derivative **/
            /* Quaternion representation */
            double lambda = -5;
            kt.q_dot = 0.5 * quatmath_casadi::quat_multiply(kt.q, SX::vertcat({0, kt.w}))
                       + 0.5 * lambda * kt.q * (SX::dot(kt.q, kt.q) - 1); // Quaternion norm stabilization term,
            // as in Gros: 'Baumgarte Stabilisation over the SO(3) Rotation Group for Control',
            // improved: lambda negative and SX::dot(q, q) instead of lambda positive and 1/SX::dot(q, q).
        }
        /** End of model **/
        /** ============================================================================================================ **/

        /** Categorize parameters **/
        sort_model_parameters(model_params, opt_params, dyn_params, default_params, static_params, filename);

        SX params0_SX = SX::vertcat({tethlat_len0, tethlon_len0, tethmain_len0,
                                       tethlat_rx, tethlat_ry, tethlat_rz,
                                       tethlon_rx, tethlon_ry, tethlon_rz});
        DM params0_DM = DM(SX::substitute(params0_SX, SymStaticParameters, SX(ValStaticParameters)));
        DMDict tethlens0 = {{"lat",  params0_DM(0)},
                            {"lon",  params0_DM(1)},
                            {"main", params0_DM(2)}};
        DMDict params0 =  {{"lat_attch",  params0_DM(Slice(3, 3+3))},
                           {"lon_attch",  params0_DM(Slice(6, 6+3))}};

//        if (tethlens0["main"].nonzeros()[0] <= 0)
//        {
//            std::cout << "Main tether OFF\n";
//            n_vel_cpoint_dot = SX::zeros(3, 1);
//            cpoint_nongrav_force = SX::zeros(3, 1);
//            tethlens0["main"] = 0;
//        }

        SX default_state_SX = SX::vertcat({posN_init, V_init});
        DM default_state_DM = DM(SX::substitute(default_state_SX, SymStaticParameters, SX(ValStaticParameters)));
        DMDict default_state = {{"posN", default_state_DM(0)},
                                {"vx", default_state_DM(1)}};

        /** State, Dynamics, Control **/
        SX state, dynamics, control, dyn_out;
        for (unsigned k = 0; k < N_kts; k++)
        {
            KiteStateSX kt = kites[k];

            state = SX::vertcat({state, kt.v, kt.w, kt.pos, kt.q});
            dynamics = SX::vertcat({dynamics, kt.v_dot, kt.w_dot, kt.r_dot, kt.q_dot}); // SX::zeros(13, 1)});
            control = SX::vertcat({control, kt.dF, kt.dE, kt.dR, kt.dA, kt.dAux0, kt.dAux1});

            const bool teth0_valid = (tethlens0["lon"].nonzeros() >= tethlens0["lat"].nonzeros());
            casadi_assert(teth0_valid, "Initial lon tether must be > lat tether length");
            // Outer radius (circle through corners)
            DM R = 0.5 * tethlens0["lat"] / std::sin(M_PI / N_kts);
            R += DM::abs(params0["lat_attch"](1));
            DM H = tethlens0["main"] + DM::sqrt(DM::sq(tethlens0["lon"]) - DM::sq(R));

            const DM brg_k = k * (2.0 * M_PI) / N_kts;
            x_default = DM::vertcat({x_default,
                                     default_state["vx"], 0, 0,
                                     0, 0, 0,
                                     DM::vertcat({default_state["posN"], 0, 0}) +
                                         DM::vertcat({R * DM::vertcat({DM::cos(brg_k), DM::sin(brg_k)}), -0.95 * H}),
                                     quatmath_casadi::quat_multiply<DM>(quatmath_casadi::T3quat(M_PI_2 - brg_k), DM::vertcat({1, 0, 0, 0}))
            });

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
            x_trim_lbound = DM::vertcat({x_trim_lbound, x_trim_bounds(Slice(), 0)});
            x_trim_ubound = DM::vertcat({x_trim_ubound, x_trim_bounds(Slice(), 1)});

            DM u_physical_bounds = DM::vertcat({DM::horzcat({0, 1}), // Throttle
                                                DM::horzcat({-21.0 * M_PI / 180.0, 21.0 * M_PI / 180.0}), // Elevator
                                                DM::horzcat({-26.5 * M_PI / 180.0, 26.5 * M_PI / 180.0}), // Rudder
                                                DM::horzcat({-20.5 * M_PI / 180.0, 20.5 * M_PI / 180.0}), // Aileron
                                                DM::horzcat({-1, 1}), // Aux0
                                                DM::horzcat({-1, 1}) // Aux1
                                               });
            u_lbound = DM::vertcat({u_lbound, u_physical_bounds(Slice(), 0)});
            u_ubound = DM::vertcat({u_ubound, u_physical_bounds(Slice(), 1)});

            DM u_trim_bounds = DM::vertcat({DM::horzcat({-0, 0}),                             // Static thrust
                                            DM::horzcat({-30 * M_PI / 180, 30 * M_PI / 180}), // Elevator deflection
                                            DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}),   // Rudder deflection
                                            DM::horzcat({-5 * M_PI / 180, 5 * M_PI / 180}),    // Aileron deflection
                                            DM::horzcat({-1, 1}), // Aux0
                                            DM::horzcat({-1, 1}) // Aux1
                                           });
            u_trim_lbound = DM::vertcat({u_trim_lbound, u_trim_bounds(Slice(), 0)});
            u_trim_ubound = DM::vertcat({u_trim_ubound, u_trim_bounds(Slice(), 1)});

            /** Output mappings **/
            SX euler_angles = quatmath_casadi::quat2euler(kt.q);
            SX jacobian_f_x = SX::jacobian(dynamics, state);
            SX jacobian_f_u = SX::jacobian(dynamics, control);

            /* Kinematic frame */
            SX g_vg = quatmath_casadi::quat_transform(kt.q, kt.v);
            SX Vg = SX::norm_2(g_vg);
            SX gamma = atan(-g_vg(2) / (Vg + 1e-4));
            SX chi = atan2(g_vg(1), g_vg(0));

            /* Full output (all in one vector) NOTE THAT debugSX only participates in vector if defined
             * anywhere else than initial declaration */
            SX dyn_out_kt = SX::vertcat(
                    {kt.Va_pitot, kt.Va, kt.alpha, kt.beta, kt.nongrav_force, kt.tether_force});

            dyn_out = SX::vertcat({dyn_out, dyn_out_kt});

            unsigned id = kt.m_id;
            output_mappings.insert(
                    {{"k" + std::to_string(id) + "_" + "dyn_out",            dyn_out_kt}, // Algebraic values before state update

                     {"k" + std::to_string(id) + "_" + "thrust",             kt.thrust},

                     {"k" + std::to_string(id) + "_" + "Va",                 kt.Va},
                     {"k" + std::to_string(id) + "_" + "alpha",              kt.alpha},
                     {"k" + std::to_string(id) + "_" + "beta",               kt.beta},
                     {"k" + std::to_string(id) + "_" + "Va_pitot",           kt.Va_pitot},

                     {"k" + std::to_string(id) + "_" + "Vg",                 Vg},
                     {"k" + std::to_string(id) + "_" + "gamma",              gamma},
                     {"k" + std::to_string(id) + "_" + "chi",                chi},

                     {"k" + std::to_string(id) + "_" + "euler_angles",       euler_angles},

                     {"k" + std::to_string(id) + "_" + "nongrav_force", kt.nongrav_force},
                     {"k" + std::to_string(id) + "_" + "tether_force",  kt.tether_force},

                     {"k" + std::to_string(id) + "_" + "jacobian_f_x",       jacobian_f_x},
                     {"k" + std::to_string(id) + "_" + "jacobian_f_u",       jacobian_f_u},

                     {"k" + std::to_string(id) + "_" + "debugSX",            debugSX}});
        }

//        x_default(Slice(9, 13)) = DM::vertcat({0, 0, 0, 1}); // First kite in opposed orientation
//        x_default(Slice(13 + 9, 13 + 13)) = DM::vertcat({0, 0, 0, 1}); // Second kite in opposed orientation

        /// conpoint
        state = SX::vertcat({state, n_vel_cpoint, pos_cpoint});
        dynamics = SX::vertcat({dynamics, n_vel_cpoint_dot, pos_cpoint_dot}); // SX::zeros(6, 1)});

        // Default state
        DM multikite_avg_posNED(0);
        for (int k = 0; k < static_cast<int>(N_kts); k++)
        {
            multikite_avg_posNED += x_default(Slice(k*13 + 6, k*13 + 9)) / static_cast<double>(N_kts);
        }
        x_default = DM::vertcat({x_default,
                                 DM::zeros(3, 1),
                                 multikite_avg_posNED / DM::norm_2(multikite_avg_posNED) * tethlens0["main"]});
        x_trim_ubound = DM::vertcat({x_trim_ubound, DM::inf(6, 1)});
        x_trim_lbound = DM::vertcat({x_trim_lbound, -DM::inf(6, 1)});

        dyn_out = SX::vertcat({dyn_out, cpoint_nongrav_force});
        output_mappings.insert({{"cpoint_nongrav_force", cpoint_nongrav_force},
                               });
        dyn_out = SX::vertcat({dyn_out, base_F_tot});
        output_mappings.insert({{"base_F_tot", base_F_tot}
                               });

        /// Tether lengths
        state = SX::vertcat({state, tethlat_len, tethlon_len, tethmain_len});
        dynamics = SX::vertcat({dynamics, tethlat_len_dot, tethlon_len_dot, tethmain_len_dot});
        control = SX::vertcat({control, tethlat_lenvel, tethlon_lenvel, tethmain_lenvel});
        x_default = DM::vertcat({x_default, tethlens0["lat"], tethlens0["lon"], tethlens0["main"]});

        u_lbound = DM::vertcat({u_lbound, -1, -1, -1});
        u_ubound = DM::vertcat({u_ubound, 1, 1, 1});
        u_trim_lbound = DM::vertcat({u_trim_lbound, -1, -1, -1});
        u_trim_ubound = DM::vertcat({u_trim_ubound, 1, 1, 1});

        /// Dynout
        dyn_out = SX::vertcat({dyn_out, debugSX});
        output_mappings.insert({"dyn_out", dyn_out});

        this->State = state;
        this->Control = control;
        this->StateDot = dynamics;
        this->Output = state;
    }
    ~MultiKite() = default;

    StateRepresentation getStateRepresentation() { return state_representation; }
protected:
    StateRepresentation state_representation{AttQuat};
};

template <unsigned N_kts = 2>
class MultiKiteMapped : public MultiKite
{
public:
    MultiKiteMapped(const std::string &filename) : MultiKite(N_kts, filename)
    {
        /* Mirror input aux to input teth{lat, lon, main}_lenvel
         *
         * In symbolic expressions, teth{lat, lon, main}_lenvel is replaced by k{0, 1}_aux{0, 1}.
         * teth{lat, lon, main}_lenvel becomes a stale input.
         */

        SX tethlat_lenvel = this->Control(N_kts * 6 + 0);
        SX k1_dAux0 = this->Control(6 + 4 + 0);
        this->StateDot = SX::substitute(this->StateDot, tethlat_lenvel, k1_dAux0);
        this->Output = SX::substitute(this->Output, tethlat_lenvel, k1_dAux0);

        SX tethlon_lenvel = this->Control(N_kts * 6 + 1);
        SX k0_dAux1 = this->Control(4 + 1);
        this->StateDot = SX::substitute(this->StateDot, tethlon_lenvel, k0_dAux1);
        this->Output = SX::substitute(this->Output, tethlon_lenvel, k0_dAux1);

        SX tethmain_lenvel = this->Control(N_kts * 6 + 2);
        SX k0_dAux0 = this->Control(4 + 0);
        this->StateDot = SX::substitute(this->StateDot, tethmain_lenvel, k0_dAux0);
        this->Output = SX::substitute(this->Output, tethmain_lenvel, k0_dAux0);
    }
};

}
}
#endif //SRC_AWEDYNAMICSCASADI_HPP
