//
// Created by Johannes Waibel on 29.09.19.
//

#ifndef SRC_AWEEIGHTCONTROLLER_HPP
#define SRC_AWEEIGHTCONTROLLER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <ros/time.h>

#define M_ONE_G 9.80665

namespace awee_control {

/* Eigen types */
using Vector2 = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
using Vector3 = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Matrix3 = Eigen::Matrix<double, 3, 3, Eigen::DontAlign>;
using Quaternion = Eigen::Quaternion<double, Eigen::DontAlign>;

struct Waypoint {
    double lat{0};
    double lon{0};
    int turnDir{0};

    Waypoint() = default;

    Waypoint(const double &latInit, const double &lonInit, const int &turnDirInit) :
            lat(latInit), lon(lonInit), turnDir(turnDirInit) {}

    Vector3 getVect() { return (Vector3{lat, lon, static_cast<double>(turnDir)}); }
};

class AWEEightController {

public:
    AWEEightController() = default;

    ~AWEEightController() = default;

    Matrix3 T_wd_ned = Matrix3::Identity();   // Transformation from NED frame to wind dome frame (NED frame rotated s.t. x = wind vector)

    /* Parameters ===================================================================== */
    struct Parameters {

        /* Constant members mean that the parameter cannot be changed at runtime/via telemetry so far. */
        double windFrom_deg{180.0};
        double windDir{0.0};
        double Va_min{10.0};
        double teth_len{150.0};

        struct GuidanceParameters {

            /* All in degrees */
            Waypoint wp1{-15.0, 25.0, -1};
            Waypoint wp2{-25.0, 25.0, 1}; // skipped
            Waypoint wp3{-15.0, -25.0, 1};
            Waypoint wp4{-25.0, -25.0, -1}; // skipped

            double wp_th_lat{30.0};             // Skip 2nd and 4th WP -> navigate with 2 WPs, R_MIN only
            double wp_th_lon{20.0};

            double lat_max{30.0};
            double lat_min{-60.0};
            double latShift_c{20.0};
            double dEtotGain{3.0};

            double aircraftMass{1.5};
            double teth_c{4.8};

            double radius_min{20.0};
            double sph_offset{10.0};

        } guidance;

        struct ControlParameters {

            double R_TC{0.4};
            double R_lim{60.0};         // constant
            double P_TC{0.4};
            double P_lim_max{50.0};     // constant
            double P_lim_min{-50.0};    // constant

        } control;

    } p;

    struct DebugValues {

        Vector2 sphLatLon{0, 0};
        double l_chiErr{0};

    } debug;

    double getWindDir(const double &windFrom_deg);

    void setParameters(const Parameters &aweParameters);

    void getLoadFactorSetpoints(const Vector3 &g_xg, const Vector3 &g_vg, const double &Va,
                                const double &pitchAng,
                                const bool state_TERS_active,

                                uint &activeWpId, Vector3 &g_lfRef, double &rollRef, double &pitchRef);

    void getRateSetpoints(const Vector3 &g_lfRef, const Vector3 &g_vg, const double &Va,
                          const Vector3 &att, const Vector3 &g_accMeas,

                          double &rollRate_sp, double &pitchRate_sp);

    bool getSwitchToMainMode(const Vector3 &g_xg);

    bool getSwitchToClimb(const Vector3 &g_xg);

    void valueTesting();

private:
    bool m_last_TERS_active{false};
    double m_E_tot_ref{0.0};

    Eigen::Matrix<double, 4, 3> m_flightPlanStart_deg;
    Vector2 m_latLonThStart_deg;

    /* General help functions ======================================================================================= */
    template<class T>
    int sign(T val);

    static bool closeEnough(const double &f1, const double &f2);

    double getShortestAngleErr(const double &angleRef, const double &angle);

    double constrain(const double &valueToConstrain, const double &lowerLimit, const double &upperLimit);

    double degrees(const double &radians);

    double radians(const double &degrees);

    /* AWE help functions =========================================================================================== */
    double getCircArc(const Vector3 &g_xg);

    static void getKinematicFrame(const Vector3 &g_vg,

                                  double &gamma, double &chi);

    static void getKinematicFrame(const Vector3 &g_vg,

                                  Matrix3 &Tkg);

    static void getKinematicFrame(const Vector3 &g_vg,

                                  double &gamma, double &chi, Matrix3 &Tkg);

    /* Elementary transformations =================================================================================== */
    static Matrix3 T1(const double &angle);

    static Matrix3 T2(const double &angle);

    static Matrix3 T3(const double &angle);

    /* Eight mode navigation and guidance =========================================================================== */
    void navigation(const Vector3 &g_xg, const Vector3 &g_vg,

                    Vector2 &presentLatLon, Matrix3 &Tlg, Matrix3 &Tlk_l,
                    double &l_gamma, double &l_chi);

    void guidance_WpManagement(const Vector3 &g_xg, const double &Va,
                               bool state_TERS_active,

                               Eigen::Matrix<double, 4, 3> &flightPln_deg, Vector2 &latLonTh_deg);

    void guidance_WpSwitching(const Eigen::Matrix<double, 4, 3> &flightPln_deg, const Vector2 &latLonTh_deg,
                              const Vector2 &presentLatLon,

                              uint &activeWpId, Vector3 &activeWp);

    void
    guidance_chiRef(const Vector3 &g_xg, const Vector3 &activeWp, const Vector2 &presentLatLon, const double &l_chi,

                    double &l_chiRef, double &l_chiErr, double &distanceToWp);

    void guidance_accRef(const Vector3 &g_xg, const Vector3 &g_vg, const double &Va,
                         const Matrix3 &Tlg, const Matrix3 &Tlk_l,
                         const double &l_chi, const double &l_chiErr,
                         const double &l_gamma, const double &distanceToWp,

                         Vector3 &g_lfRef, double &gammaRef, double &chiRef);

    void highLevelControl_rollPitchRef(const Vector3 &g_vg,
                                       const double &gammaRef, const double &chiRef,
                                       const double &pitchAng,

                                       double &rollRef, double &pitchRef);

    /* Acceleration based control =================================================================================== */
    void controlLoadFactor_rollAngProt(const double &angErr, const double &rollAng,

                                       double &angErr_limited);


    void controlLoadFactor_pitchAngProt(const double &pitchRate, const double &pitchAng,

                                        double &pitchRate_limited);


    void controlLoadFactor_qComponent(const double &lfErr, const double &measAng, const double &rollAng,

                                      double &lfErr_q);


    void vecXYZ_to_YZpolar(const Vector3 &vec,

                           double &ang, double &val);

};
}


#endif //SRC_AWEEIGHTCONTROLLER_HPP
