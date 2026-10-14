#ifndef SRC_AWEHELICONTROL_HPP
#define SRC_AWEHELICONTROL_HPP

#include <iostream>
#include <iomanip>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "awe_control/AweControl3Tools.hpp"

namespace awe_control3 {

class AweHeliController
{
public:
    static double activation(const double x, const double start, const double end)
    {
        const double x_constrained = constrain(x, start, end);
        return 0.5 * (1 + std::cos(M_PI / (end - start) * (x_constrained - start) - M_PI));
    }

    /*
     * Winddome frame:
     * NED frame rotated about z axis, such that new x axis aligns with wind vector
     * x: along wind
     * y: horizontal
     * z: down
     * */
    static Eigen::Quaterniond get_q_wn(const Eigen::Vector3d &n_vW)
    {
        const double chi_wind = std::atan2(n_vW(1), n_vW(0));
        const Eigen::Quaterniond q_wn = T3quat(chi_wind);
        return q_wn;
    }
    /*
     * Local NED frame:
     * Tangential frame on the sphere around the origin, which ressembles the global coordinate frame of Earth, with
     * the wind vector blowing towards the north pole.
     * x: towards local north pole
     * y: (right handed frame)
     * z: towards ground station
     * */
    static Eigen::Quaterniond get_q_ln(const Eigen::Vector3d &n_pos)
    {
        const Vector2 latlon = awe_control3::AweHeliController::get_local_latlon(n_pos);
        const Quaternion q_ln = T2quat(-latlon(0)) * T1quat(latlon(1));
        return q_ln;
    }
    /*
     * Circle frame:
     * Like a helicopter flying forward from the perspectie of the ground station
     * x: upward, against the wind
     * y: horizontal, left (looking from ground station)
     * z: towards ground station
     * */
    static Eigen::Quaterniond get_q_cn(const Eigen::Vector3d &n_circle_pos)
    {
        using namespace awe_control3;

        const double bearing = getBearingFromCenter(n_circle_pos.segment<2>(0));
        const double declination = std::atan2(n_circle_pos.segment<2>(0).norm(), -n_circle_pos(2));

        const Eigen::Quaterniond q_cn = T2quat(declination) * T3quat(bearing + M_PI);
        return q_cn;
    }
    /*
     * Disk frame:
     * Like a helicopter flying forward from the perspective of the ground station
     * x: forward, down, along with wind direction
     * y: horizontal, right (looking from ground station)
     * z: towards ground station
     * */
    static Eigen::Quaterniond get_q_dn(const Eigen::Vector3d &n_circle_pos)
    {
        using namespace awe_control3;

        const double bearing = getBearingFromCenter(n_circle_pos.segment<2>(0));
        const double declination = std::atan2(n_circle_pos.segment<2>(0).norm(), -n_circle_pos(2));
//        std::cout << "DISK frame (brg | decl): " << bearing * 180.0/M_PI << ", " << declination * 180.0/M_PI << "\n";

        const Eigen::Quaterniond q_dn = T2quat(-declination) * T3quat(bearing);
        return q_dn;
    }

    double wind_from{0};
    Eigen::Quaterniond q_wn{1, 0, 0, 0};

    bool latlon_ref_valid{false};
    Eigen::Vector2d latlon_ref{0, 0};

    bool liftdir_ref_valid{false};
    Eigen::Vector2d liftdir_ref{0, 0};

    Eigen::Vector2d p_liftdir_trim{-0.22, 0};
    const double p_liftdir_angular_trim{radians(60)};

    // For mission segments
    double thr_planned{0};
    double coll_planned{0};
    Vector2 liftdir_planned{0, 0};
    Vector3 tethlen_vels_planned{0, 0, 0};

    void update_winddome(const Eigen::Vector3d &n_vW)
    {
        q_wn = get_q_wn(n_vW);
    }
    void update_winddome(const double wind_from)
    {
        const Vector3 n_vW(-std::cos(wind_from), -std::sin(wind_from), 0);
        update_winddome(n_vW);
    }
    Eigen::Vector3d to_winddome(const Eigen::Vector3d &n_vec) const { return q_wn * n_vec; }
    static Eigen::Vector2d get_local_latlon(const Eigen::Vector3d &pos)
    {
        const double lat = std::atan2(pos(0), pos.segment<2>(1).norm());
        const double lon = std::atan2(pos(1), -pos(2));
        return {lat, lon};
    }
    static Eigen::Vector3d get_local_LatLonAlt(const Eigen::Vector3d &pos)
    {
        const double alt = pos.norm();
        Eigen::Vector3d LatLonAlt;
        LatLonAlt << get_local_latlon(pos), alt;
        return LatLonAlt;
    }
    static double get_direct_bearing(const Eigen::Vector2d &latlon, const Eigen::Vector2d &to_latlon)
    {
        /* delta (lat), lambda (lon) */
        const double d1 = latlon(0);
        const double l1 = latlon(1);
        const double d2 = to_latlon(0);
        const double l2 = to_latlon(1);

        /* Bearing for great circle path to WP
         * from https://www.movable-type.co.uk/scripts/latlong.html
         *
         * double y = sin(l2 - l1) * cos(d2);
         * double x = cos(d1) * sin(d2) - sin(d1) * cos(d2) * cos(l2 - l1);
         * double l_chiRef = atan2(y, x);
         */
        const double l_chiRef = std::atan2(std::sin(l2 - l1) * std::cos(d2),
                                           std::cos(d1) * std::sin(d2) -
                                           std::sin(d1) * std::cos(d2) * std::cos(l2 - l1));

        return l_chiRef;
    }
    static double get_surf_distance(const Eigen::Vector2d &latlon0, const Eigen::Vector2d &latlon1, const double R)
    {
        /* delta (lat), lambda (lon) */
        const double d1 = latlon0(0);
        const double l1 = latlon0(1);
        const double d2 = latlon1(0);
        const double l2 = latlon1(1);

        /* Bearing and distance for great circle path to WP
         * from https://www.movable-type.co.uk/scripts/latlong.html */
        const double a = std::pow(std::sin((d2 - d1) / 2), 2) +
                         std::cos(d1) * std::cos(d2) * std::pow(std::sin((l2 - l1) / 2), 2);
        const double c = 2.0 * std::atan2(sqrt(a), sqrt(1 - a));
        const double dist = R * c;
        return dist;
    }
    static double get_surf_distance(const Eigen::Vector3d &latlonalt0, const Eigen::Vector2d &latlon1)
    {
        const Eigen::Vector2d latlon0 = latlonalt0.segment<2>(0);
        const double R = latlonalt0(2);
        return get_surf_distance(latlon0, latlon1, R);
    }
    static double get_surf_distance_angle(const Eigen::Vector2d &latlon0, const Eigen::Vector2d &latlon1)
    {
        return get_surf_distance(latlon0, latlon1, 1);
    }

    /*
     * Returns the normalized lift commands for two or three kites depending on their positions.
     *
     * Assumes clockwise rotation (looking along wind)
     *
     * Parameters:
     * collective:    Command for additional lift on whole disk (both kites)
     * d_accdir:      Command for direction of acceleration in disk frame (forward-right)
     *
     * */
    Eigen::VectorXd get_normlifts(const std::vector<Eigen::Vector3d> &n_kite_posns,
                                  const Eigen::Vector2d &d_liftdir);
};

}

#endif //SRC_AWEHELICONTROL_HPP
