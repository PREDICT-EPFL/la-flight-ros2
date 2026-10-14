//
// Created by johannes on 07.04.20.
//

#include "awe_control/AweTurnController.hpp"

using namespace awe_control2;

/* Public */
void AweTurnController::updateWinddome() { q_wd_g = T3quat(p.windFrom); }
void AweTurnController::updateParameterDependencies() {

    updateWinddome();

    p.flightplan = {{-p.flightplan_wpDeclination, p.flightplan_wpLon, -p.flightplan_turnDir},
                    {-p.flightplan_wpDeclination, -p.flightplan_wpLon, p.flightplan_turnDir},
                    p.flightplan_thLon};
}


bool AweTurnController::getSwitchToMainMode(const Vector3 &g_xg) {

    Vector3 wd_xg = q_wd_g * g_xg;
    double circArg = getBearingFromCenter(wd_xg.segment<2>(0));

    /* Distance to ground station above 95 % of tether length and
     * position on climb circle suitable to enter eight */
    bool result = (g_xg.norm() > (0.9 * p.teth_len)) &&
                  (circArg > radians(135.0)) && (circArg < radians(225.0));

    if (result)
        return true;
    else
        return false;
}
bool AweTurnController::getSwitchToClimb(const Vector3 &g_xg) {

    /* Distance to ground station less than 60 % of tether length or
     * height below 40 meters */
    bool result = (g_xg.norm() < (0.6 * p.teth_len)) ||
                  (-g_xg(2) < 40.0);
    if (result)
        return true;
    else
        return false;
}


void AweTurnController::turnMode_getVelocityRef(const Vector3 &g_xg, const Vector3 &g_vg,
                                                const double &l_chiErr_commanded,

                                                Vector3 &g_vgRef) {

    /** Navigation **/
    Vector2 lLatLon;
    Quaternion q_lgk_g;
    double distToGs, l_gamma, l_chi, Vg;
    navigateOnSphere(g_xg, g_vg,
                     lLatLon, distToGs, q_lgk_g, l_gamma, l_chi, Vg);

    /** Guidance **/
    /* Guidance is done by the pilot by commanding a turn rate (l_chiErr_commanded) */

    /** Control **/
    controlFlightOnSphere(q_lgk_g, distToGs, l_gamma, Vg, p.control.distOffsetRef, l_chiErr_commanded,
                          g_vgRef);
}

void
AweTurnController::eightMode_getVelocityRef(const Vector3 &g_xg, const Vector3 &g_vg, const double &distOffsetRef,

                                            uint &activeWpId, Vector3 &g_vgRef) {

    /** Navigation **/
    Vector2 lLatLon;
    Quaternion q_lgk_g;
    double distToGs, l_gamma, l_chi, Vg;
    navigateOnSphere(g_xg, g_vg,
                     lLatLon, distToGs, q_lgk_g, l_gamma, l_chi, Vg);

    /** Guidance **/
    Waypoint activeWaypoint;
    selectWaypoint(lLatLon, p.flightplan,
                   activeWpId, activeWaypoint);

    double l_chiErr;
    guideToWaypoint(lLatLon, l_chi, activeWaypoint,
                    l_chiErr);

    /** Control **/
    controlFlightOnSphere(q_lgk_g, distToGs, l_gamma, Vg, distOffsetRef, l_chiErr,
                          g_vgRef);
}




/* Private */
/** Navigation **/
void AweTurnController::navigateOnSphere(const Vector3 &g_xg, const Vector3 &g_vg,

                                         Vector2 &lLatLon, double &distToGs, Quaternion &q_lgk_g,
                                         double &l_gamma, double &l_chi, double &Vg) {

    distToGs = g_xg.norm();
    Vg = g_vg.norm();

    /* Position in winddome frame (like NED frame, but rotated about z such that x points towards wind) */
    Vector3 wd_xg = q_wd_g * g_xg;

    /* Local latitude/longitude on winddome sphere, north pole towards wind */
    lLatLon = {atan2(wd_xg(0), wd_xg.segment<2>(1).norm()),     // atan2 for safety (possible zero division)
               atan2(wd_xg(1), -wd_xg(2))};                     // atan2 for safety (possible zero division)
    debug.sphLatLon = lLatLon;

    /* Local NED frame */
    /* Quaternion q_l_wd = T2quat(-lLatLon(0)) * T1quat(lLatLon(1));
     * Quaternion q_l_g = q_l_wd * q_wd_g; */
    Quaternion q_lg = T2quat(-lLatLon(0)) * T1quat(lLatLon(1)) * q_wd_g;

    /* Geodetic velocity in local NED frame */
    Vector3 l_vg = q_lg * g_vg;
    getGammaChi(l_vg, l_gamma, l_chi);

    /* Local ground track frame (like local NED frame, but rotated about z such that x points towards the
     * projection of the geodetic velocity) */
    /* Quaternion q_lgk_l = T3quat(l_chi);
     * Quaternion q_lgk_g = q_lgk_l * q_lg; */
    q_lgk_g = T3quat(l_chi) * q_lg;
}

/** Guidance **/
void AweTurnController::selectWaypoint(const Vector2 &lLatLon, const Flightplan2WPs &flightplan,

                                       uint &activeWaypoint_id, Waypoint &activeWaypoint) {

    //        double lLat = lLatLon(0);
    double lLon = lLatLon(1);

    /** 2-Point switching **/
    if (activeWaypoint_id == 1 && lLon > flightplan.lonTh) { activeWaypoint_id = 2; }
    else if (activeWaypoint_id == 2 && lLon < -flightplan.lonTh) { activeWaypoint_id = 1; }

    activeWaypoint = flightplan.waypoints[activeWaypoint_id - 1];

    //        /** 4-Point switching **/
    //        /* Using one-based indexing for waypoint ID */
    //        if (flightplan.waypoints[1].lat > flightplan.waypoints[0].lat) {
    //            /* Second waypoint is above first waypoint -> Fly UPDWIND turns.
    //             *
    //             * REVERSE 4-Point control switching (AWE Book 2018 Ch. 7)
    //             * Note that the index is intentionally not zero-based here.
    //             *
    //             * */
    //            switch (activeWaypoint_id) {
    //                case 1:
    //                    if (lLon > flightplan.lonTh) activeWaypoint_id = 2;
    //                    break;
    //                case 2:
    //                    if (lLat > flightplan.latTh) activeWaypoint_id = 3;
    //                    break;
    //                case 3:
    //                    if (lLon < -flightplan.lonTh) activeWaypoint_id = 4;
    //                    break;
    //                case 4:
    //                    if (lLat > flightplan.latTh) activeWaypoint_id = 1;
    //                    break;
    //                default:
    //                    /* Should not occur, still take index 1 as default */
    //                    activeWaypoint_id = 1;
    //            }
    //
    //        } else {
    //            /* Second waypoint is below first waypoint -> Fly DOWNWIND turns.
    //             *
    //             * Normal 4-Point control switching (AWE Book 2018 Ch. 7)
    //             * Note that the index is intentionally not zero-based here.
    //             *
    //             * */
    //            switch (activeWaypoint_id) {
    //                case 1:
    //                    if (lLon > flightplan.lonTh) activeWaypoint_id = 2;
    //                    break;
    //                case 2:
    //                    if (lLat < flightplan.latTh) activeWaypoint_id = 3;
    //                    break;
    //                case 3:
    //                    if (lLon < -flightplan.lonTh) activeWaypoint_id = 4;
    //                    break;
    //                case 4:
    //                    if (lLat < flightplan.latTh) activeWaypoint_id = 1;
    //                    break;
    //                default:
    //                    /* Should not occur, still take index 1 as default */
    //                    activeWaypoint_id = 1;
    //            }
    //        }
}

void
AweTurnController::guideToWaypoint(const Vector2 &lLatLon, const double &l_chi, const Waypoint &activeWaypoint,

                                   double &l_chiErr) {

    /* delta (lat), lambda (lon) */
    double d1 = lLatLon(0);
    double l1 = lLatLon(1);

    double d2 = activeWaypoint.lat;
    double l2 = activeWaypoint.lon;

    /* Bearing for great circle path to WP
     * from https://www.movable-type.co.uk/scripts/latlong.html
     *
     * double y = sin(l2 - l1) * cos(d2);
     * double x = cos(d1) * sin(d2) - sin(d1) * cos(d2) * cos(l2 - l1);
     * double l_chiRef = atan2(y, x);
     */
    double l_chiRef = atan2(sin(l2 - l1) * cos(d2),
                            cos(d1) * sin(d2) - sin(d1) * cos(d2) * cos(l2 - l1));

    //        /* Exact distance to waypoint on sphere surface */
    //        double a = sin((d2 - d1) / 2.0) * sin((d2 - d1) / 2.0) +
    //                   cos(d1) * cos(d2) * sin((l2 - l1) / 2.0) * sin((l2 - l1) / 2.0);
    //        double distanceToWp = g_xg.norm() * 2.0 * atan2(sqrt(a), sqrt(1 - a));

    /* Approximation of distance to waypoint on sphere surface
     * Radian distance of dLat: dLat portion of whole circumference: dLat * r)
     * Radian distance of dLon: dLon portion of whole circumference at mean latitude's
     * radius: dLon * r * cos((lat1+lat2)/2)
     * Distance is approximately = diagonal of these two.
     * */
    double dAngleToWaypoint = Vector2(d2 - d1, (l2 - l1) * cos((d1 + d2) / 2.0)).norm();
    //        double distanceToWp = dAngleToWaypoint * p.teth_len;

    /* Shortest turn to chiRef */
    l_chiErr = getShortestAngleErr(l_chiRef, l_chi);

    /* Long-term parameter, not subject to change */
    const double l_chiErrMax = radians(90.0);

    /* If the reference point lies outside the a 2 * 90 deg window in front of the aircraft, force a
     * right/left turn, even if the other direction is shorter. */
    if (std::abs(l_chiErr) > l_chiErrMax && dAngleToWaypoint > radians(10.0)) {
        /* If turnaround is needed (active waypoint is far away and requires a turn of more than 90 deg) */

        if (activeWaypoint.turnDir == 1 && l_chiErr < 0) {
            l_chiErr += 2.0 * M_PI;
            /* Force right turn. l_chiErr is positive and might be greater than +180 deg. */

        } else if (activeWaypoint.turnDir == -1 && l_chiErr > 0) {
            l_chiErr -= 2.0 * M_PI;
            /* Force left turn. l_chiErr is negative and might be smaller than -180 deg. */
        }

    }

    debug.l_chiErr = l_chiErr;
}

/** Control **/
void AweTurnController::controlFlightOnSphere(const Quaternion &q_lgk_ned, const double &distToGs,
                                              const double &l_gamma, const double &Vg,
                                              const double &distOffsetRef, const double &l_chiErr,

                                              Vector3 &g_vgRef) {

    debug.distOffsetRef = distOffsetRef;
    debug.l_chiErr = l_chiErr;

    /** Path angle controller **/
    /* Y direction --------------------------------------------------------------------------- */
    /* Parameters:
     *
     * lgk_chiErr_max = 30 deg
     * This is the maximum local chi offset commanded. Normally, it will be applied at the beginning of a turn,
     * when the next waypoint lies behind the aircraft.
     *
     * l_chiErr_lim = 105 deg / 60 deg (with/without tether)
     * This is the maximum local chi difference taken into account. l_chiErr comes from the guidance and is the
     * difference in local chi angle to the active waypoint. It might be greater than 180 deg if the guidance
     * forces a turn to the opposite direction than the shortest direction, as required by the flight plan.
     * */
    const double l_chiErr_lim{p.control.l_chiErr_lim};
    const double chiErrPerchiErr{p.control.lgk_chiErr_max / l_chiErr_lim};

    double l_chiErr_constrained = constrain(l_chiErr, -l_chiErr_lim, l_chiErr_lim);

    double lgk_chiErr = chiErrPerchiErr * l_chiErr_constrained;

    /* Z direction --------------------------------------------------------------------------- */
    /* Distance error to nominal tether length.
     * Positive distance error means: Above the nominal sphere,
     * Negative: Below nominal sphere */
    double distErr = (p.teth_len + distOffsetRef) - distToGs;

    const double distErr_lim{20.0};
    const double gammaRefPerDistErr{radians(40.0) / distErr_lim};

    double distErr_constrained = constrain(distErr, -distErr_lim, distErr_lim);
    double lgk_gammaRef = gammaRefPerDistErr * distErr_constrained;

    double lgk_gammaErr_constrained = constrain(lgk_gammaRef - l_gamma, -radians(30.0), radians(30.0));

    /** Path angles reference **/
    /* Velocity reference is written in a frame which is rotated from the local ground track (lgk) frame
     * by the constrained kinematic angle references
     * Detailed code:
     * Quaternion q_lgkErr_lgk = T2quat(lgk_gammaErr_constrained) * T3quat(lgk_chiErr);
     * Quaternion q_lgkErr_g = q_lgkErr_lgk * q_lgk_g;
     * Quaternion q_g_lgkErr = q_lgkErr_g.inverse();
     * */
    Quaternion q_g_lgkErr = (T2quat(lgk_gammaErr_constrained) * T3quat(lgk_chiErr) * q_lgk_ned).inverse();
    g_vgRef = q_g_lgkErr * Vector3(Vg, 0, 0);
}
