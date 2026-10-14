//
// Created by johannes on 07.04.20.
//

#ifndef SRC_AWETURNCONTROLLER_H
#define SRC_AWETURNCONTROLLER_H

#include "AweControlTools.hpp"

namespace awe_control2 {

/* Types */
enum TurnDirection {
    UP = -1,
    SHORTEST = 0,
    DOWN = 1
};

struct Waypoint {
    double lat{0};
    double lon{0};
    int turnDir{0};

    Waypoint() = default;

    Waypoint(const double &latInit, const double &lonInit, const int &turnDirInit) :
            lat(latInit), lon(lonInit), turnDir(turnDirInit) {}

    Vector3 getVect() { return (Vector3{lat, lon, static_cast<double>(turnDir)}); }
};

struct Flightplan2WPs {
    Waypoint waypoints[2]{};
    double lonTh{};

    Flightplan2WPs(const Waypoint &wp1Init, const Waypoint &wp2Init,
                   const double &lonThInit) {
        waypoints[0] = wp1Init;
        waypoints[1] = wp2Init;

        lonTh = lonThInit;
    }
};


/** AWE Eight Controller **/
struct AweTurnParameters {
    double windFrom{radians(180.0)};
    double teth_len{150.0};

    double flightplan_wpDeclination{radians(-10.0)};
    double flightplan_wpLon{radians(20.0)};
    double flightplan_thLon{radians(18.0)};
    int flightplan_turnDir{DOWN};

    Flightplan2WPs flightplan{{radians(-10.0), radians(20.0), -1},
                              {radians(-10.0), radians(-20.0), 1},
                              radians(18.0)};

    struct ControlParameters {
        double lgk_chiErr_max{radians(30.0)};
        double l_chiErr_lim{radians(60.0)};

        //double dGammaRef_dDistErr{radians(2.0)};
        double distOffsetRef{0.0};
    } control;
};
struct AweTurnDebugValues {
    Vector2 sphLatLon{0, 0};
    double distOffsetRef{0};
    double l_chiErr{0};
};

class AweTurnController {
public:
    AweTurnController() = default;
    ~AweTurnController() = default;


    AweTurnParameters p{};
    void updateWinddome();
    void updateParameterDependencies();

    bool getSwitchToMainMode(const Vector3 &g_xg);
    bool getSwitchToClimb(const Vector3 &g_xg);


    /* Turn mode: CAS (control augmentation system): l_chiErr from pilot's manual command
     * Eight mode: Autopilot: l_chiErr from guidance to waypoint */
    void turnMode_getVelocityRef(const Vector3 &g_xg, const Vector3 &g_vg, const double &l_chiErr_commanded,

                                 Vector3 &g_vgRef);

    void eightMode_getVelocityRef(const Vector3 &g_xg, const Vector3 &g_vg, const double &distOffsetRef,

                                  uint &activeWpId, Vector3 &g_vgRef);

    AweTurnDebugValues debug{};

private:
    Quaternion q_wd_g{T3quat(p.windFrom)};

    /** Navigation **/
    void navigateOnSphere(const Vector3 &g_xg, const Vector3 &g_vg,

                          Vector2 &lLatLon, double &distToGs, Quaternion &q_lgk_g,
                          double &l_gamma, double &l_chi, double &Vg);

    /** Guidance **/
    void selectWaypoint(const Vector2 &lLatLon, const Flightplan2WPs &flightplan,

                        uint &activeWaypoint_id, Waypoint &activeWaypoint);

    void guideToWaypoint(const Vector2 &lLatLon, const double &l_chi, const Waypoint &activeWaypoint,

                         double &l_chiErr);

    /** Control **/
    void controlFlightOnSphere(const Quaternion &q_lgk_ned, const double &distToGs,
                               const double &l_gamma, const double &Vg,
                               const double &distOffsetRef, const double &l_chiErr,

                               Vector3 &g_vgRef);
};

}

#endif //SRC_AWETURNCONTROLLER_H
