//
// Created by johannes on 07.04.20.
//

#include "awe_control/AweCircleController.hpp"

using namespace awe_control2;

/* Public */
void AweCircleController::updateWinddome() { q_wd_g = T3quat(p.windFrom); }
void AweCircleController::updateParameterDependencies() {

    updateWinddome();
    p.circle.heightRef = std::sqrt(p.teth_len * p.teth_len - p.circle.radiusRef * p.circle.radiusRef);

//    averageRadius.reset();
//    averageRadius.setInitValue(p.circle.radiusRef);

    movAvgRadius.init(p.circle.radiusRef);
}

void AweCircleController::getVelocityRef(const Vector3 &g_xg, const Vector3 &g_vg,
                                         const double &distOffsetRef,

                                         Vector3 &g_vgRef) {

    //DEBUG_printAllParams();

    Quaternion q_cp_g;
    Vector3 cp_xg, cp_vg;
    double cp_gamma, cp_chi, Vg;

    navigate_circlePath(g_xg, g_vg,
                        q_cp_g, cp_xg, cp_vg, cp_gamma, cp_chi, Vg);

    control_circlePath(q_cp_g, cp_xg, cp_gamma, cp_chi, Vg, distOffsetRef,
                       g_vgRef);
}

/* Private */
/** Navigation **/
void AweCircleController::navigate_circlePath(const Vector3 &g_xg, const Vector3 &g_vg,

                                              Quaternion &q_cp_g, Vector3 &cp_xg, Vector3 &cp_vg,
                                              double &cp_gamma, double &cp_chi, double &Vg) {

    /* Circle frame (Like winddome frame, but declined by circle declination, x upwards and towards wind)
     * Detailed code:
     * Quaternion q_c_wd = T2quat(p.circle.declination);
     * Quaternion q_cg = q_c_wd * q_wd_g;
     * */
    Quaternion q_cg = T2quat(p.circle.declination) * q_wd_g;

    Vector3 c_xg = q_cg * g_xg;

    double c_bearing = getBearingFromCenter(c_xg.segment<2>(0));
    debug.circArg = c_bearing;

    /* Circle Path frame (X tangential to circle, Y towards circle center, Z down)
     * Detailed code:
     * Quaternion q_cp_c = T3quat(c_bearing + M_PI_2);
     * q_cp_g = q_cp_c * q_cg;
     * */
    q_cp_g = T3quat(c_bearing + M_PI_2) * q_cg;

    /* Position and velocity in circle path frame */
    cp_xg = q_cp_g * g_xg + Vector3(0, p.circle.radiusRef, p.circle.heightRef);
    cp_vg = q_cp_g * g_vg;

    cp_gamma = atan2(-cp_vg(2), cp_vg.segment<2>(0).norm()); // atan2 for safety (possible zero division)
    cp_chi = atan2(cp_vg(1), cp_vg(0));
    Vg = g_vg.norm();
}

/** Control **/
void AweCircleController::control_circlePath(const Quaternion &q_cp_g, const Vector3 &cp_xg,
                                             const double &cp_gamma, const double &cp_chi, const double &Vg,
                                             const double &distOffsetRef,

                                             Vector3 &g_vgRef) {

    debug.distOffsetRef = distOffsetRef;

    const double cp_angleErr_lim = radians(30.0);

    debug.cp_posErrY = cp_xg(1);
    debug.cp_posErrZ = cp_xg(2);

    /* Y */
    const double cp_xgY_lim{20.0};
    const double chiRefPerY{-radians(45.0) / cp_xgY_lim};

    double cp_xgY_constrained = constrain(cp_xg(1), -cp_xgY_lim, cp_xgY_lim);
    double cp_chiRef = chiRefPerY * cp_xgY_constrained;
    //cp_chiRef = atan2(-cp_xg(1),  0.8 * Vg );

    double cp_chiErr_constrained = constrain(cp_chiRef - cp_chi, -cp_angleErr_lim, cp_angleErr_lim);

    /* Z */
    double heightNew = sqrt(std::pow(p.teth_len + distOffsetRef, 2) - std::pow(p.circle.radiusRef, 2));
    double heightOffsetRef = heightNew - p.circle.heightRef;

    double heightErr = heightOffsetRef + cp_xg(2);

    const double heightErr_lim{20.0};
    const double gammaRefPerHeightError{radians(40.0) / heightErr_lim};

    double heightErr_constrained = constrain(heightErr, -heightErr_lim, heightErr_lim);
    double cp_gammaRef = gammaRefPerHeightError * heightErr_constrained;

    double cp_gammaErr_constrained = constrain(cp_gammaRef - cp_gamma, -cp_angleErr_lim, cp_angleErr_lim);

    /* Compute velocity ref
     * Detailed code:
     * Quaternion q_cperr_cp = T2quat(cp_gammaErr_constrained) * T3quat(cp_chiErr_constrained);
     * Quaternion q_cperr_g = q_cperr_cp * q_cp_g;
     * Quaternion q_g_cperr = q_cperr_g.inverse();
     * */
    Quaternion q_g_cperr = (T2quat(cp_gammaErr_constrained) * T3quat(cp_chiErr_constrained) * q_cp_g).inverse();
    g_vgRef = q_g_cperr * Vector3(Vg, 0, 0);
}