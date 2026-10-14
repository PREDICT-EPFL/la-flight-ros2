//
// Created by johannes on 07.04.20.
//

#include "awe_control/VelocityController.hpp"

using namespace awe_control2;

/* Public */
/* Differentiate gamma/chi to get gamma_dot/chi_dot */
void VelocityController::controlVelocity(const Vector3 &g_vgRef, const double &pitchAng,

                                         double &rollRef, double &pitchRef) {

    double gammaRef, chiRef;
    getGammaChi(g_vgRef, gammaRef, chiRef);

    double gammaRateRef = (gammaRef - gamma) / p.gamma_TC;
    double chiRateRef = getShortestAngleErr(chiRef, chi) / p.chi_TC;

    double gammaRate = (gammaBuf.isInitialized()) ? gammaBuf.filterDerivative() : gammaRateRef;
    double chiRate = (chiBuf.isInitialized()) ? chiBuf.filterDerivative() : chiRateRef;

    debug.gammaRateUsed = gammaRate;
    debug.chiRateUsed = chiRate;

    controlKinematicRates(gammaRateRef, gammaRate, chiRateRef, chiRate, pitchAng,
                          rollRef, pitchRef);
}

/* Use transformed euler rates instead of gamma/chi differentiation */
void VelocityController::controlVelocity(const Vector3 &g_vgRef, const double &pitchAng,
                                         const double &rollAng, const Vector3 &bodyRates,

                                         double &rollRef, double &pitchRef) {

    double gammaRef, chiRef;
    getGammaChi(g_vgRef, gammaRef, chiRef);

    double gammaRateRef = (gammaRef - gamma) / p.gamma_TC;
    double chiRateRef = getShortestAngleErr(chiRef, chi) / p.chi_TC;

    /* Compute euler rates from body rates (kinematic differential equation) */
    double pitch_dot = bodyRates(1) * cos(rollAng) - bodyRates(2) * sin(rollAng);
    double yaw_dot = (bodyRates(1) * sin(rollAng) + bodyRates(2) * cos(rollAng)) / cos(pitchAng);

    debug.gammaRateUsed = pitch_dot;
    debug.chiRateUsed = yaw_dot;

    controlKinematicRates(gammaRateRef, pitch_dot, chiRateRef, yaw_dot, pitchAng,
                          rollRef, pitchRef);
}

///* Wrap functions above and compensate wind before */
//void VelocityController::controlVelocity(Vector3 g_vgRef, Vector3 g_vg, const Vector3 &g_vw,
//                                         const double &pitchAng,
//
//                                         double &rollRef, double &pitchRef) {
//
//    /* Transform velocities to moving air field */
//    g_vgRef -= g_vw;
//    g_vg -= g_vw;
//
//    /* Work in aerodynamic values from now on */
//    controlVelocity(g_vgRef, g_vg, pitchAng,
//                    rollRef, pitchRef);
//}
//
//void VelocityController::controlVelocity(Vector3 g_vgRef, Vector3 g_vg, const Vector3 &g_vw,
//                                         const double &pitchAng, const double &rollAng,
//                                         const Vector3 &bodyRates,
//
//                                         double &rollRef, double &pitchRef) {
//
//    /* Transform velocities to moving air field */
//    g_vgRef -= g_vw;
//    g_vg -= g_vw;
//
//    /* Work in aerodynamic values from now on */
//    controlVelocity(g_vgRef, g_vg, rollAng, pitchAng, bodyRates,
//                    rollRef, pitchRef);
//}

/* Private */
void VelocityController::controlKinematicRates(const double &gammaRateRef, const double &gammaRate,
                                               const double &chiRateRef, const double &chiRate,
                                               const double &pitchAng,

                                               double &rollRef_constrained, double &pitchRef_constrained) {

    /* Compute pitch_dot ref and inverse what will happen in attitude controller.
     * By the inversion, we actually give pitch_dot_ref to the attitude controller, in place of pitchRef. */
    double pitch_dot_ref = controlGammaRate(gammaRateRef, gammaRate);
    double pitchRef = pitch_dot_ref * p.attControl.P_TC + pitchAng;
    pitchRef_constrained = constrain(pitchRef, -p.attControl.pitch_lim, p.attControl.pitch_lim);

    double rollRef = controlChiRate(chiRateRef, chiRate, pitchAng, Vg);
    rollRef_constrained = constrain(rollRef, -p.attControl.roll_lim, p.attControl.roll_lim);
}

double VelocityController::controlGammaRate(const double &gammaRateRef, const double &gammaRate) {

    /* Return pitch_dot_ref = feedforward + P portion */
    return p.gammaRate_k_ff * gammaRateRef + p.gammaRate_k_p * (gammaRateRef - gammaRate);
}

double VelocityController::controlChiRate(const double &chiRateRef, const double &chiRate,
                                          const double &pitchAng, const double &V) {

    /* Return rollRef = feedforward (model-based) + P portion */
    double roll_ff = atan(chiRateRef * V / (M_ONE_G * cos(pitchAng)));
    return p.chiRate_k_ff * roll_ff + p.chiRate_k_p * (chiRateRef - chiRate);
}

void VelocityController::DEBUG_printAllParams() {

    std::cout << "[ VelocityController: DEBUG_printAllParams ]\n\n";

    std::cout << "AWEE_FPC_GAM_TC: " << p.gamma_TC << "\n"
              << "AWEE_FPC_CHI_TC: " << p.chi_TC << "\n"

              << "AWEE_FPC_GAMR_FF: " << p.gammaRate_k_ff << "\n"
              << "AWEE_FPC_GAMR_P: " << p.gammaRate_k_p << "\n"

              << "AWEE_FPC_CHIR_FF: " << p.chiRate_k_ff << "\n"
              << "AWEE_FPC_CHIR_P: " << p.chiRate_k_p << "\n"
              << "\n";
}