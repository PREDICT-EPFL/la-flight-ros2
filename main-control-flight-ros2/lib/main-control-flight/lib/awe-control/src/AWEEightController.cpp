//
// Created by johannes on 29.09.19.
//

#include <iostream>
#include "awe_control/AWEEightController.hpp"

namespace awee_control {

/* Parameters ======================================================================================================= */
    void AWEEightController::setParameters(const AWEEightController::Parameters &aweEightParameters) {

        p = aweEightParameters;

        p.windDir = getWindDir(p.windFrom_deg);
        T_wd_ned = T3(p.windDir);

        /* Update static flightplan and thresholds */
        m_flightPlanStart_deg = (Eigen::Matrix<double, 4, 3>() << p.guidance.wp1.getVect().transpose(),
                p.guidance.wp2.getVect().transpose(),
                p.guidance.wp3.getVect().transpose(),
                p.guidance.wp4.getVect().transpose()).finished();

        m_latLonThStart_deg = Vector2{p.guidance.wp_th_lat, p.guidance.wp_th_lon};
    }

    double AWEEightController::getWindDir(const double &windFrom_deg) {

        double windDir_deg = windFrom_deg + 180.0;

        if (windDir_deg >= 360.0) {

            windDir_deg -= 360.0;

        }

        return radians(windDir_deg);
    }

/* General help functions =========================================================================================== */
    template<class T>
    int AWEEightController::sign(T val) {
        return (T(0) < val) - (val < T(0));
    }

    bool AWEEightController::closeEnough(const double &f1, const double &f2) {
        return (std::abs(f1 - f2) < 0.001);
    }

    double AWEEightController::getShortestAngleErr(const double &angleRef, const double &angle) {

        double angleErr = angleRef - angle;

        if (angleErr > M_PI) { angleErr -= 2.0 * M_PI; }
        else if (angleErr < -M_PI) { angleErr += 2.0 * M_PI; }

        return angleErr;
    }

    double
    AWEEightController::constrain(const double &valueToConstrain, const double &lowerLimit, const double &upperLimit) {

        return fmin(fmax(valueToConstrain, lowerLimit), upperLimit);
    }

    double AWEEightController::degrees(const double &radians) {

        return radians * 180.0 / M_PI;
    }

    double AWEEightController::radians(const double &degrees) {

        return degrees * M_PI / 180.0;
    }

/* AWE help functions =========================================================================================== */
    double AWEEightController::getCircArc(const Vector3 &g_xg) {

        Vector3 wd_xg = T_wd_ned * g_xg;

        double circArg = atan2(-wd_xg(1), -wd_xg(0));
        if (circArg < 0.0) {
            circArg += 2.0 * M_PI;
        }

        return circArg;
    }

    void AWEEightController::getKinematicFrame(const Vector3 &g_vg,

                                               double &gamma, double &chi) {

        /* Calculate flight path angle and azimuth */
        gamma = atan2(-g_vg(2), g_vg.segment(0, 2).norm());
        chi = atan2(g_vg(1), g_vg(0));

        // Don't need 0 to 2pi range here, because we don't return gamma and chi, only orientation resulting from then.
    }

    void AWEEightController::getKinematicFrame(const Vector3 &g_vg,

                                               Matrix3 &Tkg) {

        double gamma{0};
        double chi{0};

        getKinematicFrame(g_vg,

                          gamma, chi);

        /* Calculate transformation matrix from geodaetic (NED) to kinematic frame */
        Tkg = T2(gamma) * T3(chi);

    }

    void AWEEightController::getKinematicFrame(const Vector3 &g_vg,

                                               double &gamma, double &chi, Matrix3 &Tkg) {

        getKinematicFrame(g_vg,

                          gamma, chi);

        /* Calculate transformation matrix from geodaetic (NED) to kinematic frame */
        Tkg = T2(gamma) * T3(chi);

    }

/* Elementary transformations ======================================================================================= */
    Matrix3 AWEEightController::T1(const double &angle) {

        return (Matrix3() << 1, 0, 0,
                0, cos(angle), sin(angle),
                0, -sin(angle), cos(angle)).finished();
    }

    Matrix3 AWEEightController::T2(const double &angle) {

        return (Matrix3() << cos(angle), 0, -sin(angle),
                0, 1, 0,
                sin(angle), 0, cos(angle)).finished();
    }

    Matrix3 AWEEightController::T3(const double &angle) {

        return (Matrix3() << cos(angle), sin(angle), 0,
                -sin(angle), cos(angle), 0,
                0, 0, 1).finished();
    }

/* Eight mode navigation and guidance =============================================================================== */
    void AWEEightController::navigation(const Vector3 &g_xg, const Vector3 &g_vg,

                                        Vector2 &presentLatLon, Matrix3 &Tlg, Matrix3 &Tlk_l,
                                        double &l_gamma, double &l_chi) {

        /* Kinematic frame
         * x: geodaetic velocity
         * y: RHS, parallel to horizon plane (NE plane)
         * z: Downwards, perpendicular to x
         * */

        /* Aircraft position in rotated wind dome NED frame */
        Vector3 wd_xg = T_wd_ned * g_xg;

        /* Local frame on sphere
         * local NED frame (letter 'l')
         * if tether sphere is a lying "earth ball" with wind blowing from north:
         * x: North (latitute on sphere)
         * y: East (longitude on sphere)
         * z: Down (to sphere center) */

        /* delta: earth latitude, positive lat = north, negative lat = south */
        double delta = atan2(-wd_xg(0), wd_xg.segment(1, 2).norm());

        /* lambda: earth longitude, positive lon = east, negative lon = west */
        double lambda = atan2(-wd_xg(1), -wd_xg(2));

        presentLatLon = Vector2(delta, lambda);

        Tlg = T2(-delta) * T1(lambda) * T3(M_PI) * T_wd_ned;

        Vector3 l_vg = Tlg * g_vg;

        /* Compute local gamma (flight path) and local chi (flight azimuth) (relative to tangential plane on sphere) */
        getKinematicFrame(l_vg,

                          l_gamma, l_chi, Tlk_l);

        /* Return by reference
         *
         * sphLatLon (rad), Tlg, l_gamma (rad), l_chi (rad), Tlk_l
         *
         * */
    }

    void AWEEightController::guidance_WpManagement(const Vector3 &g_xg, const double &Va,
                                                   bool state_TERS_active,

                                                   Eigen::Matrix<double, 4, 3> &flightPln_deg, Vector2 &latLonTh_deg) {

        Eigen::Matrix<double, 4, 1> latsStart;
        for (int i = 0; i < 4; ++i) {
            latsStart(i) = m_flightPlanStart_deg(i, 0);
        }

        double maxLatStart_deg = latsStart.maxCoeff();
        double minLatStart_deg = latsStart.minCoeff();

        double latShift_deg = 0;
        /* Total Energy Reference System (TERS) */
        if (state_TERS_active) {
            /* TERS ACTIVE*/

            double tetherLengthening = std::max(0.0, g_xg.norm() - p.teth_len);

            if (!m_last_TERS_active) {
                /* Set total energy reference if
                 * - it was never set before OR
                 * - the system has not run in the previous step */

                m_E_tot_ref = 0.5 * p.guidance.aircraftMass * Va * Va +
                              p.guidance.aircraftMass * M_ONE_G * -g_xg(2) +
                              0.5 * p.guidance.teth_c * tetherLengthening * tetherLengthening;
            }

            /* Present total energy */
            double E_tot = 0.5 * p.guidance.aircraftMass * Va * Va +
                           p.guidance.aircraftMass * M_ONE_G * -g_xg(2) +
                           0.5 * p.guidance.teth_c * tetherLengthening * tetherLengthening;

            /* Total energy ratio (adjusted by 1) */
//        double dEtot = std::pow(E_tot / m_E_tot_ref, 2.0) - 1.0;
            double dEtot = p.guidance.dEtotGain * (E_tot / m_E_tot_ref - 1);

            /* Initial latitude shift */
            latShift_deg = p.guidance.latShift_c * 2.0 * dEtot;

            /* Keep latitude within limits */
            latShift_deg = std::min(maxLatStart_deg + latShift_deg, p.guidance.lat_max) - maxLatStart_deg;
            latShift_deg = std::max(minLatStart_deg + latShift_deg, p.guidance.lat_min) - minLatStart_deg;

            m_last_TERS_active = true;

        } else {
            /* TERS INACTIVE*/

            // float dEtot = -9999;
            // latShift_deg = 0;
            m_last_TERS_active = false;

        }

//    /* Shift dramatically down if airspeed approaches stall speed */
//    if (Va < (p.guidance.Va_min + 5.0)) {
//
//        /* Shift 20 degrees down within the last 5 m/s airspeed before stall speed */
//        latShift_deg -= 20.0 * ((p.guidance.Va_min + 5.0) - Va) / 5.0;
//
//        latShift_deg = std::max(minLatStart_deg + latShift_deg, p.guidance.lat_min) - minLatStart_deg;
//    }

        /* Copy Longitude and TurnDirection from initial flightplan, shift latitude in function of Energy ratio */
        for (int i = 0; i < 4; ++i) {
            flightPln_deg(i, 0) = m_flightPlanStart_deg(i, 0) + latShift_deg;
            flightPln_deg(i, 1) = m_flightPlanStart_deg(i, 1);
            flightPln_deg(i, 2) = m_flightPlanStart_deg(i, 2);
        }

        /* Copy longitude from initial latLonThreshold, shift latitude in function of Energy ratio */
        latLonTh_deg(0) = m_latLonThStart_deg(0) + latShift_deg;
        latLonTh_deg(1) = m_latLonThStart_deg(1);

        /* Return by reference
         *
         * flightPln_deg, latLonTh_deg
         *
         * */
    }

    void
    AWEEightController::guidance_WpSwitching(const Eigen::Matrix<double, 4, 3> &flightPln_deg,
                                             const Vector2 &latLonTh_deg,
                                             const Vector2 &presentLatLon,

                                             uint &activeWpId, Vector3 &activeWp) {

        double presentLat = presentLatLon(0);
        double presentLon = presentLatLon(1);

        /* Convert thresholds from deg to rad */
        double latTh = radians(latLonTh_deg(0));
        double lonTh = radians(latLonTh_deg(1));

        if (flightPln_deg(1, 0) > flightPln_deg(0, 0)) {
            /* Second waypoint is above first waypoint -> Fly UPDWIND turns.
             *
             * REVERSE 4-Point control switching (AWE Book 2018 Ch. 7)
             * Note that the index is intentionally not zero-based here.
             *
             * */

            switch (activeWpId) {
                case 1:
                    if (presentLon > lonTh) activeWpId = 2;
                    break;
                case 2:
                    if (presentLat > latTh) activeWpId = 3;
                    break;
                case 3:
                    if (presentLon < -lonTh) activeWpId = 4;
                    break;
                case 4:
                    if (presentLat > latTh) activeWpId = 1;
                    break;
                default:
                    /* Should not occur, still take index 1 as default */
                    activeWpId = 1;
            }

        } else {
            /* Second waypoint is below first waypoint -> Fly DOWNWIND turns.
             *
             * Normal 4-Point control switching (AWE Book 2018 Ch. 7)
             * Note that the index is intentionally not zero-based here.
             *
             * */

            switch (activeWpId) {
                case 1:
                    if (presentLon > lonTh) activeWpId = 2;
                    break;
                case 2:
                    if (presentLat < latTh) activeWpId = 3;
                    break;
                case 3:
                    if (presentLon < -lonTh) activeWpId = 4;
                    break;
                case 4:
                    if (presentLat < latTh) activeWpId = 1;
                    break;
                default:
                    /* Should not occur, still take index 1 as default */
                    activeWpId = 1;
            }

        }

        // Assign activeWP
        activeWp = flightPln_deg.row(static_cast<size_t>(activeWpId - 1)).transpose();
        activeWp(0) = radians(activeWp(0));
        activeWp(1) = radians(activeWp(1));

        /* Return by reference
         *
         * activeWPid, activeWP
         *
         */
    }

    void AWEEightController::guidance_chiRef(const Vector3 &g_xg, const Vector3 &activeWp, const Vector2 &presentLatLon,
                                             const double &l_chi,

                                             double &l_chiRef, double &l_chiErr, double &distanceToWp) {

        /* activeWp contains latitude (rad), longitude (rad), direction of turn (-1/0/1) */

        /* delta (lat), lambda (lon) */
        double d1 = presentLatLon(0);
        double l1 = presentLatLon(1);

        double d2 = activeWp(0);
        double l2 = activeWp(1);

        /* Bearing for great circle path to WP
         * from https://www.movable-type.co.uk/scripts/latlong.html
         */
        double y = sin(l2 - l1) * cos(d2);
        double x = cos(d1) * sin(d2) - sin(d1) * cos(d2) * cos(l2 - l1);
        l_chiRef = atan2(y, x);
        //if (l_chiRef < 0.0) { l_chiRef += 2.0 * M_PI; } // Range from 0 to 2pi

        /* Distance to waypoint on sphere surface */
        double a = sin((d2 - d1) / 2.0) * sin((d2 - d1) / 2.0) +
                   cos(d1) * cos(d2) * sin((l2 - l1) / 2.0) * sin((l2 - l1) / 2.0);
        distanceToWp = g_xg.norm() * 2.0 * atan2(sqrt(a), sqrt(1 - a));

        /* Shortest turn to chiRef */
        l_chiErr = getShortestAngleErr(l_chiRef, l_chi);

        /* Long-term parameter, not subject to change */
        const double l_chiErrMax = radians(90.0);

        if (std::abs(l_chiErr) > l_chiErrMax) {
            /* If turnaround is needed (turn more than 90 deg) */

            if (closeEnough(1.0, activeWp(2)) && (l_chiErr < 0.0)) {
                /* Force right turn, l_chiErr ist positive and might be greater than +180 deg. */
                l_chiErr += 2.0 * M_PI;

            } else if (closeEnough(-1.0, activeWp(2)) && (l_chiErr > 0.0)) {
                /* Force left turn, l_chiErr ist negative and might be smaller than -180 deg. */
                l_chiErr -= 2.0 * M_PI;
            }

        }

        /* Return by reference
         *
         * l_chiRef, l_chiErr, distanceToWp
         *
         */
    }

    void AWEEightController::guidance_accRef(const Vector3 &g_xg, const Vector3 &g_vg, const double &Va,
                                             const Matrix3 &Tlg, const Matrix3 &Tlk_l,
                                             const double &l_chi, const double &l_chiErr,
                                             const double &l_gamma, const double &distanceToWp,

                                             Vector3 &g_lfRef, double &gammaRef, double &chiRef) {

        Matrix3 Tgl = Tlg.transpose();
        Matrix3 Tl_lk = Tlk_l.transpose();

        Vector3 l_vg = Tlg * g_vg;

        /* Demanded acceleration in local kinematic frame */
        /* X ------------------------------------------------------------------- */
        double lk_accX = 0.0;   // TODO (long-term): Does it help if this is optimized != 0 ?

        /* Y ------------------------------------------------------------------- */
        double lk_accY;

        //    /* Acceleration at full turn */
        //    double acc_At_MaxchiErr = g_vg.norm() * g_vg.norm() / p.guidance.radius_min;
        //
        //    /* chiErr Threshold - above: Full acceleration, below: scaled acceleration */
        //    double l_chiErrTH = radians(45.0);
        //
        //    if (std::abs(l_chiErr) > l_chiErrTH) {
        //
        //        /* Use max acceleration */
        //        lk_accY = sign(l_chiErr) * acc_At_MaxchiErr;
        //
        //    } else {
        //
        //        /* Scale down max acceleration */
        //        lk_accY = l_chiErr / l_chiErrTH * acc_At_MaxchiErr;
        //    }

        /* L1 controller as proposed in Park et al.: 'A New Nonlinear Guidance Logic for Trajectory Tracking'
         * L1 is limited to avoid getting turns infinitely soft in infinite distance to waypoint. */
        /* L1 controller using virtual minimal distance to waypoint */
        double l1_max = g_xg.norm() * 1.1133 / 4.0;

        double l_chiErr_constrained = constrain(l_chiErr, radians(-90.0), radians(90.0));

        double l_Vg_xy = l_vg.segment(0, 2).norm();
        lk_accY = 2 * l_Vg_xy * l_Vg_xy / std::min(l1_max, distanceToWp) * sin(l_chiErr_constrained);

        /* Z ------------------------------------------------------------------- */
        /* 1. Feedforward term
         * Assume circular flight at radius of nominal tether length */
//    double ff = l_vg.segment(0, 2).norm() * l_vg.segment(0, 2).norm() / p.teth_len;
//    double ff = l_vg.norm() * l_vg.norm() / g_xg.norm();
        double ff = l_vg.norm() * l_vg.norm() / p.teth_len;

        /* 2. Flight path angle in local frame */
//    double l_gamma_ref = radians(0.0);
        /* Push l_gamma_ref down when leaving the sphere */
        double l_gamma_ref =
                radians(-30.0) / p.guidance.sph_offset * constrain(g_xg.norm() - p.teth_len, 0, p.guidance.sph_offset);
        std::cout << "overshoot: " << constrain(g_xg.norm() - p.teth_len, 0, p.guidance.sph_offset) <<
                  "ref: " << degrees(l_gamma_ref) << " deg \n";

        /* l_gamma error and resulting acceleration ref */
        double l_gamma_error = l_gamma_ref - l_gamma;
        double gammaCorr = -1.0 / p.control.P_TC * l_gamma_error * std::max(Va, p.Va_min);

        /* Sum up terms */
        double lk_accZ = ff + gammaCorr;

        /* Load factor ref in local kinematic frame */
        Vector3 lk_lfRef(lk_accX, lk_accY, lk_accZ);
        lk_lfRef /= M_ONE_G;

        /* Load factor ref in geodaetic frame, including g compensation */
        g_lfRef = Tgl * Tl_lk * lk_lfRef + Vector3(0, 0, -1);

        /* Create kinematic angles references in local tangential frame */
        double l_gammaDiff = -lk_accZ * p.control.P_TC / std::max(Va, p.Va_min);
        double l_gammaRef = l_gamma + l_gammaDiff;
        std::cout << "l_gammaRef: " << degrees(l_gammaRef) << " deg \n";

        /* l_chiRef from l_chiErr_constrained */
        double l_chiRef = l_chi + l_chiErr_constrained;

        /* Velocity direction reference vector from l_chiRef and l_gammaRef (Length 1) */
        Vector3 l_vRef(cos(l_gammaRef) * cos(l_chiRef), cos(l_gammaRef) * sin(l_chiRef), -sin(l_gammaRef));

        /* Tranform velocity vector to geodetic (NED) frame */
        Vector3 g_vRef = Tgl * l_vRef;

        /* Usual kinematic angles reference */
        getKinematicFrame(g_vRef,

                          gammaRef, chiRef);

        /* Return by reference
         *
         * g_lfRef, gammaRef, chiRef
         *
         */
    }

    void AWEEightController::highLevelControl_rollPitchRef(const Vector3 &g_vg,
                                                           const double &gammaRef, const double &chiRef,
                                                           const double &pitchAng,

                                                           double &rollRef, double &pitchRef) {

        double gamma;
        double chi;

        getKinematicFrame(g_vg,

                          gamma, chi);

        /* Control flight path angle gamma with pith angle ---------------------------------------- */
        double gammaError = gammaRef - gamma;

        const double gammaError_limit = radians(60.0);
        double gammaError_constrained = constrain(gammaError, -gammaError_limit, gammaError_limit);

        /* Proportional dPitch angle (delta Pitch) */
        double dPitch = 0.5 * gammaError_constrained;

        /* Pitch ref = present pitch angle + delta pitch angle (constrained to reasonable angles) */
        const double pitchRef_limit = radians(70.0);
        pitchRef = constrain(pitchAng + dPitch, -pitchRef_limit, pitchRef_limit);

        /* Control flight path azimuth with roll angle -------------------------------------------- */
        double chiError = getShortestAngleErr(chiRef, chi);

        const double chiError_limit = radians(135.0);
        double chiError_constrained = constrain(chiError, -chiError_limit, chiError_limit);

        /* Scale constrained error to roll ref:
         * at chiError_limit, roll ref is rollRefAtMaxChiError */
        const double rollRefAtMaxChiError = radians(50.0);

        rollRef = chiError_constrained / chiError_limit * rollRefAtMaxChiError;

        /* Return by reference
         *
         * rollRef, pitchRef
         *
         */
    }

/* Acceleration based control ======================================================================================= */
    void AWEEightController::controlLoadFactor_rollAngProt(const double &angErr, const double &rollAng,

                                                           double &angErr_limited) {

        double rollAngLim = radians(p.control.R_lim);

        /* Roll angle setpoint */
        double rollRef = angErr + rollAng;

        /* Limit roll angle to rollAngLim */
        double rollRef_limited = constrain(rollRef, -rollAngLim, rollAngLim);

        /* Roll angle error */
        angErr_limited = rollRef_limited - rollAng;

        /* Return by reference
         *
         * angErr_limited
         *
         */
    }

    void AWEEightController::controlLoadFactor_pitchAngProt(const double &pitchRate, const double &pitchAng,

                                                            double &pitchRate_limited) {

        double pitchAngLim_max = radians(p.control.P_lim_max);
        double pitchAngLim_min = radians(p.control.P_lim_min);

        /* Limit pitch angle - set pitchRate to zero if pitchAng is high and
        pitchRate would lead to higher pitchAngle. PitchRate in opposite
        direction is allowed. */

        if (pitchAng > (pitchAngLim_max - radians(5.0))) {

            double scaler = std::min(1.0, std::max(0.0, -1.0 / radians(5.0) * (pitchAng - pitchAngLim_max)));
            pitchRate_limited = scaler * pitchRate;

        } else if (pitchAng < (pitchAngLim_min + radians(5.0f))) {

            double scaler = std::min(1.0, std::max(0.0, +1.0 / radians(5.0) * (pitchAng - pitchAngLim_min)));
            pitchRate_limited = scaler * pitchRate;

        } else {

            pitchRate_limited = pitchRate;

        }

        /* Return by reference
         *
         * pitchRate_limited
         *
         */
    }

    void
    AWEEightController::controlLoadFactor_qComponent(const double &lfErr, const double &measAng, const double &rollAng,

                                                     double &lfErr_q) {

        double smallerAng, largerAng;
        // Find larger and smaller angle of the two
        if (rollAng < measAng) {

            smallerAng = rollAng;
            largerAng = measAng;

        } else {

            smallerAng = measAng;
            largerAng = rollAng;

        }

        // Angle difference
        double angDiff = largerAng - smallerAng;

        // If angle difference is larger than 180°, take the inverse angle
        if (angDiff > radians(180.0)) {

            angDiff = radians(360.0) - angDiff;

        }

        // Limit angDiff to 90 degrees, where cosine is zero
        angDiff = std::min(angDiff, radians(90.0));

        // Component of lfErr which lies in the body-fixed z direction.
        lfErr_q = lfErr * cos(angDiff);

        /* Return by reference
         *
         * lfErr_q
         *
         */
    }

    void AWEEightController::vecXYZ_to_YZpolar(const Vector3 &vec,

                                               double &ang, double &val) {

        ang = atan2(vec(1), -vec(2));
        val = Vector2(vec(1), vec(2)).norm();

    }

/* Public functions ================================================================================================= */
    void AWEEightController::getLoadFactorSetpoints(const Vector3 &g_xg, const Vector3 &g_vg, const double &Va,
                                                    const double &pitchAng,
                                                    const bool state_TERS_active,

                                                    uint &activeWpId, Vector3 &g_lfRef, double &rollRef,
                                                    double &pitchRef) {

        /* Get present position of aircraft relative to sphere */
        Vector2 presentLatLon;
        Matrix3 Tlg;
        Matrix3 Tlk_l;
        double l_gamma{0};
        double l_chi{0};

        navigation(g_xg, g_vg,

                   presentLatLon, Tlg, Tlk_l, l_gamma, l_chi);

        /* Modify flightplan in function of energy in the system */
        Eigen::Matrix<double, 4, 3> flightPlan_deg;
        Vector2 latLonTh_deg;

        guidance_WpManagement(g_xg, Va, state_TERS_active,

                              flightPlan_deg, latLonTh_deg);

        /* Get active waypoint to approach */
        Vector3 activeWp(0, 0, 0);

        guidance_WpSwitching(flightPlan_deg, latLonTh_deg, presentLatLon,

                             activeWpId, activeWp);

        /* Get track towards active waypoint */
        double l_chiRef{0};
        double l_chiErr{0};
        double distanceToWp{0};

        guidance_chiRef(g_xg, activeWp, presentLatLon, l_chi,

                        l_chiRef, l_chiErr, distanceToWp);

        /* Get desired acceleration */
        double gammaRef{0};
        double chiRef{0};
        guidance_accRef(g_xg, g_vg, Va, Tlg, Tlk_l,
                        l_chi, l_chiErr, l_gamma, distanceToWp,

                        g_lfRef, gammaRef, chiRef);

        /* Get roll and pitch ref */
        highLevelControl_rollPitchRef(g_vg, gammaRef, chiRef, pitchAng,

                                      rollRef, pitchRef);

        /* Debug */
        debug.sphLatLon = presentLatLon;
        debug.l_chiErr = l_chiErr;

        /* Return by reference
         *
         * g_lfRef, gammaRef, chiRef
         *
         */
    }

    void AWEEightController::getRateSetpoints(const Vector3 &g_lfRef, const Vector3 &g_vg, const double &Va,
                                              const Vector3 &att, const Vector3 &g_accMeas,

                                              double &rollRate_sp, double &pitchRate_sp) {

        /* Get kinematic frame transformation matrix */
        Matrix3 Tkg;
        getKinematicFrame(g_vg,

                          Tkg);

        /* Transform to kinematic frame */
        Vector3 k_lfRef = Tkg * g_lfRef;
        Vector3 k_lfMeas = Tkg * g_accMeas / M_ONE_G;

        /* Perpendicular components to k_X direction of lfRef and lfMeas in polar coordinates */
        double refAng, refLf, measAng, measLf;

        vecXYZ_to_YZpolar(k_lfRef,

                          refAng, refLf);

        /* Protect against references which demand to pitch down more than free fall, which would lead to
         * oscillating behavior in bank movement */
        if (refAng > radians(170.0)) {

            refAng = 0.0;
            refLf = 0.0;

        }

        vecXYZ_to_YZpolar(k_lfMeas,

                          measAng, measLf);

        /* Errors: Angle error and load factor error */
        double angErr = refAng - measAng;
        double lfErr = refLf - measLf;

        /* Roll angle protection */
        double angErr_limited;
        controlLoadFactor_rollAngProt(angErr, att(0),

                                      angErr_limited);

        /* Use only q component of lfErr */
        double lfErr_q{0};
        controlLoadFactor_qComponent(lfErr, measAng, att(1),

                                     lfErr_q);

        /* Generate rate setpoints */
        rollRate_sp = angErr_limited / p.control.R_TC;
        double pitchRate = lfErr_q * M_ONE_G / std::max(Va, p.Va_min);

        /* Pitch angle protection */
        double pitchRate_limited{0};
        controlLoadFactor_pitchAngProt(pitchRate, att(1),

                                       pitchRate_limited);

        pitchRate_sp = pitchRate_limited;

        /* Return by reference
         *
         * roll_rate_sp (rad/s), pitch_rate_sp (rad/s)
         *
         */

        /* Return by reference
         *
         * rollrate_sp, pitchrate_sp, thrust_sp
         *
         */
    }

    bool AWEEightController::getSwitchToMainMode(const Vector3 &g_xg) {

        double circArg = getCircArc(g_xg);

        /* Distance to ground station less than 60 % of tether length or
         * height below 40 meters */
        bool result = (g_xg.norm() > (0.9 * p.teth_len)) &&
                      (circArg > radians(135.0)) && (circArg < radians(225.0));

        if (result)
            return true;
        else
            return false;
    }

    bool AWEEightController::getSwitchToClimb(const Vector3 &g_xg) {

        /* Distance to ground station above 95 % of tether length and
         * position on climb circle suitable to enter eight */
        bool result = (g_xg.norm() < (0.6 * p.teth_len)) ||
                      (-g_xg(2) < 40.0);
        if (result)
            return true;
        else
            return false;
    }

    void AWEEightController::valueTesting() {

        p.windDir = getWindDir(135.0);
        T_wd_ned = T3(p.windDir);

        /* Update static flightplan and thresholds */
        m_flightPlanStart_deg = (Eigen::Matrix<double, 4, 3>() << p.guidance.wp1.getVect().transpose(),
                p.guidance.wp2.getVect().transpose(),
                p.guidance.wp3.getVect().transpose(),
                p.guidance.wp4.getVect().transpose()).finished();

        m_latLonThStart_deg = Vector2{p.guidance.wp_th_lat, p.guidance.wp_th_lon};

        /* Insert states from Matlab here (copy/paste): ================================================================= */
        Vector3 g_xg(-17.9486, -54.1904, -140.1406);

        Vector3 g_vg(-6.2984, -10.8705, 4.1707);

        double Va = 14.3866;

        Vector3 att(0.2005, -0.2840, -2.7852);

        /* ============================================================================================================== */

        //climb_getLoadFactorSetpoints(g_xg, g_vg, Va, g_lfRef);
        m_last_TERS_active = true;
        m_E_tot_ref = 2364.832097;

        Vector3 g_lfRef;
        double roll_sp{0}, pitch_sp{0};
        uint activeWPid{1}; // Range: 1 - 4

        /* Get load factor reference */
        getLoadFactorSetpoints(g_xg, g_vg, Va, att(1), false,

                               activeWPid, g_lfRef, roll_sp, pitch_sp);

        //std::cout << g_lfRef << std::endl;

//    double rr_sp;
//    double pr_sp;
//    double thrust_sp;
//
//    getRateSetpoints(g_lfRef, g_vg, Va, att, Tfg.transpose() * acc,
//
//            rr_sp, pr_sp);

        //std::cout << Vector3(rr_sp, pr_sp, thrust_sp) << std::endl;

        double test = 1;
    }

}

