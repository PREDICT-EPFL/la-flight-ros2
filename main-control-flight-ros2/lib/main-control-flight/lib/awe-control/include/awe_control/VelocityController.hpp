//
// Created by johannes on 07.04.20.
//

#ifndef SRC_VELOCITYCONTROLLER_H
#define SRC_VELOCITYCONTROLLER_H

#include "AweControlTools.hpp"

namespace awe_control2 {

struct VelocityControllerParameters {
    struct AttControlParams {
        double P_TC{0.4};
        double roll_lim{radians(70.0)};
        double pitch_lim{radians(70.0)};
    } attControl;

    double gamma_TC{0.4};
    double chi_TC{0.9};

    double gammaRate_k_ff{0.7};
    double gammaRate_k_p{0.8}; // TODO

    double chiRate_k_ff{1.1};
    double chiRate_k_p{0.7}; // TODO
};
struct VelocityControllerDebugValues {
    double gammaRateUsed{0};
    double chiRateUsed{0};

    double gammaRateUsed2{0};
    double chiRateUsed2{0};
};

class VelocityController {
public:
    VelocityController() = default;
    ~VelocityController() = default;

    VelocityControllerParameters p{};

    void reset() {
        gammaBuf.reset();
        chiBuf.reset();
    }
    void update(const ros::Time &timestamp, const Vector3 &g_vg) {

        getGammaChi(g_vg, gamma, chi, Vg);

        /* Write gamma and chi to their buffers to allow to filter them later */
//        double time_now = ros::Time::now().toSec();
        double stamp = timestamp.toSec();
        gammaBuf.write(stamp, gamma);
        chiBuf.write2(stamp, chi);
    }

    /* Differentiate gamma/chi to get gamma_dot/chi_dot */
    void controlVelocity(const Vector3 &g_vgRef, const double &pitchAng,

                         double &rollRef, double &pitchRef);

    /* Use transformed euler rates instead of gamma/chi differentiation */
    void controlVelocity(const Vector3 &g_vgRef, const double &pitchAng,
                         const double &rollAng, const Vector3 &bodyRates,

                         double &rollRef, double &pitchRef);

//    /* Wrap functions above and compensate wind before */
//    void controlVelocity(Vector3 g_vgRef, Vector3 g_vg, const Vector3 &g_vw,
//                         const double &pitchAng,
//
//                         double &rollRef, double &pitchRef);
//
//    void controlVelocity(Vector3 g_vgRef, Vector3 g_vg, const Vector3 &g_vw,
//                         const double &pitchAng, const double &rollAng,
//                         const Vector3 &bodyRates,
//
//                         double &rollRef, double &pitchRef);

    VelocityControllerDebugValues debug{};

    double gamma{0};
    double chi{0};
    double Vg{0};

private:
    PolyFilter<2,8> gammaBuf{};
    PolyFilter<1,8> chiBuf{};

    void controlKinematicRates(const double &gammaRateRef, const double &gammaRate,
                               const double &chiRateRef, const double &chiRate,
                               const double &pitchAng,

                               double &rollRef_constrained, double &pitchRef_constrained);

    double controlGammaRate(const double &gammaRateRef, const double &gammaRate);

    double controlChiRate(const double &chiRateRef, const double &chiRate,
                          const double &pitchAng, const double &V);

    void DEBUG_printAllParams();
};

}

#endif //SRC_VELOCITYCONTROLLER_H
