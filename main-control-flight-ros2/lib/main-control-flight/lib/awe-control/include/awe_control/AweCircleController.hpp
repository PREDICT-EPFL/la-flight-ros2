//
// Created by johannes on 07.04.20.
//

#ifndef SRC_AWECIRCLECONTROLLER_H
#define SRC_AWECIRCLECONTROLLER_H

#include "AweControlTools.hpp"

namespace awe_control2 {

struct AweCircleParameters {
    /* Constant members mean that the parameter cannot be changed at runtime/via telemetry so far. */
    double windFrom{radians(180.0)};
    double teth_len{150.0};

    struct CircleParameters {
        double radiusRef{30.0};
        double declination{radians(35.0)};
        double heightRef{100.0};
    } circle;
};
struct AweCircleDebugValues {
    double circArg{0};
    double cp_posErrY{0};
    double cp_posErrZ{0};
    double distOffsetRef{0};
};

class AweCircleController {
public:
    AweCircleController() = default;
    ~AweCircleController() = default;

    AweCircleParameters p{};
    void updateWinddome();
    void updateParameterDependencies();

    void getVelocityRef(const Vector3 &g_xg, const Vector3 &g_vg,
                        const double &distOffsetRef,

                        Vector3 &g_vgRef);

    AweCircleDebugValues debug{};
//    Averager averageRadius{};
    MovingAverageFilter<> movAvgRadius{};
    MovingAverageFilter<> movAvgDist{};

    awe_control2::Integral reelOutLength{}; // Additional tether length on reel-out

public:
    Quaternion q_wd_g{T3quat(p.windFrom)};

    /** Navigation **/
    void navigate_circlePath(const Vector3 &g_xg, const Vector3 &g_vg,

                             Quaternion &q_cp_g, Vector3 &cp_xg, Vector3 &cp_vg,
                             double &cp_gamma, double &cp_chi, double &Vg);

    /** Guidance **/
    /* Guidance is given through the fact that it is a circle */

    /** Control **/
    void control_circlePath(const Quaternion &q_cp_g, const Vector3 &cp_xg,
                            const double &cp_gamma, const double &cp_chi, const double &Vg,
                            const double &distOffsetRef,

                            Vector3 &g_vgRef);
};

}

#endif //SRC_AWECIRCLECONTROLLER_H
