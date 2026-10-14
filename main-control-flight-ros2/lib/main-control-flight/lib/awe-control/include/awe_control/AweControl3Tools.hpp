#ifndef SRC_AWECONTROL3TOOLS_HPP
#define SRC_AWECONTROL3TOOLS_HPP

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace awe_control3 {

/* Eigen types */
using Vector2 = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
using Vector3 = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Matrix3 = Eigen::Matrix<double, 3, 3, Eigen::DontAlign>;
using Quaternion = Eigen::Quaternion<double, Eigen::DontAlign>;

/* Math */
static double constrain(const double valueToConstrain, const double lowerLimit, const double upperLimit)
{
    return std::min(std::max(valueToConstrain, lowerLimit), upperLimit);
}

static double degrees(const double radians) { return radians * 180.0 / M_PI; }
static double radians(const double degrees) { return degrees * M_PI / 180.0; }

/* Transformations */
static Quaternion T1quat(const double rotAng) { return {std::cos(-rotAng / 2), std::sin(-rotAng / 2), 0, 0}; }
static Quaternion T2quat(const double rotAng) { return {std::cos(-rotAng / 2), 0, std::sin(-rotAng / 2), 0}; }
static Quaternion T3quat(const double rotAng) { return {std::cos(-rotAng / 2), 0, 0, std::sin(-rotAng / 2)}; }

/* Navigation */
static double getBearingFromCenter(const double positionX, const double positionY)
{
    /* QTE: True bearing from the station (here: origin of the coordinate frame) */
    double bearingFromCenter = std::atan2(positionY, positionX);
    if (bearingFromCenter < 0.0) { bearingFromCenter += 2.0 * M_PI; }

    // Return value is on 0 ... 2*pi
    return bearingFromCenter;
}
static double getBearingFromCenter(const Eigen::Vector2d &position)
{
    return getBearingFromCenter(position(0), position(1));
}

/* Helpers */
static double wrapTo(double value, const double lower_bound, const double upper_bound)
{
    if (value > upper_bound)
    {
        value -= (upper_bound - lower_bound);
    }
    else if (value < lower_bound)
    {
        value += (upper_bound - lower_bound);
    }
    return value;
}

}

#endif //SRC_AWECONTROL3TOOLS_HPP
