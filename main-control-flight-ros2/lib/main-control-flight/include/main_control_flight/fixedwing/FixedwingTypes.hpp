#ifndef SRC_FIXEDWINGTYPES_HPP
#define SRC_FIXEDWINGTYPES_HPP

#include <iostream>

#include "main_control_common/ControlTypes.hpp"

namespace fixedwing_types {

/* RC_Raw */
template<typename T  = double>
struct RC_Raw_s
{
    T roll{0};     // RC roll stick position, range: 0 ... 1
    T pitch{0};    // RC pitch stick position,  range: 0 ... 1
    T thrust{0};   // RC throttle stick position,  range: 0 ... 1
    T yaw{0};      // RC yaw stick position,  range: 0 ... 1
    T flaps{0};    // RC flaps switch, true if switch on the RC transmitter points away from the pilot
    T gear{0};     // RC gear switch, true if switch on the RC transmitter points away from the pilot

    RC_Raw_s() = default;

    RC_Raw_s(const T &rollInit, const T &pitchInit, const T &thrustInit, const T &yawInit, const T &flapsInit,
             const T &gearInit) :
            roll(rollInit), pitch(pitchInit), thrust(thrustInit), yaw(yawInit), flaps(flapsInit), gear(gearInit) {}
};

/* Manual */
template<typename T  = double>
struct Manual_s
{
    T roll{0};          // RC roll stick position, range: -1 ... 1, positive: right
    T pitch{0};         // RC pitch stick position,  range: -1 ... 1, positive: down
    T thrust{0};        // RC throttle stick position,  range: 0 ... 1, positive: +thrust
    T yaw{0};           // RC yaw stick position,  range: -1 ... 1, positive: right
    T trigger_0{0};     // Trigger axis 0
    T trigger_1{0};     // Trigger axis 1
    uint buttons{0};

    T param_right_left{0};
    T param_up_down{0};
    bool activation_button{false};
    bool button_0{false};
    bool button_1{false};
    bool button_2{false};
    bool button_3{false};

    Manual_s() = default;
};
using Manual = Manual_s<double>;

template<typename T  = double>
std::ostream &operator<<(std::ostream &os, const Manual_s<T> &manual)
{
    os << "roll: " << manual.roll << ", pitch: " << manual.pitch << ", "
       << "thrust: " << manual.thrust << ", yaw: " << manual.yaw << ", "
       << "trigger(0, 1): " << manual.trigger_0 << ", " << manual.trigger_1 << ", buttons: " << manual.buttons;
    return os;
}

/* FixedwingCommand - commands to be passed through by the Flight Controller Boards to the servos */
template<typename T = double>
struct FixedwingCommand_s
{
    T aileron{0};       // Positive: right
    T elevator{0};      // Positive: up
    T throttle{0};      // Positive: +thrust
    T rudder{0};        // Positive: right
    T aux0{0};          // Auxillary
    T aux1{0};          // Auxillary

    FixedwingCommand_s() = default;
};
using FixedwingCommand = FixedwingCommand_s<double>;

template<typename T = double>
std::ostream &operator<<(std::ostream &os, const FixedwingCommand_s<T> &command)
{
    os << "dR: " << command.dR << ", dP: " << command.dP << ", dF: " << command.dF << ", dFdiff: " << command.dFdiff;
    return os;
}

} // namespace fixedwing_types

#endif //SRC_FIXEDWINGTYPES_HPP
