#ifndef SRC_STATEREPRESENTATION_HPP
#define SRC_STATEREPRESENTATION_HPP

namespace flight_model {

enum StateRepresentation
{
    Undefined,
    AttQuat, AttEuler, LongitudinalEulerAoa, LongitudinalUW, LongitudinalFlightPath
};

}

#endif //SRC_STATEREPRESENTATION_HPP
