//
// Created by johannes on 09.04.20.
//

#ifndef SRC_SPEEDCONTROLLER_HPP
#define SRC_SPEEDCONTROLLER_HPP


#include "AweControlTools.hpp"

namespace awe_control2 {

struct SpeedControllerParameters
{
    double speedProt{11};
    double predictionTime{5};
    double speedErrLimNeg{5};
    double integralGain{0.2};
};
struct SpeedControllerDebugValues
{
    double dAirspeed{0};
    double speedPred{0};
    double speedErr{0};
};

class SpeedController
{
public:
    SpeedController()
    {
        reset();
    }
    void setSampletime(const double &sampletime) { integral.setSampletime(sampletime); }
    void reset()
    {
        VaBuf.reset();
        integral.reset();
    }

    SpeedControllerParameters p{};
    SpeedControllerDebugValues debug{};

    void setThrottle(const double &throttle) { integral.set_value(throttle); }

    void update(const ros::Time &timestamp, const double &airspeed)
    {
        VaBuf.write(timestamp.toSec(), airspeed);
    }

    double controlMinSpeed()
    {
        double speedPred = VaBuf.getUnfilteredValue() + p.predictionTime * VaBuf.filterDerivative();
        double speedErr = constrain(p.speedProt - speedPred, -p.speedErrLimNeg, 9999);

        debug.dAirspeed = VaBuf.filterDerivative();
        debug.speedPred = speedPred;
        debug.speedErr = speedErr;

        return integral.integrate(p.integralGain * speedErr);
    }

    PolyFilter<2, 20> VaBuf{};

private:
    Integral integral{0, 1};
};


enum AutoThrottleMode
{
    OFF,
    THRUSTLOCK,
    TOGA,
    SPEED,
    SPEEDLVR,
    THRUSTCLIMB,
    THRUSTLVR
};

class AutoThrottle
{
public:
//    AutoThrottle() { //TODO
//        AutoThrottle(void *(const std::string &modeStr){});
//    }
//    AutoThrottle(void (*modeChangeCallback_)(const std::string &modeStr)) {
//        modeChangeCallback = modeChangeCallback_;
//    }
    bool CONNECTED{true};
    bool modeHasChanged()
    {
        if (mode_hasChanged)
        {
            mode_hasChanged = false;
            return true;
        }
        return false;
    }
    AutoThrottleMode getMode() { return mode; }
    void getModeName(std::string &modeStr)
    {
        switch (mode)
        {
            case THRUSTLOCK:
                modeStr = "THRUST LOCK";
                break;
            case TOGA:
                modeStr = "TOGA";
                break;
            case SPEED:
                modeStr = "SPEED";
                break;
            case SPEEDLVR:
                modeStr = "SPEED LVR";
                break;
            case THRUSTCLIMB:
                modeStr = "THRUST CLIMB";
                break;
            case THRUSTLVR:
                modeStr = "THRUST LVR";
                break;
            default:
                modeStr = "OFF";
        }
    }
    void lock(const double &leverPosition)
    {
        thrust_locked = leverPosition;
        changeMode(THRUSTLOCK);
    }
    void unlock() { unlock_isRequested = true; }

    void disengage()
    {
        mode = OFF;
        unlock_isRequested = false;
        thrust_locked = 0.0;
    }

    void armSpeedMode() { mode_armed = SPEED; }
    void armSpeedMode(const double &speedRef)
    {
        armSpeedMode();
        speedController.p.speedProt = speedRef;
    }

    void armThrustMode() { mode_armed = THRUSTCLIMB; }
    void trackThrust(const double &leverPosition)
    {
        isTracked = true;
        thrust_tracked = leverPosition;
    }

    double getThrottle(const double &leverPosition)
    {

        if (!CONNECTED)
            unlock();

        if (isTracked)
        {
            isTracked = false;
            return thrust_tracked;
        }

        if (CONNECTED && mode != THRUSTLOCK)
        {
            /* Select mode other than THRUSTLOCK */

            if (leverPosition > TOGA_threshold)
            {

                if (changeMode(TOGA))
                {
                    speedController.setThrottle(max_cont_thrust);
                }

            }
            else if (leverPosition > engage_threshold)
            {

                if (mode_armed == THRUSTCLIMB)
                {
                    changeMode(THRUSTCLIMB);

                }
                else
                {
                    if (changeMode(SPEED))
                    {
                        speedController.setThrottle(max_cont_thrust);
                    }
                }

            }
            else if (leverPosition > disengage_threshold)
            {

                /* Lever coming from above,
                 * already in SPEED- or THRUST- mode */
                if (mode == THRUSTCLIMB)
                {
                    changeMode(THRUSTLVR);

                }
                else if (mode == SPEED)
                {
                    changeMode(SPEEDLVR);

                }
                else if (mode == TOGA)
                {
                    /* Lever coming from above, but missed upper mode due to stick discretization */
                    if (mode_armed == THRUSTCLIMB)
                    {
                        changeMode(THRUSTLVR);
                    }
                    else
                    {
                        changeMode(SPEEDLVR);
                    }

                }
                else if (mode == THRUSTLVR && mode_armed == SPEED)
                {
                    /* Lever position is already in this region, but SPEED/THRUST armed has been changed */
                    changeMode(SPEEDLVR);

                }
                else if (mode == SPEEDLVR && mode_armed == THRUSTCLIMB)
                {
                    /* Lever position is already in this region, but SPEED/THRUST armed has been changed */
                    changeMode(THRUSTLVR);
                }

            }
            else
            {
                /* Below disengage_threshold */
                changeMode(OFF);
            }
        }

        switch (mode)
        {

            case THRUSTLOCK:
                if (unlock_isRequested && std::abs(thrust_locked - leverPosition) < 0.1)
                {

                    unlock_isRequested = false;
                    changeMode(OFF);
                    return leverPosition;

                }
                else
                {
                    return thrust_locked;
                }

            case TOGA:
                return 1.0;

            case SPEED:
                return std::min(speedController.controlMinSpeed(), max_cont_thrust);

            case SPEEDLVR:
                return std::min(speedController.controlMinSpeed(), leverPosition);

            case THRUSTCLIMB:
                return max_cont_thrust;

            case THRUSTLVR:
            case OFF:
            default:
                return leverPosition;
        }
    }
    double getMaxContThrottle() { return max_cont_thrust; }

    SpeedController speedController{};
//    void (*modeChangeCallback)(const std::string &modeStr); //TODO

private:
    double thrust_locked{0};
    bool unlock_isRequested{false};

    double thrust_tracked{0};
    bool isTracked{false};

    const double TOGA_threshold{0.95};
    const double engage_threshold{0.8};
    const double max_cont_thrust{0.8};
    const double disengage_threshold{0.1};

    AutoThrottleMode mode{OFF};
    AutoThrottleMode mode_armed{SPEED};
    bool mode_hasChanged{false};

    bool changeMode(const AutoThrottleMode modeToSet)
    {
        if (mode != modeToSet)
        {
            mode = modeToSet;

            /* Print mode change */
            std::string modeStr;
            getModeName(modeStr);
            std::cout << "[  AUTO THR   ]: " << modeStr << "\n";

            mode_hasChanged = true;
//            modeChangeCallback(modeStr);  // TODO
            return true;
        }
        else
        {
            return false;
        }
    }
};

class SimpleAutoThrottle
{
public:
    bool CONNECTED{true};
    void lock(const double &leverPosition)
    {
        thrust_locked = leverPosition;
        changeMode(THRUSTLOCK);
    }
    void unlock() { unlock_isRequested = true; }
    void disengage()
    {
        mode = OFF;
        unlock_isRequested = false;
        thrust_locked = 0.0;
    }

    void armSpeedMode() { mode_armed = SPEED; }
    void armSpeedMode(const double &speedRef)
    {
        armSpeedMode();
        speedController.p.speedProt = speedRef;
    }

    void armThrustMode() { mode_armed = THRUSTCLIMB; }

    double getThrottle(const double &leverPosition)
    {

        if (CONNECTED && mode != THRUSTLOCK)
        {
            /* Select mode other than THRUSTLOCK */

            /* SPEED/THRUSTCLIMB is activated only coming from TOGA, but the lever position is wider to hit it.
             * LVR modes are not very relevant in practice and are deleted */
            if (leverPosition > TOGA_threshold)
            {

                if (changeMode(TOGA))
                {
                    speedController.setThrottle(max_cont_thrust);
                }

            }
            else if (leverPosition > engage_threshold)
            {

                if (mode == TOGA || mode == THRUSTCLIMB || mode == SPEED)
                {
                    /* Coming from TOGA or already in THRUST/SPEED mode */

                    if (mode_armed == THRUSTCLIMB)
                    {
                        changeMode(THRUSTCLIMB);

                    }
                    else
                    {
                        if (changeMode(SPEED))
                        {
                            speedController.setThrottle(max_cont_thrust);
                        }
                    }
                }

            }
            else if (leverPosition < disengage_threshold)
            {
                /* Below disengage_threshold */
                changeMode(OFF);
            }
        }

        switch (mode)
        {

            case THRUSTLOCK:
                if (unlock_isRequested && std::abs(thrust_locked - leverPosition) < 0.1)
                {

                    unlock_isRequested = false;
                    changeMode(OFF);
                    return leverPosition;

                }
                else
                {
                    return thrust_locked;
                }

            case TOGA:
                return 1.0;

            case SPEED:
                return std::min(speedController.controlMinSpeed(), max_cont_thrust);

            case THRUSTCLIMB:
                return max_cont_thrust;

            case OFF:
            default:
                return leverPosition;
        }
    }

    SpeedController speedController{};

private:
    double thrust_locked{0};
    bool unlock_isRequested{false};

    const double TOGA_threshold{0.95};
    const double engage_threshold{0.8};
    const double max_cont_thrust{0.8};
    const double disengage_threshold{0.1};

    AutoThrottleMode mode{OFF};
    AutoThrottleMode mode_armed{SPEED};

    bool changeMode(const AutoThrottleMode modeToSet)
    {
        if (mode != modeToSet)
        {
            mode = modeToSet;

            /* Print mode change */
            std::cout << "[  AUTO THR   ]: ";
            switch (mode)
            {
                case THRUSTLOCK:
                    std::cout << "THRUST LOCK";
                    break;
                case TOGA:
                    std::cout << "TOGA";
                    break;
                case SPEED:
                    std::cout << "SPEED";
                    break;
                case THRUSTCLIMB:
                    std::cout << "THRUST CLIMB";
                    break;
                default:
                    std::cout << "OFF";
            }
            std::cout << "\n";

            return true;
        }
        else
        {
            return false;
        }
    }
};


}

#endif //SRC_SPEEDCONTROLLER_HPP
