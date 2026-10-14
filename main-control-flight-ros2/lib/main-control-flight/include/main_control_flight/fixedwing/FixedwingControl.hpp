#ifndef SRC_FIXEDWINGCONTROL_HPP
#define SRC_FIXEDWINGCONTROL_HPP

#include "main_control_common/ControlBase.hpp"
#include "main_control_common/ControlMath.hpp"
#include "main_control_common/ControlUtils.hpp"

#include "main_control_flight/fixedwing/FixedwingTypes.hpp"
#include "main_control_flight/fixedwing/FixedwingAttitudeRateControl.hpp"

#include "main_control_flight/MpcManager.hpp"
#include "main_control_flight/MissionCommander.hpp"

#include "awe_control/AweControlTools.hpp"
#include "awe_control/AweCircleController.hpp"
#include "awe_control/AweHeliControl.hpp"

namespace control {
namespace flight {
namespace fixedwing {

using namespace control_common::math;
using namespace control_common::utils;
using namespace control_common::types;
using namespace fixedwing_types;

struct TestState
{
    const Vector3 posNED{4, 1, -5};
    const Quaternion q_ned_b{1, 0, 0, 0};
    const Vector3 vel_bodyFRD{0.1, -0.3, 0.4};
    const Vector3 ratesFRD{radians(20), radians(-15), radians(10)};
    const Vector3 accFRD{0.3, 0.2, -9.8};
    const Vector2 windEstNE{0.2, 0.1};

    const Manual manual{};
};

class FlightParams
{
public:
    struct FlightState
    {
        TimePoint timestamp;

        /* Depending on global positioning (GNSS) */
        bool pos_avail{false};
        bool vel_avail{false};
        Eigen::Vector3d pos{0, 0, 0};
        Eigen::Vector3d vel_ned{0, 0, 0};
        Eigen::Vector3d vel_body{0, 0, 0};

        bool windEst_avail{false};
        Eigen::Vector2d windEst{0, 0};

        /* Inertial measurements (IMU) */
        bool att_avail{false};
        bool rates_avail{false};
        Eigen::Quaterniond q_nb{1, 0, 0, 0};
        Vector3 rates{0, 0, 0};              // Measured rates (angular velocity) in aircraft body-fixed frame
        Vector3 acc{0, 0, 0};                // Measured acceleration (angular velocity) in aircraft body-fixed frame

        /* Additional quantities (redundant to existing) */
        Matrix3 T_b_ned = Matrix3::Identity(); // Transformation matrix from local NED to aircraft body-fixed frame
        Vector3 att{0, 0, 0}; // Euler angles (roll, pitch, yaw) from local NED to aircraft body-fixed frame

        void update_body_vel() { if (vel_avail) vel_body = q_nb.inverse() * vel_ned; }
        void update_ned_vel() { if (vel_avail) vel_ned = q_nb * vel_body; }
        void update_euler_angles()
        {
            /* Quat to euler angles*/
            T_b_ned = q_nb.inverse().toRotationMatrix();
            att = {atan2(T_b_ned(1, 2), T_b_ned(2, 2)),
                   -asin(T_b_ned(0, 2)),
                   atan2(T_b_ned(0, 1), T_b_ned(0, 0))};
        }
    };
    std::array<FlightState, 4> states{};
    Eigen::VectorXd x_comp; // composed state
};

/* Flight control modes */
class FixedwingControlMode
{
public:
    enum ControlMode
    {
        UNDEFINED, MANUAL, RATES, ATTITUDE, VELOCITY, POSITION,
        MAWE_MANUAL, MAWE_CLIMB, MAWE_LATLON, MAWE_DESCENT,
    };

    FixedwingControlMode() = default;
    explicit FixedwingControlMode(const ControlMode &control_mode) :
            m_control_mode(control_mode),
            m_control_mode_prev(control_mode) {}

    ControlMode get() const { return m_control_mode; }
    void set(const std::string &controlMode)
    {
        if (controlMode == "mawe_latlon") { set(MAWE_LATLON); }
        else if (controlMode == "mawe_descent") { set(MAWE_DESCENT); }
        else if (controlMode == "mawe_climb") { set(MAWE_CLIMB); }
        else if (controlMode == "mawe_manual") { set(MAWE_MANUAL); }
        else if (controlMode == "position") { set(POSITION); }
        else if (controlMode == "velocity") { set(VELOCITY); }
        else if (controlMode == "attitude") { set(ATTITUDE); }
        else if (controlMode == "rates") { set(RATES); }
        else if (controlMode == "manual") { set(MANUAL); }
        else
        {
            std::cerr << "Control mode """ << controlMode << """ could not be found. Falling back to MANUAL.\n"
                      << "Available modes are: {mawe_latlon, mawe_climb, mawe_descent, mawe_manual, "
                         "position, velocity, attitude, rates, manual}\n";
            set(MANUAL);
        }
    }
    void set(const ControlMode &controlMode)
    {
        m_control_mode_prev = m_control_mode;
        m_control_mode = controlMode;
        m_control_mode_has_changed = true;
    }
    void spin_once() { m_control_mode_has_changed = false; }
    bool has_changed() const { return m_control_mode_has_changed; }

    ControlMode operator()() const { return m_control_mode; }
private:
    ControlMode m_control_mode{UNDEFINED};
    ControlMode m_control_mode_prev{UNDEFINED};
    bool m_control_mode_has_changed{false};
};

class FixedwingControl : public FlightParams, public virtual control_common::ControlBase
{
protected:
    void init_impl() override;
    void test_impl() override;

    void compute_control_impl() override;

    /** Capsuled flight management tasks **/
    MissionCommander m_mission_commander;
    void executeMission();

    void mission_mawe();

    void chooseFlightControlMode();
    void changeFlightControlMode(const FixedwingControlMode::ControlMode &newFlightControlMode,
                                 bool inhibitParamUpdate = false);

    /** States **/
    /* Manual control */
    Manual manual{};
    bool manual_avail{false};
    control_common::utils::ActiveParser m_param_parser_up;
    control_common::utils::ActiveParser m_param_parser_dn;
    control_common::utils::ActiveParser m_param_parser_right;
    control_common::utils::ActiveParser m_param_parser_left;

    /** Control **/
    FixedwingControlMode m_control_mode;

    /** Multi-kite control **/
    unsigned p_control_kites{0};
    double p_argmax{radians(90)};
    double p_ampl_max{0};

//    awe_control2::AweCircleController m_awe_circle_control;
    awe_control3::AweHeliController m_awec;
    AttitudeRateControl m_posatt_control;
    control::flight::MpcInterface m_mpc_interface;

    /* Control setpoints */
    bool posvel_ref_needs_update{false};
    Vector3 pos_ref{0, 0, -1.5};
    bool pos_ref_valid{false};
    double heading_ref{0};
    bool heading_ref_valid{false};

    Vector3 velNED_ref{0, 0, 0};
    bool velNED_ref_valid{false};

    Vector3 n_acc_ref{0, 0, 0};
    bool n_acc_ref_valid{false};

    Vector3 rates_ref{0, 0, 0};
    bool rates_ref_valid{false};

    struct ControlDiag
    {
        Vector3 pos_ref{0, 0, 0};
        Vector3 lf_centri{0, 0, 0};
        Vector3 lf_centri_cond{0};
        Vector3 ICR_offs{0, 0, 0};
        Vector3 speed{0, 0, 0};
    } m_diag;

    /* Control command */
    std::array<FixedwingCommand, 3> commands{};
    bool command_valid{false};
    void setAuxCommands(const Vector3 &tetherLatLonMain_vels);
    void prepareCommand();

private:
    void control_flight_complete();

    void control_flight_high_level();
    void control_flight_low_level();

    Eigen::VectorXd control_centrifugal_sanity(const Eigen::VectorXd &normlift_tilt_cmds);

    Eigen::VectorXd control_multi_lift(const Eigen::Vector2d &d_lift_dir);

    void print_test_res_impl() override
    {
        std::cout << "Command: "
                  << commands[0].throttle << ", "
                  << commands[0].elevator << ", "
                  << commands[0].rudder << ", "
                  << commands[0].aileron << "\n";
    }

    double getDiscretizedStickPosition(double manualAxis)
    {
        double stickPosition = manualAxis;
        /* Discretize in 3 (positive/negative) steps and deadzone */
        if (stickPosition > 0.7) stickPosition = 1.0;
        else if (stickPosition > 0.4) stickPosition = 0.666666;
        else if (stickPosition > 0.1) stickPosition = 0.333333;

        else if (stickPosition < -0.7) stickPosition = -1.0;
        else if (stickPosition < -0.4) stickPosition = -0.666666;
        else if (stickPosition < -0.1) stickPosition = -0.333333;

        else /* Deadzone */
            stickPosition = 0;

        return stickPosition;
    }

};

} // namespace fixedwing
} // namespace flight
} // namespace control

#endif //SRC_FIXEDWINGCONTROL_HPP
