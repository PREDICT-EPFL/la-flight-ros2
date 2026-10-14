#ifndef SRC_FIXEDWINGCONTROLROS2_HPP
#define SRC_FIXEDWINGCONTROLROS2_HPP

#include "main_control_common/ControlTypes.hpp"
#include "main_control_common/ControlMath.hpp"
#include "main_control_common/ControlUtils.hpp"

#include "main_control_flight/fixedwing/FixedwingControl.hpp"
#include "main_control_flight_ros2/ControlRos2Base.hpp"

/* Messages to interface Mavros/Pixhawk */
//#include <mavros_msgs/ActuatorControl.h>
//#include <mavros_msgs/EstimatorStatus.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

/* Messages to interface simulator */
#include "sensor_msgs/msg/multi_dof_joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

/* Messages for diagnostics */
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace control_ros2 {
namespace flight {
namespace fixedwing {

using namespace control_common::types;
using namespace control_common::math;
using namespace control_common::utils;

class FixedwingControlRos2_px : public control::flight::fixedwing::FixedwingControl, public ControlRos2Base
{
public:
    explicit FixedwingControlRos2_px(const std::string &name) : ControlRos2Base(name)
    {
        std::cout << "Running FIXEDWING Control (ROS2 interface to MAVROS/PIXHAWK)\n";

        /* Register subscribers */
//        m_sub_px.estStat = m_nh.subscribe("/mavros/estimator_status", 1,
//                                          &FixedwingControlRos_px::estimatorStatusCallback, this);
        m_sub_px.pose = create_subscription<PoseStampedM>("/mavros/local_position/pose", 1,
                                                          std::bind(&FixedwingControlRos2_px::poseCallback, this, _1));
        m_sub_px.velBod = create_subscription<TwistStampedM>("/mavros/local_position/velocity_body", 1,
                                                             std::bind(&FixedwingControlRos2_px::velBodCallback, this,
                                                                       _1));
        m_sub_px.imuData = create_subscription<ImuM>("/mavros/imu/data", 1,
                                                     std::bind(&FixedwingControlRos2_px::imuDataCallback, this, _1));
        m_sub_px.windEst = create_subscription<TwistWithCovarianceStampedM>(
                "/mavros/wind_estimation", 1,
                std::bind(&FixedwingControlRos2_px::windEstimationCallback, this, _1));

        /* Register publishers */
//        m_pub_px.actuatorControl = m_nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);

//        /* Wait 1 s for publishers to be established */
//        chrono_sleep(std::chrono::seconds(1));
    }

protected:
    Quaternion q_enu_flu{1, 0, 0, 0};
    const Quaternion q_ned_enu{0, sqrt(2) / 2.0, sqrt(2) / 2.0, 0};
    const Quaternion q_flu_b{0, 1, 0, 0};
    Vector2 windEst_px{0, 0}; // Custom wind estimate (mean average)

private:
    using PoseStampedM = geometry_msgs::msg::PoseStamped;
    using TwistStampedM = geometry_msgs::msg::TwistStamped;
    using ImuM = sensor_msgs::msg::Imu;
    using TwistWithCovarianceStampedM = geometry_msgs::msg::TwistWithCovarianceStamped;
    using Vector3StampedM = geometry_msgs::msg::Vector3Stamped;

    struct Subscribers
    {
//        RosSub estStat;
        RosSub<PoseStampedM> pose;
        RosSub<TwistStampedM> velBod;
        RosSub<ImuM> imuData;
        RosSub<TwistWithCovarianceStampedM> windEst;
    } m_sub_px;
//    void estimatorStatusCallback(const mavros_msgs::EstimatorStatusConstPtr &msg)
//    {
//        bool pos_wasAvail = pos_isAvail;
//        pos_isAvail = vel_isAvail = (msg->pos_horiz_abs_status_flag && msg->pos_vert_abs_status_flag
//                                     && msg->velocity_horiz_status_flag && msg->velocity_vert_status_flag);
//
//        if (pos_isAvail && !pos_wasAvail)
//        {
//            std::cout << "Position AVAIL\n";
////            publishOffboardStatus();
//        }
//    }
    void poseCallback(const PoseStampedM::SharedPtr msg)
    {
        /* Message in ENU -> transform to NED */
        states[0].pos(0) = msg->pose.position.y;
        states[0].pos(1) = msg->pose.position.x;
        states[0].pos(2) = -msg->pose.position.z;

        /* Rotation from Front-Left-Up body frame to ENU frame */
        q_enu_flu = {msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                     msg->pose.orientation.z};
    }
    void velBodCallback(const TwistStampedM::SharedPtr msg)
    {
        states[0].timestamp = TimePoint(std::chrono::nanoseconds(msg->header.stamp.nanosec));

        /* Message in Front-Left-Up body frame */
        states[0].vel_body = {msg->twist.linear.x, -msg->twist.linear.y, -msg->twist.linear.z};

        if (states[0].pos_avail)
        {
            states[0].rates = {msg->twist.angular.x, -msg->twist.angular.y, -msg->twist.angular.z};
        }
    }
    void imuDataCallback(const ImuM::SharedPtr msg)
    {
        /* Message in Front-Left-Up body frame */
        if (!states[0].pos_avail)
        {
            states[0].rates = {msg->angular_velocity.x, -msg->angular_velocity.y, -msg->angular_velocity.z};
        }
        states[0].acc = {msg->linear_acceleration.x, -msg->linear_acceleration.y, -msg->linear_acceleration.z};
    }
    void windEstimationCallback(const TwistWithCovarianceStampedM::SharedPtr msg)
    {
        /* Message in ENU -> transform to NED */
        windEst_px(0) = msg->twist.twist.linear.y;
        windEst_px(1) = msg->twist.twist.linear.x;
        states[0].windEst_avail = true;
    }
    void updateStates()
    {
        states[0].q_nb = q_ned_enu * q_enu_flu * q_flu_b;
        states[0].update_ned_vel();
    }
    void update_in() override
    {
        //TODO
//        ros::spinOnce();
        updateStates();
    }

    struct Publishers
    {
//        RosPub actuatorControl;
    } m_pub_px;
    void update_out() override
    {
//        /* Actuator Control message */
//        mavros_msgs::ActuatorControl actuatorControl_msg{}; // Everything = 0
//
//        actuatorControl_msg.header.stamp = ros::Time::now();
//        actuatorControl_msg.group_mix = 0;  // Control Group #0 (Flight Control), see https://dev.px4.io/v1.9.0/en/concept/mixing.html
//
//        actuatorControl_msg.controls[0] = static_cast<float>(commands[0].dR * radians(15)); // Map command to +-15 degrees
//        actuatorControl_msg.controls[1] = static_cast<float>(commands[0].dP * radians(15)); // Map command to +-15 degrees
//        actuatorControl_msg.controls[2] = static_cast<float>(commands[0].dF);
//        actuatorControl_msg.controls[3] = static_cast<float>(commands[0].dFdiff);
//
//        m_pub_px.actuatorControl.publish(actuatorControl_msg);
    }

    void publish_test_impl() override
    {
//        RosPub estStat_pub = m_nh.advertise<mavros_msgs::EstimatorStatus>("/mavros/estimator_status", 1);
//        RosPub pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1);
//        RosPub velBod_pub = m_nh.advertise<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 1);
//        RosPub imuData_pub = m_nh.advertise<sensor_msgs::Imu>("/mavros/imu/data", 1);
//        RosPub windEst_pub = m_nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/mavros/wind_estimation", 1);
//
//        /* Wait for publishers/subscribers to be established */
//        while (ok() && (estStat_pub.getNumSubscribers() < 1 ||
//                        pose_pub.getNumSubscribers() < 1 ||
//                        velBod_pub.getNumSubscribers() < 1 ||
//                        imuData_pub.getNumSubscribers() < 1 ||
//                        windEst_pub.getNumSubscribers() < 1)) { chrono_sleep(std::chrono::milliseconds(1)); }
//
//        /* Prepare and send messages */
//        const control::flight::fixedwing::TestState testState;
//        mavros_msgs::EstimatorStatus estStatMsg{};
//        {
//            estStatMsg.pos_horiz_abs_status_flag = true;
//            estStatMsg.pos_vert_abs_status_flag = true;
//            estStatMsg.velocity_horiz_status_flag = true;
//            estStatMsg.velocity_vert_status_flag = true;
//        }
//        geometry_msgs::PoseStamped poseMsg{};
//        {
//            poseMsg.pose.position.x = testState.posNED(1); // Emulate ENU
//            poseMsg.pose.position.y = testState.posNED(0);
//            poseMsg.pose.position.z = -testState.posNED(2);
//
//            Quaternion q_enu_flu_test = q_ned_enu.inverse() * testState.q_ned_b * q_flu_b.inverse();
//            poseMsg.pose.orientation.w = q_enu_flu_test.w();
//            poseMsg.pose.orientation.x = q_enu_flu_test.x();
//            poseMsg.pose.orientation.y = q_enu_flu_test.y();
//            poseMsg.pose.orientation.z = q_enu_flu_test.z();
//        }
//        geometry_msgs::TwistStamped velBodMsg{};
//        {
//            velBodMsg.header.stamp = ros::Time::now();
//
//            velBodMsg.twist.linear.x = testState.vel_bodyFRD(0); // Emulate FLU
//            velBodMsg.twist.linear.y = -testState.vel_bodyFRD(1);
//            velBodMsg.twist.linear.z = -testState.vel_bodyFRD(2);
//
//            velBodMsg.twist.angular.x = testState.ratesFRD(0); // Emulate FLU
//            velBodMsg.twist.angular.y = -testState.ratesFRD(1);
//            velBodMsg.twist.angular.z = -testState.ratesFRD(2);
//        }
//        sensor_msgs::Imu imuMsg{};
//        {
//            imuMsg.angular_velocity.x = testState.ratesFRD(0); // Emulate FLU
//            imuMsg.angular_velocity.y = -testState.ratesFRD(1);
//            imuMsg.angular_velocity.z = -testState.ratesFRD(2);
//
//            imuMsg.linear_acceleration.x = testState.accFRD(0); // Emulate FLU
//            imuMsg.linear_acceleration.y = -testState.accFRD(1);
//            imuMsg.linear_acceleration.z = -testState.accFRD(2);
//        }
//        geometry_msgs::TwistWithCovarianceStamped windEstMsg{};
//        {
//            windEstMsg.twist.twist.linear.x = testState.windEstNE(1); // Emulate EN(U)
//            windEstMsg.twist.twist.linear.y = testState.windEstNE(0);
//        }
//
//        /* Keep publishing messages until they are actually received by own node again.
//         *
//         * We don't have avail flags for all topics. We assume the remaining ones have
//         * also been received by the time the three have.*/
//        while (ok() && (!pos_isAvail ||
//                        !vel_isAvail ||
//                        !windEst_isAvail))
//        {
//            estStat_pub.publish(estStatMsg);
//            pose_pub.publish(poseMsg);
//            velBod_pub.publish(velBodMsg);
//            imuData_pub.publish(imuMsg);
//            windEst_pub.publish(windEstMsg);
//
//            update_in();
//        }
    }
    void print_test_res_impl() override
    {
        std::cout << "Command: "
                  << commands[0].throttle << ", "
                  << commands[0].elevator << ", "
                  << commands[0].rudder << ", "
                  << commands[0].aileron << "\n";
    }
};

class FixedwingControlRos2_sim : public control::flight::fixedwing::FixedwingControl, public ControlRos2Base
{
public:
    explicit FixedwingControlRos2_sim(const std::string &name);

protected:
    void update_in() override;
    void update_out() override;

private:
    using MultiDOFJointStateM = sensor_msgs::msg::MultiDOFJointState;
    using JoyM = sensor_msgs::msg::Joy;
    using Vector3StampedM = geometry_msgs::msg::Vector3Stamped;
    using DiagnosticArrayM = diagnostic_msgs::msg::DiagnosticArray;

    struct Subscribers
    {
        RosSub<MultiDOFJointStateM> state;
        RosSub<MultiDOFJointStateM> state2;
        RosSub<MultiDOFJointStateM> state3;
        RosSub<MultiDOFJointStateM> stateConPoint;

        RosSub<JoyM> joy;
    } m_sub_sim;
    void stateCallback(const MultiDOFJointStateM::SharedPtr msg);
    void state2Callback(const MultiDOFJointStateM::SharedPtr msg);
    void state3Callback(const MultiDOFJointStateM::SharedPtr msg);
    void stateConPointCallback(const MultiDOFJointStateM::SharedPtr msg);

    void joyCallback(const JoyM::SharedPtr msg);
    void updateStates();

    struct Publishers
    {
        RosPub<JoyM> control;
        RosPub<JoyM> control2;
        RosPub<JoyM> control3;

        RosPub<DiagnosticArrayM> ctrl_status;

        RosPub<Vector3StampedM> diag_LFc;
        RosPub<Vector3StampedM> diag_LFc_cond;
        RosPub<Vector3StampedM> diag3;
    } m_pub_sim;

    void publish_test_impl() override;
    void print_test_res_impl() override;
};

} // namespace fixedwing
} // namespace flight
} // namespace control_ros2

#endif //SRC_FIXEDWINGCONTROLROS2_HPP
