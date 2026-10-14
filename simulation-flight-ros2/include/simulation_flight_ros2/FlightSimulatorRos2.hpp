#ifndef SRC_FLIGHTSIMULATORROS2_HPP
#define SRC_FLIGHTSIMULATORROS2_HPP

#include <fstream>

#include "simulation_flight/FlightSimulator.hpp"
#include "simulation_flight_ros2/FlightSimulatorRos2Base.hpp"

/* Messages for state and control */
#include "sensor_msgs/msg/multi_dof_joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

/* Messages for Mavros emulation */
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace simulation_ros2 {
namespace flight {

class FlightSimulatorRos2 : public simulation::flight::FlightSimulator, public FlightSimulatorRos2Base
{
public:
    explicit FlightSimulatorRos2(const std::string &name);
    std::string get_rosparam_model_params_filepath();
    Eigen::VectorXd get_rosparam_init_state();
    Eigen::VectorXd get_rosparam_init_control();

protected:
    using JoyM = sensor_msgs::msg::Joy;
    using MultiDofJointStateM = sensor_msgs::msg::MultiDOFJointState;

    struct Subscribers
    {
        RosSub<JoyM> control;
    } m_sub;
    void control_callback(const JoyM::ConstSharedPtr msg);

    struct Messages
    {
        MultiDofJointStateM state{};
        JoyM actuator{};
    } m_msg{};
    struct Publishers
    {
        RosPub<MultiDofJointStateM> state;
        RosPub<JoyM> actuator;
    } m_pub;
    void publish_sim_data();

    void update_in_impl() override;
    void update_out_impl() override;

    void prepare_state_msg(MultiDofJointStateM &msg,
                           const rclcpp::Time &stamp,
                           const Eigen::Vector3d &b_vel,
                           const Eigen::Vector3d &w,
                           const Eigen::Vector3d &pos,
                           const Eigen::Quaterniond &q_nb,
                           const Eigen::Vector4d &Va_bet_alph,
                           const Eigen::Vector3d &nongravForce,
                           const Eigen::Vector3d &tethForce);


private:
    static bool file_exists(const std::string &filepath)
    {
        std::ifstream model_params(filepath);
        return model_params.good();
    }
};

class FlightSimulatorMavros2 : public FlightSimulatorRos2
{
public:
    explicit FlightSimulatorMavros2(const std::string &name);
protected:
    void update_out_impl() override;

private:
    using PoseStampedM = geometry_msgs::msg::PoseStamped;
    using TwistStampedM = geometry_msgs::msg::TwistStamped;
    using ImuM = sensor_msgs::msg::Imu;

    struct Publishers
    {
        RosPub<PoseStampedM> pose;
        RosPub<TwistStampedM> vel_body;
        RosPub<TwistStampedM> vel_local;
        RosPub<ImuM> imu_data;
    } m_mavpub;
    struct Messages
    {
        PoseStampedM pose{};
        TwistStampedM vel{};
        ImuM imu{};
    } m_mavmsg;
    void publish_mavros_data();

};

class FlightSimulator2KitesRos2 : public FlightSimulatorRos2
{
public:
    explicit FlightSimulator2KitesRos2(const std::string &name);

    MultiDofJointStateM m_msg_state2{};
    JoyM m_msg_actuator2{};
    MultiDofJointStateM m_msg_state_conpoint{};
    MultiDofJointStateM m_msg_state_base{};
    MultiDofJointStateM m_msg_tethers{};

    void control_callback(const JoyM::ConstSharedPtr msg);
    RosSub<JoyM>  m_sub_control2;
    void control2_callback(const JoyM::ConstSharedPtr msg);

    RosPub<MultiDofJointStateM> m_pub_state2;
    RosPub<JoyM> m_pub_actuator2;
    RosPub<MultiDofJointStateM> m_pub_state_conpoint;
    RosPub<MultiDofJointStateM> m_pub_state_base;
    RosPub<MultiDofJointStateM> m_pub_tethers;

    /* Override publish function to extract second kite state */
    void update_out_impl() override;
    void publish_sim_data2(const rclcpp::Time &publish_time);

    /* Helper functions */
    void prepare_conpoint_msg(MultiDofJointStateM &msg,
                              const rclcpp::Time &stamp,
                              const Eigen::Vector3d &vel,
                              const Eigen::Vector3d &pos,
                              const Eigen::Vector3d &nongravForce,
                              const Eigen::Vector3d &tethForce);
    void prepare_base_msg(MultiDofJointStateM &msg,
                          const rclcpp::Time &stamp,
                          const Eigen::Vector3d &tethForce);
    void prepare_teth_msg(MultiDofJointStateM &msg,
                          const rclcpp::Time &stamp,
                          const Eigen::Vector3d &teth_lengths,
                          const Eigen::Vector3d &teth_lenvels);
};

class FlightSimulator3KitesRos2 : public FlightSimulator2KitesRos2
{
public:
    explicit FlightSimulator3KitesRos2(const std::string &name);

    MultiDofJointStateM m_msg_state3{};
    JoyM m_msg_actuator3{};

//    void control_callback(const JoyM::ConstSharedPtr msg);
    RosSub<JoyM>  m_sub_control3;
    void control3_callback(const JoyM::ConstSharedPtr msg);

    RosPub<MultiDofJointStateM> m_pub_state3;
    RosPub<JoyM> m_pub_actuator3;

    /* Override publish function to extract second kite state */
    void update_out_impl() override;
    void publish_sim_data3(const rclcpp::Time &publish_time);
};

} // namespace flight
} // namespace simulation_ros2

#endif //SRC_FLIGHTSIMULATORROS2_HPP
