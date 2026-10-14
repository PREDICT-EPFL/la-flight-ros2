#include "simulation_flight_ros2/FlightSimulatorRos2.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace simulation_ros2::flight;

FlightSimulatorRos2::FlightSimulatorRos2(const std::string &name) : FlightSimulatorRos2Base(name)
{
    std::cout << "Running Flight Simulator (ROS2 interface)\n";

    /** Read atmosphere and simulation options from ROS params **/
    /* Setup Atmosphere */
    declare_parameter<double>("windFrom_deg", 160.0);
    declare_parameter<double>("windSpeed", 1.5);

    const double mean_windFrom_deg = get_parameter("windFrom_deg").get_value<double>();
    const double mean_wind_speed = get_parameter("windSpeed").get_value<double>();
    setup_atmosphere(mean_windFrom_deg, mean_wind_speed);

    declare_parameter<bool>("simulate_atmosphere", true);
    declare_parameter<bool>("simulate_tether", false);
    p_sim_opts.atmosphere = get_parameter("simulate_atmosphere").get_value<bool>();
    p_sim_opts.tether = get_parameter("simulate_tether").get_value<bool>();

    declare_parameter<double>("actuator_rate", 50);
    const double actuator_rate = get_parameter("actuator_rate").get_value<double>();
    set_actuator_rate(actuator_rate);

    /** Setup subscribers, publishers, messages **/
    /* Register subscribers */
    m_sub.control = create_subscription<JoyM>("/sim/set/control", 1,
                                              std::bind(&FlightSimulatorRos2::control_callback, this, _1));

    /* Register publishers */
    m_pub.state = create_publisher<MultiDofJointStateM>("/sim/state", 1);
    m_pub.actuator = create_publisher<JoyM>("/sim/control", 1);

    /* Prepare messages */
    m_msg.state.transforms.resize(1);
    m_msg.state.twist.resize(2);
    m_msg.state.wrench.resize(2);

    m_msg.actuator.axes.resize(4);
}
FlightSimulatorMavros2::FlightSimulatorMavros2(const std::string &name) : FlightSimulatorRos2(name)
{
    std::cout << "MAVROS emulation\n";

    /** Setup subscribers, publishers **/
    m_mavpub.pose = create_publisher<PoseStampedM>("/mavros/local_position/pose", 1);
    m_mavpub.vel_body = create_publisher<TwistStampedM>("/mavros/local_position/velocity_body", 1);
    m_mavpub.vel_local = create_publisher<TwistStampedM>("/mavros/local_position/velocity_local", 1);
    m_mavpub.imu_data = create_publisher<ImuM>("/mavros/imu/data", 1);
}
FlightSimulator2KitesRos2::FlightSimulator2KitesRos2(const std::string &name) : FlightSimulatorRos2(name)
{
    std::cout << "Running Flight Simulator 2 Kites (ROS2 interface)\n";

    /** Setup subscribers, publishers, messages **/
    /* Register subscribers */
    m_sub.control = create_subscription<JoyM>("/sim/set/control", 1,
                                              std::bind(&FlightSimulator2KitesRos2::control_callback, this, _1));
    m_sub_control2 = create_subscription<JoyM>("/sim/set/control2", 1,
                                               std::bind(&FlightSimulator2KitesRos2::control2_callback, this, _1));

    /* Register publishers */
    m_pub_state2 = create_publisher<MultiDofJointStateM>("/sim/state2", 1);
    m_pub_actuator2 = create_publisher<JoyM>("/sim/control2", 1);
    m_pub_state_conpoint = create_publisher<MultiDofJointStateM>("/sim/state_conpoint", 1);
    m_pub_state_base = create_publisher<MultiDofJointStateM>("/sim/state_base", 1);
    m_pub_tethers = create_publisher<MultiDofJointStateM>("/sim/tethers", 1);

    /// Kite 2
    m_msg_state2.transforms.resize(1);
    m_msg_state2.twist.resize(2);
    m_msg_state2.wrench.resize(2);

    /// Conpoint & Base
    m_msg_state_conpoint.transforms.resize(1);
    m_msg_state_conpoint.twist.resize(1);
    m_msg_state_conpoint.wrench.resize(1);
    m_msg_state_base.wrench.resize(1);

    m_msg_actuator2.axes.resize(4);
    m_msg_tethers.transforms.resize(1);
    m_msg_tethers.twist.resize(1);
}
FlightSimulator3KitesRos2::FlightSimulator3KitesRos2(const std::string &name) : FlightSimulator2KitesRos2(name)
{
    std::cout << "Running Flight Simulator 3 Kites (ROS2 interface)\n";

    /** Setup subscribers, publishers, messages **/
    /* Register subscribers */
    m_sub_control3 = create_subscription<JoyM>("/sim/set/control3", 1,
                                               std::bind(&FlightSimulator3KitesRos2::control3_callback, this, _1));

    /* Register publishers */
    m_pub_state3 = create_publisher<MultiDofJointStateM>("/sim/state3", 1);
    m_pub_actuator3 = create_publisher<JoyM>("/sim/control3", 1);

    /// Kite 3
    m_msg_state3.transforms.resize(1);
    m_msg_state3.twist.resize(2);
    m_msg_state3.wrench.resize(2);

    m_msg_actuator3.axes.resize(4);
}

std::string FlightSimulatorRos2::get_rosparam_model_params_filepath()
{
    declare_parameter<std::string>("model_params", "");
    std::string model_params_file = get_parameter("model_params").get_value<std::string>();

    declare_parameter<std::string>("variant", "multikite");
    std::string variant = get_parameter("variant").get_value<std::string>();

    /** Manage model_params file **/
    /* If no model params specified, try to use default files for variants */
    if (model_params_file.empty())
    {
        if (variant == "kite") { model_params_file = "eg4_xflr-Pvw-YR"; }
        else if (variant == "multikite") { model_params_file = "eg4_xflr-Pvw-YR-Multi"; }
    }
    /* Complete model params file path if necessary */
    if (!model_params_file.empty() && !file_exists(model_params_file))
    {
        if (model_params_file.at(0) != '/')
        {
            model_params_file = ament_index_cpp::get_package_share_directory("simulation_flight_ros2") +
                                "/config/" + model_params_file;
        }
        if (model_params_file.find(".yaml") == std::string::npos) { model_params_file.append(".yaml"); }
    }
    return model_params_file;
}
Eigen::VectorXd FlightSimulatorRos2::get_rosparam_init_state()
{
    declare_parameter<std::vector<double>>("init_state", std::vector<double>());
    const std::vector<double> init_state = get_parameter("init_state").get_value<std::vector<double>>();
    return simulation::vec2EigenXd(init_state);
}
Eigen::VectorXd FlightSimulatorRos2::get_rosparam_init_control()
{
    declare_parameter<std::vector<double>>("init_control", std::vector<double>());
    const std::vector<double> init_control = get_parameter("init_control").get_value<std::vector<double>>();
    return simulation::vec2EigenXd(init_control);
}

/** Protected members ============================================================================ **/
void FlightSimulatorRos2::update_in_impl()
{
    spinOnce_wrapper();
}

void FlightSimulatorMavros2::update_out_impl()
{
    publish_sim_data();
    publish_mavros_data();
}
void FlightSimulatorRos2::update_out_impl()
{
    publish_sim_data();
}
void FlightSimulator2KitesRos2::update_out_impl()
{
    const rclcpp::Time publish_time = rclcpp::Node::now();
    publish_sim_data2(publish_time);
}
void FlightSimulator3KitesRos2::update_out_impl()
{
    const rclcpp::Time publish_time = rclcpp::Node::now();
    publish_sim_data3(publish_time);
}

void FlightSimulatorRos2::control_callback(const JoyM::ConstSharedPtr msg)
{
    m_control_in(0) = msg->axes[0];
    m_control_in(1) = msg->axes[1];
    m_control_in(2) = msg->axes[2];
    m_control_in(3) = msg->axes[3];
}
void FlightSimulator2KitesRos2::control_callback(const JoyM::ConstSharedPtr msg)
{
    FlightSimulatorRos2::control_callback(msg);
    if (msg->axes.size() > 4) m_control_in(4) = msg->axes[4];
    if (msg->axes.size() > 5) m_control_in(5) = msg->axes[5];
}
void FlightSimulator2KitesRos2::control2_callback(const JoyM::ConstSharedPtr msg)
{
    const int nu0 = 6;
    m_control_in(nu0 + 0) = msg->axes[0];
    m_control_in(nu0 + 1) = msg->axes[1];
    m_control_in(nu0 + 2) = msg->axes[2];
    m_control_in(nu0 + 3) = msg->axes[3];
    if (msg->axes.size() > 4) m_control_in(nu0 + 4) = msg->axes[4];
    if (msg->axes.size() > 5) m_control_in(nu0 + 5) = msg->axes[5];
}
void FlightSimulator3KitesRos2::control3_callback(const JoyM::ConstSharedPtr msg)
{
    const int nu0 = 2*6;
    m_control_in(nu0 + 0) = msg->axes[0];
    m_control_in(nu0 + 1) = msg->axes[1];
    m_control_in(nu0 + 2) = msg->axes[2];
    m_control_in(nu0 + 3) = msg->axes[3];
    if (msg->axes.size() > 4) m_control_in(nu0 + 4) = msg->axes[4];
    if (msg->axes.size() > 5) m_control_in(nu0 + 5) = msg->axes[5];
}

void FlightSimulatorRos2::publish_sim_data()
{
    //    ROS_INFO("Flightsim publish_sim_data");
    rclcpp::Time publish_time = rclcpp::Node::now();

    /* Parse state and publish it in flight simulator specific message */
    /** State message */
//    std::cout << "state: " << m_state.transpose() << "\n";
    m_q_nb.coeffs() << m_state.segment(10, 3), m_state(9); // state contains quaternion
    prepare_state_msg(m_msg.state,
                      publish_time,
                      m_state.segment<3>(0), m_state.segment<3>(3), m_state.segment<3>(6), m_q_nb,
                      Eigen::Vector4d(m_Va_pitot, m_Va, m_beta, m_alpha), m_nongravForce, m_tethForce);

    /** Control states message **/
    m_msg.actuator.header.stamp = publish_time;
    if (p_sim_opts.actuator_dynamics)
    {
        //    msg_control.axes[0] = (std::abs(thrust) < 0.001 ? 0.0 : thrust); // Thrust
        m_msg.actuator.axes[0] = (std::abs(m_state(13)) < 0.001 ? 0.0 : m_state(13)); // Thrust
        m_msg.actuator.axes[1] = (std::abs(m_state(14)) < 0.001 ? 0.0 : m_state(14)); // Elevator
        m_msg.actuator.axes[2] = (std::abs(m_state(15)) < 0.001 ? 0.0 : m_state(15)); // Rudder
        m_msg.actuator.axes[3] = (std::abs(m_state(16)) < 0.001 ? 0.0 : m_state(16)); // Aileron
    }
    else
    {
        //    msg_control.axes[0] = (std::abs(thrust) < 0.001 ? 0.0 : thrust); // Thrust
        m_msg.actuator.axes[0] = m_control(0); // Thrust
        m_msg.actuator.axes[1] = m_control(1); // Elevator
        m_msg.actuator.axes[2] = m_control(2); // Rudder
        m_msg.actuator.axes[3] = m_control(3); // Aileron
    }

    m_pub.state->publish(m_msg.state);
    m_pub.actuator->publish(m_msg.actuator);

    //    std::cout << "state_msg pos " << state_msg.transforms[0].translation.x << "\n";
}
void FlightSimulatorMavros2::publish_mavros_data()
{
    /* Publish sim message for visualization */
    FlightSimulatorRos2::publish_sim_data();

    /* Publish MAVROS topics additionally */
    rclcpp::Time publish_time = rclcpp::Node::now();

    /* Pose in ENU frame */
    m_mavmsg.pose.header.stamp = publish_time;

    m_mavmsg.pose.pose.position.x = m_state(7);  // E
    m_mavmsg.pose.pose.position.y = m_state(6);  // N
    m_mavmsg.pose.pose.position.z = -m_state(8); // U

    const Eigen::Quaternion<double> q_enu_ned = {0, -sqrt(2) / 2.0, -sqrt(2) / 2.0, 0};
    const Eigen::Quaternion<double> q_b_flu = {0, -1, 0, 0};

    m_q_nb.coeffs() << m_state.segment(10, 3), m_state(9); // state contains quaternion
    const Eigen::Quaternion<double> q_enu_flu = q_enu_ned * m_q_nb * q_b_flu;

    m_mavmsg.pose.pose.orientation.w = q_enu_flu.w();
    m_mavmsg.pose.pose.orientation.x = q_enu_flu.x();
    m_mavmsg.pose.pose.orientation.y = q_enu_flu.y();
    m_mavmsg.pose.pose.orientation.z = q_enu_flu.z();
    m_mavpub.pose->publish(m_mavmsg.pose);

    /* Body velocity in Front-Left-Up frame */
    m_mavmsg.vel.header.stamp = publish_time;

    m_mavmsg.vel.twist.linear.x = m_state(0);
    m_mavmsg.vel.twist.linear.y = -m_state(1);
    m_mavmsg.vel.twist.linear.z = -m_state(2);

    m_mavmsg.vel.twist.angular.x = m_state(3);
    m_mavmsg.vel.twist.angular.y = -m_state(4);
    m_mavmsg.vel.twist.angular.z = -m_state(5);
    m_mavpub.vel_body->publish(m_mavmsg.vel);

    /* Velocity in ENU frame */
    const Eigen::Vector3d vel_NED = m_q_nb * m_state.segment(0, 3);
    m_mavmsg.vel.twist.linear.x = vel_NED(1);
    m_mavmsg.vel.twist.linear.y = vel_NED(0);
    m_mavmsg.vel.twist.linear.z = -vel_NED(2);
    m_mavpub.vel_local->publish(m_mavmsg.vel);

    /* IMU data in Front-Left-Up frame */
    m_mavmsg.imu.header.stamp = publish_time;

    m_mavmsg.imu.linear_acceleration.x = m_nongravForce(0);
    m_mavmsg.imu.linear_acceleration.y = -m_nongravForce(1);
    m_mavmsg.imu.linear_acceleration.z = -m_nongravForce(2);

    m_mavmsg.imu.angular_velocity.x = m_state(3);
    m_mavmsg.imu.angular_velocity.y = -m_state(4);
    m_mavmsg.imu.angular_velocity.z = -m_state(5);
    m_mavpub.imu_data->publish(m_mavmsg.imu);
}
void FlightSimulator2KitesRos2::publish_sim_data2(const rclcpp::Time &publish_time)
{
    /* Parse state and publish it in flight simulator specific message */
/// Kite 1
    {
        m_q_nb.coeffs() << m_state.segment(10, 3), m_state(9); // state contains quaternion
        prepare_state_msg(m_msg.state,
                          publish_time,
                          m_state.segment<3>(0), m_state.segment<3>(3), m_state.segment<3>(6), m_q_nb,
                          Eigen::Vector4d(m_Va_pitot, m_Va, m_alpha, m_beta), m_nongravForce, m_tethForce);

        m_msg.actuator.header.stamp = publish_time;
        if (p_sim_opts.actuator_dynamics)
        {
            //    msg_control.axes[0] = (std::abs(thrust) < 0.001 ? 0.0 : thrust); // Thrust
            m_msg.actuator.axes[0] = (std::abs(m_state(13)) < 0.001 ? 0.0 : m_state(13)); // Thrust
            m_msg.actuator.axes[1] = (std::abs(m_state(14)) < 0.001 ? 0.0 : m_state(14)); // Elevator
            m_msg.actuator.axes[2] = (std::abs(m_state(15)) < 0.001 ? 0.0 : m_state(15)); // Rudder
            m_msg.actuator.axes[3] = (std::abs(m_state(16)) < 0.001 ? 0.0 : m_state(16)); // Aileron
        }
        else
        {
            m_msg.actuator.axes[0] = m_control(0); // Thrust
            m_msg.actuator.axes[1] = m_control(1); // Elevator
            m_msg.actuator.axes[2] = m_control(2); // Rudder
            m_msg.actuator.axes[3] = m_control(3); // Aileron
        }
    }

    /// Kite 2
    {
        unsigned nx0 = 1 * 13;
        unsigned nz0 = 1 * 10;
        unsigned nu0 = 1 * 6;

        Eigen::Quaternion<double> q_nb;
        q_nb.coeffs() << m_state.segment(nx0 + 10, 3), m_state(nx0 + 9); // state contains quaternion

        prepare_state_msg(m_msg_state2,
                          publish_time,
                          m_state.segment<3>(nx0 + 0), m_state.segment<3>(nx0 + 3), m_state.segment<3>(nx0 + 6), q_nb,
                          m_dyn_out.segment<4>(nz0 + 0), m_dyn_out.segment<3>(nz0 + 4), m_dyn_out.segment<3>(nz0 + 7));

        /** Control states message **/
        m_msg_actuator2.header.stamp = publish_time;
        if (p_sim_opts.actuator_dynamics)
        {
            m_msg_actuator2.axes[0] = (std::abs(m_state(nx0 + 13)) < 0.001 ? 0.0 : m_state(nx0 + 13)); // Thrust
            m_msg_actuator2.axes[1] = (std::abs(m_state(nx0 + 14)) < 0.001 ? 0.0 : m_state(nx0 + 14)); // Elevator
            m_msg_actuator2.axes[2] = (std::abs(m_state(nx0 + 15)) < 0.001 ? 0.0 : m_state(nx0 + 15)); // Rudder
            m_msg_actuator2.axes[3] = (std::abs(m_state(nx0 + 16)) < 0.001 ? 0.0 : m_state(nx0 + 16)); // Aileron
        }
        else
        {
            m_msg_actuator2.axes[0] = m_control(nu0 + 0); // Thrust
            m_msg_actuator2.axes[1] = m_control(nu0 + 1); // Elevator
            m_msg_actuator2.axes[2] = m_control(nu0 + 2); // Rudder
            m_msg_actuator2.axes[3] = m_control(nu0 + 3); // Aileron
        }
    }

    /// Conpoint & Base
    {
        unsigned nx0 = 2 * 13;
        unsigned nz0 = 2 * 10;

        prepare_conpoint_msg(m_msg_state_conpoint, publish_time,
                             m_state.segment<3>(nx0 + 0), m_state.segment<3>(nx0 + 3),
                             m_dyn_out.segment<3>(nz0),
                             m_dyn_out.segment<3>(nz0));

        prepare_base_msg(m_msg_state_base, publish_time,
                         m_dyn_out.segment<3>(nz0 + 3));
    }

    /// Tethers
    {
        unsigned nx0 = 2 * 13 + 6;
        unsigned nz0 = 2 * 10 + 3;

        prepare_teth_msg(m_msg_tethers, publish_time,
                         m_state.segment<3>(nx0 + 0),
                         {m_control(1 * 6 + 4), m_control(0 * 6 + 5), m_control(0 * 6 + 4)});
    }
    m_pub.state->publish(m_msg.state);
    m_pub.actuator->publish(m_msg.actuator);

    m_pub_state2->publish(m_msg_state2);
    m_pub_actuator2->publish(m_msg_actuator2);

    m_pub_state_conpoint->publish(m_msg_state_conpoint);
    m_pub_state_base->publish(m_msg_state_base);
    m_pub_tethers->publish(m_msg_tethers);
}
void FlightSimulator3KitesRos2::publish_sim_data3(const rclcpp::Time &publish_time)
{
    /* Parse state and publish it in flight simulator specific message */
    /// Kite 1
    {
        m_q_nb.coeffs() << m_state.segment(10, 3), m_state(9); // state contains quaternion
        prepare_state_msg(m_msg.state,
                          publish_time,
                          m_state.segment<3>(0), m_state.segment<3>(3), m_state.segment<3>(6), m_q_nb,
                          Eigen::Vector4d(m_Va_pitot, m_Va, m_alpha, m_beta), m_nongravForce, m_tethForce);

        m_msg.actuator.header.stamp = publish_time;
        if (p_sim_opts.actuator_dynamics)
        {
            //    msg_control.axes[0] = (std::abs(thrust) < 0.001 ? 0.0 : thrust); // Thrust
            m_msg.actuator.axes[0] = (std::abs(m_state(13)) < 0.001 ? 0.0 : m_state(13)); // Thrust
            m_msg.actuator.axes[1] = (std::abs(m_state(14)) < 0.001 ? 0.0 : m_state(14)); // Elevator
            m_msg.actuator.axes[2] = (std::abs(m_state(15)) < 0.001 ? 0.0 : m_state(15)); // Rudder
            m_msg.actuator.axes[3] = (std::abs(m_state(16)) < 0.001 ? 0.0 : m_state(16)); // Aileron
        }
        else
        {
            m_msg.actuator.axes[0] = m_control(0); // Thrust
            m_msg.actuator.axes[1] = m_control(1); // Elevator
            m_msg.actuator.axes[2] = m_control(2); // Rudder
            m_msg.actuator.axes[3] = m_control(3); // Aileron
        }
    }

    /// Kite 2
    {
        unsigned nx0 = 1 * 13;
        unsigned nz0 = 1 * 10;
        unsigned nu0 = 1 * 6;

        Eigen::Quaternion<double> q_nb;
        q_nb.coeffs() << m_state.segment(nx0 + 10, 3), m_state(nx0 + 9); // state contains quaternion

        prepare_state_msg(m_msg_state2,
                          publish_time,
                          m_state.segment<3>(nx0 + 0), m_state.segment<3>(nx0 + 3), m_state.segment<3>(nx0 + 6), q_nb,
                          m_dyn_out.segment<4>(nz0 + 0), m_dyn_out.segment<3>(nz0 + 4), m_dyn_out.segment<3>(nz0 + 7));

        /** Control states message **/
        m_msg_actuator2.header.stamp = publish_time;
        if (p_sim_opts.actuator_dynamics)
        {
            m_msg_actuator2.axes[0] = (std::abs(m_state(nx0 + 13)) < 0.001 ? 0.0 : m_state(nx0 + 13)); // Thrust
            m_msg_actuator2.axes[1] = (std::abs(m_state(nx0 + 14)) < 0.001 ? 0.0 : m_state(nx0 + 14)); // Elevator
            m_msg_actuator2.axes[2] = (std::abs(m_state(nx0 + 15)) < 0.001 ? 0.0 : m_state(nx0 + 15)); // Rudder
            m_msg_actuator2.axes[3] = (std::abs(m_state(nx0 + 16)) < 0.001 ? 0.0 : m_state(nx0 + 16)); // Aileron
        }
        else
        {
            m_msg_actuator2.axes[0] = m_control(nu0 + 0); // Thrust
            m_msg_actuator2.axes[1] = m_control(nu0 + 1); // Elevator
            m_msg_actuator2.axes[2] = m_control(nu0 + 2); // Rudder
            m_msg_actuator2.axes[3] = m_control(nu0 + 3); // Aileron
        }
    }

    /// Kite 3
    {
        unsigned nx0 = 2 * 13;
        unsigned nz0 = 2 * 10;
        unsigned nu0 = 2 * 6;

        Eigen::Quaternion<double> q_nb;
        q_nb.coeffs() << m_state.segment(nx0 + 10, 3), m_state(nx0 + 9); // state contains quaternion

        prepare_state_msg(m_msg_state3,
                          publish_time,
                          m_state.segment<3>(nx0 + 0), m_state.segment<3>(nx0 + 3), m_state.segment<3>(nx0 + 6), q_nb,
                          m_dyn_out.segment<4>(nz0 + 0), m_dyn_out.segment<3>(nz0 + 4), m_dyn_out.segment<3>(nz0 + 7));

        /** Control states message **/
        m_msg_actuator3.header.stamp = publish_time;
        if (p_sim_opts.actuator_dynamics)
        {
            m_msg_actuator3.axes[0] = (std::abs(m_state(nx0 + 13)) < 0.001 ? 0.0 : m_state(nx0 + 13)); // Thrust
            m_msg_actuator3.axes[1] = (std::abs(m_state(nx0 + 14)) < 0.001 ? 0.0 : m_state(nx0 + 14)); // Elevator
            m_msg_actuator3.axes[2] = (std::abs(m_state(nx0 + 15)) < 0.001 ? 0.0 : m_state(nx0 + 15)); // Rudder
            m_msg_actuator3.axes[3] = (std::abs(m_state(nx0 + 16)) < 0.001 ? 0.0 : m_state(nx0 + 16)); // Aileron
        }
        else
        {
            m_msg_actuator3.axes[0] = m_control(nu0 + 0); // Thrust
            m_msg_actuator3.axes[1] = m_control(nu0 + 1); // Elevator
            m_msg_actuator3.axes[2] = m_control(nu0 + 2); // Rudder
            m_msg_actuator3.axes[3] = m_control(nu0 + 3); // Aileron
        }
    }

    /// Conpoint & Base
    {
        unsigned nx0 = 3 * 13;
        unsigned nz0 = 3 * 10;

        prepare_conpoint_msg(m_msg_state_conpoint,
                             publish_time,
                             m_state.segment<3>(nx0 + 0), m_state.segment<3>(nx0 + 3),
                             m_dyn_out.segment<3>(nz0),
                             m_dyn_out.segment<3>(nz0));

        prepare_base_msg(m_msg_state_base,
                         publish_time,
                         m_dyn_out.segment<3>(nz0 + 3));
    }

    m_pub.state->publish(m_msg.state);
    m_pub.actuator->publish(m_msg.actuator);

    m_pub_state2->publish(m_msg_state2);
    m_pub_actuator2->publish(m_msg_actuator2);

    m_pub_state3->publish(m_msg_state3);
    m_pub_actuator3->publish(m_msg_actuator3);

    m_pub_state_conpoint->publish(m_msg_state_conpoint);
    m_pub_state_base->publish(m_msg_state_base);
}

/* Helpers */
void FlightSimulatorRos2::prepare_state_msg(MultiDofJointStateM &msg,
                                            const rclcpp::Time &stamp,
                                            const Eigen::Vector3d &pos,
                                            const Eigen::Vector3d &b_vel,
                                            const Eigen::Vector3d &w,
                                            const Eigen::Quaterniond &q_nb,
                                            const Eigen::Vector4d &Va_bet_alph,
                                            const Eigen::Vector3d &nongravForce,
                                            const Eigen::Vector3d &tethForce)
{
    msg.header.stamp = stamp;
    msg.header.frame_id = "flight_sim";

    msg.twist[0].linear.x = pos(0);
    msg.twist[0].linear.y = pos(1);
    msg.twist[0].linear.z = pos(2);

    msg.twist[0].angular.x = b_vel(0);
    msg.twist[0].angular.y = b_vel(1);
    msg.twist[0].angular.z = b_vel(2);

    msg.transforms[0].translation.x = w(0);
    msg.transforms[0].translation.y = w(1);
    msg.transforms[0].translation.z = w(2);

    msg.transforms[0].rotation.w = q_nb.w();
    msg.transforms[0].rotation.x = q_nb.x();
    msg.transforms[0].rotation.y = q_nb.y();
    msg.transforms[0].rotation.z = q_nb.z();

    /* Using wrench 0 as for acceleration, as measured by IMU (spec nongravitational forces) */
    msg.wrench[0].force.x = nongravForce(0);
    msg.wrench[0].force.y = nongravForce(1);
    msg.wrench[0].force.z = nongravForce(2);

    /* Using wrench 1 as for specific total tether force */
    msg.wrench[1].force.x = tethForce(0);
    msg.wrench[1].force.y = tethForce(1);
    msg.wrench[1].force.z = tethForce(2);

    /* Aero values */
    msg.twist[1].linear.x = Va_bet_alph(1); //m_Va;
    msg.twist[1].linear.y = Va_bet_alph(3); //m_beta;
    msg.twist[1].linear.z = Va_bet_alph(2); //m_alpha;
    msg.twist[1].angular.x = Va_bet_alph(0); //m_Va_pitot;
    /* Wind velocity vector */
    msg.twist[1].angular.y = m_vW_ned(0);
    msg.twist[1].angular.z = m_vW_ned(1);
}

void FlightSimulator2KitesRos2::prepare_conpoint_msg(MultiDofJointStateM &msg,
                                                     const rclcpp::Time &stamp,
                                                     const Eigen::Vector3d &vel,
                                                     const Eigen::Vector3d &pos,
                                                     const Eigen::Vector3d &nongravForce,
                                                     const Eigen::Vector3d &tethForce)
{
    msg.header.frame_id = "flight_sim";
    msg.header.stamp = stamp;

    msg.twist[0].linear.x = vel(0);
    msg.twist[0].linear.y = vel(1);
    msg.twist[0].linear.z = vel(2);

    msg.transforms[0].translation.x = pos(0);
    msg.transforms[0].translation.y = pos(1);
    msg.transforms[0].translation.z = pos(2);

    msg.wrench[0].force.x = nongravForce(0);
    msg.wrench[0].force.y = nongravForce(1);
    msg.wrench[0].force.z = nongravForce(2);

    /* Using wrench 1 as for total tether force */
    msg.wrench[1].force.x = tethForce(0);
    msg.wrench[1].force.y = tethForce(1);
    msg.wrench[1].force.z = tethForce(2);
}
void FlightSimulator2KitesRos2::prepare_base_msg(MultiDofJointStateM &msg,
                                                 const rclcpp::Time &stamp,
                                                 const Eigen::Vector3d &nongravForce)
{
    msg.header.frame_id = "flight_sim";
    msg.header.stamp = stamp;

    msg.wrench[0].force.x = nongravForce(0);
    msg.wrench[0].force.y = nongravForce(1);
    msg.wrench[0].force.z = nongravForce(2);
}
void FlightSimulator2KitesRos2::prepare_teth_msg(MultiDofJointStateM &msg,
                                                 const rclcpp::Time &stamp,
                                                 const Eigen::Vector3d &teth_lengths,
                                                 const Eigen::Vector3d &teth_lenvels)
{
    msg.header.frame_id = "flight_sim";
    msg.header.stamp = stamp;

    msg.transforms[0].translation.x = teth_lengths(0);
    msg.transforms[0].translation.y = teth_lengths(1);
    msg.transforms[0].translation.z = teth_lengths(2);

    msg.twist[0].linear.x = teth_lenvels(0); // lat
    msg.twist[0].linear.y = teth_lenvels(1); // lon
    msg.twist[0].linear.z = teth_lenvels(2); // main
}
