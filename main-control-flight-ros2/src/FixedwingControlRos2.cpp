#include "main_control_flight_ros2/FixedwingControlRos2.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace control_ros2::flight::fixedwing;

FixedwingControlRos2_sim::FixedwingControlRos2_sim(const std::string &name) : ControlRos2Base(name)
{
    std::cout << "Running FIXEDWING Control Node (ROS2 interface to SIMULATOR)\n";

    /* Register subscribers */
    m_sub_sim.state = this->create_subscription<MultiDOFJointStateM>(
            "/sim/state", 1, std::bind(&FixedwingControlRos2_sim::stateCallback, this, _1));
    m_sub_sim.state2 = this->create_subscription<MultiDOFJointStateM>(
            "/sim/state2", 1, std::bind(&FixedwingControlRos2_sim::state2Callback, this, _1));
    m_sub_sim.state3 = this->create_subscription<MultiDOFJointStateM>(
            "/sim/state3", 1, std::bind(&FixedwingControlRos2_sim::state3Callback, this, _1));
    m_sub_sim.stateConPoint = this->create_subscription<MultiDOFJointStateM>(
        "/sim/state_conpoint", 1, std::bind(&FixedwingControlRos2_sim::stateConPointCallback, this, _1));

    m_sub_sim.joy = this->create_subscription<JoyM>("/joy", 1,
                                                    std::bind(&FixedwingControlRos2_sim::joyCallback, this, _1));

    /* Register publishers */
    m_pub_sim.control = this->create_publisher<JoyM>("/sim/set/control", 1);
    m_pub_sim.control2 = this->create_publisher<JoyM>("/sim/set/control2", 1);
    m_pub_sim.control3 = this->create_publisher<JoyM>("/sim/set/control3", 1);

    m_pub_sim.ctrl_status = this->create_publisher<DiagnosticArrayM>("/status/main_control", 1);

    m_pub_sim.diag_LFc = this->create_publisher<Vector3StampedM>("/diag/LFc", 1);
    m_pub_sim.diag_LFc_cond = this->create_publisher<Vector3StampedM>("/diag/LFc_cond", 1);
    m_pub_sim.diag3 = this->create_publisher<Vector3StampedM>("/diag/diag3", 1);

    /* Wait 1 s for publishers to be established */
    chrono_sleep(std::chrono::seconds(1));

    declare_parameter<std::string>("mission_mode", "");
    const std::string mission_mode = get_parameter("mission_mode").as_string();
    m_mission_commander.set(mission_mode);

    if (m_mission_commander.getMission() == control::flight::fixedwing::NOMISSION)
    {
        declare_parameter<std::string>("control_mode", "mawe_manual");
        const std::string control_mode = get_parameter("control_mode").as_string();
        m_control_mode.set(control_mode);
    }

    declare_parameter<double>("windFrom_deg", 160.0);
    m_awec.wind_from = radians(get_parameter("windFrom_deg").as_double());
    std::cout << "Wind from: " << degrees(m_awec.wind_from) << " deg\n";

    declare_parameter<int>("control_kites", 2); // Control one kite, or several up to 999
    p_control_kites = get_parameter("control_kites").as_int();
    if (p_control_kites > 3)
    {
        std::cerr << "control_kites cannot exceed 3 for the moment!";
        exit(EXIT_FAILURE);
    }

    declare_parameter<bool>("activate", false);
    if (get_parameter("activate").as_bool())
    {
        manual.activation_button = true;
    }
}

/** Protected members ============================================================================ **/
void FixedwingControlRos2_sim::update_in()
{
    spinOnce_wrapper();
    for (unsigned k = 0; k < p_control_kites; k++)
    {
        states[k].update_ned_vel();
        states[k].update_euler_angles();
    }
}
void FixedwingControlRos2_sim::update_out()
{
    rclcpp::Time pub_time = rclcpp::Node::now();

    std::array<JoyM, 3> joy_msgs{};
    for (unsigned int k = 0; k < joy_msgs.size(); ++k)
    {
        auto &joy_msg = joy_msgs[k];
        joy_msg.header.stamp = pub_time;
        joy_msg.axes.resize(6);
        joy_msg.axes[0] = static_cast<float>(commands[k].throttle);
        joy_msg.axes[1] = static_cast<float>(commands[k].elevator);
        joy_msg.axes[2] = static_cast<float>(commands[k].rudder);
        joy_msg.axes[3] = static_cast<float>(commands[k].aileron);
        joy_msg.axes[4] = static_cast<float>(commands[k].aux0);
        joy_msg.axes[5] = static_cast<float>(commands[k].aux1);
    }

    m_pub_sim.control->publish(joy_msgs[0]);
    if (p_control_kites >= 2) { m_pub_sim.control2->publish(joy_msgs[1]); }
    if (p_control_kites >= 3) { m_pub_sim.control3->publish(joy_msgs[2]); }

    if (m_mission_commander.getMarkerActive())
    {
        DiagnosticArrayM ctrl_status{};
        ctrl_status.header.stamp = pub_time;
        ctrl_status.status.resize(1);
        ctrl_status.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        ctrl_status.status[0].name = "mission";
        ctrl_status.status[0].message = std::to_string(m_mission_commander.getMissionPhase());
        m_pub_sim.ctrl_status->publish(ctrl_status);
        m_mission_commander.deactivateMarker();
    }

    // Diagnostics
    struct DiagMsgs
    {
        Vector3StampedM LFc;
        Vector3StampedM LFc_cond;
        Vector3StampedM diag3;
    } diag_msgs;

    {
        auto &msg = diag_msgs.LFc;
        msg.header.stamp = pub_time;
        msg.vector.x = m_diag.lf_centri(0);
        msg.vector.y = m_diag.lf_centri(1);
        msg.vector.z = m_diag.lf_centri(2);
    }
    {
        auto &msg = diag_msgs.LFc_cond;
        msg.header.stamp = pub_time;
        msg.vector.x = m_diag.lf_centri_cond(0);
        msg.vector.y = m_diag.lf_centri_cond(1);
    }
    {
        auto &msg = diag_msgs.diag3;
        msg.header.stamp = pub_time;
//        msg.vector.x = m_diag.ICR_offs(0);
//        msg.vector.y = m_diag.ICR_offs(1);
//        msg.vector.z = m_diag.ICR_offs(2);
        msg.vector.x = m_diag.speed(0);
        msg.vector.y = m_diag.speed(1);
        msg.vector.z = m_diag.speed(2);
    }
    m_pub_sim.diag_LFc->publish(diag_msgs.LFc);
    m_pub_sim.diag_LFc_cond->publish(diag_msgs.LFc_cond);
    m_pub_sim.diag3->publish(diag_msgs.diag3);
}

/** Private members ============================================================================== **/
void FixedwingControlRos2_sim::stateCallback(const MultiDOFJointStateM::SharedPtr msg)
{
    if (m_VERBOSE >= PrintLevel2) { std::cout << "   stateCallback\n"; }
    states[0].timestamp = TimePoint(std::chrono::nanoseconds(msg->header.stamp.nanosec));

    states[0].vel_body = {msg->twist[0].linear.x,
                          msg->twist[0].linear.y,
                          msg->twist[0].linear.z};
    states[0].vel_avail = true;

    states[0].rates = {msg->twist[0].angular.x,
                       msg->twist[0].angular.y,
                       msg->twist[0].angular.z};

    states[0].pos = {msg->transforms[0].translation.x,
                     msg->transforms[0].translation.y,
                     msg->transforms[0].translation.z};
    states[0].pos_avail = true;

    states[0].q_nb = {msg->transforms[0].rotation.w,
                      msg->transforms[0].rotation.x,
                      msg->transforms[0].rotation.y,
                      msg->transforms[0].rotation.z};

    /* Wrench is used for acceleration */
    states[0].acc = {msg->wrench[0].force.x,
                     msg->wrench[1].force.y,
                     msg->wrench[2].force.z};
}
void FixedwingControlRos2_sim::state2Callback(const MultiDOFJointStateM::SharedPtr msg)
{
    if (m_VERBOSE >= PrintLevel2) { std::cout << "   state2Callback\n"; }
    states[1].timestamp = TimePoint(std::chrono::nanoseconds(msg->header.stamp.nanosec));

    states[1].vel_body = {msg->twist[0].linear.x,
                          msg->twist[0].linear.y,
                          msg->twist[0].linear.z};
    states[1].vel_avail = true;

    states[1].rates = {msg->twist[0].angular.x,
                       msg->twist[0].angular.y,
                       msg->twist[0].angular.z};

    states[1].pos = {msg->transforms[0].translation.x,
                     msg->transforms[0].translation.y,
                     msg->transforms[0].translation.z};
    states[1].pos_avail = true;

    states[1].q_nb = {msg->transforms[0].rotation.w,
                      msg->transforms[0].rotation.x,
                      msg->transforms[0].rotation.y,
                      msg->transforms[0].rotation.z};

    /* Wrench is used for acceleration */
    states[1].acc = {msg->wrench[0].force.x,
                     msg->wrench[1].force.y,
                     msg->wrench[2].force.z};
}
void FixedwingControlRos2_sim::state3Callback(const MultiDOFJointStateM::SharedPtr msg)
{
    if (m_VERBOSE >= PrintLevel2) { std::cout << "   state3Callback\n"; }
    states[2].timestamp = TimePoint(std::chrono::nanoseconds(msg->header.stamp.nanosec));

    states[2].vel_body = {msg->twist[0].linear.x,
                          msg->twist[0].linear.y,
                          msg->twist[0].linear.z};
    states[2].vel_avail = true;

    states[2].rates = {msg->twist[0].angular.x,
                       msg->twist[0].angular.y,
                       msg->twist[0].angular.z};

    states[2].pos = {msg->transforms[0].translation.x,
                     msg->transforms[0].translation.y,
                     msg->transforms[0].translation.z};
    states[2].pos_avail = true;

    states[2].q_nb = {msg->transforms[0].rotation.w,
                      msg->transforms[0].rotation.x,
                      msg->transforms[0].rotation.y,
                      msg->transforms[0].rotation.z};

    /* Wrench is used for acceleration */
    states[2].acc = {msg->wrench[0].force.x,
                     msg->wrench[1].force.y,
                     msg->wrench[2].force.z};
}
void FixedwingControlRos2_sim::stateConPointCallback(const MultiDOFJointStateM::SharedPtr msg)
{
    states[3].timestamp = TimePoint(std::chrono::nanoseconds(msg->header.stamp.nanosec));

    states[3].vel_ned = {msg->twist[0].linear.x,
                         msg->twist[0].linear.y,
                         msg->twist[0].linear.z};
    states[3].vel_avail = true;

    states[3].pos = {msg->transforms[0].translation.x,
                     msg->transforms[0].translation.y,
                     msg->transforms[0].translation.z};
    states[3].pos_avail = true;
}

void FixedwingControlRos2_sim::joyCallback(const JoyM::SharedPtr msg)
{
    if (m_VERBOSE >= PrintLevel2) { std::cout << "   joyCallback\n"; }
    manual.yaw = msg->axes[0];
    manual.thrust = msg->axes[1];
    manual.roll = msg->axes[3];
    manual.pitch = msg->axes[4];

    manual.trigger_0 = (1.0 - msg->axes[2]) / 2.0;
    manual.trigger_1 = (1.0 - msg->axes[5]) / 2.0;

    manual.param_right_left = -msg->axes[6];
    manual.param_up_down = msg->axes[7];

    manual.activation_button = msg->buttons[4];

    manual.button_0 = msg->buttons[0];
    manual.button_1 = msg->buttons[1];
    manual.button_2 = msg->buttons[2];
    manual.button_3 = msg->buttons[3];
}
void FixedwingControlRos2_sim::publish_test_impl()
{
    RosPub<MultiDOFJointStateM> sim_pub = this->create_publisher<MultiDOFJointStateM>("/sim/state", 5);
    RosPub<JoyM> joy_pub = this->create_publisher<JoyM>("/joy", 5);

    /* Wait for publishers/subscribers to be established */
    while (ok() && (sim_pub->get_subscription_count() < 1 ||
                    joy_pub->get_subscription_count() < 1)) { chrono_sleep(std::chrono::milliseconds(1)); }

    /* Prepare and send messages */
    const control::flight::fixedwing::TestState testState;
    MultiDOFJointStateM simMsg{};
    {
        simMsg.header.stamp = rclcpp::Node::now();

        simMsg.twist.resize(1);
        simMsg.twist[0].linear.x = testState.vel_bodyFRD(0);
        simMsg.twist[0].linear.y = testState.vel_bodyFRD(1);
        simMsg.twist[0].linear.z = testState.vel_bodyFRD(2);

        simMsg.twist[0].angular.x = testState.ratesFRD(0);
        simMsg.twist[0].angular.y = testState.ratesFRD(1);
        simMsg.twist[0].angular.z = testState.ratesFRD(2);

        simMsg.transforms.resize(1);
        simMsg.transforms[0].translation.x = testState.posNED(0);
        simMsg.transforms[0].translation.y = testState.posNED(1);
        simMsg.transforms[0].translation.z = testState.posNED(2);

        simMsg.transforms[0].rotation.w = testState.q_ned_b.w();
        simMsg.transforms[0].rotation.x = testState.q_ned_b.x();
        simMsg.transforms[0].rotation.y = testState.q_ned_b.y();
        simMsg.transforms[0].rotation.z = testState.q_ned_b.z();

        simMsg.wrench.resize(1);
        simMsg.wrench[0].force.x = testState.accFRD(0);
        simMsg.wrench[0].force.y = testState.accFRD(1);
        simMsg.wrench[0].force.z = testState.accFRD(2);

    }
    JoyM joyMsg{};
    {
        joyMsg.axes.resize(8);
        joyMsg.axes[0] = (float) -testState.manual.yaw;
        joyMsg.axes[1] = (float) testState.manual.thrust;
        joyMsg.axes[3] = (float) -testState.manual.roll;
        joyMsg.axes[4] = (float) testState.manual.pitch;

        joyMsg.buttons.resize(13);
    }

    /* Publish messages a couple of times to make sure they are received) */
    for (unsigned i = 0; i < 5; i++)
    {
        sim_pub->publish(simMsg);
        joy_pub->publish(joyMsg);

        update_in();
    }

}
void FixedwingControlRos2_sim::print_test_res_impl()
{
    std::cout << "Command: "
              << commands[0].throttle << ", "
              << commands[0].elevator << ", "
              << commands[0].rudder << ", "
              << commands[0].aileron << "\n";
}
