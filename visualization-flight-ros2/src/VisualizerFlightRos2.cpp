#include "visualization_flight_ros2/VisualizerFlightRos2.hpp"

using namespace visualization_ros2::flight;

void KiteVisualizer::init_impl()
{
    VisualizerFlightRos2Base::init_impl();

    std::cout << "Fixed-Wing (Kite)\n";

    /* Clear trajectory if still displayed from earlier execution */
//    publishResetMpcStateTraj();

    updateParameters();

    /* Set up persistent data in markers */
    broadcastWinddomeTransform();
    broadcastCircleTransform();

    publishSphere();
    publishPathRef();

    if (simulate_tether)
    {
        setup_tether();

        setup_teth_force(m_msg.tetherForceArrow, m_msg.tetherForceText);

        if (N_vehicles >= 2)
        {
            setup_teth_force(m_msg.tetherForceArrow2, m_msg.tetherForceText2);
            m_msg.tetherForceArrow2.header.frame_id = "body2";
            m_msg.tetherForceArrow2.id = 1;
            m_msg.tetherForceText2.id = 1;
        }
        if (N_vehicles >= 3)
        {
            setup_teth_force(m_msg.tetherForceArrow3, m_msg.tetherForceText3);
            m_msg.tetherForceArrow3.header.frame_id = "body3";
            m_msg.tetherForceArrow3.id = 2;
            m_msg.tetherForceText3.id = 2;
        }

        /* Connection point */
        setup_teth_force(m_msg.tetherForceArrowConpoint, m_msg.tetherForceTextConpoint);
        m_msg.tetherForceArrowConpoint.header.frame_id = "ned";
        m_msg.tetherForceArrowConpoint.id = 3;
        m_msg.tetherForceArrowConpoint.scale.x = 0.05 * 2.0; // Shaft diameter
        m_msg.tetherForceArrowConpoint.scale.y = 0.2 * 2.0; // Head diameter
        m_msg.tetherForceArrowConpoint.scale.z = 0.2 * 2.0; // Head Length
        m_msg.tetherForceTextConpoint.id = 3;
        setup_conpoint(); // Point to indicate conpoint position

        /* Base point */
        m_states[4].pos.setZero();
        m_states[4].b_nongravForce.setZero();
        setup_teth_force(m_msg.tetherForceArrowBase, m_msg.tetherForceTextBase);
        m_msg.tetherForceArrowBase.header.frame_id = "ned";
        m_msg.tetherForceArrowBase.id = 4;
        m_msg.tetherForceArrowBase.scale.x = 0.05 * 2.0; // Shaft diameter
        m_msg.tetherForceArrowBase.scale.y = 0.2 * 2.0; // Head diameter
        m_msg.tetherForceArrowBase.scale.z = 0.2 * 2.0; // Head Length
        m_msg.tetherForceTextBase.id = 4;

        setup_circle_plane();
        setup_circle_plane_force();
    }

//        publishWind();
    publishBody();
}
void KiteVisualizer::update_in_impl()
{
    VisualizerFlightRos2Base::update_in_impl();
    m_states[0].vel_transformed = m_states[0].vel_in;
    m_states[1].vel_transformed = m_states[1].vel_in;
    m_states[2].vel_transformed = m_states[2].vel_in;
}
void KiteVisualizer::update_out_impl()
{
    VisualizerFlightRos2Base::update_out_impl();

    broadcastCircleTransform();

    prepareControl();

    if (simulate_tether)
    {
        prepareTether();

        prepareTethForce(m_msg.tetherForceArrow, m_msg.tetherForceText, m_states[0], b_tethForce[0]);

        prepareCirclePlane();
        prepareCirclePlaneForce();

        // Append markers to fast array
        m_markerArray_fast.markers.push_back(m_msg.tether);

        m_markerArray_fast.markers.push_back(m_msg.tetherForceArrow);
        m_markerArray_fast.markers.push_back(m_msg.tetherForceText);

        m_markerArray_fast.markers.push_back(m_msg.circlePlane);
        m_markerArray_fast.markers.push_back(m_msg.circlePlaneForce);

        m_markerArray_fast.markers.push_back(m_msg.control);

        if (N_vehicles >= 2)
        {
            prepareTethForce(m_msg.tetherForceArrow2, m_msg.tetherForceText2, m_states[1], b_tethForce[1]);
            m_markerArray_fast.markers.push_back(m_msg.tetherForceArrow2);
            m_markerArray_fast.markers.push_back(m_msg.tetherForceText2);

            m_markerArray_fast.markers.push_back(m_msg.control2);

            if (N_vehicles >= 3)
            {
                prepareTethForce(m_msg.tetherForceArrow3, m_msg.tetherForceText3, m_states[2], b_tethForce[2]);
                m_markerArray_fast.markers.push_back(m_msg.tetherForceArrow3);
                m_markerArray_fast.markers.push_back(m_msg.tetherForceText3);

                m_markerArray_fast.markers.push_back(m_msg.control3);
            }
        }

        /* Connection point */
        prepareConpoint();
        m_markerArray_fast.markers.push_back(m_msg.conpoint);

        prepareTethForce(m_msg.tetherForceArrowConpoint, m_msg.tetherForceTextConpoint,
                         m_states[3], m_states[3].b_nongravForce); // b_nongravForce is actually in NED here
        // Therefore, need to translate end points of arrow
        m_msg.tetherForceArrowConpoint.points[0].x = m_states[3].pos(0);
        m_msg.tetherForceArrowConpoint.points[0].y = m_states[3].pos(1);
        m_msg.tetherForceArrowConpoint.points[0].z = m_states[3].pos(2);
        m_msg.tetherForceArrowConpoint.points[1].x += m_states[3].pos(0);
        m_msg.tetherForceArrowConpoint.points[1].y += m_states[3].pos(1);
        m_msg.tetherForceArrowConpoint.points[1].z += m_states[3].pos(2);
        m_markerArray_fast.markers.push_back(m_msg.tetherForceArrowConpoint);
        m_markerArray_fast.markers.push_back(m_msg.tetherForceTextConpoint);

        /* Base point */
        prepareTethForce(m_msg.tetherForceArrowBase, m_msg.tetherForceTextBase,
                         m_states[4], m_states[4].b_nongravForce); // b_nongravForce is actually in NED here
        // Therefore, need to translate end points of arrow
        m_msg.tetherForceArrowBase.points[0].x = m_states[4].pos(0);
        m_msg.tetherForceArrowBase.points[0].y = m_states[4].pos(1);
        m_msg.tetherForceArrowBase.points[0].z = m_states[4].pos(2);
        m_msg.tetherForceArrowBase.points[1].x += m_states[4].pos(0);
        m_msg.tetherForceArrowBase.points[1].y += m_states[4].pos(1);
        m_msg.tetherForceArrowBase.points[1].z += m_states[4].pos(2);
        m_markerArray_fast.markers.push_back(m_msg.tetherForceArrowBase);
        m_markerArray_fast.markers.push_back(m_msg.tetherForceTextBase);
    }
    m_pub.fastArray->publish(m_markerArray_fast);

//        if (mpc_traj_poses[0].position.x != 0)
//        {
//            publishMpcStateTraj();
//        }
}

/* Setup reusable markers */
void KiteVisualizer::setup_circle_plane()
{
    /* Namespace and identifier */
    m_msg.circlePlane.ns = "circle_plane";
    m_msg.circlePlane.id = 0;

    /* Frame and time */
//    m_msg.tether.header.stamp = RosTime(0);
    m_msg.circlePlane.header.frame_id = "circle";
    m_msg.circlePlane.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_msg.circlePlane.frame_locked = false;

    /* Position and orientation */
    m_msg.circlePlane.type = MarkerM::CYLINDER;

//    m_msg.circlePlane.scale.x = m_msg.circlePlane.scale.y = 10;
    m_msg.circlePlane.scale.z = 0.1;

    m_msg.circlePlane.pose.orientation.w = 1;

    /* Color */
    // White
    m_msg.circlePlane.color.r = 1.0;
    m_msg.circlePlane.color.g = 1.0;
    m_msg.circlePlane.color.b = 1.0;
    m_msg.circlePlane.color.a = 0.1;
}
void KiteVisualizer::setup_circle_plane_force()
{
    MarkerM &arrowMarker = m_msg.circlePlaneForce;

    /* Namespace and identifier */
    arrowMarker.ns = "circle_plane_force";
    arrowMarker.id = 0;

    /* Frame and time */
    arrowMarker.header.frame_id = "circle";
    arrowMarker.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    arrowMarker.frame_locked = true;

    /* Position and orientation */
    arrowMarker.pose.orientation.w = 1;

    arrowMarker.points.resize(2);

    /* Shape and scale */
    arrowMarker.type = MarkerM::ARROW;
    arrowMarker.scale.x = 0.05 * 5.0; // Shaft diameter
    arrowMarker.scale.y = 0.2 * 5.0; // Head diameter
    arrowMarker.scale.z = 0.2 * 5.0; // Head Length

    /* Color */
    // Yellow
    arrowMarker.color.r = 1.0;
    arrowMarker.color.g = 1.0;
    arrowMarker.color.b = 0.0;
    arrowMarker.color.a = 1.0;
}
void KiteVisualizer::setup_tether()
{
    /* Namespace and identifier */
    m_msg.tether.ns = "tether";
    m_msg.tether.id = 0;

    /* Frame and time */
//    m_msg.tether.header.stamp = RosTime(0);
    m_msg.tether.header.frame_id = "ned";
    m_msg.tether.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_msg.tether.frame_locked = false;

    /* Position and orientation */
    m_msg.tether.type = MarkerM::LINE_LIST;
    m_msg.tether.points.resize(2 * (1 + 3 + 3));

    m_msg.tether.scale.x = m_msg.tether.scale.y = m_msg.tether.scale.z = vehicle.force_scale * 0.1;

    m_msg.tether.pose.orientation.w = 1;

    /* Color */
    // White
    m_msg.tether.color.r = 1.0;
    m_msg.tether.color.g = 1.0;
    m_msg.tether.color.b = 1.0;
    m_msg.tether.color.a = 1.0;
}
void KiteVisualizer::setup_conpoint()
{
    /* Namespace and identifier */
    m_msg.conpoint.ns = "conpoint";
    m_msg.conpoint.id = 0;

    /* Frame and time */
    m_msg.conpoint.header.stamp = RosTime(0);
    m_msg.conpoint.header.frame_id = "ned";
    m_msg.conpoint.frame_locked = false;

    /* Color */
    // White
    m_msg.conpoint.color.r = 1.0;
    m_msg.conpoint.color.g = 1.0;
    m_msg.conpoint.color.b = 1.0;
    m_msg.conpoint.color.a = 1.0;

    /* Position and orientation */
    m_msg.conpoint.type = MarkerM::SPHERE;
    m_msg.conpoint.scale.x = m_msg.conpoint.scale.y = m_msg.conpoint.scale.z = 4 * vehicle.mesh_scale;

    m_msg.conpoint.pose.orientation.w = 1;
}
void KiteVisualizer::setup_teth_force(MarkerM &arrowMarker, MarkerM &textMarker)
{
    /** Arrow **/
    arrowMarker = MarkerM();

    /* Namespace and identifier */
    arrowMarker.ns = "teth_force";
    arrowMarker.id = 0;

    /* Frame and time */
    arrowMarker.header.frame_id = "body";
    arrowMarker.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    arrowMarker.frame_locked = true;

    /* Position and orientation */
    arrowMarker.pose.orientation.w = 1;

    arrowMarker.points.resize(2);

    /* Shape and scale */
    arrowMarker.type = MarkerM::ARROW;
    arrowMarker.scale.x = 0.05; // Shaft diameter
    arrowMarker.scale.y = 0.2; // Head diameter
    arrowMarker.scale.z = 0.2; // Head Length

    /* Color */
    // Yellow
    arrowMarker.color.r = 1.0;
    arrowMarker.color.g = 1.0;
    arrowMarker.color.b = 0.0;
    arrowMarker.color.a = 1.0;

    /** Label **/
    textMarker = MarkerM();

    /* Namespace and identifier */
    textMarker.ns = "tether_load_text";
    textMarker.header.frame_id = "ned";
    textMarker.id = 0;

    /* Shape and scale */
    textMarker.scale.x = textMarker.scale.y = textMarker.scale.z = 2.0;

    textMarker.type = MarkerM::TEXT_VIEW_FACING;
    textMarker.text = "-- KG";

    /* Color */
    // White
    textMarker.color.r = 1.0;
    textMarker.color.g = 1.0;
    textMarker.color.b = 1.0;
    textMarker.color.a = 1.0;
}

/* Callback functions ============================================================================= */
void KiteVisualizer::simulationStateCallback(const MultiDofJointStateM::SharedPtr msg)
{
    m_states[0].pos = {msg->transforms[0].translation.x,
                    msg->transforms[0].translation.y,
                    msg->transforms[0].translation.z};

    m_states[0].vel_in = {msg->twist[0].linear.x,
                       msg->twist[0].linear.y,
                       msg->twist[0].linear.z};

    m_states[0].q_ned_b = {msg->transforms[0].rotation.w,
                        msg->transforms[0].rotation.x,
                        msg->transforms[0].rotation.y,
                        msg->transforms[0].rotation.z};

    m_states[0].b_nongravForce = {msg->wrench[0].force.x,
                                  msg->wrench[0].force.y,
                                  msg->wrench[0].force.z};

    b_tethForce[0] = {msg->wrench[1].force.x,
                          msg->wrench[1].force.y,
                          msg->wrench[1].force.z};

    m_states[0].Va = msg->twist[1].linear.x;
    m_states[0].beta = msg->twist[1].linear.y;
    m_states[0].alpha = msg->twist[1].linear.z;

    m_states[0].Va_pitot = msg->twist[1].angular.x;

    sim_vW = {msg->twist[1].angular.y, msg->twist[1].angular.z};
}
void KiteVisualizer::controlCallback(const JoyM::SharedPtr msg)
{
    control[0] = {msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3]};
//    std::cout << "control[0]: " << control[0].transpose() << "\n";
}

void KiteVisualizer::simulationState2Callback(const MultiDofJointStateM::SharedPtr msg)
{
    m_states[1].pos = {msg->transforms[0].translation.x,
                       msg->transforms[0].translation.y,
                       msg->transforms[0].translation.z};

    m_states[1].vel_in = {msg->twist[0].linear.x,
                          msg->twist[0].linear.y,
                          msg->twist[0].linear.z};

    m_states[1].q_ned_b = {msg->transforms[0].rotation.w,
                           msg->transforms[0].rotation.x,
                           msg->transforms[0].rotation.y,
                           msg->transforms[0].rotation.z};

    m_states[1].b_nongravForce = {msg->wrench[0].force.x,
                                      msg->wrench[0].force.y,
                                      msg->wrench[0].force.z};

    b_tethForce[1] = {msg->wrench[1].force.x,
                          msg->wrench[1].force.y,
                          msg->wrench[1].force.z};

    m_states[1].Va = msg->twist[1].linear.x;
    m_states[1].beta = msg->twist[1].linear.y;
    m_states[1].alpha = msg->twist[1].linear.z;

    m_states[1].Va_pitot = msg->twist[1].angular.x;

//    sim_vW = {msg->twist[1].angular.y, msg->twist[1].angular.z};
}
void KiteVisualizer::control2Callback(const JoyM::SharedPtr msg)
{
    control[1] = {msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3]};
//    std::cout << "control[1]: " << control[1].transpose() << "\n";
}

void KiteVisualizer::simulationState3Callback(const MultiDofJointStateM::SharedPtr msg)
{
    m_states[2].pos = {msg->transforms[0].translation.x,
                    msg->transforms[0].translation.y,
                    msg->transforms[0].translation.z};

    m_states[2].vel_in = {msg->twist[0].linear.x,
                       msg->twist[0].linear.y,
                       msg->twist[0].linear.z};

    m_states[2].q_ned_b = {msg->transforms[0].rotation.w,
                        msg->transforms[0].rotation.x,
                        msg->transforms[0].rotation.y,
                        msg->transforms[0].rotation.z};

    m_states[2].b_nongravForce = {msg->wrench[0].force.x,
                                   msg->wrench[0].force.y,
                                   msg->wrench[0].force.z};

    b_tethForce[2] = {msg->wrench[1].force.x,
                          msg->wrench[1].force.y,
                          msg->wrench[1].force.z};

    m_states[2].Va = msg->twist[1].linear.x;
    m_states[2].beta = msg->twist[1].linear.y;
    m_states[2].alpha = msg->twist[1].linear.z;

    m_states[2].Va_pitot = msg->twist[1].angular.x;

    //    sim_vW = {msg->twist[1].angular.y, msg->twist[1].angular.z};
}
void KiteVisualizer::control3Callback(const JoyM::SharedPtr msg)
{
    control[2] = {msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3]};
    //    std::cout << "control[1]: " << control[1].transpose() << "\n";
}

void KiteVisualizer::simulationStateConpointCallback(const MultiDofJointStateM::SharedPtr msg)
{
    m_states[3].pos = {msg->transforms[0].translation.x,
                       msg->transforms[0].translation.y,
                       msg->transforms[0].translation.z};

    m_states[3].vel_in = m_states[2].vel_transformed = {msg->twist[0].linear.x,
                                                        msg->twist[0].linear.y,
                                                        msg->twist[0].linear.z};

    m_states[3].b_nongravForce = {msg->wrench[0].force.x,
                                  msg->wrench[0].force.y,
                                  msg->wrench[0].force.z};
}
void KiteVisualizer::simulationStateBaseCallback(const MultiDofJointStateM::SharedPtr msg)
{
    m_states[4].b_nongravForce = {msg->wrench[0].force.x,
                                  msg->wrench[0].force.y,
                                  msg->wrench[0].force.z};
}

void KiteVisualizer::pxPoseCallback(const PoseStampedM::SharedPtr &msg)
{
    /* Message in ENU -> transform to NED */
    m_states[0].pos = {msg->pose.position.y, msg->pose.position.x, -msg->pose.position.z};

    /* Rotation from Front-Left-Up body frame to ENU frame */
    const Eigen::Quaterniond q_enu_flu = {msg->pose.orientation.w,
                                          msg->pose.orientation.x,
                                          msg->pose.orientation.y,
                                          msg->pose.orientation.z};
    m_states[0].q_ned_b = q_ned_enu * q_enu_flu * q_flu_b;
}
void KiteVisualizer::pxVelBodCallback(const TwistStampedM::SharedPtr msg)
{
    /* Message in Front-Left-Up body frame */
    m_states[0].vel_in = {msg->twist.linear.x, -msg->twist.linear.y, -msg->twist.linear.z};
}
void KiteVisualizer::pxImuDataCallback(const ImuM::SharedPtr msg)
{
    /* Message in Front-Left-Up body frame */
    m_states[0].b_nongravForce = {msg->linear_acceleration.x, -msg->linear_acceleration.y,
                                   -msg->linear_acceleration.z};
}
void KiteVisualizer::pxWindEstimationCallback(const TwistWithCovarianceStampedM::SharedPtr msg)
{
    /* Filtered wind estimation by PX4 */
    sim_vW = {msg->twist.twist.linear.y, msg->twist.twist.linear.x};
}
void KiteVisualizer::pxVfrHudCallback(const VfrHudM::SharedPtr msg)
{
    m_states[0].Va_pitot = msg->airspeed;
}
void KiteVisualizer::controlCmdCallback(const JoyM::SharedPtr msg)
{
    control[0] = {msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3]};
}

//void KiteVisualizer::get_circle_transform(TransformStampedM &tf_c_wd)
//{
//    // Define circle frame from visualization node parameters
//      tf_c_wd.header.frame_id = "winddome";
//
//    /* Position */
//    p.circle.hRef = std::sqrt(p.teth_len * p.teth_len - p.circle.radiusRef * p.circle.radiusRef);
//    tf_c_wd.transform.translation.x = -p.circle.hRef * sin(p.circle.declination);
//    tf_c_wd.transform.translation.z = -p.circle.hRef * cos(p.circle.declination);
//
//    /* Rotation */
//    const Eigen::Quaterniond q_c_wd(Eigen::AngleAxisd(p.circle.declination, Eigen::Vector3d::UnitY()));
//    tf_c_wd.transform.rotation.w = q_c_wd.w();
//    tf_c_wd.transform.rotation.x = q_c_wd.x();
//    tf_c_wd.transform.rotation.y = q_c_wd.y();
//    tf_c_wd.transform.rotation.z = q_c_wd.z();
//}
void KiteVisualizer::get_circle_transform(TransformStampedM &tf_cn)
{
    // Define circle frame from kite positions
    tf_cn.header.frame_id = "ned";

    // Multi-kite average position defines "soft" (time-varying winddome) frame wd2
    Eigen::Vector3d n_circle_pos(0, 0, 0);
    for (unsigned k = 0; k < N_vehicles; k++)
    {
        n_circle_pos += m_states[k].pos / static_cast<double>(N_vehicles);
    }

    const double bearing = std::atan2(n_circle_pos(1), n_circle_pos(0));
    const double declination = std::atan2(n_circle_pos.segment<2>(0).norm(), -n_circle_pos(2));

//    std::cout << "brg: " << bearing * 180.0/M_PI << ", decl: " << declination * 180.0/M_PI << "\n";
    using namespace Eigen;
    const Quaterniond q_cn = Quaterniond(AngleAxisd(bearing + M_PI, Vector3d::UnitZ())) *
                             Quaterniond(AngleAxisd(declination, Vector3d::UnitY()));

    /* Position */
    tf_cn.transform.translation.x = n_circle_pos(0);
    tf_cn.transform.translation.y = n_circle_pos(1);
    tf_cn.transform.translation.z = n_circle_pos(2);

    /* Rotation */
    tf_cn.transform.rotation.w = q_cn.w();
    tf_cn.transform.rotation.x = q_cn.x();
    tf_cn.transform.rotation.y = q_cn.y();
    tf_cn.transform.rotation.z = q_cn.z();
}

/* Publishing functions =========================================================================== */
/* Publish when parameters have changed */
void KiteVisualizer::broadcastWinddomeTransform()
{
    TransformStampedM tf_wd_ned{};     // All entries = 0

    /* Reference */
    tf_wd_ned.header.stamp = rclcpp::Node::now();
    tf_wd_ned.header.frame_id = "ned";
    tf_wd_ned.child_frame_id = "winddome";

    /* Rotation */
    const Eigen::Quaterniond q_wd_ned(Eigen::AngleAxisd(p.windFrom_deg * M_PI / 180.0 - M_PI, Eigen::Vector3d::UnitZ()));
    tf_wd_ned.transform.rotation.w = q_wd_ned.w();
    tf_wd_ned.transform.rotation.x = q_wd_ned.x();
    tf_wd_ned.transform.rotation.y = q_wd_ned.y();
    tf_wd_ned.transform.rotation.z = q_wd_ned.z();

    m_tf_static_broadcaster->sendTransform(tf_wd_ned);
}
void KiteVisualizer::publishSphere() const
{
    MarkerM sphere{};    // All entries = 0

    /* Namespace and identifier */
    sphere.ns = "sphere";
    sphere.id = 0;

    /* Frame and time */
//        sphere.header.stamp = rclcpp::Node::now();
    sphere.header.frame_id = "winddome";

    /* Position and orientation */
    sphere.pose.orientation.w = 1;

    /* Shape and scale */
    sphere.type = MarkerM::LINE_STRIP;
    sphere.scale.x = 0.4; // Linewidth

    /* Color and state[0].alpha */
    // White, transparent
    sphere.color.r = 1;
    sphere.color.g = 1;
    sphere.color.b = 1;
    sphere.color.a = 0.1;

    const int nLatsPerHemisphere = 7;
    const int nLats = 2 * nLatsPerHemisphere + 1;
    double lats[nLats];
    double degreesPerLine = 90.0 / (nLatsPerHemisphere + 1);

    int idCounter{0};
    int nPoints = 100;
    /* Distribute latitudes */
    for (int iLat = 0; iLat < nLats; ++iLat)
    {
        lats[iLat] = -90.0 + (iLat + 1) * degreesPerLine;
        /* Draw one line */
        sphere.points.resize(nPoints);
        for (int iPoint = 0; iPoint < nPoints; ++iPoint)
        {
            sphere.points[iPoint].x = p.teth_len * sin(lats[iLat] * M_PI / 180.0);
            sphere.points[iPoint].y =
                    p.teth_len * sin(iPoint * M_PI / nPoints - M_PI / 2.0) * cos(lats[iLat] * M_PI / 180.0);
            sphere.points[iPoint].z =
                    p.teth_len * -cos(iPoint * M_PI / nPoints - M_PI / 2.0) * cos(lats[iLat] * M_PI / 180.0);
        }

        sphere.id = idCounter++;
        /* Publish */
        m_pub.env->publish(sphere);
    }

    const int nLonsPerHemisphere = 3;
    const int nLons = 2 * nLonsPerHemisphere + 3;
    double lons[nLons];
    degreesPerLine = 90.0 / (nLonsPerHemisphere + 1);
    /* Distribute longitudes */
    for (int iLon = 0; iLon < nLons; ++iLon)
    {
        lons[iLon] = -90.0 + iLon * degreesPerLine;
        /* Draw one line */
        sphere.points.resize(nPoints);
        for (int iPoint = 0; iPoint < nPoints; ++iPoint)
        {
            sphere.points[iPoint].x = p.teth_len * sin(iPoint * M_PI / nPoints - M_PI / 2.0);
            sphere.points[iPoint].y =
                    p.teth_len * sin(lons[iLon] * M_PI / 180.0) * cos(iPoint * M_PI / nPoints - M_PI / 2.0);
            sphere.points[iPoint].z =
                    p.teth_len * -cos(lons[iLon] * M_PI / 180.0) * cos(iPoint * M_PI / nPoints - M_PI / 2.0);
        }

        sphere.id = idCounter++;
        /* Publish */
        m_pub.env->publish(sphere);
    }
}
void KiteVisualizer::publishPathRef() const
{
    MarkerM pathref{};

    /* Namespace and identifier */
    pathref.ns = "pathRef";
    pathref.id = 0;

    /* Frame and time */
    pathref.header.stamp = rclcpp::Node::now();
    pathref.header.frame_id = "circle";

    /* Position and orientation */
    pathref.pose.orientation.w = 1;

    /* Shape and scale */
    pathref.type = MarkerM::LINE_LIST;
    pathref.scale.x = 0.15; // Linewidth

    /* Color and state[0].alpha */
    // magenta
    pathref.color.r = 0.89;
    pathref.color.g = 0.19;
    pathref.color.b = 0.89;
    pathref.color.a = 1.0;

    /* Points */
    int nPoints = 100;
    pathref.points.resize(nPoints);

    for (int iPoint = 0; iPoint < nPoints; ++iPoint)
    {
        pathref.points[iPoint].x = p.circle.radiusRef * cos(iPoint * 2 * M_PI / nPoints);
        pathref.points[iPoint].y = p.circle.radiusRef * sin(iPoint * 2 * M_PI / nPoints);
    }

    /* Publish */
    m_pub.env->publish(pathref);
}
void KiteVisualizer::publishResetMpcStateTraj()
{
    MarkerArrayM poseMarkers{};
    MarkerArrayM virtStateMarkers{};

    /* Loop through prediction time steps */
    for (int iStep = 0; iStep < N_TRAJ_POINTS; ++iStep)
    {

        /* Pose marker -------------------------------------------------- */
        MarkerM iPoseMarker{};     // All entries = 0

        iPoseMarker.ns = "predicted_trajectory";
        iPoseMarker.id = iStep;
        iPoseMarker.action = MarkerM::DELETEALL;

        iPoseMarker.header.stamp = rclcpp::Node::now();
        iPoseMarker.header.frame_id = "ned";

        poseMarkers.markers.push_back(iPoseMarker);

        /* Virtual state marker ------------------------------------------- */
        MarkerM iVirtStateMarker{};     // All entries = 0

        iVirtStateMarker.ns = "virtual_state";
        iVirtStateMarker.id = iStep;

        iVirtStateMarker.header.frame_id = "circle";

        iVirtStateMarker.action = MarkerM::DELETEALL;

        virtStateMarkers.markers.push_back(iVirtStateMarker);
    }

    m_pub.fastArray->publish(poseMarkers);
    m_pub.fastArray->publish(virtStateMarkers);
}

/* Publish once */
void KiteVisualizer::publishBody()
{
    /* Namespace and identifier */
    m_msg.body.ns = "kite";
    m_msg.body.id = 0;
    /* Frame and time */
    m_msg.body.header.stamp = RosTime(0);
    m_msg.body.header.frame_id = "body";
    m_msg.body.frame_locked = true;
    /* Color */
    // White
    m_msg.body.color.r = 1.0;
    m_msg.body.color.g = 1.0;
    m_msg.body.color.b = 1.0;
    m_msg.body.color.a = 1.0;
    /* Position and orientation */
    getBodyMarker(m_msg.body, m_states[0].q_ned_b);
    /* Publish */
    m_pub.env->publish(m_msg.body);

    if (N_vehicles >= 2)
    {
        /* Namespace and identifier */
        m_msg.body.ns = "kite2";
        m_msg.body.id = 0;
        /* Frame and time */
        m_msg.body.header.stamp = RosTime(0);
        m_msg.body.header.frame_id = "body2";
        m_msg.body.frame_locked = true;
        /* Color */
        // White
        m_msg.body.color.r = 1.0;
        m_msg.body.color.g = 1.0;
        m_msg.body.color.b = 1.0;
        m_msg.body.color.a = 1.0;
        /* Position and orientation */
        getBodyMarker(m_msg.body, m_states[0].q_ned_b);
        /* Publish */
        m_pub.env->publish(m_msg.body);

        if (N_vehicles >= 3)
        {
            /* Namespace and identifier */
            m_msg.body.ns = "kite3";
            m_msg.body.id = 0;
            /* Frame and time */
            m_msg.body.header.stamp = RosTime(0);
            m_msg.body.header.frame_id = "body3";
            m_msg.body.frame_locked = true;
            /* Color */
            // White
            m_msg.body.color.r = 1.0;
            m_msg.body.color.g = 1.0;
            m_msg.body.color.b = 1.0;
            m_msg.body.color.a = 1.0;
            /* Position and orientation */
            getBodyMarker(m_msg.body, m_states[0].q_ned_b);
            /* Publish */
            m_pub.env->publish(m_msg.body);
        }
    }
}

/* Publish continuously */
void KiteVisualizer::broadcastCircleTransform()
{
//    TransformStampedM tf_c_wd{};     // All entries = 0

    /* Reference */
    m_tfm.c_ned.header.stamp = m_pub_time;
//    m_tfs.c_ned.header.frame_id = "winddome"; | "ned" // Set in get_winddome_to_circle
    m_tfm.c_ned.child_frame_id = "circle";

    get_circle_transform(m_tfm.c_ned);

    m_tf_broadcaster->sendTransform(m_tfm.c_ned);
}

void KiteVisualizer::prepareCirclePlane()
{
    Eigen::Vector<double, 3> Rs;
    for (unsigned k = 0; k < N_vehicles; k++)
    {
        double dist = (k < N_vehicles - 1) ?
                                (m_states[k].pos - m_states[k + 1].pos).norm() :
                                (m_states[k].pos - m_states[0].pos).norm();
        Rs[k] = dist / (2.0 * std::cos((M_PI / N_vehicles) / 2.0));
    }
    m_msg.circlePlane.scale.x = m_msg.circlePlane.scale.y = 2.0 * Rs.mean();
}
void KiteVisualizer::prepareCirclePlaneForce()
{
    Eigen::Vector3d n_nongravForce(0, 0, 0);
    Eigen::Vector3d n_tetherForce(0, 0, 0);

    for (unsigned k = 0; k < N_vehicles; k++)
    {
        n_nongravForce += m_states[k].q_ned_b * m_states[k].b_nongravForce;
        n_tetherForce += m_states[k].q_ned_b * b_tethForce[k];
    }

    const Eigen::Quaterniond q_c_ned(m_tfm.c_ned.transform.rotation.w,
                                     m_tfm.c_ned.transform.rotation.x,
                                     m_tfm.c_ned.transform.rotation.y,
                                     m_tfm.c_ned.transform.rotation.z);
    const Eigen::Vector3d c_nongravForce_tot = q_c_ned * (n_nongravForce + n_tetherForce - N_vehicles * Eigen::Vector3d(0, 0, -9.81));
//    const Eigen::Vector3d c_nongravForce_tot = q_c_ned * (n_nongravForce - N_kts * Eigen::Vector3d(0, 0, -9.81));
//    const Eigen::Vector3d c_nongravForce_tot = q_c_ned * (n_nongravForce);

    MarkerM &arrowMarker = m_msg.circlePlaneForce;
    arrowMarker.header.stamp = m_pub_time;
    const double scaler{0.5 / 9.81};
    arrowMarker.points[1].x = scaler * c_nongravForce_tot(0);
    arrowMarker.points[1].y = scaler * c_nongravForce_tot(1);
    arrowMarker.points[1].z = scaler * c_nongravForce_tot(2);
}

void KiteVisualizer::prepareControl()
{
    /* Control surface markers ------------------------------------------- */
    m_msg.control.ns = "control";
    m_msg.control.id = 0;

    m_msg.control.header.stamp = m_pub_time;
    m_msg.control.header.frame_id = "body";
    m_msg.control.frame_locked = true;

    /* Position and orientation */
    m_msg.control.pose.orientation.w = 1;

    m_msg.control.type = MarkerM::LINE_LIST;
    m_msg.control.scale.x = 0.02;

    m_msg.control.color.r = 1.0;
    m_msg.control.color.g = 1.0;
    m_msg.control.color.b = 1.0;
    m_msg.control.color.a = 1.0;

    getControlMarkerPoints(m_msg.control, control[0]);

//    /* Thrust (propeller) marker ------------------------------------------- */
//    MarkerM propMarker{};
//
//    propMarker.ns = "control";
//    propMarker.id = 1;
//
//    propMarker.header.stamp = rclcpp::Node::now();
//    propMarker.header.frame_id = "body";
//
//    /* Position and orientation */
//    propMarker.pose.position.x += 0.25;
//    propMarker.pose.orientation.w = 0.707;
//    propMarker.pose.orientation.y = 0.707;
//
//    propMarker.type = MarkerM::CYLINDER;
//    propMarker.scale.x = propMarker.scale.y = control(0) / 6.3739;
//    propMarker.scale.z = 0.01;
//
//    propMarker.color.r = 1.0;
//    propMarker.color.g = 1.0;
//    propMarker.color.b = 1.0;
//    propMarker.color.a = 1.0;

    /* Control surface markers ------------------------------------------- */
    m_msg.control2.ns = "control";
    m_msg.control2.id = 1;

    m_msg.control2.header.stamp = m_pub_time;
    m_msg.control2.header.frame_id = "body2";
    m_msg.control2.frame_locked = true;

    /* Position and orientation */
    m_msg.control2.pose.orientation.w = 1;

    m_msg.control2.type = MarkerM::LINE_LIST;
    m_msg.control2.scale.x = 0.02;

    m_msg.control2.color.r = 1.0;
    m_msg.control2.color.g = 1.0;
    m_msg.control2.color.b = 1.0;
    m_msg.control2.color.a = 1.0;

    getControlMarkerPoints(m_msg.control2, control[1]);

    /* Control surface markers ------------------------------------------- */
    m_msg.control3.ns = "control";
    m_msg.control3.id = 2;

    m_msg.control3.header.stamp = m_pub_time;
    m_msg.control3.header.frame_id = "body3";
    m_msg.control3.frame_locked = true;

    /* Position and orientation */
    m_msg.control3.pose.orientation.w = 1;

    m_msg.control3.type = MarkerM::LINE_LIST;
    m_msg.control3.scale.x = 0.02;

    m_msg.control3.color.r = 1.0;
    m_msg.control3.color.g = 1.0;
    m_msg.control3.color.b = 1.0;
    m_msg.control3.color.a = 1.0;

    getControlMarkerPoints(m_msg.control3, control[2]);
}

void KiteVisualizer::prepareTether()
{
    // Precompute kite wingtip positions in NED frame
    const Eigen::Vector3d b_tethlat_attch_pos = kite.b_tethlat_attch_pos;
    const Eigen::Vector3d b_tethlon_attch_pos = kite.b_tethlon_attch_pos;

    const Eigen::Vector3d n_tethlat_attch_pos_k2 = m_states[2].pos + m_states[2].q_ned_b * b_tethlat_attch_pos;

    m_msg.tether.header.stamp = m_pub_time;

    // Tether origin - conpoint
    m_msg.tether.points[0].x = 0;
    m_msg.tether.points[0].y = 0;
    m_msg.tether.points[0].z = 0;

    m_msg.tether.points[1].x = m_states[3].pos(0);
    m_msg.tether.points[1].y = m_states[3].pos(1);
    m_msg.tether.points[1].z = m_states[3].pos(2);

    // Tether conpoint - kite0
    m_msg.tether.points[2].x = m_states[3].pos(0);
    m_msg.tether.points[2].y = m_states[3].pos(1);
    m_msg.tether.points[2].z = m_states[3].pos(2);

    const Eigen::Vector3d n_tethlon_attch_pos_k0 = m_states[0].pos + m_states[0].q_ned_b * b_tethlon_attch_pos;
    m_msg.tether.points[3].x = n_tethlon_attch_pos_k0(0);
    m_msg.tether.points[3].y = n_tethlon_attch_pos_k0(1);
    m_msg.tether.points[3].z = n_tethlon_attch_pos_k0(2);

    if (N_vehicles >= 2)
    {
        // Tether conpoint - kite1
        m_msg.tether.points[4].x = m_states[3].pos(0);
        m_msg.tether.points[4].y = m_states[3].pos(1);
        m_msg.tether.points[4].z = m_states[3].pos(2);

        const Eigen::Vector3d n_tethlon_attch_pos_k1 = m_states[1].pos + m_states[1].q_ned_b * b_tethlon_attch_pos;
        m_msg.tether.points[5].x = n_tethlon_attch_pos_k1(0);
        m_msg.tether.points[5].y = n_tethlon_attch_pos_k1(1);
        m_msg.tether.points[5].z = n_tethlon_attch_pos_k1(2);

        // Tether kite0 - kite1
        const Eigen::Vector3d n_tethlat_attch_pos_k0 = m_states[0].pos + m_states[0].q_ned_b * b_tethlat_attch_pos;
        const Eigen::Vector3d n_tethlat_attch_pos_k1 = m_states[1].pos + m_states[1].q_ned_b * b_tethlat_attch_pos;

        m_msg.tether.points[8].x = n_tethlat_attch_pos_k0(0);
        m_msg.tether.points[8].y = n_tethlat_attch_pos_k0(1);
        m_msg.tether.points[8].z = n_tethlat_attch_pos_k0(2);

        m_msg.tether.points[9].x = n_tethlat_attch_pos_k1(0);
        m_msg.tether.points[9].y = n_tethlat_attch_pos_k1(1);
        m_msg.tether.points[9].z = n_tethlat_attch_pos_k1(2);

        if (N_vehicles >= 3)
        {
            // Tether conpoint - kite2
            m_msg.tether.points[6].x = m_states[3].pos(0);
            m_msg.tether.points[6].y = m_states[3].pos(1);
            m_msg.tether.points[6].z = m_states[3].pos(2);

            const Eigen::Vector3d n_tethlon_attch_pos_k2 = m_states[2].pos + m_states[2].q_ned_b * b_tethlon_attch_pos;
            m_msg.tether.points[7].x = n_tethlon_attch_pos_k2(0);
            m_msg.tether.points[7].y = n_tethlon_attch_pos_k2(1);
            m_msg.tether.points[7].z = n_tethlon_attch_pos_k2(2);

            // Tether kite1 - kite2
            m_msg.tether.points[10].x = n_tethlat_attch_pos_k1(0);
            m_msg.tether.points[10].y = n_tethlat_attch_pos_k1(1);
            m_msg.tether.points[10].z = n_tethlat_attch_pos_k1(2);

            m_msg.tether.points[11].x = n_tethlat_attch_pos_k2(0);
            m_msg.tether.points[11].y = n_tethlat_attch_pos_k2(1);
            m_msg.tether.points[11].z = n_tethlat_attch_pos_k2(2);

            // Tether kite2 - kite0
            m_msg.tether.points[12].x = n_tethlat_attch_pos_k2(0);
            m_msg.tether.points[12].y = n_tethlat_attch_pos_k2(1);
            m_msg.tether.points[12].z = n_tethlat_attch_pos_k2(2);

            m_msg.tether.points[13].x = n_tethlat_attch_pos_k0(0);
            m_msg.tether.points[13].y = n_tethlat_attch_pos_k0(1);
            m_msg.tether.points[13].z = n_tethlat_attch_pos_k0(2);
        }
    }
}

void KiteVisualizer::prepareTethForce(MarkerM &arrowMarker, MarkerM &textMarker,
                                      const FlightState &state, const Eigen::Vector3d &b_tethForce)
{
    /* Arrow */
    arrowMarker.header.stamp = m_pub_time;
    const double scaler{0.5 / 9.81};
    arrowMarker.points[1].x = scaler * b_tethForce(0);
    arrowMarker.points[1].y = scaler * b_tethForce(1);
    arrowMarker.points[1].z = scaler * b_tethForce(2);

    /* Label */
    textMarker.header.stamp = m_pub_time;
    textMarker.pose.position.x = state.pos(0);
    textMarker.pose.position.y = state.pos(1);
    textMarker.pose.position.z = state.pos(2) + vehicle.trace_scale * 15;

    const double load_rounded = std::round(b_tethForce.norm() / 9.81 * 10.0) / 10.0;
    std::string load_rounded_str = std::to_string(load_rounded);
    const auto decimal_pos = load_rounded_str.find_first_of('.');
    load_rounded_str = load_rounded_str.substr(0, decimal_pos + 2);

    textMarker.text = load_rounded_str + " KG";
}
void KiteVisualizer::prepareConpoint()
{
    m_msg.conpoint.pose.position.x = m_states[3].pos(0);
    m_msg.conpoint.pose.position.y = m_states[3].pos(1);
    m_msg.conpoint.pose.position.z = m_states[3].pos(2);
}

void KiteVisualizer::prepareMpcStateTraj()
{
    MarkerArrayM poseMarkers{};
    MarkerArrayM virtStateMarkers{};
    MarkerArrayM controlMarkers{};

    RosTime stamp = rclcpp::Node::now();

    MarkerM control_ref_line{};
    control_ref_line.ns = "sngl_ctrl_trj_ntrl";
    control_ref_line.id = 0;
    control_ref_line.header.stamp = stamp;
    control_ref_line.header.frame_id = "ned";
    control_ref_line.pose.orientation.w = 1;
    control_ref_line.type = MarkerM::LINE_STRIP;
    control_ref_line.scale.x = 0.025;
    control_ref_line.points.resize(n_traj_points);
    control_ref_line.color.a = 1.0;
    control_ref_line.color.r = 0.7;
    control_ref_line.color.g = 0.7;
    control_ref_line.color.b = 0.7;

    MarkerM singleControlTraj{};
    singleControlTraj.ns = "sngl_ctrl_trj";
    singleControlTraj.id = 0;
    singleControlTraj.header.stamp = stamp;
    singleControlTraj.header.frame_id = "ned";
    singleControlTraj.pose.orientation.w = 1;
    singleControlTraj.type = MarkerM::LINE_STRIP;
    singleControlTraj.scale.x = 0.025;
    singleControlTraj.points.resize(n_traj_points);
    singleControlTraj.color.a = 1.0;
    singleControlTraj.color.r = 1.0;
    singleControlTraj.color.g = 1.0;
    singleControlTraj.color.b = 1.0;

    MarkerM singleControlTrajDefl{};
    singleControlTrajDefl.ns = "sngl_ctrl_trj_dfl";
    singleControlTrajDefl.id = 0;
    singleControlTraj.header.stamp = stamp;
    singleControlTrajDefl.header.frame_id = "ned";
    singleControlTrajDefl.pose.orientation.w = 1;
    singleControlTrajDefl.type = MarkerM::LINE_LIST;
    singleControlTrajDefl.scale.x = 0.025;
    singleControlTrajDefl.points.resize(2 * n_traj_points);
    singleControlTrajDefl.color.a = 1.0;
    singleControlTrajDefl.color.r = 1.0;
    singleControlTrajDefl.color.g = 0.0;
    singleControlTrajDefl.color.b = 0.0;

    /* Loop through prediction time steps */
    for (unsigned iStep = 0; iStep < n_traj_points; ++iStep)
    {
        if (mpc_traj_poses[iStep].position.x != 0.0)
        {
            /* Pose marker -------------------------------------------------- */
            MarkerM iPoseMarker{};     // All entries = 0

            if (mpc_traj_poses[iStep].orientation.y != 0)
                getBodyMarker(iPoseMarker, mpc_traj_poses[iStep]);
            else
                getBallMarker(iPoseMarker, mpc_traj_poses[iStep]);

            iPoseMarker.ns = "state_traj";
            iPoseMarker.id = iStep;

            iPoseMarker.header.stamp = stamp;
            iPoseMarker.header.frame_id = "ned";

            /* Color */
            if (mpc_traj_isValid)
            {
                iPoseMarker.color.r = blue.r;
                iPoseMarker.color.g = blue.g;
                iPoseMarker.color.b = blue.b;
                iPoseMarker.color.a = 1.0;
            }
            else
            {
                // yellow
                iPoseMarker.color.r = yellow.r;
                iPoseMarker.color.g = yellow.g;
                iPoseMarker.color.b = yellow.b;
                iPoseMarker.color.a = 1.0;
            }

            poseMarkers.markers.push_back(iPoseMarker);

            /* Virtual state marker ------------------------------------------- */
            MarkerM iVirtStateMarker{};     // All entries = 0

            iVirtStateMarker.ns = "virt_state_traj";
            iVirtStateMarker.id = iStep;

            /* Frame and time */
            iVirtStateMarker.header.stamp = stamp;
            iVirtStateMarker.header.frame_id = "circle";

            /* Position and orientation */
            iVirtStateMarker.pose.orientation.w = 1;
            iVirtStateMarker.pose.position.x = p.circle.radiusRef * cos(mpc_traj_virtStates[iStep] * 2 * M_PI);
            iVirtStateMarker.pose.position.y = p.circle.radiusRef * sin(mpc_traj_virtStates[iStep] * 2 * M_PI);

            /* Shape and scale */
//            iVirtStateMarker.type = MarkerM::SPHERE;
//            iVirtStateMarker.scale.x = iVirtStateMarker.scale.y = iVirtStateMarker.scale.z = 0.7;
            iVirtStateMarker.type = MarkerM::TEXT_VIEW_FACING;
            iVirtStateMarker.text = std::to_string(
                    iStep);// + " (" + std::to_string(mpc_traj_virtStates[iStep]) + ")";
            iVirtStateMarker.scale.z = 1;

            /* Color */
            if (mpc_traj_isValid)
            {
                iVirtStateMarker.color.r = blue.r;
                iVirtStateMarker.color.g = blue.g;
                iVirtStateMarker.color.b = blue.b;
                iVirtStateMarker.color.a = 1.0;
            }
            else
            {
                // yellow
                iVirtStateMarker.color.r = yellow.r;
                iVirtStateMarker.color.g = yellow.g;
                iVirtStateMarker.color.b = yellow.b;
                iVirtStateMarker.color.a = 1.0;
            }

            virtStateMarkers.markers.push_back(iVirtStateMarker);

            /* Control surface markers ------------------------------------------- */
            MarkerM controlMarker{};
            controlMarker.ns = "control_traj";
            controlMarker.id = iStep;

            controlMarker.header.stamp = stamp;
            controlMarker.header.frame_id = "ned";

            /* Position and orientation */
            controlMarker.pose.position = mpc_traj_poses[iStep].position;
            controlMarker.pose.orientation = mpc_traj_poses[iStep].orientation;

            controlMarker.type = MarkerM::LINE_LIST;
            controlMarker.scale.x = 0.05;

            getControlMarkerPoints(controlMarker, mpc_control_traj.row(iStep));

            controlMarker.color.r = 1.0;
            controlMarker.color.g = 1.0;
            controlMarker.color.b = 1.0;
            controlMarker.color.a = 1.0;

            controlMarkers.markers.push_back(controlMarker);

            /* Aileron line ------------------------------------------- */
            /* Ref line */
            control_ref_line.points[iStep] = mpc_traj_poses[iStep].position;
            /* Deflected line */
            singleControlTraj.points[iStep] = mpc_traj_poses[iStep].position;
//        singleControlTraj.points[iStep].z += 4 * mpc_control_traj(iStep, 0) / 6.3739; // thrust
            singleControlTraj.points[iStep].z += 4 * mpc_control_traj(iStep, 1) / 0.3665; // elev
//        singleControlTraj.points[iStep].z += 4 * mpc_control_traj(iStep, 2) / 0.4625; // rudder
//        singleControlTraj.points[iStep].z += 4 * mpc_control_traj(iStep, 3) / 0.3578; // aileron
            /* Deflection lines */
            singleControlTrajDefl.points[2 * iStep] = mpc_traj_poses[iStep].position;
            singleControlTrajDefl.points[2 * iStep + 1] = mpc_traj_poses[iStep].position;
//        singleControlTrajDefl.points[2 * iStep + 1].z += 4 * mpc_control_traj(iStep, 0) / 6.3739; // thrust
            singleControlTrajDefl.points[2 * iStep + 1].z += 4 * mpc_control_traj(iStep, 1) / 0.3665; // elev
//        singleControlTrajDefl.points[2 * iStep + 1].z += 4 * mpc_control_traj(iStep, 2) / 0.4625; // rudder
//        singleControlTrajDefl.points[2 * iStep + 1].z += 4 * mpc_control_traj(iStep, 3) / 0.3578; // aileron
        }
    }
//    std::cout << "\n";

    m_pub.fastArray->publish(poseMarkers);
    m_pub.fastArray->publish(virtStateMarkers);
    m_pub.fastArray->publish(controlMarkers);
    m_kitepub.control->publish(control_ref_line);
    m_kitepub.control->publish(singleControlTraj);
    m_kitepub.control->publish(singleControlTrajDefl);
}

/* Helper functions =============================================================================== */
void KiteVisualizer::getBallMarker(MarkerM &marker, const Eigen::Vector3d &position)
{
    /* Position and orientation */
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
    marker.pose.orientation.w = 1;

    /* Shape and scale */
    marker.type = MarkerM::SPHERE;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
}
void KiteVisualizer::getBallMarker(MarkerM &marker, const PoseM &pose)
{
    Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);

    getBallMarker(marker, position);
}

void KiteVisualizer::getControlMarkerPoints(MarkerM &marker, const Eigen::Matrix<double, 1, 4> &ctrl) const
{
    marker.points.resize(10);
    /* Thrust */
    const Eigen::Vector3d b_thr_r = kite.b_thr_r;
    marker.points[0].x = marker.points[1].x = b_thr_r(0);
    marker.points[0].y = marker.points[1].y = b_thr_r(1);
    marker.points[0].z = marker.points[1].z = b_thr_r(2);

    marker.points[1].x += ctrl(0) / 6.3739 * cos(3.0 * M_PI / 180.0);
    marker.points[1].z += ctrl(0) / 6.3739 * sin(3.0 * M_PI / 180.0);

    /* Elevator */
    marker.points[2].x = marker.points[3].x = -0.55;
    marker.points[2].z = marker.points[3].z = 0;
    marker.points[3].z += ctrl(1) / 0.3665;

    /* Rudder */
    marker.points[4].x = marker.points[5].x = -0.44;
    marker.points[4].y = marker.points[5].y = 0;
    marker.points[4].z = marker.points[5].z = -0.1;
    marker.points[5].y += -ctrl(2) / 0.4625;

    /* Aileron right */
    marker.points[6].x = marker.points[7].x = 0.03;
    marker.points[6].y = marker.points[7].y = 0.8;
    marker.points[6].z = marker.points[7].z = -0.05;
    marker.points[7].z += ctrl(3) / 0.3578;

    /* Aileron left */
    marker.points[8].x = marker.points[9].x = 0.03;
    marker.points[8].y = marker.points[9].y = -0.8;
    marker.points[8].z = marker.points[9].z = -0.05;
    marker.points[9].z += -ctrl(3) / 0.3578;
}
