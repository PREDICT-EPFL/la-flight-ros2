#include "visualization_flight_ros2/VisualizerFlightRos2Base.hpp"

using namespace visualization_ros2::flight;

void VisualizerFlightRos2Base::init_impl()
{
    setup_trace();
    setup_vel_geo();
    setup_vel_aero();
    setup_nongrav_force();
    setup_wind();

    m_pub_time = rclcpp::Node::now();
    clear_all_markers();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    broadcastNedTransfrom();
    broadcastBodyTransform();

    if (N_vehicles >= 2)
    {
        m_msg.trace2 = m_msg.trace;
        m_msg.trace2.ns = "trace2";

        m_msg.vel_geo2 = m_msg.vel_geo;
        m_msg.vel_geo2.id = 1;
        m_msg.vel_geo2.header.frame_id = "body2";

        m_msg.vel_aero2 = m_msg.vel_aero;
        m_msg.vel_aero2.id = 1;
        m_msg.vel_aero2.header.frame_id = "body2";

        m_msg.nongravForce2 = m_msg.nongravForce;
        m_msg.nongravForce2.id = 1;
        m_msg.nongravForce2.header.frame_id = "body2";

        broadcastBodyTransform2();
    }
    if (N_vehicles >= 3)
    {
        m_msg.trace3 = m_msg.trace;
        m_msg.trace3.ns = "trace3";

        m_msg.vel_geo3 = m_msg.vel_geo;
        m_msg.vel_geo3.id = 2;
        m_msg.vel_geo3.header.frame_id = "body3";

        m_msg.vel_aero3 = m_msg.vel_aero;
        m_msg.vel_aero3.id = 2;
        m_msg.vel_aero3.header.frame_id = "body3";

        m_msg.nongravForce3 = m_msg.nongravForce;
        m_msg.nongravForce3.id = 2;
        m_msg.nongravForce3.header.frame_id = "body3";

        broadcastBodyTransform3();
    }
}

void VisualizerFlightRos2Base::update_out_impl()
{
    m_pub_time = rclcpp::Node::now();
    m_markerArray_fast.markers.clear();

    broadcastBodyTransform();

    publishTrace();

    prepareGeodeticVel();
    m_markerArray_fast.markers.push_back(m_msg.vel_geo);

    prepareAerodynamicVel();
    m_markerArray_fast.markers.push_back(m_msg.vel_aero);

    prepareNongravForce();
    m_markerArray_fast.markers.push_back(m_msg.nongravForce);

    prepareWind();
    m_markerArray_fast.markers.push_back(m_msg.wind_arrow);
    m_markerArray_fast.markers.push_back(m_msg.wind_label);

    if (N_vehicles >= 2)
    {
        broadcastBodyTransform2();
        m_markerArray_fast.markers.push_back(m_msg.vel_geo2);
        m_markerArray_fast.markers.push_back(m_msg.vel_aero2);
        m_markerArray_fast.markers.push_back(m_msg.nongravForce2);

        if (N_vehicles >= 3)
        {
            broadcastBodyTransform3();
            m_markerArray_fast.markers.push_back(m_msg.vel_geo3);
            m_markerArray_fast.markers.push_back(m_msg.vel_aero3);
            m_markerArray_fast.markers.push_back(m_msg.nongravForce3);
        }
    }

    publishFastMarkerArray();
}

/* Setup reusable markers */
void VisualizerFlightRos2Base::setup_trace()
{
    m_msg.trace = MarkerM();

    /* Namespace and identifier */
    m_msg.trace.ns = "trace";
    m_msg.trace.id = 0; // Is going to be iterated

    /* Frame and time */
    m_msg.trace.header.frame_id = "ned";
    m_msg.trace.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_msg.trace.frame_locked = false; // Not necessary

    /* Position and orientation */
    m_msg.trace.pose.orientation.w = 1;

    /* Shape and scale */
    m_msg.trace.type = MarkerM::SPHERE;
    m_msg.trace.scale.x = m_msg.trace.scale.y = m_msg.trace.scale.z = vehicle.trace_scale * 1;
    m_msg.trace.action = MarkerM::ADD;

    /* Color */
    // grey
    m_msg.trace.color.r = 0.8;
    m_msg.trace.color.g = 0.8;
    m_msg.trace.color.b = 0.8;
    m_msg.trace.color.a = 1;
}
void VisualizerFlightRos2Base::setup_vel_geo()
{
    m_msg.vel_geo = MarkerM();

    /* Namespace and identifier */
    m_msg.vel_geo.ns = "vg";
    m_msg.vel_geo.id = 0;

    /* Frame and time */
    m_msg.vel_geo.header.frame_id = "body";
    m_msg.vel_geo.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_msg.vel_geo.frame_locked = true;

    /* Position and orientation */
    m_msg.vel_geo.pose.orientation.w = 1;

    m_msg.vel_geo.points.resize(2);
    // points[0] is 0,0,0
    // points[1] will be updated

    /* Shape and scale */
    m_msg.vel_geo.type = MarkerM::ARROW;
    m_msg.vel_geo.scale.x = vehicle.vel_geo_scale * 0.16; // Shaft diameter
    m_msg.vel_geo.scale.y = vehicle.vel_geo_scale * 0.8; // Head diameter
    m_msg.vel_geo.scale.z = vehicle.vel_geo_scale * 0.8; // Head Length

    /* Color */
    // White
    m_msg.vel_geo.color.r = 1.0;
    m_msg.vel_geo.color.g = 1.0;
    m_msg.vel_geo.color.b = 1.0;
    m_msg.vel_geo.color.a = 1.0;
}
void VisualizerFlightRos2Base::setup_vel_aero()
{
    m_msg.vel_aero = MarkerM();

    /* Namespace and identifier */
    m_msg.vel_aero.ns = "va";
    m_msg.vel_aero.id = 0;

    /* Frame and time */
    m_msg.vel_aero.header.frame_id = "body";
    m_msg.vel_aero.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_msg.vel_aero.frame_locked = true;

    /* Position and orientation */
    m_msg.vel_aero.pose.orientation.w = 1;

    m_msg.vel_aero.points.resize(2);
    // points[0] is 0,0,0
    // points[1] will be updated

    /* Shape and scale */
    m_msg.vel_aero.type = MarkerM::ARROW;
    m_msg.vel_aero.scale.x = vehicle.vel_air_scale * 0.16; // Shaft diameter
    m_msg.vel_aero.scale.y = vehicle.vel_air_scale * 0.8; // Head diameter
    m_msg.vel_aero.scale.z = vehicle.vel_air_scale * 0.8; // Head Length

    /* Color */
    // Skyblue
    m_msg.vel_aero.color.r = 0.0;
    m_msg.vel_aero.color.g = 0.0;
    m_msg.vel_aero.color.b = 1.0;
    m_msg.vel_aero.color.a = 1.0;
}
void VisualizerFlightRos2Base::setup_nongrav_force()
{
    m_msg.nongravForce = MarkerM();

    /* Namespace and identifier */
    m_msg.nongravForce.ns = "spec_nongrav_force";
    m_msg.nongravForce.id = 0;

    /* Frame and time */
    m_msg.nongravForce.header.frame_id = "body";
    m_msg.nongravForce.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_msg.nongravForce.frame_locked = true;

    /* Position and orientation */
    m_msg.nongravForce.pose.orientation.w = 1;

    m_msg.nongravForce.points.resize(2);
    // points[0] is 0,0,0
    // points[1] will be updated

    /* Shape and scale */
    m_msg.nongravForce.type = MarkerM::ARROW;
    m_msg.nongravForce.scale.x = vehicle.force_scale * 0.2; // Shaft diameter
    m_msg.nongravForce.scale.y = vehicle.force_scale * 0.8; // Head diameter
    m_msg.nongravForce.scale.z = vehicle.force_scale * 0.8; // Head Length

    /* Color */
    /* Blue */
    m_msg.nongravForce.color.r = 0.2;
    m_msg.nongravForce.color.g = 0.7;
    m_msg.nongravForce.color.b = 0.9;
    m_msg.nongravForce.color.a = 1.0;
}
void VisualizerFlightRos2Base::setup_wind()
{
    /** Arrow **/
    /* Namespace and identifier */
    m_msg.wind_arrow.ns = "wind";
    m_msg.wind_arrow.id = 0;

    /* Frame and time */
    m_msg.wind_arrow.header.frame_id = "ned";
    m_msg.wind_arrow.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_msg.wind_arrow.frame_locked = false;

    /* Shape and scale */
    m_msg.wind_arrow.type = MarkerM::ARROW;
    m_msg.wind_arrow.scale.x = 1e-3;
    m_msg.wind_arrow.scale.y = m_msg.wind_arrow.scale.z = vehicle.wind_scale;


    /* Color and alpha */
    // White
    m_msg.wind_arrow.color.r = 1;
    m_msg.wind_arrow.color.g = 1;
    m_msg.wind_arrow.color.b = 1;
    m_msg.wind_arrow.color.a = 0.6;

    /** Label **/
    /* Namespace and identifier */
    m_msg.wind_label.ns = "wind";
    m_msg.wind_label.id = 1;

    /* Frame and time */
    m_msg.wind_label.header.frame_id = "ned";
    m_msg.wind_label.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_msg.wind_label.frame_locked = false;

    /* Shape and scale */
    m_msg.wind_label.type = MarkerM::TEXT_VIEW_FACING;
    m_msg.wind_label.pose.position.z -= vehicle.wind_scale;

    /* Color and alpha */
    // White
    m_msg.wind_label.color.r = 1;
    m_msg.wind_label.color.g = 1;
    m_msg.wind_label.color.b = 1;
    m_msg.wind_label.color.a = 0.6;
}

/* Publish functions ================= */
/* Publish once */
void VisualizerFlightRos2Base::clear_trace()
{
    for (int i = 0; i <= trace_n_points; i++)
    {
        m_msg.trace.id = i;
        m_msg.trace.header.stamp = m_pub_time;
        m_msg.trace.action = MarkerM::DELETEALL;
        m_pub.trace->publish(m_msg.trace);
    }
    m_msg.trace.action = MarkerM::ADD; // Change back to ADD
}

void VisualizerFlightRos2Base::clear_all_markers()
{
    std::cout << "Clear all markers\n";
    clear_trace();
}

void VisualizerFlightRos2Base::broadcastNedTransfrom()
{
    TransformStampedM tf_ned_world{};     // All entries = 0

    /* Reference */
    tf_ned_world.header.stamp = rclcpp::Node::now();
    tf_ned_world.header.frame_id = "world";
    tf_ned_world.child_frame_id = "ned";

    /* Rotation */
    //const Quaternion q_ned_enu(Eigen::AngleAxisd(M_PI / 2.0, Vector3(0, 0, 1)) * Eigen::AngleAxisd(M_PI, Vector3(1, 0, 0)));
    tf_ned_world.transform.rotation.w = q_ned_enu.w();
    tf_ned_world.transform.rotation.x = q_ned_enu.x();
    tf_ned_world.transform.rotation.y = q_ned_enu.y();
    tf_ned_world.transform.rotation.z = q_ned_enu.z();

    std::cout << "Broadcast NED\n";
    m_tf_static_broadcaster->sendTransform(tf_ned_world);
}

/* Publish continuously */
void VisualizerFlightRos2Base::broadcastBodyTransform()
{
    /* Reference */
    m_msg.tf_b_ned.header.stamp = m_pub_time;
    m_msg.tf_b_ned.header.frame_id = "ned";
    m_msg.tf_b_ned.child_frame_id = "body";

    /* Position */
    m_msg.tf_b_ned.transform.translation.x = m_states[0].pos(0);
    m_msg.tf_b_ned.transform.translation.y = m_states[0].pos(1);
    m_msg.tf_b_ned.transform.translation.z = m_states[0].pos(2);

    /* Rotation */
    m_msg.tf_b_ned.transform.rotation.w = m_states[0].q_ned_b.w();
    m_msg.tf_b_ned.transform.rotation.x = m_states[0].q_ned_b.x();
    m_msg.tf_b_ned.transform.rotation.y = m_states[0].q_ned_b.y();
    m_msg.tf_b_ned.transform.rotation.z = m_states[0].q_ned_b.z();

    m_tf_broadcaster->sendTransform(m_msg.tf_b_ned);
}
void VisualizerFlightRos2Base::broadcastBodyTransform2()
{
    /* Reference */
    m_msg.tf_b_ned2.header.stamp = m_pub_time;
    m_msg.tf_b_ned2.header.frame_id = "ned";
    m_msg.tf_b_ned2.child_frame_id = "body2";

    /* Position */
    m_msg.tf_b_ned2.transform.translation.x = m_states[1].pos(0);
    m_msg.tf_b_ned2.transform.translation.y = m_states[1].pos(1);
    m_msg.tf_b_ned2.transform.translation.z = m_states[1].pos(2);

    /* Rotation */
    m_msg.tf_b_ned2.transform.rotation.w = m_states[1].q_ned_b.w();
    m_msg.tf_b_ned2.transform.rotation.x = m_states[1].q_ned_b.x();
    m_msg.tf_b_ned2.transform.rotation.y = m_states[1].q_ned_b.y();
    m_msg.tf_b_ned2.transform.rotation.z = m_states[1].q_ned_b.z();

    m_tf_broadcaster->sendTransform(m_msg.tf_b_ned2);
}
void VisualizerFlightRos2Base::broadcastBodyTransform3()
{
    /* Reference */
    m_msg.tf_b_ned3.header.stamp = m_pub_time;
    m_msg.tf_b_ned3.header.frame_id = "ned";
    m_msg.tf_b_ned3.child_frame_id = "body3";

    /* Position */
    m_msg.tf_b_ned3.transform.translation.x = m_states[2].pos(0);
    m_msg.tf_b_ned3.transform.translation.y = m_states[2].pos(1);
    m_msg.tf_b_ned3.transform.translation.z = m_states[2].pos(2);

    /* Rotation */
    m_msg.tf_b_ned3.transform.rotation.w = m_states[2].q_ned_b.w();
    m_msg.tf_b_ned3.transform.rotation.x = m_states[2].q_ned_b.x();
    m_msg.tf_b_ned3.transform.rotation.y = m_states[2].q_ned_b.y();
    m_msg.tf_b_ned3.transform.rotation.z = m_states[2].q_ned_b.z();

    m_tf_broadcaster->sendTransform(m_msg.tf_b_ned3);
}

void VisualizerFlightRos2Base::publishTrace()
{
    // Only publish every trace_sample_period step
    if (trace_pause_counter >= (trace_sample_period - 1))
    {
        trace_pause_counter = 0;

        // Reset id counter such that only nTracePoints objects are displayed at the same time
        if (trace_id_counter++ >= (trace_n_points - 1)) { trace_id_counter = 0; }
        m_msg.trace.id = trace_id_counter;

        /* Frame and time */
        m_msg.trace.header.stamp = m_pub_time;

        /* Position and orientation */
        m_msg.trace.pose.position.x = m_states[0].pos(0);
        m_msg.trace.pose.position.y = m_states[0].pos(1);
        m_msg.trace.pose.position.z = m_states[0].pos(2);

        m_pub.trace->publish(m_msg.trace);

        if (N_vehicles >= 2)
        {
            m_msg.trace2.id = trace_id_counter;

            /* Frame and time */
            m_msg.trace2.header.stamp = m_pub_time;

            m_msg.trace2.pose.position.x = m_states[1].pos(0);
            m_msg.trace2.pose.position.y = m_states[1].pos(1);
            m_msg.trace2.pose.position.z = m_states[1].pos(2);

            m_pub.trace->publish(m_msg.trace2);
        }
        if (N_vehicles >= 3)
        {
            m_msg.trace3.id = trace_id_counter;

            /* Frame and time */
            m_msg.trace3.header.stamp = m_pub_time;

            m_msg.trace3.pose.position.x = m_states[2].pos(0);
            m_msg.trace3.pose.position.y = m_states[2].pos(1);
            m_msg.trace3.pose.position.z = m_states[2].pos(2);

            m_pub.trace->publish(m_msg.trace3);
        }
    }
    else { trace_pause_counter++; }
}

void VisualizerFlightRos2Base::prepareGeodeticVel()
{
    m_msg.vel_geo.header.stamp = m_pub_time;

    m_msg.vel_geo.points[1].x = vehicle.vel_geo_scale * m_states[0].vel_transformed(0);
    m_msg.vel_geo.points[1].y = vehicle.vel_geo_scale * m_states[0].vel_transformed(1);
    m_msg.vel_geo.points[1].z = vehicle.vel_geo_scale * m_states[0].vel_transformed(2);

    if (N_vehicles >= 2)
    {
        m_msg.vel_geo2.points[1].x = vehicle.vel_geo_scale * m_states[1].vel_transformed(0);
        m_msg.vel_geo2.points[1].y = vehicle.vel_geo_scale * m_states[1].vel_transformed(1);
        m_msg.vel_geo2.points[1].z = vehicle.vel_geo_scale * m_states[1].vel_transformed(2);
    }
    if (N_vehicles >= 3)
    {
        m_msg.vel_geo3.points[1].x = vehicle.vel_geo_scale * m_states[2].vel_transformed(0);
        m_msg.vel_geo3.points[1].y = vehicle.vel_geo_scale * m_states[2].vel_transformed(1);
        m_msg.vel_geo3.points[1].z = vehicle.vel_geo_scale * m_states[2].vel_transformed(2);
    }
}
void VisualizerFlightRos2Base::prepareAerodynamicVel()
{
    if (vehicle.vel_air_scale > 0.0)
    {
        m_msg.vel_aero.header.stamp = m_pub_time;

        Eigen::Quaterniond q_ba_beta(Eigen::AngleAxisd(m_states[0].beta, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond q_ba_alpha(Eigen::AngleAxisd(-m_states[0].alpha, Eigen::Vector3d::UnitY()));
        Eigen::Quaterniond q_ba = q_ba_alpha * q_ba_beta;
        Eigen::Vector3d b_va = q_ba * Eigen::Vector3d(m_states[0].Va, 0, 0);
        m_msg.vel_aero.points[1].x = vehicle.vel_air_scale * b_va(0);
        m_msg.vel_aero.points[1].y = vehicle.vel_air_scale * b_va(1);
        m_msg.vel_aero.points[1].z = vehicle.vel_air_scale * b_va(2);

        if (N_vehicles >= 2)
        {
            Eigen::Quaterniond q_ba_beta(Eigen::AngleAxisd(m_states[1].beta, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond q_ba_alpha(Eigen::AngleAxisd(-m_states[1].alpha, Eigen::Vector3d::UnitY()));
            Eigen::Quaterniond q_ba = q_ba_alpha * q_ba_beta;
            Eigen::Vector3d b_va = q_ba * Eigen::Vector3d(m_states[1].Va, 0, 0);
            m_msg.vel_aero2.points[1].x = vehicle.vel_air_scale * b_va(0);
            m_msg.vel_aero2.points[1].y = vehicle.vel_air_scale * b_va(1);
            m_msg.vel_aero2.points[1].z = vehicle.vel_air_scale * b_va(2);
        }
        if (N_vehicles >= 3)
        {
            Eigen::Quaterniond q_ba_beta(Eigen::AngleAxisd(m_states[2].beta, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond q_ba_alpha(Eigen::AngleAxisd(-m_states[2].alpha, Eigen::Vector3d::UnitY()));
            Eigen::Quaterniond q_ba = q_ba_alpha * q_ba_beta;
            Eigen::Vector3d b_va = q_ba * Eigen::Vector3d(m_states[2].Va, 0, 0);
            m_msg.vel_aero3.points[1].x = vehicle.vel_air_scale * b_va(0);
            m_msg.vel_aero3.points[1].y = vehicle.vel_air_scale * b_va(1);
            m_msg.vel_aero3.points[1].z = vehicle.vel_air_scale * b_va(2);
        }
    }
}
void VisualizerFlightRos2Base::prepareNongravForce()
{
    m_msg.nongravForce.header.stamp = m_pub_time;

    m_msg.nongravForce.points[1].x = vehicle.force_scale * m_states[0].b_nongravForce(0);
    m_msg.nongravForce.points[1].y = vehicle.force_scale * m_states[0].b_nongravForce(1);
    m_msg.nongravForce.points[1].z = vehicle.force_scale * m_states[0].b_nongravForce(2);

    if (N_vehicles >= 2)
    {
        m_msg.nongravForce2.points[1].x = vehicle.force_scale * m_states[1].b_nongravForce(0);
        m_msg.nongravForce2.points[1].y = vehicle.force_scale * m_states[1].b_nongravForce(1);
        m_msg.nongravForce2.points[1].z = vehicle.force_scale * m_states[1].b_nongravForce(2);
    }
    if (N_vehicles >= 3)
    {
        m_msg.nongravForce3.points[1].x = vehicle.force_scale * m_states[2].b_nongravForce(0);
        m_msg.nongravForce3.points[1].y = vehicle.force_scale * m_states[2].b_nongravForce(1);
        m_msg.nongravForce3.points[1].z = vehicle.force_scale * m_states[2].b_nongravForce(2);
    }
}
void VisualizerFlightRos2Base::prepareWind()
{
    sim_windFrom_deg = std::atan2(-sim_vW(1), -sim_vW(0)) * 180.0 / M_PI;
    if (sim_windFrom_deg < 0) sim_windFrom_deg += 360;

    sim_windSpeed = sim_vW.norm();

    m_msg.wind_arrow.header.stamp = m_pub_time;

    /* Position, orientation, size */
    m_msg.wind_arrow.pose.position.x = vehicle.wind_scale * 15 * std::cos(sim_windFrom_deg * M_PI / 180.0);
    m_msg.wind_arrow.pose.position.y = vehicle.wind_scale * 15 * std::sin(sim_windFrom_deg * M_PI / 180.0);
    m_msg.wind_arrow.pose.position.z = -6;

    const Eigen::Quaterniond q_wind_ned(
            Eigen::AngleAxisd((sim_windFrom_deg + 180.0) * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
    m_msg.wind_arrow.pose.orientation.w = q_wind_ned.w();
    m_msg.wind_arrow.pose.orientation.x = q_wind_ned.x();
    m_msg.wind_arrow.pose.orientation.y = q_wind_ned.y();
    m_msg.wind_arrow.pose.orientation.z = q_wind_ned.z();

    m_msg.wind_arrow.scale.x = vehicle.wind_scale * sim_windSpeed;
    m_msg.wind_arrow.scale.y = m_msg.wind_arrow.scale.z = 0.1 * m_msg.wind_arrow.scale.x;

    /** Text **/
    m_msg.wind_label.header.stamp = m_pub_time;

    m_msg.wind_label.pose.position.x = m_msg.wind_arrow.pose.position.x;
    m_msg.wind_label.pose.position.y = m_msg.wind_arrow.pose.position.y;

    const double speed_rounded = std::round(sim_windSpeed * 10.0) / 10.0;
    std::string speed_rounded_str = std::to_string(speed_rounded);
    const auto decimal_pos = speed_rounded_str.find_first_of('.');
    speed_rounded_str = speed_rounded_str.substr(0, decimal_pos + 2);
    m_msg.wind_label.text = std::to_string(static_cast<int>(sim_windFrom_deg)) + " @ " + speed_rounded_str;
    m_msg.wind_label.scale.x = m_msg.wind_label.scale.y = m_msg.wind_label.scale.z = vehicle.wind_scale;
}
