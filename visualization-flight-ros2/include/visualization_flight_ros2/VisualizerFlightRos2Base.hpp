#ifndef SRC_VISUALIZERFLIGHTROS2BASE_HPP
#define SRC_VISUALIZERFLIGHTROS2BASE_HPP

#include "visualization_common_ros2/VisualizerRos2Base.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

/* Pub */
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace visualization_ros2 {
namespace flight {

struct VehicleProperties
{
    std::string mesh_path;
    Eigen::Quaterniond q_b_mesh{1, 0, 0, 0};
    Eigen::Vector3d mesh_body_offset{0, 0, 0};
    double mesh_scale{1};

    double force_scale{1};
    double trace_scale{1};
    double vel_air_scale{1};
    double vel_geo_scale{1};
    double wind_scale{1};
};

class VisualizerFlightRos2Base : public VisualizerRos2Base
{
public:
    explicit VisualizerFlightRos2Base(const std::string &name) : VisualizerRos2Base(name, 100)
    {
        declare_parameter<int>("numTracePoints", 200);
        trace_n_points = get_parameter("numTracePoints").get_value<int>();

        /* Publishers */
        m_tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        m_pub.env = create_publisher<MarkerM>("/vis/env", 100);
        m_pub.envArray = create_publisher<MarkerArrayM>("/vis/env_array", 100);

        m_pub.trace = create_publisher<MarkerM>("/vis/trace", 5);
        m_pub.fastArray = create_publisher<MarkerArrayM>("/vis/fast_array", 5);

        /* Default vehicle visual properties */
        vehicle.force_scale = 0.5;
        vehicle.trace_scale = 0.5;
        vehicle.vel_air_scale = 0.5;
        vehicle.vel_geo_scale = 0.5;
        vehicle.wind_scale = 0.5;
    }

    void set_N_vehicle(const unsigned N) { N_vehicles = N; }

protected:
    using MarkerM = visualization_msgs::msg::Marker;
    using MarkerArrayM = visualization_msgs::msg::MarkerArray;
    using TransformStampedM = geometry_msgs::msg::TransformStamped;

    void init_impl() override;

    void update_out_impl() override;

    VehicleProperties vehicle;
    unsigned N_vehicles{1};

    /* Constant quaternions */
    const Eigen::Quaterniond q_ned_enu{0, sqrt(2) / 2.0, sqrt(2) / 2.0, 0};
    const Eigen::Quaterniond q_flu_b{0, 1, 0, 0};

    /* Trace */
    const int trace_sample_period{
            static_cast<int>(std::floor(300 / (p_nodeRate.period().count() / 1000000)))}; // 300 ms
    int trace_pause_counter{trace_sample_period};
    int trace_n_points{200};
    int trace_id_counter{0};

    /** Data **/
    struct FlightState
    {
        Eigen::Vector3d pos{2, 0, -10};
        Eigen::Vector3d vel_in{0, 0, 0};
        Eigen::Quaterniond q_ned_b{1, 0, 0, 0}; // Quaternion transforms a vector from body to geodetic (ned) frame
        Eigen::Vector3d b_nongravForce{0, 0, -1};

        Eigen::Vector3d vel_transformed{0, 0, 0};

        double Va_pitot{0}; // unused so far for visualization
        double Va{0};
        double alpha{0};
        double beta{0};
    };
    std::array<FlightState, 5> m_states{};

    double sim_windFrom_deg{180.0};
    double sim_windSpeed{1.0};
    Eigen::Vector2d sim_vW{};

    /** Markers **/
    void setup_trace();
    void setup_vel_geo();
    void setup_vel_aero();
    void setup_nongrav_force();
    void setup_wind();

    /** Subscribing and publishing **/
    struct Messages
    {
        TransformStampedM tf_b_ned{};
        MarkerM trace{};
        MarkerM vel_geo{};
        MarkerM vel_aero{};
        MarkerM nongravForce{};
        MarkerM control{};

        TransformStampedM tf_b_ned2{};
        MarkerM trace2{};
        MarkerM vel_geo2{};
        MarkerM vel_aero2{};
        MarkerM nongravForce2{};
        MarkerM control2{};

        TransformStampedM tf_b_ned3{};
        MarkerM trace3{};
        MarkerM vel_geo3{};
        MarkerM vel_aero3{};
        MarkerM nongravForce3{};
        MarkerM control3{};

        MarkerM wind_arrow{};
        MarkerM wind_label{};
    } m_msg;
    MarkerArrayM m_markerArray_fast{};
    struct Publishers
    {
        RosPub<MarkerM> env;
        RosPub<MarkerArrayM> envArray;
        RosPub<MarkerM> trace;
        RosPub<MarkerArrayM> fastArray;
    } m_pub;
    RosTime m_pub_time;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_static_broadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    // Once
    void clear_all_markers();
    void clear_trace();

    void broadcastNedTransfrom();

    // Continuously
    void broadcastBodyTransform();
    void broadcastBodyTransform2();
    void broadcastBodyTransform3();

    void publishTrace();

    void prepareGeodeticVel();
    void prepareAerodynamicVel();
    void prepareNongravForce();
    void prepareWind();

    void publishFastMarkerArray() { m_pub.fastArray->publish(m_markerArray_fast); }

    /* Helpers */
    void getBodyMarker(MarkerM &marker, const Eigen::Vector3d &position, const Eigen::Quaterniond &quat)
    {
        /* Returns the aircraft body mesh marker on a certain position and orientation */
        /* Fixed-wing */
        marker.type = MarkerM::MESH_RESOURCE;
        marker.mesh_resource = vehicle.mesh_path;
        marker.scale.x = marker.scale.y = marker.scale.z = vehicle.mesh_scale;
        Eigen::Quaterniond q_ned_mesh = quat * vehicle.q_b_mesh;
        q_ned_mesh.normalize();
        marker.pose.position.x = position(0) + vehicle.mesh_body_offset(0);
        marker.pose.position.y = position(1) + vehicle.mesh_body_offset(1);
        marker.pose.position.z = position(2) + vehicle.mesh_body_offset(2);

        /* Mesh needs to be rotated*/
        marker.pose.orientation.w = q_ned_mesh.w();
        marker.pose.orientation.x = q_ned_mesh.x();
        marker.pose.orientation.y = q_ned_mesh.y();
        marker.pose.orientation.z = q_ned_mesh.z();
    }
    void getBodyMarker(MarkerM &marker, const geometry_msgs::msg::Pose &pose)
    {
        Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        getBodyMarker(marker, position, quat);
    }
    void getBodyMarker(MarkerM &marker, const Eigen::Quaterniond &quat)
    {
        getBodyMarker(marker, Eigen::Vector3d(0, 0, 0), quat);
    }
};

} // namespace flight
} // namespace visualization_ros2

#endif //SRC_VISUALIZERFLIGHTROS2BASE_HPP
