#ifndef SRC_VISUALIZERFLIGHTROS2_HPP
#define SRC_VISUALIZERFLIGHTROS2_HPP

#include "VisualizerFlightRos2Base.hpp"

/* Sub */
#include "sensor_msgs/msg/multi_dof_joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mavros_msgs/msg/vfr_hud.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "visualization_flight_ros2/yaml_read.hpp"

namespace visualization_ros2 {
namespace flight {

struct KiteProperties
{
    Eigen::Vector3d b_thr_r{0, 0, 0};
    Eigen::Vector3d b_tethlat_attch_pos{0, 0, 0};
    Eigen::Vector3d b_tethlon_attch_pos{0, 0, 0};
};

#define N_TRAJ_POINTS 40 // generic max
class KiteVisualizer : public VisualizerFlightRos2Base
{
public:
    explicit KiteVisualizer(const std::string &name) : VisualizerFlightRos2Base(name)
    {
        this->declare_parameter<bool>("tether", true);
        simulate_tether = this->get_parameter("tether").get_value<bool>();

        if (simulate_tether)
        {
            std::cout << "[KiteVisualizer]: Tether ON\n";
        }

        m_sub.simState = this->create_subscription<MultiDofJointStateM>(
                "/sim/state", 1, std::bind(&KiteVisualizer::simulationStateCallback, this, _1));
        m_sub.simState2 = this->create_subscription<MultiDofJointStateM>(
                "/sim/state2", 1, std::bind(&KiteVisualizer::simulationState2Callback, this, _1));
        m_sub.simState3 = this->create_subscription<MultiDofJointStateM>(
                "/sim/state3", 1, std::bind(&KiteVisualizer::simulationState3Callback, this, _1));
        m_sub.simStateConpoint = this->create_subscription<MultiDofJointStateM>(
                "/sim/state_conpoint", 1, std::bind(&KiteVisualizer::simulationStateConpointCallback, this, _1));
        m_sub.simStateBase = this->create_subscription<MultiDofJointStateM>(
                "/sim/state_base", 1, std::bind(&KiteVisualizer::simulationStateBaseCallback, this, _1));

        m_sub.control = this->create_subscription<JoyM>(
                "/sim/control", 1, std::bind(&KiteVisualizer::controlCallback, this, _1));
        m_sub.control2 = this->create_subscription<JoyM>(
                "/sim/control2", 1, std::bind(&KiteVisualizer::control2Callback, this, _1));
        m_sub.control3 = this->create_subscription<JoyM>(
                "/sim/control3", 1, std::bind(&KiteVisualizer::control3Callback, this, _1));

//    /* Subscribers - from real data (rosbag) */
//    pose_sub = nh.subscribe("/mavros/local_position/pose", 1, &KiteVisualizer::pxPoseCallback, this);
//    velBod_sub = nh.subscribe("/mavros/local_position/velocity_body", 1, &KiteVisualizer::pxVelBodCallback, this);
//    imuData_sub = nh.subscribe("/mavros/imu/data", 1, &KiteVisualizer::pxImuDataCallback, this);
//    windEst_sub = nh.subscribe("/mavros/wind_estimation", 1, &KiteVisualizer::pxWindEstimationCallback, this);
//    vfrHud_sub = nh.subscribe("/mavros/vfr_hud", 1, &KiteVisualizer::pxVfrHudCallback, this);

        /* Kite markers */
        m_kitepub.control = create_publisher<MarkerM>("/vis/control", 5);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        mpc_control_traj.setZero();
        for (unsigned i = 0; i < sizeof(mpc_traj_poses); i++) { mpc_traj_poses[0] = PoseM(); }

        /* Set up vehicle visual properties */

        const std::string default_kite = ament_index_cpp::get_package_share_directory("simulation_flight_ros2") +
                                "/config/" + "eg4_xflr-Pvw-YR-Multi.yaml";
        declare_parameter<std::string>("kite_params", default_kite);
        load_kite_params(get_parameter("kite_params").as_string());

        vehicle.mesh_path = "package://visualization_flight_ros2/resources/kite_rviz.dae";
        vehicle.mesh_scale = 0.187;
        vehicle.q_b_mesh = Eigen::Quaterniond(0, 0.999, 0, -0.044);
        vehicle.mesh_body_offset(0) = 0.07;

        vehicle.force_scale = 0.2;
        vehicle.vel_air_scale = 0.2;
        vehicle.vel_geo_scale = 0.2;
        vehicle.wind_scale = 5;
    }

    void load_kite_params(const std::string &kite_param_filepath)
    {
        const YAML::Node yamlNode = YAML::LoadFile(kite_param_filepath);

        using namespace yaml_read;
        get_as_double(yamlNode, "b_thrust_rx", kite.b_thr_r.x());
        get_as_double(yamlNode, "b_thrust_ry", kite.b_thr_r.y());
        get_as_double(yamlNode, "b_thrust_rz", kite.b_thr_r.z());

        get_as_double(yamlNode, "tethlat_rx", kite.b_tethlat_attch_pos.x());
        get_as_double(yamlNode, "tethlat_ry", kite.b_tethlat_attch_pos.y());
        get_as_double(yamlNode, "tethlat_rz", kite.b_tethlat_attch_pos.z());

        get_as_double(yamlNode, "tethlon_rx", kite.b_tethlon_attch_pos.x());
        get_as_double(yamlNode, "tethlon_ry", kite.b_tethlon_attch_pos.y());
        get_as_double(yamlNode, "tethlon_rz", kite.b_tethlon_attch_pos.z());
    }

protected:
    using MultiDofJointStateM = sensor_msgs::msg::MultiDOFJointState;
    using JoyM = sensor_msgs::msg::Joy;

    using PoseM = geometry_msgs::msg::Pose;
    using PoseStampedM = geometry_msgs::msg::PoseStamped;
    using PointStampedM = geometry_msgs::msg::PointStamped;
    using TwistStampedM = geometry_msgs::msg::TwistStamped;
    using TwistWithCovarianceStampedM = geometry_msgs::msg::TwistWithCovarianceStamped;
    using ImuM = sensor_msgs::msg::Imu;
    using VfrHudM = mavros_msgs::msg::VfrHud;

    void init_impl() override;

    void update_in_impl() override;
    void update_out_impl() override;

private:
    /* Parameters ===================================================================== */
    struct Parameters
    {
        /* Constant members mean that the parameter cannot be changed at runtime/via telemetry so far. */
        double windFrom_deg{160};
        double teth_len{60};

        struct CircleParameters
        {
            double radiusRef{40.0};
            double declination{25.0 * M_PI / 180.0};
            double hRef{100.0}; // Overwritten later, computed from radiusRef and teth_len
        } circle;
    } p;
    bool simulate_tether{false};
    KiteProperties kite;

    std::array<Eigen::Vector4d, 3> control{};

    std::array<Eigen::Vector3d, 3> b_tethForce{};

    unsigned n_traj_points{N_TRAJ_POINTS};
    PoseM mpc_traj_poses[N_TRAJ_POINTS];
    double mpc_traj_virtStates[N_TRAJ_POINTS]{};
    bool mpc_traj_isValid{false};

    Eigen::Matrix<double, N_TRAJ_POINTS, 4> mpc_control_traj;

    void updateParameters()
    {
        p.circle.hRef = std::sqrt(p.teth_len * p.teth_len - p.circle.radiusRef * p.circle.radiusRef);
    }

    /** Callbacks **/
    struct Subscribers
    {
        RosSub<MultiDofJointStateM> simState;
        RosSub<MultiDofJointStateM> simState2;
        RosSub<MultiDofJointStateM> simState3;
        RosSub<MultiDofJointStateM> simStateConpoint;
        RosSub<MultiDofJointStateM> simStateBase;

        RosSub<PoseStampedM> pose;
        RosSub<TwistStampedM> velBod;
        RosSub<ImuM> imuData;
        RosSub<TwistWithCovarianceStampedM> windEst;
        RosSub<VfrHudM> vfrHud;
        RosSub<JoyM> controlCmd;

        RosSub<JoyM> control;
        RosSub<JoyM> control2;
        RosSub<JoyM> control3;

//        RosPub<> mpc_state_traj;
//        RosPub<> mpc_control_traj;
    } m_sub;
    void simulationStateCallback(const MultiDofJointStateM::SharedPtr msg);
    void controlCallback(const JoyM::SharedPtr msg);

    void simulationState2Callback(const MultiDofJointStateM::SharedPtr msg);
    void control2Callback(const JoyM::SharedPtr msg);

    void simulationState3Callback(const MultiDofJointStateM::SharedPtr msg);
    void control3Callback(const JoyM::SharedPtr msg);

    void simulationStateConpointCallback(const MultiDofJointStateM::SharedPtr msg);
    void simulationStateBaseCallback(const MultiDofJointStateM::SharedPtr msg);

    void pxPoseCallback(const PoseStampedM::SharedPtr &msg);
    void pxVelBodCallback(const TwistStampedM::SharedPtr msg);
    void pxImuDataCallback(const ImuM::SharedPtr msg);
    void pxWindEstimationCallback(const TwistWithCovarianceStampedM::SharedPtr msg);
    void pxVfrHudCallback(const VfrHudM::SharedPtr msg);
    void controlCmdCallback(const JoyM::SharedPtr msg);

    void get_circle_transform(TransformStampedM &tf_c_wd);

    /** Markers **/
    void setup_circle_plane();
    void setup_circle_plane_force();
    void setup_tether();
    void setup_conpoint();
    void setup_teth_force(MarkerM &arrowMarker, MarkerM &textMarker);

    /** Subscribing and publishing **/
    struct Transforms
    {
        TransformStampedM c_ned{};
    } m_tfm{};
    struct Messages
    {
        MarkerM body{};
        MarkerM control{};
        MarkerM control2{};
        MarkerM control3{};

        MarkerM tether{};
        MarkerM circlePlane{};
        MarkerM circlePlaneForce{};

        MarkerM conpoint{};
        MarkerM tetherForceArrow{};
        MarkerM tetherForceText{};
        MarkerM tetherForceArrow2{};
        MarkerM tetherForceText2{};
        MarkerM tetherForceArrow3{};
        MarkerM tetherForceText3{};

        MarkerM tetherForceArrowConpoint{};
        MarkerM tetherForceTextConpoint{};

        MarkerM tetherForceArrowBase{};
        MarkerM tetherForceTextBase{};
    } m_msg{};
    struct KitePub
    {
        RosPub<MarkerM> control;
    } m_kitepub;

    // On parameter change
    void broadcastWinddomeTransform();
    void publishSphere() const;
    void publishPathRef() const;
    void publishResetMpcStateTraj();

    // Once
    void publishBody();

    // Continuously
    void broadcastCircleTransform();

    void prepareCirclePlane();
    void prepareCirclePlaneForce();

    void prepareControl();

    void prepareTether();
    void prepareTethForce(MarkerM &arrowMarker, MarkerM &textMarker,
                          const FlightState &state, const Eigen::Vector3d &b_tethForce);
    void prepareConpoint();
    void prepareMpcStateTraj();

    void publishFastMarkerArray() {} // Shadow base class function to control moment of publishing

    /* Helpers */
    static void getBallMarker(MarkerM &marker, const Eigen::Vector3d &position);
    static void getBallMarker(MarkerM &marker, const PoseM &pose);

    void getControlMarkerPoints(MarkerM &marker, const Eigen::Matrix<double, 1, 4> &ctrl) const;
};

} // namespace flight
} // namespace visualization_ros2

#endif //SRC_VISUALIZERFLIGHTROS2_HPP
