#include "main_control_flight_ros2/FixedwingControlRos2.hpp"

int main(int argc, char **argv)
{
    /* Init node */
    rclcpp::init(argc, argv);
    const std::string node_name = "fixedwing_control";

    /* Get node parameters */
//    std::string sensor_str;
//    nh.param<std::string>("sensor", sensor_str, "sim");
    const std::string sensor_str = "sim";

    if (sensor_str == "sim")
    {
        control_ros2::flight::fixedwing::FixedwingControlRos2_sim fixedwingControlRos2Sim(node_name);
        fixedwingControlRos2Sim.run();
    }
    else
    {
        if (sensor_str != "pixhawk")
        {
            std::cout << "Hint: For safety reasons, please specify the sensor to use:\n"
                         "rosrun offboard_control offboard_control_node _sensor:=pixhawk (default) | xsens | sim\n\n";
        }
        control_ros2::flight::fixedwing::FixedwingControlRos2_px fixedwingControlRos2Px(node_name);
        fixedwingControlRos2Px.run();
    }

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
