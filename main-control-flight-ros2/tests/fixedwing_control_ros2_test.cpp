#include "main_control_flight_ros2/FixedwingControlRos2.hpp"

int main(int argc, char **argv)
{
    /* Init node */
    rclcpp::init(argc, argv);

    using namespace control_ros2::flight::fixedwing;
//    {
//        FixedwingControlRos2_sim fixedwingControlRos2("fixedwing_control_ros2_test");
//        fixedwingControlRos2.test_ros();
//    }
//    std::cout << "\n";
    {
        FixedwingControlRos2_sim fixedwingControlRos2("fixedwing_control_test");
//        while (rclcpp::ok())
        {
            fixedwingControlRos2.test_ros();
            chrono_sleep(std::chrono::milliseconds(100));
        }
    }
    std::cout << "\n";

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
