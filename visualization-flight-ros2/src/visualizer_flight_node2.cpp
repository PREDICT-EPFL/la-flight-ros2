#include "visualization_flight_ros2/VisualizerFlightRos2.hpp"

void print_options(const std::string &param, const std::vector<std::string> &options)
{
    std::cout << "\nERROR: Available options for node parameter '" << param << "': ";
    for (const auto &option: options) { std::cout << option << " "; }
    std::cout << "\n\n";
}

int main(int argc, char **argv) {

    /* Init node */
    rclcpp::init(argc, argv);

    /* Get node parameters */
//    std::string variant_str;
//    nh.param<std::string>("variant", variant_str, "kite");
//
//    if (variant_str == "kite")
//    {
      visualization_ros2::flight::KiteVisualizer kiteVisualizer("visualizer_kite");
//        kiteVisualizer.test();
        kiteVisualizer.set_N_vehicle(2);
        kiteVisualizer.run();
//    }

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}