// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber() : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                "topic", 6, std::bind(&MinimalSubscriber::topic_callback, this, _1));

        m_executor.add_node(rclcpp::Node::get_node_base_interface());
    }

    rclcpp::executors::SingleThreadedExecutor m_executor;
    void spinOnce_wrapper()
    {
        m_executor.spin_some(std::chrono::nanoseconds(static_cast<unsigned>(1e9)));
    }

    unsigned m_counter{0};
    void run()
    {
        std::cout << "run\n";
        while (rclcpp::ok())
        {
            std::cout << "spin " << m_counter + 1 << "\n";
            using namespace std::chrono;
            const steady_clock::time_point start = steady_clock::now();

            // Process one unit of work (callback once message). If none available, wait for up to 1 sec
//        executor.spin_once(std::chrono::nanoseconds(static_cast<unsigned>(1e9)));
            spinOnce_wrapper();

            // If work available, process it for up to 1 second
//        executor.spin_some(std::chrono::nanoseconds(static_cast<unsigned>(1e9)));

            const steady_clock::time_point stop = steady_clock::now();
            const auto duration = duration_cast<microseconds>(stop - start);
            std::cout << "done (" << duration.count() << " us)\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Loop at 50Hz
            m_counter++;
        }
    }
private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto sub = std::make_shared<MinimalSubscriber>();
//    rclcpp::spin(sub);
    std::cout << "no of callback groups: " << sub->get_callback_groups().size() << "\n";

    std::cout << "Sleeping for 4 ..." << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    std::cout << "done\n";

//    rclcpp::executors::SingleThreadedExecutor executor;
//    executor.add_node(sub);

    sub->run();

    for (int i = 0; i < 0; i++)
    {
        if (!rclcpp::ok()) { continue; }
        std::cout << "spin " << i + 1 << "\n";
        using namespace std::chrono;
        const steady_clock::time_point start = steady_clock::now();

        // Process one unit of work (callback once message). If none available, wait for up to 1 sec
//        executor.spin_once(std::chrono::nanoseconds(static_cast<unsigned>(1e9)));
        sub->spinOnce_wrapper();

        // If work available, process it for up to 1 second
//        executor.spin_some(std::chrono::nanoseconds(static_cast<unsigned>(1e9)));

        const steady_clock::time_point stop = steady_clock::now();
        const auto duration = duration_cast<microseconds>(stop - start);
        std::cout << "done (" << duration.count() << " us)\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Loop at 50Hz
    }

    rclcpp::shutdown();

    return 0;
}