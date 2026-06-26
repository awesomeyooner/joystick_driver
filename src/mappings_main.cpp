#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "mapping_node.hpp"


using namespace rclcpp;
using namespace status_utils;


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MappingNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}