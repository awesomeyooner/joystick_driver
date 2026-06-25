#ifndef MAPPING_NODE
#define MAPPING_NODE


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


class MappingNode : public rclcpp::Node
{

    public:

        MappingNode();

        

    private:


} // class MappingNode


#endif // MAPPING_NODE