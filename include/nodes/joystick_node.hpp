#ifndef JOYSTICK_NODE_HPP
#define JOYSTICK_NODE_HPP


#include <filesystem>
#include <memory>

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <ament_index_cpp/ament_index_cpp/get_package_share_directory.hpp>

#include <joystick_driver/joystick_driver_parameters.hpp>

#include "handles/gamepad.hpp"
#include "preconfigured_mappings.hpp"


class JoystickNode : public rclcpp::Node
{

    public:

        JoystickNode();

    private:

        /**
         * @brief Updates the button and axis states given the incoming joystick message
         * 
         * @param message `const sensor_msgs::msg::Joy&`
         */
        void topic_callback(const sensor_msgs::msg::Joy& message);

        /**
         * @brief Create a 2D TwistStamped message whose `linear.x` is the axis of `linear_axis` and whose
         * `angular.z` is the axis of `angular_axis`. These are scaled with `max_tangential_velocity` and
         * `max_angular_velocity` respectively.
         * 
         * @param linear_axis `int` The axis to use for populating the `linear.x` component
         * @param angular_axis `int` The axis to use for populating the `angular.z` component
         * @return `geometry_msgs::msg::TwistStamped` 
         */
        geometry_msgs::msg::TwistStamped create_twist(int linear_axis, int angular_axis);

        /**
         * @brief Get the `GamepadMapping` object from the given ROS parameters
         * 
         * @return `GamepadMapping` 
         */
        GamepadMapping get_mapping();

        /**
         * @brief Get the mapping from a predefined yaml file in the `/config` folder
         * 
         * @param filename `std::string` The filename within the `/config` folder, such as `ps4.yaml`
         * @return `GamepadMapping` 
         */
        GamepadMapping get_mapping_from_yaml(std::string filename);

        // Generate Parameter Library
        std::shared_ptr<joystick_driver::ParamListener> m_param_listener;
        joystick_driver::Params m_params;

        // Incoming Raw Joystick data
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;

        // Twist Publisher
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher;

        // Gamepad API
        Gamepad gamepad;

}; // class JoystickNode : public rclcpp::Node


#endif // JOYSTICK_NODE_HPP