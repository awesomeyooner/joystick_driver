#ifndef MAPPING_NODE
#define MAPPING_NODE


#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/emitter.h"

#include "handles/button.hpp"

#include "plib/util/status.hpp"
#include "plib/util/logger.hpp"


enum class PromptState
{
    READY,
    WAITING_FOR_ACTIVITY

}; // enum class PromptState


class MappingNode : public rclcpp::Node
{

    public:

        /**
         * @brief Initializes the node
         * 
         */
        MappingNode();

        /**
         * @brief Creates a .yaml file whose contents are the maps 
         * `button_mappings` and `axis_mappings` 
         * 
         * @return `status_utils::StatusCode` `OK` if file made successfully, `FAILED` otherwise 
         */
        status_utils::StatusCode create_yaml();

        status_utils::StatusCode topic_callback(const sensor_msgs::msg::Joy& message);

    private:

        // List of bindings that are strictly either axes or buttons
        const std::vector<std::string> prompts = {
            "home",

            "button_up",
            "button_down",
            "button_left",
            "button_right",

            "bumper_left",
            "bumper_right",
            
            "button_left_stick",
            "button_right_stick",

            "stick_left_x",
            "stick_left_y",
            "stick_right_x",
            "stick_right_y"
        };

        // List of the bindings that could be axes, buttons, or both
        const std::vector<std::string> dynamic_prompts = {
            "trigger_left",
            "trigger_right",
        };

        // How big the change-of-states needs to be for a button
        // to be considered active
        const int ACTIVE_BUTTON_THRESHOLD = 1;

        // How big the change-of-states needs to be for an axis
        // to be considered active
        const float ACTIVE_AXIS_THRESHOLD = 0.90;

        // How much allowed difference there is between axis values to be considered resting
        const int RESTING_AXIS_THRESHOLD = 0.1;

        // The resting state of the controller to use for comparison
        std::vector<int> initial_button_states;
        std::vector<float> initial_axis_states;

        PromptState prompt_state = PromptState::READY;

        bool ask_for_home = false;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;

        // The current maps in the form of {map_name, index}
        // Example: `{bumper_left, 9}`
        std::map<std::string, int> button_mappings;
        std::map<std::string, int> axis_mappings;

        std::string get_prompt();

        std::vector<int> get_active_buttons(const std::vector<int>& button_states);

        std::vector<int> get_active_axes(const std::vector<float>& axis_states);

        bool is_resting(const std::vector<int>& button_states, const std::vector<float>& axis_states);

        bool get_home_button(const std::vector<int>& button_states);

        std::vector<int> get_values(const std::map<std::string, int>& map);

        void info(std::string message);

}; // class MappingNode


#endif // MAPPING_NODE