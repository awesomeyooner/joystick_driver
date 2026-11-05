#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include <map>
#include <vector>

#include "sensor_msgs/msg/joy.hpp"

#include "handles/gamepad_mapping.hpp"
#include "handles/axis.hpp"
#include "handles/button.hpp"
#include "universal_ids.hpp"


/**
 * @brief Contains all Axis and Buttons of a standard gamepad controller (like an XBox controller)
 * 
 */
class Gamepad
{

    public:

        Gamepad() = default;

        /**
         * @brief Assigns the mappings for each axis from the given mappings struct
         * 
         * @param mappings `GamepadMapping` The gamepad mapping (the index of each button / axis)
         */
        Gamepad(GamepadMapping mappings);

        void init(GamepadMapping mappings);

        /**
         * @brief Update the current values of all axes and buttons given
         * the array of axis and button values
         * 
         * @param axes `std::vector<float>` - The array of axis data
         * @param buttons `std::vector<int>` - The array of button data
         */
        void update(std::vector<float> axes, std::vector<int> buttons);

        /**
         * @brief Update the values of all axes and buttons directly from
         * a `joy_msg`
         * 
         * @param joy_packet `sensor_msgs::msg::Joy` - The `joy_msg` 
         */
        void update(const sensor_msgs::msg::Joy& joy_packet);

        /**
         * @brief Gets the button handle given the Universal ID
         * 
         * @param id `int` - The Universal ID this button is associated with.
         * @return `Button*` - Pointer to the button handle 
         */
        Button* get_button(int id);

        /**
         * @brief Gets the axis handle given the Universal ID
         * 
         * @param id `int` - The Universal ID this axis is associated with.
         * @return `Button*` - Pointer to the axis handle 
         */
        Axis* get_axis(int id);

    private:

        std::pair<int, Axis*> create_axis(int id, int index);

        std::pair<int, Button*> create_button(int id, int index);

        // Map containing pointers to all Buttons and Axes at their corresponding Universal IDs
        std::map<int, Button*> m_button_map;
        std::map<int, Axis*> m_axis_map;
        

}; // class Gamepad


#endif // GAMEPAD_HPP