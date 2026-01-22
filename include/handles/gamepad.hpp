#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP


#include "handles/axis.hpp"
#include "handles/button.hpp"
#include "handles/gamepad_mapping.hpp"
#include "handles/gamepad_status.hpp"

class Gamepad
{

    public:

        Gamepad() = default;

        Gamepad(GamepadMapping mapping);

        void initialize(GamepadMapping mapping);

        void update(const sensor_msgs::msg::Joy& joy_packet);

        Button* get_button(int id);

        Axis* get_axis(int id);

    private:

        // The status to contain all button and axis handles
        GamepadStatus m_status;

        // Map containing pointers to all Buttons at their corresponding
        // Universal ID
        std::map<int, Button*> m_buttons;

        // Map containing pointers to all Axes at their corresponding
        // Universal ID
        std::map<int, Axis*> m_axes;
    
}; // end of "Gamepad"

#endif // GAMEPAD_HPP