#include "handles/gamepad_status.hpp"


GamepadStatus::GamepadStatus(GamepadMapping mappings) :

    // Stick Axes
    m_stick_left_x(mappings.stick_left_x, GamepadAxis::LEFT_X),
    m_stick_left_y(mappings.stick_left_y, GamepadAxis::LEFT_Y),
    m_stick_right_x(mappings.stick_right_x, GamepadAxis::RIGHT_X),
    m_stick_right_y(mappings.stick_right_y, GamepadAxis::RIGHT_Y),

    // Trigger Axes
    m_trigger_axis_left(mappings.trigger_left, GamepadAxis::LEFT_TRIGGER),
    m_trigger_axis_right(mappings.trigger_right, GamepadAxis::RIGHT_TRIGGER),

    // Trigger Buttons
    m_trigger_button_left(mappings.trigger_left, GamepadButton::LEFT_TRIGGER),
    m_trigger_button_right(mappings.trigger_right, GamepadButton::RIGHT_TRIGGER),

    // Bumper Buttons
    m_bumper_left(mappings.bumper_left, GamepadButton::LEFT_BUMPER),
    m_bumper_right(mappings.bumper_right, GamepadButton::RIGHT_BUMPER),

    // Face Buttons
    m_button_down(mappings.button_down, GamepadButton::ACTION_DOWN),
    m_button_right(mappings.button_right, GamepadButton::ACTION_RIGHT),
    m_button_left(mappings.button_left, GamepadButton::ACTION_LEFT),
    m_button_up(mappings.button_up, GamepadButton::ACTION_UP),

    // Stick Buttons
    m_button_left_stick(mappings.button_left_stick, GamepadButton::LEFT_STICK),
    m_button_right_stick(mappings.button_right_stick, GamepadButton::RIGHT_STICK)

{} // end of constructor


void GamepadStatus::update(std::vector<float> axes, std::vector<int> buttons)
{
    m_stick_left_x.update(axes);
    m_stick_left_y.update(axes);
    m_stick_right_x.update(axes);
    m_stick_right_y.update(axes);

    // Update the triggers for whatever type they are
    if(m_trigger_as_buttons)
    {
        m_trigger_button_left.update(buttons);
        m_trigger_button_right.update(buttons);
    }
    else
    {
        m_trigger_axis_left.update(axes);
        m_trigger_axis_right.update(axes);
    }    

    m_bumper_left.update(buttons);
    m_bumper_right.update(buttons);

    m_button_down.update(buttons);
    m_button_right.update(buttons);
    m_button_left.update(buttons);
    m_button_up.update(buttons);

    m_button_left_stick.update(buttons);
    m_button_right_stick.update(buttons);

} // end of "update"


void GamepadStatus::update(sensor_msgs::msg::Joy joy_packet)
{
    update(joy_packet.axes, joy_packet.buttons);

} // end of "update"