#include "handles/gamepad.hpp"


using namespace std;


Gamepad::Gamepad(GamepadMapping mappings)
{
    init(mappings);

} // end of constructor


void Gamepad::init(GamepadMapping mappings)
{
    // Stick Axes
    m_axis_map.insert(create_axis(GamepadAxis::LEFT_X, mappings.stick_left_x));
    m_axis_map.insert(create_axis(GamepadAxis::LEFT_Y, mappings.stick_left_y));
    m_axis_map.insert(create_axis(GamepadAxis::RIGHT_X, mappings.stick_right_x));
    m_axis_map.insert(create_axis(GamepadAxis::RIGHT_Y, mappings.stick_right_y));

    // Trigger Axes
    m_axis_map.insert(create_axis(GamepadAxis::LEFT_TRIGGER, mappings.axis_trigger_left));
    m_axis_map.insert(create_axis(GamepadAxis::RIGHT_TRIGGER, mappings.axis_trigger_right));

    // DPAD Axes
    m_axis_map.insert(create_axis(GamepadAxis::DPAD_X, mappings.axis_dpad_x));
    m_axis_map.insert(create_axis(GamepadAxis::DPAD_Y, mappings.axis_dpad_y));

    // Trigger Buttons
    m_button_map.insert(create_button(GamepadButton::LEFT_TRIGGER, mappings.button_trigger_left));
    m_button_map.insert(create_button(GamepadButton::RIGHT_TRIGGER, mappings.button_trigger_right));

    // Bumper Buttons
    m_button_map.insert(create_button(GamepadButton::LEFT_BUMPER, mappings.bumper_left));
    m_button_map.insert(create_button(GamepadButton::RIGHT_BUMPER, mappings.bumper_right));

    // Face Buttons
    m_button_map.insert(create_button(GamepadButton::ACTION_DOWN, mappings.button_down));
    m_button_map.insert(create_button(GamepadButton::ACTION_RIGHT, mappings.button_right));
    m_button_map.insert(create_button(GamepadButton::ACTION_LEFT, mappings.button_left));
    m_button_map.insert(create_button(GamepadButton::ACTION_UP, mappings.button_up));

    // Stick Buttons
    m_button_map.insert(create_button(GamepadButton::LEFT_STICK, mappings.button_left_stick));
    m_button_map.insert(create_button(GamepadButton::RIGHT_STICK, mappings.button_right_stick));

    // DPAD Buttons
    m_button_map.insert(create_button(GamepadButton::DPAD_DOWN, mappings.button_dpad_down));
    m_button_map.insert(create_button(GamepadButton::DPAD_RIGHT, mappings.button_dpad_right));
    m_button_map.insert(create_button(GamepadButton::DPAD_LEFT, mappings.button_dpad_left));
    m_button_map.insert(create_button(GamepadButton::DPAD_UP, mappings.button_dpad_up));

    // Special Buttons
    m_button_map.insert(create_button(GamepadButton::HOME, mappings.button_home));
    m_button_map.insert(create_button(GamepadButton::HOME, mappings.button_settings));

} // end of "init(GampepadMapping)"


void Gamepad::update(vector<float> axes, vector<int> buttons)
{
    // Update all buttons
    for(const auto& button : m_button_map)
    {
        button.second->update(buttons);
    }

    // Update all axes
    for(const auto& axis : m_axis_map)
    {
        axis.second->update(axes);
    }

} // end of "update"


void Gamepad::update(const sensor_msgs::msg::Joy& joy_packet)
{
    update(joy_packet.axes, joy_packet.buttons);

} // end of "update"


Button* Gamepad::get_button(int id)
{
    return m_button_map.at(id);

} // end of "get_button"


Axis* Gamepad::get_axis(int id)
{
    return m_axis_map.at(id);

} // end of "get_axis"


pair<int, Axis*> Gamepad::create_axis(int id, int index)
{
    return {id, new Axis(index, id)};

} // end of "create_axis(int, int)"


pair<int, Button*> Gamepad::create_button(int id, int index)
{
    return {id, new Button(index, id)};

} // end of "create_axis(int, int)"