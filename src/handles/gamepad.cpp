#include "handles/gamepad.hpp"


using namespace std;


Gamepad::Gamepad(GamepadMapping mappings)
{
    init(mappings);

} // end of constructor


void Gamepad::init(GamepadMapping mappings)
{
    // Stick Axes
    m_stick_left_x = Axis(mappings.stick_left_x,   GamepadAxis::LEFT_X);
    m_stick_left_y = Axis(mappings.stick_left_y,   GamepadAxis::LEFT_Y);
    m_stick_right_x = Axis(mappings.stick_right_x, GamepadAxis::RIGHT_X);
    m_stick_right_y = Axis(mappings.stick_right_y, GamepadAxis::RIGHT_Y);

    // Trigger Axes
    m_trigger_axis_left = Axis(mappings.trigger_left,      GamepadAxis::LEFT_TRIGGER);
    m_trigger_axis_right = Axis(mappings.trigger_right,    GamepadAxis::RIGHT_TRIGGER);

    // Trigger Buttons
    m_trigger_button_left = Button(mappings.trigger_left,    GamepadButton::LEFT_TRIGGER);
    m_trigger_button_right = Button(mappings.trigger_right,  GamepadButton::RIGHT_TRIGGER);

    // Bumper Buttons
    m_bumper_left = Button(mappings.bumper_left,     GamepadButton::LEFT_BUMPER);
    m_bumper_right = Button(mappings.bumper_right,   GamepadButton::RIGHT_BUMPER);

    // Face Buttons
    m_button_down = Button(mappings.button_down,     GamepadButton::ACTION_DOWN);
    m_button_right = Button(mappings.button_right,   GamepadButton::ACTION_RIGHT);
    m_button_left = Button(mappings.button_left,     GamepadButton::ACTION_LEFT);
    m_button_up = Button(mappings.button_up,         GamepadButton::ACTION_UP);

    // Stick Buttons
    m_button_left_stick = Button(mappings.button_left_stick,     GamepadButton::LEFT_STICK);
    m_button_right_stick = Button(mappings.button_right_stick,   GamepadButton::RIGHT_STICK);

    // Configure the axes map
    m_axis_map[m_stick_left_x.get_id()] = &m_stick_left_x;
    m_axis_map[m_stick_left_y.get_id()] = &m_stick_left_y;

    m_axis_map[m_stick_right_x.get_id()] = &m_stick_right_x;
    m_axis_map[m_stick_right_y.get_id()] = &m_stick_right_y;

    m_axis_map[m_trigger_axis_left.get_id()] = &m_trigger_axis_left;
    m_axis_map[m_trigger_axis_right.get_id()] = &m_trigger_axis_right;

    // Configure the buttons map
    m_button_map[m_trigger_button_left.get_id()] = &m_trigger_button_left;
    m_button_map[m_trigger_button_right.get_id()] = &m_trigger_button_right;

    m_button_map[m_bumper_left.get_id()] = &m_bumper_left;
    m_button_map[m_bumper_right.get_id()] = &m_bumper_right;

    m_button_map[m_button_down.get_id()] = &m_button_down;
    m_button_map[m_button_right.get_id()] = &m_button_right;
    m_button_map[m_button_left.get_id()] = &m_button_left;
    m_button_map[m_button_up.get_id()] = &m_button_up;

    m_button_map[m_button_left_stick.get_id()] = &m_button_left_stick;
    m_button_map[m_button_right_stick.get_id()] = &m_button_right_stick;

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