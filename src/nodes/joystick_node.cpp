#include "nodes/joystick_node.hpp"

using std::placeholders::_1;
using namespace joystick_driver;
using namespace rclcpp;
using namespace std;

using TwistStamped = geometry_msgs::msg::TwistStamped;
using Joy = sensor_msgs::msg::Joy;

//0 left x 
//1 left y
//2 left trigger

//3 right x
//4 right y
//5 right trigger
Joystick::Joystick() : Node("joystick_teleop")
{
    // Initialize the parameters
    m_param_listener = std::make_shared<ParamListener>(this);
    m_params = m_param_listener->get_params();

    // Assign the mapping to the Gamepad API
    gamepad.initialize(
        get_mapping()
    );

    // Create the TwistStamped publisher
    publisher = this->create_publisher<TwistStamped>(m_params.twist_topic, SystemDefaultsQoS());

    // Create the raw joystick subscriber
    subscription = this->create_subscription<Joy>(
        "joy", 
        SystemDefaultsQoS(), 
        bind(&Joystick::topic_callback, this, _1)
    );

} // end of "Joystick()"


void Joystick::topic_callback(const Joy& message) {

    gamepad.update(message);

    if(gamepad.get_button(PS4::X)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed X");
    
    if(gamepad.get_button(PS4::X)->on_release())
        RCLCPP_INFO(this->get_logger(), "released X");

    if(gamepad.get_button(PS4::CIRCLE)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed CIRCLE");

    if(gamepad.get_button(PS4::SQUARE)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed SQUARE");

    if(gamepad.get_button(PS4::TRIANGLE)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed TRIANGLE");

    if(gamepad.get_button(PS4::LEFT_STICK)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed LEFT STICk");

    if(gamepad.get_button(PS4::RIGHT_STICK)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed RIGHT STICK");

    if(gamepad.get_button(PS4::LEFT_BUMPER)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed LEFT BUMPER");

    if(gamepad.get_button(PS4::RIGHT_BUMPER)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed RIGHT BUMPER");

    // Publish the TwistStamped message
    publisher->publish(
        create_twist(
            GamepadAxis::LEFT_Y, 
            GamepadAxis::RIGHT_X
        )
    );
    
} // end of "topic_callback(const Joy&)"


TwistStamped Joystick::create_twist(int linear_axis, int angular_axis)
{
    TwistStamped twist_stamped = TwistStamped();

    // Populate the header
    twist_stamped.header.frame_id = m_params.twist_frame_id;
    twist_stamped.header.stamp = this->now();

    // Populate the actual Twist part
    twist_stamped.twist.linear.x = gamepad.get_axis(linear_axis)->get() * m_params.max_tangential_velocity;
    twist_stamped.twist.angular.z = gamepad.get_axis(angular_axis)->get() * m_params.max_angular_velocity;

    return twist_stamped;

} // end of "create_twist(int, int)"


GamepadMapping Joystick::get_mapping()
{
    GamepadMapping mapping;

    // Bumpers
    mapping.bumper_left =   m_params.bumper_left;
    mapping.bumper_right =  m_params.bumper_right;

    // Buttons
    mapping.button_up =     m_params.button_up;
    mapping.button_down =   m_params.button_down;
    mapping.button_left =   m_params.button_left;
    mapping.button_right =  m_params.button_right;

    // Joystick Buttons (click)
    mapping.button_left_stick =     m_params.button_left_stick;
    mapping.button_right_stick =    m_params.button_right_stick;

    // Joystick Axes
    mapping.stick_left_x =  m_params.stick_left_x;
    mapping.stick_left_y =  m_params.stick_left_y;
    mapping.stick_right_x = m_params.stick_right_x;
    mapping.stick_right_y = m_params.stick_right_y;

    // Trigger Axes
    mapping.trigger_left =  m_params.trigger_left;
    mapping.trigger_right = m_params.trigger_right;

    return mapping;

} // end of "get_mapping()"