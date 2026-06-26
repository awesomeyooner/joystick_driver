#include "joystick_node.hpp"

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
    m_param_listener = std::make_shared<ParamListener>(this);
    m_params = m_param_listener->get_params();

    if(m_params.joystick_type == "ps4")
      gamepad.initialize(PS4::MAP);
    else if(m_params.joystick_type == "xbox")
      gamepad.initialize(Xbox::MAP);
    else if(m_params.joystick_type == "gamesir")
      gamepad.initialize(GameSir::MAP);

    publisher = this->create_publisher<TwistStamped>("cmd_vel", SystemDefaultsQoS());

    subscription = this->create_subscription<Joy>(
        "joy", 
        SystemDefaultsQoS(), 
        bind(&Joystick::topic_callback, this, _1)
    );
}

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

    publisher->publish(
        create_twist(
            GamepadAxis::LEFT_Y, 
            GamepadAxis::RIGHT_X
        )
    );
    
}


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