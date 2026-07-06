#include "nodes/joystick_node.hpp"

using std::placeholders::_1;
using namespace joystick_driver;
using namespace rclcpp;
using namespace std;

using TwistStamped = geometry_msgs::msg::TwistStamped;
using Joy = sensor_msgs::msg::Joy;


JoystickNode::JoystickNode() : Node("joystick_teleop")
{
    // Initialize the parameters
    m_param_listener = std::make_shared<ParamListener>(this);
    m_params = m_param_listener->get_params();

    // If the user specified a preset joystick_type
    // Then use that mapping
    if(m_params.joystick_type != "none")
    {
        RCLCPP_INFO(this->get_logger(), "Using preset file for mappings");

        gamepad.initialize(
            get_mapping_from_yaml(m_params.joystick_type + ".yaml"),
            get_yaml_params(m_params.joystick_type + ".yaml")["use_triggers_as_buttons"].as<bool>()
        );
    }
    // If not then fallback to the explicit mapping
    else
    {
        RCLCPP_INFO(this->get_logger(), "Using custom mappings");
        // Assign the ROS Params mapping to the Gamepad API
        gamepad.initialize(
            get_mapping(),
            m_params.use_triggers_as_buttons
        );
    }

    // Create the TwistStamped publisher
    publisher = this->create_publisher<TwistStamped>(m_params.twist_topic, SystemDefaultsQoS());

    // Create the raw joystick subscriber
    subscription = this->create_subscription<Joy>(
        "joy", 
        SystemDefaultsQoS(), 
        bind(&JoystickNode::topic_callback, this, _1)
    );

} // end of "JoystickNode()"


void JoystickNode::topic_callback(const Joy& message) {

    gamepad.update(message);

    if(gamepad.get_button(GamepadButton::ACTION_UP)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed UP");

    if(gamepad.get_button(GamepadButton::ACTION_DOWN)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed DOWN");

    if(gamepad.get_button(GamepadButton::ACTION_LEFT)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed LEFT");

    if(gamepad.get_button(GamepadButton::ACTION_RIGHT)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed RIGHT");

    if(gamepad.get_button(GamepadButton::LEFT_STICK)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed LEFT STICK");

    if(gamepad.get_button(GamepadButton::RIGHT_STICK)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed RIGHT STICK");

    if(gamepad.get_button(GamepadButton::LEFT_BUMPER)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed LEFT BUMPER");

    if(gamepad.get_button(GamepadButton::RIGHT_BUMPER)->on_press())
        RCLCPP_INFO(this->get_logger(), "pressed RIGHT BUMPER");

    // Publish the TwistStamped message
    publisher->publish(
        create_twist(
            GamepadAxis::LEFT_Y, 
            GamepadAxis::RIGHT_X
        )
    );
    
} // end of "topic_callback(const Joy&)"


TwistStamped JoystickNode::create_twist(int linear_axis, int angular_axis)
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


GamepadMapping JoystickNode::get_mapping()
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


GamepadMapping JoystickNode::get_mapping_from_yaml(string filename)
{
    auto params = get_yaml_params(filename);

    // Populate the actual mapping object
    GamepadMapping mapping;
    
    // Bumpers
    mapping.bumper_left =   params["bumper_left"].as<int>();
    mapping.bumper_right =  params["bumper_right"].as<int>();

    // Buttons
    mapping.button_up =     params["button_up"].as<int>();
    mapping.button_down =   params["button_down"].as<int>();
    mapping.button_left =   params["button_left"].as<int>();
    mapping.button_right =  params["button_right"].as<int>();

    // Joystick Buttons (click)
    mapping.button_left_stick =     params["button_left_stick"].as<int>();
    mapping.button_right_stick =    params["button_right_stick"].as<int>();

    // Joystick Axes
    mapping.stick_left_x =  params["stick_left_x"].as<int>();
    mapping.stick_left_y =  params["stick_left_y"].as<int>();
    mapping.stick_right_x = params["stick_right_x"].as<int>();
    mapping.stick_right_y = params["stick_right_y"].as<int>();

    // Trigger Axes
    mapping.trigger_left =  params["trigger_left"].as<int>();
    mapping.trigger_right = params["trigger_right"].as<int>();

    return mapping;

} // end of "get_mapping_from_yaml(string)"


YAML::Node JoystickNode::get_yaml_params(string filename)
{
    // The share folder of the given package
    string share_folder = ament_index_cpp::get_package_share_directory("joystick_driver");

    // The path to the yaml file
    filesystem::path config_path = filesystem::path(share_folder) / "config" / filename;

    // Parsed contents
    YAML::Node contents = YAML::LoadFile(config_path.string());

    // Get specifically the ros__parameters
    auto params = contents[get_name()]["ros__parameters"];

    return params;

} // end of "get_yaml_params(string)"
