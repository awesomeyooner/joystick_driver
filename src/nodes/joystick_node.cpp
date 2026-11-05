#include "nodes/joystick_node.hpp"

using std::placeholders::_1;
using namespace joystick_driver;
using namespace rclcpp;
using namespace std;

using TwistStamped = geometry_msgs::msg::TwistStamped;
using Twist = geometry_msgs::msg::Twist;
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

        gamepad.init(
            get_mapping_from_yaml(m_params.joystick_type + ".yaml")
            // get_yaml_params(m_params.joystick_type + ".yaml")["use_triggers_as_buttons"].as<bool>()
        );
    }
    // If not then fallback to the explicit mapping
    else
    {
        RCLCPP_INFO(this->get_logger(), "Using custom mappings");
        // Assign the ROS Params mapping to the Gamepad API
        gamepad.init(
            get_mapping()
            // m_params.use_triggers_as_buttons
        );
    }

    // `twist_topic`/stamped if the param is empty, if not then use the param
    string stamped_topic = m_params.stamped_topic.empty() ? m_params.twist_topic + "/stamped" : m_params.stamped_topic;

    // If only publishing stamped, then use the normal twist_topic
    if(m_params.twist_type == "stamped")
        stamped_topic = m_params.twist_topic;

    // Create the TwistStamped and Twist publisher
    if(m_params.twist_type == "stamped" || m_params.twist_type == "both")
        stamped_publisher = this->create_publisher<TwistStamped>(stamped_topic, SystemDefaultsQoS());
    if(m_params.twist_type == "unstamped" || m_params.twist_type == "both")
        unstamped_publisher = this->create_publisher<Twist>(m_params.twist_topic, SystemDefaultsQoS());

    // Create the raw joystick subscriber
    subscription = this->create_subscription<Joy>(
        "joy", 
        SystemDefaultsQoS(), 
        bind(&JoystickNode::topic_callback, this, _1)
    );

    m_timer = this->create_wall_timer(
        chrono::milliseconds(1000 / m_params.publish_rate), // Publish every 100ms
        bind(&JoystickNode::publish, this)
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

    publish();
    
} // end of "topic_callback(const Joy&)"


void JoystickNode::publish()
{
    TwistStamped twist_stamped = create_twist(
        GamepadAxis::LEFT_Y, 
        GamepadAxis::RIGHT_X
    );

    // Publish the twist type based on the given parameter
    if(m_params.twist_type == "stamped")
    {
        stamped_publisher->publish(twist_stamped);
    }
    else if(m_params.twist_type == "unstamped")
    {
        unstamped_publisher->publish(twist_stamped.twist);
    }
    else if(m_params.twist_type == "both")
    {
        stamped_publisher->publish(twist_stamped);
        unstamped_publisher->publish(twist_stamped.twist);
    }

} // end of "publish()"


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

    mapping.stick_left_x = m_params.stick_left_x;
    mapping.stick_left_y = m_params.stick_left_y;
    mapping.stick_right_x = m_params.stick_right_x;
    mapping.stick_right_y = m_params.stick_right_y;

    // Triggers as buttons (optional)
    mapping.button_trigger_left = m_params.button_trigger_left;
    mapping.button_trigger_right = m_params.button_trigger_right;

    // Triggers as axes (optional)
    mapping.axis_trigger_left = m_params.axis_trigger_left;
    mapping.axis_trigger_right = m_params.axis_trigger_right;

    // Bumper Buttons
    mapping.bumper_left = m_params.bumper_left;
    mapping.bumper_right = m_params.bumper_right;

    // Face Buttons
    mapping.button_down = m_params.button_down;
    mapping.button_right = m_params.button_right;
    mapping.button_left = m_params.button_left;
    mapping.button_up = m_params.button_up;

    // Thumbstick Buttons
    mapping.button_left_stick = m_params.button_left_stick;
    mapping.button_right_stick = m_params.button_right_stick;

    // DPAD as buttons (optional)
    mapping.button_dpad_up = m_params.button_dpad_up;
    mapping.button_dpad_down = m_params.button_dpad_down;
    mapping.button_dpad_left = m_params.button_dpad_left;
    mapping.button_dpad_right = m_params.button_dpad_right;

    // DPAD as axes (optional)
    mapping.axis_dpad_x = m_params.axis_dpad_x;
    mapping.axis_dpad_y = m_params.axis_dpad_y;

    // Home Button
    mapping.button_home = m_params.button_home;
    mapping.button_settings = m_params.button_settings;

    return mapping;

} // end of "get_mapping()"


GamepadMapping JoystickNode::get_mapping_from_yaml(string filename)
{
    auto params = get_yaml_params(filename);

    // Populate the actual mapping object
    GamepadMapping mapping;
    
    // Joystick Axes
    mapping.stick_left_x =  params["stick_left_x"] ? params["stick_left_x"].as<int>() : -1;
    mapping.stick_left_y =  params["stick_left_y"] ? params["stick_left_y"].as<int>() : -1;
    mapping.stick_right_x = params["stick_right_x"] ? params["stick_right_x"].as<int>() : -1;
    mapping.stick_right_y = params["stick_right_y"] ? params["stick_right_y"].as<int>() : -1;

    // Triggers as Buttons
    mapping.button_trigger_left =  params["button_trigger_left"] ? params["button_trigger_left"].as<int>() : -1;
    mapping.button_trigger_right = params["button_trigger_right"] ? params["button_trigger_right"].as<int>() : -1;

    // Triggers as Axes
    mapping.axis_trigger_left =  params["axis_trigger_left"] ? params["axis_trigger_left"].as<int>() : -1;
    mapping.axis_trigger_right = params["axis_trigger_right"] ? params["axis_trigger_right"].as<int>() : -1;

    // Bumper Buttons
    mapping.bumper_left =  params["bumper_left"] ? params["bumper_left"].as<int>() : -1;
    mapping.bumper_right = params["bumper_right"] ? params["bumper_right"].as<int>() : -1;

    // Face Buttons
    mapping.button_down =  params["button_down"] ? params["button_down"].as<int>() : -1;
    mapping.button_right = params["button_right"] ? params["button_right"].as<int>() : -1;
    mapping.button_left =  params["button_left"] ? params["button_left"].as<int>() : -1;
    mapping.button_up =    params["button_up"] ? params["button_up"].as<int>() : -1;

    // Thumbstick Buttons
    mapping.button_left_stick =  params["button_left_stick"] ? params["button_left_stick"].as<int>() : -1;
    mapping.button_right_stick = params["button_right_stick"] ? params["button_right_stick"].as<int>() : -1;

    // DPAD as Buttons
    mapping.button_dpad_up =    params["button_dpad_up"] ? params["button_dpad_up"].as<int>() : -1;
    mapping.button_dpad_down =  params["button_dpad_down"] ? params["button_dpad_down"].as<int>() : -1;
    mapping.button_dpad_left =  params["button_dpad_left"] ? params["button_dpad_left"].as<int>() : -1;
    mapping.button_dpad_right = params["button_dpad_right"] ? params["button_dpad_right"].as<int>() : -1;

    // DPAD as Axes
    mapping.axis_dpad_x = params["axis_dpad_x"] ? params["axis_dpad_x"].as<int>() : -1;
    mapping.axis_dpad_y = params["axis_dpad_y"] ? params["axis_dpad_y"].as<int>() : -1;

    // Home and Settings Buttons
    mapping.button_home = params["button_home"] ? params["button_home"].as<int>() : -1;
    mapping.button_settings = params["button_settings"] ? params["button_settings"].as<int>() : -1;

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
