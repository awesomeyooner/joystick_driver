#include "nodes/mapping_node.hpp"


using namespace std;
using namespace status_utils;
using std::placeholders::_1;


MappingNode::MappingNode() : Node("create_mappings")
{
    subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 
        rclcpp::SystemDefaultsQoS(), 
        std::bind(&MappingNode::topic_callback, this, _1)
    );

} // end of "MappingNode()"


StatusCode MappingNode::topic_callback(const sensor_msgs::msg::Joy& msg)
{
    // Set the initial button and axis states to
    // compare future events
    if(initial_button_states.size() == 0)
        initial_button_states = msg.buttons;
    if(initial_axis_states.size() == 0)
        initial_axis_states = msg.axes;

    string current_prompt = get_current_prompt();

    // If we have populated all prompts
    // Then create the file and shutdown
    if(current_prompt == "")
    {
        create_file();
        rclcpp::shutdown();

        return StatusCode::OK;
    }

    if(!waiting_for_input)
    {
        Logger::info("Please activate: " + current_prompt);
        waiting_for_input = true;
    }

    switch(prompt_state)
    {
        case PromptState::BUTTONS:
        {

            vector<int> active_buttons = get_active_buttons(msg.buttons, button_mappings);

            if(active_buttons.size() != 1)
                return StatusCode::OK;

            button_mappings.insert({current_prompt, active_buttons.at(0)});

            break;
        }

        case PromptState::AXES:
        {
            vector<int> active_axes = get_active_axes(msg.axes, axis_mappings);

            if(active_axes.size() != 1)
                return StatusCode::OK;

            axis_mappings.insert({current_prompt, active_axes.at(0)});
            break;
        }

        case PromptState::DYNAMIC:
        {
            vector<int> active_axes = get_active_axes(msg.axes, axis_mappings);
            vector<int> active_buttons = get_active_buttons(msg.buttons, button_mappings);

            // If there's NOT only one active action
            // Then return
            if(active_axes.size() + active_buttons.size() != 1)
                return StatusCode::OK;

            if(active_axes.size() == 1)
            {
                axis_mappings.insert({current_prompt, active_axes.at(0)});
            }
            else if(active_buttons.size() == 1)
            {
                button_mappings.insert({current_prompt, active_buttons.at(0)});
                triggers_as_buttons = true;
            }

            break;
        }
    }

    waiting_for_input = false;

    return StatusCode::OK;

} // end of "topic_callback(const sensor_msgs::msg::Joy&)"


StatusCode MappingNode::create_file()
{
    Logger::info("Preparing to create joystick_mappings.yaml file...");

    YAML::Emitter yaml_out;
    ofstream output_file("joystick_mappings.yaml");

    if(!output_file.is_open())
    {
        Logger::error("Failed to write to file!");
        output_file.close();
        return StatusCode::FAILED;
    }

    // Create one map to work with YAML
    map<string, int> mappings = button_mappings;
    mappings.merge(axis_mappings);

    // Begin joystick_teleop
    yaml_out << YAML::BeginMap;
    yaml_out << YAML::Key << "joystick_teleop" << YAML::Value;

        // Begin ros__parameters
        yaml_out << YAML::BeginMap;
        yaml_out << YAML::Key << "ros__parameters" << YAML::Value;

            yaml_out << YAML::BeginMap;
                yaml_out << YAML::Key << "use_triggers_as_buttons";
                yaml_out << YAML::Value << triggers_as_buttons;

                // Manually enter map because normal << needs a parent
                for(const auto& [key, value] : mappings)
                {
                    yaml_out << YAML::Key << key;
                    yaml_out << YAML::Value << value;
                }

            yaml_out << YAML::EndMap;
            
        // End ros__parameters
        yaml_out << YAML::EndMap;
    
    // End joystick_teleop
    yaml_out << YAML::EndMap;

    // Write to the file
    output_file << yaml_out.c_str();

    output_file.close();

    Logger::info("Successfully created .yaml file!");
    
    return StatusCode::OK;
} // end of "create_file()"


string MappingNode::get_current_prompt()
{
    // Buttons first then axes

    for(string prompt : button_prompts)
    {
        // If this prompt was already set
        // Then skip it
        if(button_mappings.count(prompt) == 1)
            continue;

        return prompt;
    }

    // If code gets here, then all buttom prompts are set
    prompt_state = PromptState::AXES;

    for(string prompt : axes_prompts)
    {
        // If this prompt was already set
        // Then skip it
        if(axis_mappings.count(prompt) == 1)
            continue;

        return prompt;
    }

    // If code gets here, then all buttom and axis prompts are set
    prompt_state = PromptState::DYNAMIC;

    for(string prompt : dynamic_prompts)
    {
        // If this prompt was already set in buttons
        // Then skip it
        if(button_mappings.count(prompt) == 1)
            continue;

        // If this prompt was already set in axes
        // Then skip it
        if(axis_mappings.count(prompt) == 1)
            continue;

        return prompt;
    }

    return "";

} // end of "get_current_prompt()"


vector<int> MappingNode::get_active_buttons(const vector<int>& button_states, const map<string, int>& excluding)
{
    // Indexes of which buttons are actively pressed
    vector<int> active_buttons;

    vector<int> excluded_buttons = get_values(button_mappings);

    // Push back all the indexes whose values are equal to 1 (active)
    for(int i = 0; i < button_states.size(); i++)
    {
        // Skip this button if it's in the exlcuded list
        if(count(excluded_buttons.begin(), excluded_buttons.end(), i) == 1)
            continue;

        int initial_state = initial_button_states.at(i);
        int current_state = button_states.at(i);

        // If the button is active
        if(abs(current_state - initial_state) >= ACTIVE_BUTTON_THRESHOLD)
            active_buttons.push_back(i);
    }

    return active_buttons;

} // end of "get_active_buttons(const vector<int>&)"


vector<int> MappingNode::get_active_axes(const vector<float>& axis_states, const map<string, int>& excluding)
{
    // Indexes of which axes are actively pressed
    vector<int> active_axes;

    vector<int> excluded_buttons = get_values(axis_mappings);

    // Push back all the indexes whose values are equal to 1 (active)
    for(int i = 0; i < axis_states.size(); i++)
    {
        // Skip this button if it's in the exlcuded list
        if(count(excluded_buttons.begin(), excluded_buttons.end(), i) == 1)
            continue;

        float initial_state = initial_axis_states.at(i);
        float current_state = axis_states.at(i);

        // If the button is active
        if(abs(current_state - initial_state) >= ACTIVE_AXIS_THRESHOLD)
            active_axes.push_back(i);
    }

    return active_axes;

} // end of "get_active_buttons(const vector<int>&)"


vector<int> MappingNode::get_values(const map<string, int>& map)
{
    vector<int> values;

    for(const auto& pair : map)
    {
        values.push_back(pair.second);
    }

    return values;

} // end of "get_values(const map<string, int>&)"