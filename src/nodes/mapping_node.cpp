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

    string prompt = get_prompt();

    // If we have populated all prompts
    // Then create the file and shutdown
    if(prompt == "")
    {
        create_yaml();
        rclcpp::shutdown();

        return StatusCode::OK;
    }

    // If we're ready to send a prompt
    // Then send it and change the state accordingly
    if(prompt_state == PromptState::READY)
    {
        if(ask_for_home)
            info("Please activate: " + prompt + " and press HOME to bind");
        else
            info("Please activate: " + prompt);

        prompt_state = PromptState::WAITING_FOR_ACTIVITY;
    }

    vector<int> active_buttons = get_active_buttons(msg.buttons);
    vector<int> active_axes = get_active_axes(msg.axes);

    // Don't advance until there is activity
    if(active_buttons.size() == 0 && active_axes.size() == 0)
        return StatusCode::OK;

    // If we need to wait for home button to be pressed and it's not pressed
    // Then don't advance
    if(ask_for_home && !get_home_button(msg.buttons))
        return StatusCode::OK;  

    // If we're using a dual binding (axis and button)
    // Then add the mapping to both buttons and axes
    if(active_buttons.size() == 1 && active_axes.size() == 1)
    {
        button_mappings.insert({"button_" + prompt, active_buttons.at(0)});
        axis_mappings.insert({"axis_" + prompt, active_axes.at(0)});

        prompt_state = PromptState::READY;

        return StatusCode::OK;
    }
    else if(active_buttons.size() == 1)
    {
        button_mappings.insert({prompt, active_buttons.at(0)});

        prompt_state = PromptState::READY;

        return StatusCode::OK;
    }
    else if(active_axes.size() == 1)
    {
        axis_mappings.insert({prompt, active_axes.at(0)});

        prompt_state = PromptState::READY;

        return StatusCode::OK;
    }

    return StatusCode::OK;

} // end of "topic_callback(const sensor_msgs::msg::Joy&)"


StatusCode MappingNode::create_yaml()
{
    info("Preparing to create joystick_mappings.yaml file...");

    YAML::Emitter yaml_out;
    ofstream output_file("joystick_mappings.yaml");

    if(!output_file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to file!");
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
                // yaml_out << YAML::Key << "use_triggers_as_buttons";
                // yaml_out << YAML::Value << triggers_as_buttons;

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

    info("Successfully created .yaml file!");
    
    return StatusCode::OK;

} // end of "create_yaml()"


string MappingNode::get_prompt()
{
    // For every prompt
    // See if we've already assigned this prompt to a mapping
    // If so, skip
    // If not, return that prompt
    for(string prompt : prompts)
    {
        // If the prompt is already assigned in the mappings
        bool in_buttons = button_mappings.find(prompt) != button_mappings.end();
        bool in_axes = axis_mappings.find(prompt) != axis_mappings.end();

        // If this given prompt is already assigned in the mappings
        // Then skip it
        if(in_buttons || in_axes)
            continue;

        return prompt;
    }

    // If code gets here then that means we've filled up all normal prompts, so switch to dynamic
    ask_for_home = true;

    for(string prompt : dynamic_prompts)
    {
        // If the prompt is already assigned in the mappings
        bool in_buttons = button_mappings.find("button_" + prompt) != button_mappings.end() || button_mappings.find(prompt) != button_mappings.end();
        bool in_axes = axis_mappings.find("axis_" + prompt) != axis_mappings.end() || axis_mappings.find(prompt) != axis_mappings.end();

        // If this given prompt is already assigned in the mappings
        // Then skip it
        if(in_buttons || in_axes)
            continue;

        return prompt;
    }

    return "";

} // end of "get_prompt()"


vector<int> MappingNode::get_active_buttons(const vector<int>& button_states)
{
    // Indexes of which buttons are actively pressed
    vector<int> active;

    // Indexes of buttons we've already mapped
    vector<int> excluded = get_values(button_mappings);

    // Push back all the indexes whose values are equal to 1 (active)
    for(int i = 0; i < button_states.size(); i++)
    {
        // Skip this button if it's in the exlcuded list
        if(find(excluded.begin(), excluded.end(), i) != excluded.end())
            continue;

        int initial_state = initial_button_states.at(i);
        int current_state = button_states.at(i);

        // If the button is active
        if(abs(current_state - initial_state) >= ACTIVE_BUTTON_THRESHOLD)
            active.push_back(i);
    }

    return active;

} // end of "get_active_buttons(const vector<int>&)"


vector<int> MappingNode::get_active_axes(const vector<float>& axis_states)
{
    // Indexes of which axes are actively pressed
    vector<int> active;

    vector<int> excluded = get_values(axis_mappings);

    // Push back all the indexes whose values are equal to 1 (active)
    for(int i = 0; i < axis_states.size(); i++)
    {
        // Skip this axis if it's in the exlcuded list
        if(find(excluded.begin(), excluded.end(), i) != excluded.end())
            continue;

        float initial_state = initial_axis_states.at(i);
        float current_state = axis_states.at(i);

        // If the button is active
        if(abs(current_state - initial_state) >= ACTIVE_AXIS_THRESHOLD)
            active.push_back(i);
    }

    return active;

} // end of "get_active_buttons(const vector<int>&)"


bool MappingNode::is_resting(const vector<int>& button_states, const vector<float>& axis_states)
{
    // Check all current button states against the initial states
    // If one is not matching
    // Then return false
    for(int i = 0; i < button_states.size(); i++)
    {
        if(button_states.at(i) != initial_button_states.at(i))
            return false;
    }

    // Check all current axis states against initial states
    // If one differs too much
    // Then return false
    for(int i = 0; i < axis_states.size(); i++)
    {
        if(abs(axis_states.at(i) - initial_axis_states.at(i)) > RESTING_AXIS_THRESHOLD)
            return false;
    }

    return true;

} // end of "is_resting(const vector<int>&, const vector<float>&)"


bool MappingNode::get_home_button(const std::vector<int>& button_states)
{
    // If the home button hasn't been set yet
    // Then return false
    if(button_mappings.find("home") == button_mappings.end())
        return false;

    int home_button_index = button_mappings.at("home");

    return abs(button_states.at(home_button_index) - initial_button_states.at(home_button_index));
 
} // end of "get_home_button(const std::vector<int>&)"


vector<int> MappingNode::get_values(const map<string, int>& map)
{
    vector<int> values;

    // Push back every value in the map
    for(const auto& pair : map)
    {
        values.push_back(pair.second);
    }

    return values;

} // end of "get_values(const map<string, int>&)"


void MappingNode::info(string message)
{
    RCLCPP_INFO(this->get_logger(), message.c_str());

} // end of "info(string)"