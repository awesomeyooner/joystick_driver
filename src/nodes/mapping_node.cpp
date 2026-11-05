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
        std::string message = "Please activate: " + prompt;

        if(ask_for_home)
            prompt += ". Press HOME to bind";
        if(allow_skipping)
            prompt += ". Press SETTINGS to skip";
        
        info("Please activate: " + prompt);

        prompt_state = PromptState::WAITING_FOR_ACTIVITY;
    }

    home_button.update(get_button_state("button_home", msg.buttons));
    settings_button.update(get_button_state("button_settings", msg.buttons));

    // If we allow skipping and the settings button was pressed (to skip)
    // Then remove this prompt and continue to the next one
    if(allow_skipping && settings_button.on_press())
    {
        remove_prompt(prompt);
        prompt_state = PromptState::READY;
        return StatusCode::OK;
    }

    vector<int> active_buttons = get_active_buttons(msg.buttons);
    vector<int> active_axes = get_active_axes(msg.axes);

    // Don't advance until there is activity
    if(active_buttons.size() == 0 && active_axes.size() == 0)
        return StatusCode::OK;

    // If we need to wait for home button to be pressed and it's not pressed
    // Then don't advance
    if(ask_for_home && !home_button.on_press())
        return StatusCode::OK;  

    // Prioritize buttons over axes
    if(active_buttons.size() == 1)
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
    for(string prompt : button_prompts)
    {
        // If this given prompt is already assigned in the mappings
        // Then skip it
        if(button_mappings.find(prompt) != button_mappings.end())
            continue;

        return prompt;
    }

    for(string prompt : axis_prompts)
    {
        // If this given prompt is already assigned in the mappings
        // Then skip it
        if(axis_mappings.find(prompt) != axis_mappings.end())
            continue;

        return prompt;
    }

    // If code gets here then that means we've filled up all normal prompts, so switch to dynamic
    ask_for_home = true;
    allow_skipping = true;

    for(string prompt : dynamic_prompts)
    {
        bool in_buttons = button_mappings.find(prompt) != button_mappings.end();
        bool in_axes = axis_mappings.find(prompt) != axis_mappings.end();

        // If this prompt is already assigned in either mapping
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


bool MappingNode::get_button_state(std::string button, const std::vector<int>& button_states)
{
    // If the button hasn't been set yet
    // Then return false
    if(button_mappings.find(button) == button_mappings.end())
        return false;

    int home_button_index = button_mappings.at(button);

    return abs(button_states.at(home_button_index) - initial_button_states.at(home_button_index));

} // end of "get_button_state(std::string, const std::vector<int>&)"


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


bool MappingNode::remove_prompt(std::string prompt)
{
    auto button_it = find(button_prompts.begin(), button_prompts.end(), prompt);
    // If the prompt is in buttons
    if(button_it != button_prompts.end())
    {   
        button_prompts.erase(button_it);
        return true;    
    }   

    auto axis_it = find(axis_prompts.begin(), axis_prompts.end(), prompt);
    // If the prompt is in axes
    if(axis_it != axis_prompts.end())
    {   
        axis_prompts.erase(axis_it);
        return true;    
    }   

    auto dynamic_it = find(dynamic_prompts.begin(), dynamic_prompts.end(), prompt);
    // If the prompt is in dynamic prompts
    if(dynamic_it != dynamic_prompts.end())
    {   
        dynamic_prompts.erase(dynamic_it);
        return true;    
    }   

    return false;

}


void MappingNode::info(string message)
{
    RCLCPP_INFO(this->get_logger(), message.c_str());

} // end of "info(string)"