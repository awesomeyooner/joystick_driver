#ifndef GAMEPAD_MAPPING_HPP
#define GAMEPAD_MAPPING_HPP

/**
 * @brief Stores the indexes of each axis / button
 * for where they are in their respective arrays in the `joy_msg`
 * 
 */
struct GamepadMapping
{
    // Thumbstick axes
    int stick_left_x = -1;
    int stick_left_y = -1;
    int stick_right_x = -1;
    int stick_right_y = -1;

    // Triggers as buttons (optional)
    int button_trigger_left = -1;
    int button_trigger_right = -1;

    // Triggers as axes (optional)
    int axis_trigger_left = -1;
    int axis_trigger_right = -1;

    // Bumper Buttons
    int bumper_left = -1;
    int bumper_right = -1;

    // Face Buttons
    int button_down = -1;
    int button_right = -1;
    int button_left = -1;
    int button_up = -1;

    // Thumbstick Buttons
    int button_left_stick = -1;
    int button_right_stick = -1;

    // DPAD as buttons (optional)
    int button_dpad_up = -1;
    int button_dpad_down = -1;
    int button_dpad_left = -1;
    int button_dpad_right = -1;

    // DPAD as axes (optional)
    int axis_dpad_x = -1;
    int axis_dpad_y = -1;

    // Special Buttons
    int button_home = -1;
    int button_settings = -1;
    
}; // struct GamepadMapping


#endif // GAMEPAD_MAPPING_HPP