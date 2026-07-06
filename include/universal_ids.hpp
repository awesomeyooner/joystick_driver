#ifndef UNIVERSAL_IDS_HPP
#define UNIVERSAL_IDS_HPP


 // Universal Button ID
namespace GamepadButton
{
    enum
    {
        ACTION_DOWN = 0,
        ACTION_RIGHT = 1,
        ACTION_LEFT = 2,
        ACTION_UP = 3,

        LEFT_BUMPER = 4,
        RIGHT_BUMPER = 5,

        LEFT_STICK = 6,
        RIGHT_STICK = 7,

        DPAD_UP = 8,
        DPAD_DOWN = 9,
        DPAD_LEFT = 10,
        DPAD_RIGHT = 11,

        // In the case that triggers are buttons
        // These possibly might not be used
        LEFT_TRIGGER = 12,
        RIGHT_TRIGGER = 13

    }; // enum

} // namespace GamepadButton

// Universal Axis ID
namespace GamepadAxis
{
    enum
    {
        LEFT_X = 0,
        LEFT_Y = 1,
        RIGHT_X = 2,
        RIGHT_Y = 3,

        // In the case that triggers are axes
        // These possibly might not be used
        LEFT_TRIGGER = 4,
        RIGHT_TRIGGER = 5

    }; // enum

} // namespace GamepadAxis


#endif // UNIVERSAL_IDS_HPP