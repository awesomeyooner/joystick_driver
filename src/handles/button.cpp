#include "handles/button.hpp"


using namespace std;


Button::Button(int index, int id) :
    m_index_in_msg(index), m_universal_id(id)
{
    // Initialze with all states to false
    m_is_pressed = false;
    m_on_press = false;
    m_on_release = false;
    m_previously_pressed = false;

} // end of constructor


bool Button::is_pressed()
{
    return m_is_pressed;

} // end of "is_pressed"


bool Button::on_press()
{
    return m_on_press;

} // end of "on_press"


bool Button::on_release()
{
    return m_on_release;

} // end of "on_release"


void Button::update(bool is_pressed)
{
    // Set the previous state to the "current" state
    // prior to updating it
    m_previously_pressed = m_is_pressed;

    // Update the current state
    m_is_pressed = is_pressed;

    // TRUE if currently PRESSED and previously NOT PRESSED
    m_on_press = m_is_pressed && !m_previously_pressed;
    
    // TRUE if currently NOT PRESSED and previously PRESSED
    m_on_release = !m_is_pressed && m_previously_pressed;

} // end of "update"


void Button::update(const vector<int>& buttons)
{
    // OOB Check
    if(m_index_in_msg < buttons.size() && m_index_in_msg > 0)
        return;

    // Get the current state of the button from the message
    bool is_pressed = buttons.at(m_index_in_msg) == 1 ? true : false;

    // Call the update function with the new state
    update(is_pressed);

} // end of "update"


int Button::get_id()
{
    return m_universal_id;

} // end of "get_id"


int Button::get_index()
{
    return m_index_in_msg;

} // end of "get_index"

