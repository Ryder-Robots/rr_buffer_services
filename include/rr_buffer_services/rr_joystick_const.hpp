#ifndef RR_JOYSTICK_CONST_HPP
#define RR_JOYSTICK_CONST_HPP

/**
 * The following constants are true for PS4 joysticks not sure if it is true about all controllers.
 * 
 * Note lists for games controllers are split into two parts, sticks which are an anolog component which 
 * gives a value between -1 to 1,  where 0 is centered. 
 * 
 * Buttons which are the various button values on the game controller, these are generally boolean values
 * that can be represented as 1 or 0. Not all buttons are mapped out below.
 */

 namespace rrobot {

    // Left stick X and Y axis
    #define CTRL_AXIS_XL 0
    #define CTRL_AXIS_YL 1

    // Right stick X and Y axis
    #define CTRL_AXIS_XR 2
    #define CTRL_AXIS_YR 2

    // buttons
    #define CTRL_X_BUTTON 0
    #define CTRL_SCROLL_DOWN 11
    #define CTRL_SCROLL_UP 12
    #define CTRL_SCROLL_LEFT 13
    #define CTRL_SCROLL_RIGHT 14


    // I could not find any frame_id standard for joysticks,  this may be because they do
    // not reflect, spacial relations or coordinate systems in the traditional sense,  but 
    // it urks me not to have one,  so have named it joy-ps4
    #define FRAME_ID_JOY_PS4 "joy-ps4"
    #define TOPIC_JOY  "/joy"
 }

#endif