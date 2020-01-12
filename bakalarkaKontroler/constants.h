/**
 * \file constants.h
 * \author Daniel Adamkovic
 * \date 20/1/2019
 */


#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <Adafruit_ST7735.h>

/**
 * \defgroup CONSTANTS_AND_MACROS
 * \brief All constants and macros, used throughout the project.
 * \{
 */


/**CS pin of the TFT display.*/
#define TFT_CS     8
/**Reset pin of the display can be wired directly to Arduino (in that case set to -1).*/
#define TFT_RST    9                         
/**DC pin of the TFT display.*/
#define TFT_DC     10

/**State where will will enter switch clause.*/
#define DEFAULT_STATE 0x00
/**Set to represent the width in pexels of the used display.*/
#define DISPLAY_WIDTH 160
/**Set t orepresent the height in pixels of the used display.*/
#define DISPLAY_HEIGHT 128
/**Width of a single character with text size set to 2.*/
#define CHAR_WIDTH 10
/**Space between characetrs with text size set to 2.*/
#define CHAR_SPACE 2
/**Delay in ms when drawing progress bars.*/
#define CALIBRATION_STEP_DELAY 1300
/**Total time we can wait for response from the robot.*/
#define ALLOWED_WAIT_TIME 1000
/**Constant used when transforming from radians to degrees.*/
#define RAD_TO_DEG 57.2957
/**Set sensitivity for joypad movement in menus closer to 1024 higher the sensitivity on the edges.*/
#define UPPER_THRESHOLD 715
/**No need to change, updated automaticaly by changing UPPER_THRESHOLD.*/
#define LOWER_THRESHOLD (1024 - UPPER_THRESHOLD)



//Defines for the colors that the adafruit library understands.
#define BLUE ST77XX_BLUE
#define RED ST77XX_RED
#define YELLOW ST77XX_YELLOW
#define GREEN ST77XX_GREEN
#define BLACK ST77XX_BLACK
#define WHITE ST77XX_WHITE

//pins selected for interacting with peripherals
#define JOYPAD_X A0
#define JOYPAD_Y A1
#define SEL_BUTTON !digitalRead(2)
#define MENU_BUTTON !digitalRead(3)

//codenames of the various requests send to the robot
#define CONTROL_INFO 0xff
#define SEND_CONTROL_INFO 0x01
#define REQ_BATTERY_LVL 0x02
#define REQ_TILT_ANGLE 0x03
#define REQ_MPU_CALIBRATION 0x04
#define REQ_SPEED 0x06

#define TOGGLE_STOP 0x07
#define REQ_DISTANCE 0x08
#define REQ_CURRENT 0x09
#define DATA_TRANSFER 0xf0




/**
 * \}
 */
#endif
