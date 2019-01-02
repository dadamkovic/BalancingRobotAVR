#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <Adafruit_ST7735.h>
#define TFT_CS     10
#define TFT_RST    9  // you can also connect this to the Arduino reset
                       // in which case, set this #define pin to -1!
#define TFT_DC     8

#define DEFAULT_STATE 0x00
#define DISPLAY_WIDTH 160
#define DISPLAY_HEIGHT 128
#define CHAR_WIDTH 10
#define CHAR_SPACE 2
#define CALIBRATION_STEP_DELAY 500
#define ALLOWED_WAIT_TIME 2000
#define RAD_TO_DEG 57.2957
#define UPPER_THRESHOLD 60
#define LOWER_THRESHOLD (100 - UPPER_THRESHOLD)

#define BLUE ST77XX_BLUE
#define RED ST77XX_RED
#define YELLOW ST77XX_YELLOW
#define GREEN ST77XX_GREEN
#define BLACK ST77XX_BLACK
#define WHITE ST77XX_WHITE

#define JOYPAD_X A0
#define JOYPAD_Y A1
#define SEL_BUTTON !digitalRead(3)
#define MENU_BUTTON digitalRead(2)

#define SEND_CONTROL_INFO 0x00
#define REQ_BATTERY_LVL 0x01
#define REQ_TILT_ANGLE 0x20
#define REQ_MPU_CALIBRATION 0x22
#define REQ_ANGLE_CALIBRATION 0x23
#define REQ_SPEED 0x30
#define REQ_REGULATOR_P 0x40
#define REQ_REGULATOR_I 0x41
#define REQ_REGULATOR_D 0x42
#define CHANGE_P 0x43
#define CHANGE_I 0x44
#define CHANGE_D 0x45




#endif
