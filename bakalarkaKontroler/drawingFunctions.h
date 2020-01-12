/** \file drawingFunctions.h
 *  \author Daniel Adamkovic
 *  \date 20/1/2019
 *  \brief Includes declarations of functions used for drawing on TFT display.
 */


#ifndef DRAWINGFUNCTIONS_H
#define DRAWINGFUNCTIONS_H
#include "inttypes.h"
#include "constants.h"




/**
 * \defgroup DRAWING_GROUP
 * \brief All functions that have something to do with drawing on TFT display.
 * \{
*/


/**
* \brief Draws a changing icon of battery cahrge lvl in upper right corner.
* \param charge From 0-100, determines how the battery is drawn.
*/
void drawBattery(uint8_t charge);

/** 
 *  \brief Extends the functionality of adafruit library and allows to position drawn text.
 *  \param text Text to be drawn.
 *  \param color In the form of 0x0000-0xffff determines the color of he text.
 *  \param x Pos in X axis (default 0).
 *  \param y Pos in Y axis (default 0).
*/
void drawText(char *text, uint16_t color, uint16_t x=0, uint16_t y=0);

/**
 * \brief Draws two circles with small arrows in them pointing to the left and right.
 * \param invertColors If set color scheme will slightly change.
 */
void drawMenuArrows(uint8_t invertColors);

/**
 * \brief Used for drawing "progress bar" during calibration routines. 
 * \param stripeNum Depending on this number we will be drawing the corrsponding part of the progress bar.
 * \param numSegments Total number of parts that prgoress bar will consist of.
 * \param spaces If set, every part of progress bar that is multiple of this number will be skipped.
 * \param color Color of the progress bar 0x0000-0xffff
 * \see drawWaitCircle
 * 
 * \code
 * for(uint16_t i=0;i<16;i++){
 *    drawWaitPolygon(i+1,16);
 *    oneStepOfSomeProcess();         
 * }
 * \endcode
 * 
 * Essencially aproximates the proces of drawing a circle, but instead of drawing lines from center to the 
 * rim, it finds two points on the outside border that are always a fixed distance apart and passes the set center
 * point along with these two points to the adafruit library finction for drawing triangles. User can chose the number of these traingles  
 * by setting the \p numSegments to this number. Triangles are added one after another and the function assumes that display
 * wont't be cleared in between the calls (polygons that were added are not redrawn over the calls. With large \p numSegments
 * the function draws a circle, but this use is dicouraged as it is comparably slower than the adafruit function. 
 */
void drawWaitPolygon(uint16_t stripeNum,uint16_t numSegments,uint8_t spaces=0,uint16_t color=BLUE);

/**
 * \brief Will draw picture of something of otherworldy beauty.
 * \param delayTime Determins how long the picture will be displayed, beforehanding control back to main loop.
 *
 */
void showMeSomethingBeutiful(uint16_t delayTime);

/**
 * \brief Displays logo of Zillinska Univerzita
 * \param delayTime Determins how long the picture will be displayed, beforehanding control back to main loop.
 */
void showUnizaLogo(uint16_t delayTime);

/**
 * \brief Draws screens for menu navigation. 
 * \param bitmap Bitmap of the picture stored in flash.
 * \param text Text to be displayed on the menu screen
 * \param delayTime Determins how long the picture will be displayed, beforehanding control back to main loop.
 * 
 * Draws 60x60  bitmap in the upper center of the screen, wrties text underneath it and draws arrows 
 * to the sides to let user know its possible to scroll left and right.
 * 
 * \note This function asssumes the background has been set to solid black before it was called.
 */
void drawMenuScreen(const uint8_t* bitmap,char* text, uint16_t delayTime);

/**
 * \brief Drawn during calibration wait time and to visualize some parameter setting routines.
 * \param section Sets how big part of the section circle should be drawn (0-8).
 * \param color Color of the drawn circle.
 * 
 *  \see drawWaitPolygon
 *  
 *  \code
 *  for(uint8_t i=0;i<8;i++){
          drawWaitCircle(i,WHITE);
          oneStepOfSomeProcess();
    }
 *  \endcode
 *  
 *  We obtain the coordinate in the X axis drom the formula CENTER_POINT_OF_CIRCLE_IN_X_AXIS-RADIUS_OF_THE_CIRCLE*cos(ANGLE)
 *  and for the Y axis CENTER_POINT_OF_CIRCLE_IN_Y_AXIS-RADIUS_OF_THE_CIRCLE*cos(ANGLE), where angle ranges from 0-2*PI. By 
 *  drawing lines while gradualy chanigng the \p angle we can draw any part of the circle. 
 *  
 *  \note In this function the circle is split into 8 consecutive sections.
 *  
 */
void drawWaitCircle(uint8_t section,uint16_t color);

/** 
 *  \brief Draws information screens during calibration routines.
 *  \param bitmap Bitmap of the picture stored in flash.
 *  \param text Text to be displayed on the menu screen.
 *  \param bitmapColor Sets the color of the drawn picure.
 *  \param textColor Sets the color of the drawn text.

 */
void drawCalibrationScreen(const uint8_t* bitmap,char* text,uint16_t bitmapColor=ST77XX_GREEN,uint16_t textColor=ST77XX_WHITE);

/**
 * \brief Samples user inputs and handles screen changing and action selection in the menu.
 * \param lMove Set if the user is moving to the left, otherwise 0.
 * \param rMove Set if the user is moving to the right, otherwise 0.
 * \param moveToken Tracks validity of the sampled data. 
 * 
 * If we only read joypad value and moved to the left or right after reaching a certain threshold, we would
 * skip over all the menus immediately. This function makes sure that we only switch menus once if the user
 * moves the joypad from the center to the side and then prevents any further movement until we return back to
 * the approximate midpoint. The sensitivity of the joypad is determined by the UPPER_THRESHOLD macro.
 */
void resolveMenuInput(uint8_t *rMove,uint8_t *lMove,uint8_t *moveToken);

/**
 * \brief Visulizes real time movements of the robot.
 * \param angle Tilt of the robot in radians.
 * \param robotSpeed Speed of the robot in the form of: real_robto_speed = robtoSpeed/100;
 */
void drawPendulum(float angle,int16_t robotSpeed);


//void drawIcon(const uint8_t*, uint16_t);
/**
 * \}
 */
#endif
