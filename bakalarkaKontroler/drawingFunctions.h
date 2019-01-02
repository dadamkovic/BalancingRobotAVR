#ifndef DRAWINGFUNCTIONS_H
#define DRAWINGFUNCTIONS_H
#include "inttypes.h"
#include "constants.h"
void drawBattery(uint8_t charge);
void drawText(char *text, uint16_t color, uint16_t x=0, uint16_t y=0);
void drawMenuArrows(uint8_t);
void drawWaitPolygon(uint16_t stripeNum,uint16_t numSegments,uint8_t spaces=0,uint16_t color=BLUE);
void showMeSomethingBeutiful(uint16_t);
void showUnizaLogo(uint16_t);
void drawIcon(const uint8_t*, uint16_t);
void drawCalibrationSreen(const uint8_t*,char*,uint16_t bitmapColor=ST77XX_GREEN,uint16_t textColor=ST77XX_WHITE);
void resolveMenuInput(uint8_t*,uint8_t*,uint8_t*);
void drawMenuScreen(const uint8_t* ,char* , uint16_t);
void drawWaitCircle(uint8_t,uint16_t);
void drawPendulum(float,int16_t);




#endif
