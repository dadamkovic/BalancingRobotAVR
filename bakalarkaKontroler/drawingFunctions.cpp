#include "drawingFunctions.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <avr/interrupt.h>

#include "NewBitmap.h"
#include <SPI.h>
#include "communicationFunctions.h"
extern Adafruit_ST7735 tft;
void drawBattery(uint8_t charge){
  tft.drawRect(120, 0, 30, 15, ST77XX_WHITE);
  tft.fillRect(115,5,5,5,ST77XX_WHITE);        //small battery tip
  if(charge > 80)tft.fillRect(122, 2, 26, 11, ST77XX_GREEN);   //removed 2 bits from all direction to keep outline visible
  else if(charge > 60)tft.fillRect(122, 2, 15, 11, ST77XX_GREEN);
  else if(charge > 40)tft.fillRect(122, 2, 5, 11, ST77XX_GREEN);
  else tft.fillRect(122, 2, 26, 11, ST77XX_RED);
}


void drawText(char *text, uint16_t color, uint16_t x=0, uint16_t y=0) {
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}


void drawMenuArrows(uint8_t flash){
  if(flash==0){
    tft.fillCircle(120,100,20,ST77XX_RED);
    tft.fillTriangle(115,85,115,115,135,100,ST77XX_WHITE); //bottom right arrow
    tft.fillCircle(40,100,20,ST77XX_RED);
    tft.fillTriangle(45,85,45,115,25,100,ST77XX_WHITE); //bottom left arrow
  }
  else{
    tft.fillCircle(120,100,20,ST77XX_WHITE);
    tft.fillTriangle(115,85,115,115,135,100,ST77XX_RED); //bottom right arrow
    tft.fillCircle(40,100,20,ST77XX_WHITE);
    tft.fillTriangle(45,85,45,115,25,100,ST77XX_RED); //bottom left arrow
    delay(200);
    };
}

void drawWaitPolygon(uint16_t stripeNum,uint16_t numSegments,uint8_t spaces=0,uint16_t color=ST77XX_BLUE){
  float angle = PI/2 + stripeNum*PI/((numSegments)/2.0);
  uint8_t xAxisA = 80-50*cos(angle);
  uint8_t yAxisA = 64-50*sin(angle);
  uint8_t xAxisB = 80-50*cos(angle+PI/((numSegments)/2.0));
  uint8_t yAxisB = 64-50*sin(angle+PI/((numSegments)/2.0));
  if(spaces & (stripeNum %2 == 0))tft.fillTriangle(80,64,xAxisA,yAxisA,xAxisB,yAxisB,ST77XX_BLACK); 
  else tft.fillTriangle(80,64,xAxisA,yAxisA,xAxisB,yAxisB,color); 
}

void showMeSomethingBeutiful(uint16_t delayTime){
  tft.fillScreen(ST77XX_BLACK);
  tft.drawBitmap(0,0,DANIEL,160,128,ST77XX_WHITE);//displays logo
  delay(delayTime);
}

void showUnizaLogo(uint16_t delayTime){
  tft.fillScreen(ST77XX_BLACK);
  tft.drawBitmap(0,0,UNIZA_LOGO,160,128,ST77XX_WHITE);
  delay(delayTime);
}


/*
 * function assumes BLACK background when it is called
 */
void drawMenuScreen(const uint8_t* bitmap,char* text, uint16_t delayTime){
  uint8_t textLength = 0;
  while(text[textLength]){
    textLength++;
  }
  uint8_t textStart = (DISPLAY_WIDTH - (textLength*CHAR_WIDTH + (textLength-1)*CHAR_SPACE))/2;
  tft.fillRect(50,0,60,60,ST77XX_WHITE);
  tft.drawBitmap(50,0,bitmap,60,60,ST77XX_BLACK);//displays logo
  drawText(text,ST77XX_WHITE,textStart,60);
  drawMenuArrows(0);
  delay(delayTime);
}

void drawCalibrationSreen(const uint8_t* bitmap,char* text,uint16_t bitmapColor=ST77XX_GREEN,uint16_t textColor=ST77XX_WHITE){
  tft.fillRect(50,0,60,60,ST77XX_WHITE);
  tft.drawBitmap(50,0,bitmap,60,60,bitmapColor);//displays logo
  tft.drawLine(0,70,160,70,ST77XX_WHITE);
  tft.setTextSize(1);
  drawText(text,textColor,0,75);
  tft.setTextSize(2);
}


void drawPendulum(float angle,int16_t robotSpeed){
  angle += PI/2;
  tft.fillRect(0,20,160,108,ST77XX_BLACK); //premaze to co bolo pod tym
  tft.drawLine(80,90,80+70*cos(angle),90-70*sin(angle),ST77XX_YELLOW);
  tft.fillCircle(80,90,10,ST77XX_BLUE);
  char text[7];
  int16_t angleDeg = (angle-PI/2)*RAD_TO_DEG*100;
  num2Str(angleDeg,text);
  drawText("ANGLE:",ST77XX_WHITE,40,110);
  drawText(text,ST77XX_WHITE,80,110);
  num2Str(robotSpeed,text);
  if(robotSpeed > 0){
    drawText("<--",ST77XX_WHITE,0,60);
    drawText(text,ST77XX_WHITE,30,60);
  }
  else{
    drawText(text,ST77XX_WHITE,100,60);
    drawText("-->",ST77XX_WHITE,140,60);
  }
  
}

void drawWaitCircle(uint8_t section,uint16_t color){
  
  for(float angle =(PI/2);angle<(2*PI+PI/2);angle+=0.1){
    uint8_t xAxis = 80-50*cos(angle);
    uint8_t yAxis = 64-50*sin(angle);
    tft.drawLine(80,64,xAxis,yAxis,color);
    if(angle>(((2*PI)/8)*section+PI/2))break;
  }
}



void resolveMenuInput(uint8_t *rMove,uint8_t *lMove,uint8_t *moveToken){
  uint8_t joypadXPos = analogRead(JOYPAD_X);
  uint8_t joypadYPos = analogRead(JOYPAD_Y);
  if(((joypadXPos > UPPER_THRESHOLD))&&(moveToken))rMove = 1;
  else if(((joypadXPos < LOWER_THRESHOLD))&&(moveToken))lMove = 1;
  else {
    moveToken = 1;
    return;
  }
  moveToken = 0;    //if we moved we used up the move token
}

