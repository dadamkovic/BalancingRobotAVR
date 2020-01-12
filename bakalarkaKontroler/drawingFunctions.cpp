/**
 * \file drawingFunctions.cpp
 * \author Daniel Adamkovic
 * \brief Defintions of all funcions that have anyhting to do with draiwing on TFT.
 */


#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <avr/interrupt.h>

#include "NewBitmap.h"
#include <SPI.h>
#include "communicationFunctions.h"
#include "drawingFunctions.h"


extern Adafruit_ST7735 tft;       //tft defined in bakalarkaKontroler

/**
 * \addtogroup DRAWING_GROUP
 *
 * \{
 */
void drawBattery(uint8_t charge){
  tft.drawRect(120, 0, 30, 15, ST77XX_WHITE);
  tft.fillRect(115,5,5,5,ST77XX_WHITE);                           //small battery tip
  if(charge > 80)tft.fillRect(122, 2, 26, 11, ST77XX_GREEN);      //removed 2 bits from all direction to keep outline visible
  else if(charge > 60){
    tft.fillRect(133, 2, 15, 11, ST77XX_GREEN); //different levels of charge change the size and color of the infill
  }
  else if(charge > 30){   
    tft.fillRect(140, 2, 8, 11, ST77XX_GREEN);
  }
  else tft.fillRect(122, 2, 26, 11, ST77XX_RED);
}


void drawText(char *text, uint16_t color, uint16_t x=0, uint16_t y=0) {
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextWrap(true);        //text will continue to the next line if we run out of screen
  tft.print(text);
}


void drawMenuArrows(uint8_t invertColors){
  
  if(invertColors==0){
    tft.fillCircle(120,100,20,ST77XX_RED);
    tft.fillTriangle(115,85,115,115,135,100,ST77XX_WHITE); //bottom right arrow
    tft.fillCircle(40,100,20,ST77XX_RED);
    tft.fillTriangle(45,85,45,115,25,100,ST77XX_WHITE); //bottom left arrow
  }
  //inverts the color of the arrows
  else{
    tft.fillCircle(120,100,20,ST77XX_WHITE);
    tft.fillTriangle(115,85,115,115,135,100,ST77XX_RED); //bottom right arrow
    tft.fillCircle(40,100,20,ST77XX_WHITE);
    tft.fillTriangle(45,85,45,115,25,100,ST77XX_RED); //bottom left arrow
    delay(200);
    };
}

void drawWaitPolygon(uint16_t stripeNum,uint16_t numSegments,uint8_t spaces=0,uint16_t color=ST77XX_BLUE){ 
  float angle = PI/2 + stripeNum*PI/((numSegments)/2.0);      //sets the angle at which we begin drawing section of the polygon
  uint8_t xAxisA = 80-50*cos(angle);                          //see function detailed describtion
  uint8_t yAxisA = 64-50*sin(angle);        
  uint8_t xAxisB = 80-50*cos(angle+PI/((numSegments)/2.0));   //using the same formula asa above, but the angle is shifted to give a second point 
  uint8_t yAxisB = 64-50*sin(angle+PI/((numSegments)/2.0));
  if(spaces & (stripeNum %2 == 0))tft.fillTriangle(80,64,xAxisA,yAxisA,xAxisB,yAxisB,ST77XX_BLACK);   //used whene we want to skip coloring certain parts
  else tft.fillTriangle(80,64,xAxisA,yAxisA,xAxisB,yAxisB,color);     //uses generated point and center point to draw a triangle filled with color
}

void showUnizaLogo(uint16_t delayTime){
  tft.fillScreen(ST77XX_BLACK);
  tft.drawBitmap(0,0,UNIZA_LOGO,160,128,ST77XX_WHITE);  //displays bitmap
  delay(delayTime);
}

void drawMenuScreen(const uint8_t* bitmap,char* text, uint16_t delayTime){
  uint8_t textLength = 0;       //will be used to store text length
  while(text[textLength]){      //finds the correct text length
    textLength++;
  }
  uint8_t textStart = (DISPLAY_WIDTH - (textLength*CHAR_WIDTH + (textLength-1)*CHAR_SPACE))/2;    //will make sure the text is always centered
  tft.fillRect(50,0,60,60,ST77XX_WHITE);                //image s nicer when its background is black and everything else white
  tft.drawBitmap(50,0,bitmap,60,60,ST77XX_BLACK);       //draws icon, inverted colors 
  drawText(text,ST77XX_WHITE,textStart,60);
  drawMenuArrows(0);
  delay(delayTime);
}

void drawCalibrationScreen(const uint8_t* bitmap,char* text,uint16_t bitmapColor=ST77XX_GREEN,uint16_t textColor=ST77XX_WHITE){
  tft.fillRect(50,0,60,60,ST77XX_WHITE);              //image s nicer when its background is black and everything else white
  tft.drawBitmap(50,0,bitmap,60,60,bitmapColor);      //draws icon, inverted colors
  tft.drawLine(0,70,160,70,ST77XX_WHITE);
  tft.setTextSize(1);                                 //decreases text size so that everything could fit on the screen
  drawText(text,textColor,0,75);
  tft.setTextSize(2);                                 //text back to normal size
}


void drawPendulum(float angle,int16_t robotSpeed){
  angle += PI/2;                            //the circle should begin to be draw from the top of the display
  tft.fillRect(0,20,160,108,ST77XX_BLACK);  //instead of redrawing entire screen we only redraw everything that changes
  tft.drawLine(80,90,80+70*cos(angle),90-70*sin(angle),ST77XX_YELLOW);    //corresponds to the angle of the body
  tft.fillCircle(80,90,10,ST77XX_BLUE);     //center point that represents the wheels        
  char text[7];
  int16_t angleDeg = (angle-PI/2)*RAD_TO_DEG*100;   //transforms the angle to degrees for better user experience
  num2Str(angleDeg,text);                           //angle to text
  drawText("ANGLE:",ST77XX_WHITE,40,110);
  drawText(text,ST77XX_WHITE,80,110);
  num2Str(robotSpeed,text);                         //speed to text
  if(robotSpeed > 0){                       //will either draw left or right pointing arrow, depending on the direction
    drawText("<--",ST77XX_WHITE,0,60);
    drawText(text,ST77XX_WHITE,30,60);      //speed next to the arrow
  }
  else{
    drawText(text,ST77XX_WHITE,100,60);     //speed next to the arrow
    drawText("-->",ST77XX_WHITE,140,60);
  }
  
}

void drawWaitCircle(uint8_t section,uint16_t color){
  for(float angle =(PI/2);angle<(2*PI+PI/2);angle+=0.1){      //angle changes by 0.1 rad 
    uint8_t xAxis = 80-50*cos(angle);
    uint8_t yAxis = 64-50*sin(angle);
    tft.drawLine(80,64,xAxis,yAxis,color);                    //draws line from center to the point on the circumference of the circle
    if(angle>(((2*PI)/8)*section+PI/2))break;                 //stops drawning if we reached the end of the section
  }
}

void resolveMenuInput(uint8_t *rMove,uint8_t *lMove,uint8_t *moveToken){
  int16_t joypadXPos = analogRead(JOYPAD_X);
  int16_t joypadYPos = analogRead(JOYPAD_Y);
  if(((joypadYPos > UPPER_THRESHOLD))&(*moveToken==1)){         //joypad got to the threshold from zero position
    *moveToken = 0;                                             //we have already performed one movement read
    *rMove = 1;
  }
  else if(((joypadYPos < LOWER_THRESHOLD))&(*moveToken==1)){    //same as above
    *moveToken = 0;
    *lMove = 1;
  }
  else if((joypadYPos > LOWER_THRESHOLD)&(joypadXPos < UPPER_THRESHOLD)){   //we can allow for the next movement to be registered if we have gotten to zero
    *moveToken = 1;
  }
  delay(50);      //no need to sample data that fast
}

/**
 * \}
 */
