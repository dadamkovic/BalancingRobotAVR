/***************************************************
  This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <avr/interrupt.h>
#include <SPI.h>
#include "drawingFunctions.h"
#include "communicationFunctions.h"
#include "NewBitmap.h"
//128X160




// For 1.44" and 1.8" TFT with ST7735 use
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);



volatile uint8_t chargeLvl = 100;
unsigned long timer;
volatile uint8_t state = 0x30;
uint8_t oldState = 0xff;

void setup(void) {
  Serial.begin(9600);
  tft.setTextSize(2);
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setRotation(1);          //turns display may need to be changed later

  timer = millis();
  tft.fillScreen(ST77XX_BLACK);
  timer = millis() - timer;
  showUnizaLogo(500);
  pinMode(2, INPUT_PULLUP);     //MENU
  pinMode(3, INPUT_PULLUP);     //SEL
  showMeSomethingBeutiful(60000);
}

void loop() {
  if((millis()-timer) > 60000){  //kazdu minutu update baterie
    chargeLvl = requestBatteryUpdate();         //TODO
    timer = millis();
  }
  static uint8_t rMove, lMove, moveToken = 0;
  attachInterrupt(digitalPinToInterrupt(2), MENU, LOW);
  if(state != DEFAULT_STATE)resolveMenuInput(&rMove,&lMove,&moveToken);
  drawBattery(chargeLvl);
  
  switch(state){
    case 0x00:   
    { 
      if(state != oldState){
        tft.fillScreen(ST77XX_BLACK);   
        tft.fillCircle(53, 80, 15, WHITE);
        drawText("MENU",WHITE,30, 100);
        tft.fillCircle(106,80, 15, RED);
        drawText("HONK",WHITE,83, 100);   //ak bude cas vymysliet a implementovat custom bitmap
      }
      oldState = state;
      handleControl(50);                  //function that manages sending data and reading joypad
      delay(1000);
      
      break;
    }
    case 0x20:
    {
      if(state != oldState){
        tft.fillScreen(ST77XX_BLACK);
        drawMenuScreen(CALIBRATION_ICON, "KALIBRACIA", 1000);
      }
      if(rMove){            //treba vynulovat pouzitie pohybu aby sme nepreleteli vsetky menu
        rMove = 0;
        state=0x30;
      }
      else if(lMove){
        lMove = 0;
        state=0x40;
      }
      oldState = state;
      break;
    }
   //kalibracne menu   
   case 0x21:
   {   
      if(state != oldState){    
        tft.fillScreen(ST77XX_BLACK);
        drawBattery(chargeLvl);
        drawText("UHOL",ST77XX_WHITE,10,50);        //left side
        drawText("MPU6050",ST77XX_WHITE,70,50);    //right side 
        drawMenuArrows(0);
      }
      if(rMove){
        rMove = 0;
        state=0x22;
      }
      else if(lMove){
        lMove = 0;
        state=0x23;
      }
      oldState = state;
      break;
   }
   //kresli progress bar, ak prijime od robota potvrdenie o dokonceni kalibracie tak vypise spravu a vyziada umiestnenie robota
   //do zvislej polohy - aby mohol znova zacat balansovat
   //bez potvrdenia prejde do predchadzajuceho stavu po vypisani chyby
   case 0x22:
   {
      tft.fillScreen(BLACK);
      float tiltAngle = getRobotAngle();
      if(tiltAngle < -80 || tiltAngle > 80){
        initMPUCalibration();          //bude treba chvilku pockat kym ho naozaj operator pusti z ruky
        for(uint8_t i=0;i<8;i++){
          drawWaitCircle(i,WHITE);
          delay(CALIBRATION_STEP_DELAY);    //moze sa zmenit
          if(Serial.available()){
            Serial.read();
            tft.fillScreen(ST77XX_BLACK);
            drawBattery(chargeLvl);
            drawCalibrationSreen(SUCCESS_ICON, "PRE UKONCENIE KALIBRACIE\nPODRZTE ROBOTA VO VZPRIA-\nMENEJ POLOHE DO PIPNUTIA",GREEN);
            while(!Serial.available()){}  //robot musi potvrdit vzpriamenu polohu
            Serial.read();
            state=0x21;         
            break;    //ak prijmeme potvrdenie od robota netreba cakat
          }
        }
        if(state==0x22){
          tft.fillScreen(ST77XX_BLACK);
          drawBattery(chargeLvl);
          tft.fillScreen(ST77XX_BLACK);
          drawBattery(chargeLvl);
          for(uint8_t i=0;i<4;i++){
            drawText("KALIBRACIA\nNEUSPESNA\n!!!!!!!!!!",ST77XX_RED,0,30);
            delay(500);
            drawText("KALIBRACIA\nNEUSPESNA\n!!!!!!!!!!",ST77XX_WHITE,0,30);
            delay(500);
          }
          state = 0x21;
        } 
      }
      else drawCalibrationSreen(LAY_FLAT_ICON, "UVEDTE ROBOTA DO POLOHY\nLEZMO!",RED,WHITE);
      break;
  }
    //silno podobne ako 0x22, nezlucil som ich z dovodu prehladnosti
    //robot po zacai kalibracie balansuje a vytvori si priemer z nameranych hodnot z ktorych urci kal. uhol  
    case 0x23:
    {
      tft.fillScreen(BLACK);
      initAngleCalibration();
      for(uint16_t i=0;i<16;i++){
          drawWaitPolygon(i+1,16);
          delay(CALIBRATION_STEP_DELAY);    //moze sa zmenit
          if(Serial.available()){
            Serial.read();
            tft.fillScreen(ST77XX_BLACK);
            drawBattery(chargeLvl);
            drawCalibrationSreen(SUCCESS_ICON, "KALIBRACIA USPESNA!",GREEN,GREEN);
            delay(2000);
            state=0x21;
            break;    //ak prijmeme potvrdenie od robota netreba cakat
          }
        }
        if(state==0x23){
          tft.fillScreen(BLACK);
          drawBattery(chargeLvl);
          drawCalibrationSreen(SUCCESS_ICON, "KALIBRACIA NEUSPESNA!",RED,RED);
          delay(500);
          drawCalibrationSreen(SUCCESS_ICON, "KALIBRACIA NEUSPESNA!",RED,WHITE);
          delay(500);
          drawCalibrationSreen(SUCCESS_ICON, "KALIBRACIA NEUSPESNA!",RED,RED);
          delay(500);
          drawCalibrationSreen(SUCCESS_ICON, "KALIBRACIA NEUSPESNA!",RED,WHITE);
          state = 0x21;
        }
        break;
        }
      //vykresluje polohu a rychlost robota
      case 0x30:
      {
        int16_t robotSpeed,robotAngle;
        tft.fillScreen(BLACK);
        drawBattery(chargeLvl);
        robotSpeed = getRobotSpeed();   //will just receive int16
        robotAngle = getRobotAngle();        
        drawPendulum(robotAngle,robotSpeed);
        handleControl(50);
        //zo stavu sa prejde pri preruseni
        break;  
      }   
      case 0x40:
      {
        tft.fillScreen(BLACK);
        drawMenuScreen(TUNING_ICON, "PID TUNING", 500);
        if(rMove){            //treba vynulovat pouzitie pohybu aby sme nepreleteli vsetky menu
          rMove = 0;
          state=0x41;
        }
        else if(lMove){
          lMove = 0;
          state=0x42;
        }
        break;
      }
        //vstup do P menu
      case 0x41:
      {
        if(state != oldState){
          tft.fillScreen(BLACK);
          drawMenuArrows(0);
          tft.setTextSize(3);
          drawText("P",WHITE,60,60);
        }
        tft.setTextSize(2);
        if(rMove){            //treba vynulovat pouzitie pohybu aby sme nepreleteli vsetky menu
          rMove = 0;
          state=0x43;
        }
        else if(lMove){
          lMove = 0;
          state=0x45;
        }
        oldState = state;
        break;
      }
        
        //vstup do I menu
     case 0x43:
     {
        if(state != oldState){
          tft.fillScreen(BLACK);
          drawMenuArrows(0);
          tft.setTextSize(3);
          drawText("I",WHITE,60,60);
        }
        tft.setTextSize(2);
        if(rMove){            //treba vynulovat pouzitie pohybu aby sme nepreleteli vsetky menu
          rMove = 0;
          state=0x45;
        }
        else if(lMove){
          lMove = 0;
          state=0x41;
        }
        oldState = state;
        break;
     }
     case 0x45:
     {
        if(state != oldState){
          tft.fillScreen(BLACK);
          drawMenuArrows(0);
          tft.setTextSize(3);
          drawText("D",WHITE,60,60);
        }
        tft.setTextSize(2);
        if(rMove){            //treba vynulovat pouzitie pohybu aby sme nepreleteli vsetky menu
          rMove = 0;
          state=0x45;
        }
        else if(lMove){
          lMove = 0;
          state=0x41;
        }
        oldState = state;
        break;
     }
     //P tuning
     case 0x42:
     {
        tft.fillScreen(BLACK);
        while(!SEL_BUTTON & state==0x42){     
          uint16_t calP = getRobotRegulatorParameter('P');
          char text[7]; 
          tft.fillRect(0,0,100,20,BLACK);
          for(uint16_t i=0;i<(calP/50);i++)drawWaitPolygon(i,100,0,WHITE);
          tft.setTextSize(1);
          drawText("P KONSTANTA : ",WHITE);
          num2Str(calP,text);
          drawText(text,WHITE,80,0);
          tft.setTextSize(2);
          if((map(analogRead(JOYPAD_Y),0,255,0,100)>UPPER_THRESHOLD)&calP<5000){
            calP+=50;            
            sendRobotCommand(CHANGE_P,calP);
          }
          else if((map(analogRead(JOYPAD_Y),0,255,0,100)<LOWER_THRESHOLD)&calP>0){
            calP-=50;
            sendRobotCommand(CHANGE_P,calP);
          }
          delay(100);
        }
        break;
     }
     //I tuning
     case 0x44:
     {
        tft.fillScreen(BLACK);
        while(!SEL_BUTTON & state==0x44){
          uint16_t calI = getRobotRegulatorParameter('I');
          char text[7]; 
          tft.fillRect(0,0,100,20,BLACK);
          for(uint16_t i=0;i<calI;i++)drawWaitPolygon(i+1,100,0,RED);
          tft.setTextSize(1);
          drawText("I KONSTANTA : ",WHITE);
          
          num2Str(calI,text);
          drawText(text,WHITE,80,0);
          tft.setTextSize(2);
          if((map(analogRead(JOYPAD_Y),0,255,0,100)>UPPER_THRESHOLD)&calI<100){
            calI+=5;
            sendRobotCommand(CHANGE_I,calI);
          }
          else if((map(analogRead(JOYPAD_Y),0,255,0,100)<LOWER_THRESHOLD)&calI>0){
            calI-=5;
            sendRobotCommand(CHANGE_I,calI);
          }
          delay(100);
        }
        break;
     }
     //D tuning
     case 0x46:
     {
        tft.fillScreen(BLACK);
        while(!SEL_BUTTON & state==0x46){
          uint16_t calD = getRobotRegulatorParameter('D');
          char text[7]; 
          tft.fillRect(0,0,100,20,BLACK);
          for(uint16_t i=0;i<(calD/20);i++)drawWaitPolygon(i,100,0,GREEN);
          tft.setTextSize(1);
          drawText("D KONSTANTA : ",WHITE);    
          num2Str(calD,text);
          drawText(text,WHITE,80,0);
          tft.setTextSize(2);
          if((map(analogRead(JOYPAD_Y),0,255,0,100)>UPPER_THRESHOLD)&calD<2000){
            calD+=50;
            sendRobotCommand(CHANGE_D,calD);
          }
          else if((map(analogRead(JOYPAD_Y),0,255,0,100)<LOWER_THRESHOLD)&calD>0){
            calD-=50;
            sendRobotCommand(CHANGE_D,calD);
          }
          delay(100);
        }
        break;
     }
  }
}
    
  

void MENU(){
  if(state != 0x00)state=0x20;
  else state=0x00;
}




void mediabuttons() {
  // play
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRoundRect(25, 10, 78, 60, 8, ST77XX_WHITE);
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_RED);
  delay(500);
  // pause
  tft.fillRoundRect(25, 90, 78, 60, 8, ST77XX_WHITE);
  tft.fillRoundRect(39, 98, 20, 45, 5, ST77XX_GREEN);
  tft.fillRoundRect(69, 98, 20, 45, 5, ST77XX_GREEN);
  delay(500);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_BLUE);
  delay(50);
  // pause color
  tft.fillRoundRect(39, 98, 20, 45, 5, ST77XX_RED);
  tft.fillRoundRect(69, 98, 20, 45, 5, ST77XX_RED);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_GREEN);
}
