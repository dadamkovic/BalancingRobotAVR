#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <avr/interrupt.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "drawingFunctions.h"
#include "communicationFunctions.h"
#include "NewBitmap.h"



/*******************************************************************************************/
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
SoftwareSerial BTSerial(6, 7);
/*******************************************************************************************/

volatile uint8_t chargeLvl = 100;
unsigned long timer;
volatile uint8_t state = 0x00;      //remove after testing is done
uint8_t oldState = 0xff;            //used to make sure we only redraw display when we have to


/*******************************************************************************************/

void setup(void) {
  Serial.begin(57600); 
  BTSerial.begin(57600);
  BTSerial.setTimeout(600);
  Serial.setTimeout(600);
  tft.setTextSize(2);
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setRotation(1);          //turns display

  timer = millis();
  tft.fillScreen(ST77XX_BLACK);
  timer = millis() - timer;
  showUnizaLogo(500);
  //tft.print(map(constrain(analogRead(JOYPAD_X)-170,0,1000),0,911,1,100));
  delay(500);
  pinMode(2, INPUT_PULLUP);     //MENU
  pinMode(3, INPUT_PULLUP);     //SEL
  attachInterrupt(digitalPinToInterrupt(3), MENU, FALLING);
  state = 0x20;
}








void loop() {
  float timerB = millis();
  if(((millis()-timer) > 10000) && (state != 0x31)&& (state != 0x21)){  //kazdu minutu update baterie

    timer = millis();
      chargeLvl = requestBatteryUpdate();
      tft.fillRect(122, 2, 26, 11, BLACK);
      drawBattery(chargeLvl);
  }
  
  static uint8_t rMove, lMove, moveToken = 0;
  static int8_t stopped=1;
  
  if(state != DEFAULT_STATE)resolveMenuInput(&rMove,&lMove,&moveToken);
  

/*******************************************************************************************/
/*******************************************************************************************/
/*******************************************************************************************/

  switch(state){
    case 0x00:      //deafult state, control of the robot enabled
    { 
      if(state != oldState){
        tft.fillScreen(ST77XX_BLACK);   
        drawMenuScreen(STEERING, "RIADENIE", 0);
        tft.fillRect(0, 80, 160, 128, ST77XX_BLACK);                       
        oldState = state;
      }   
      
      handleControl(30);                  //function that manages sending data and reading joypad
      break;
    }                               
/*******************************************************************************************/
/*******************************************************************************************/    
    case 0x20:        //calibration menu screen
    {
      if(state != oldState){
        tft.print(chargeLvl);
        tft.fillScreen(ST77XX_BLACK);
        drawBattery(chargeLvl);
        drawMenuScreen(CALIBRATION_ICON, "KALIBRACIA", 0);
        oldState = state;                                                    
      }
      if(rMove){            //neccessary to zero out used up move token
        rMove = 0;
        state=0x30;
      }
      else if(lMove){       //neccessary to zero out used up move token
        lMove = 0;
        state=0x40;
      }   
      else if(SEL_BUTTON)state=0x21;      
      break;
    }

/*******************************************************************************************/
   /*draws the progress bar, once the calibration is complete reqests robot position change so it could start balancing 
    *if error occurs return to state 0x21
    */
/*******************************************************************************************/
   case 0x21:
   {
      if(oldState!=state){
        tft.fillScreen(BLACK);
        oldState = state;
        drawBattery(chargeLvl);
      }
      
      float tiltAngle;
      tiltAngle = getRobotAngle();
      Serial.println(tiltAngle);
      if(tiltAngle == -10000){
        tft.fillScreen(ST77XX_BLACK);
        drawCalibrationScreen(ERROR_ICON, "SPOJENIE NEUSPESNE",RED);
        delay(2000);
        state = 0x20;
        break;
      }
      
      else if((tiltAngle < -70.0) || (tiltAngle > 70.0)){
        initMPUCalibration();          //bude treba chvilku pockat kym ho naozaj operator pusti z ruky
        tft.fillScreen(BLACK);
        BTSerial.flush();
        char tmp;
        uint8_t success = 0;
        for(uint8_t i=0;i<150;i++){
          tmp = BTSerial.read();
          Serial.println(tmp);
          delay(100);
          if(tmp == '\n'){
            success = 1;
            tft.fillScreen(BLACK);
            drawBattery(chargeLvl);
            drawCalibrationScreen(SUCCESS_ICON, "PO PIPNUTI UVEDTE ROBOTA DO VZPRIAMENEJ POLOHY",GREEN);
            delay(1000);
            state=0x20;         
            break;    //ak prijmeme potvrdenie od robota netreba cakat
          }
        
          //drawWaitCircle(i,WHITE);
          drawWaitPolygon(i,150,1);
          //delay(CALIBRATION_STEP_DELAY);    //variable
        }
        
        if(success == 0){        //in case the calibration was not successful
          tft.fillScreen(ST77XX_BLACK);
          drawBattery(chargeLvl);
          drawCalibrationScreen(ERROR_ICON, "KALIBRACIA NEUSPESNA",RED);
          delay(2000);
          state = 0x20;
          }
           
      }
      
      else {
        drawCalibrationScreen(LAY_FLAT_ICON, "UVEDTE ROBOTA DO POLOHY\nLEZMO!",RED,WHITE);
        delay(1000);
        oldState = state;
      }
      
      break;
  }

/*******************************************************************************************/
/*******************************************************************************************/        
      case 0x30:        //draws realtime tilt and speed of the robot, control of robot possible
      {
        if(state != oldState){
          tft.fillScreen(ST77XX_BLACK);
          drawBattery(chargeLvl);
          drawMenuScreen(SPEED_TILT_ICON, "INFO", 0);
          
          oldState = state; 
        }
        if(rMove){            //neccessary to zero out used up move token
          rMove = 0;
          state=0x40;
        }
        else if(lMove){       //neccessary to zero out used up move token
          lMove = 0;
          state=0x20;
        }   
        else if(SEL_BUTTON)state=0x31;      
      break; 
      }   
/*******************************************************************************************/
/*******************************************************************************************/
      case 0x31:        //realization of 0x30
      {
        oldState = state;
        float robotSpeed;
        float robotDistance;
        tft.fillScreen(BLACK);
        drawBattery(chargeLvl);
        
        robotDistance = getRobotDistance();
        Serial.println(robotDistance);
        robotSpeed = getRobotSpeed();   
        Serial.println(robotSpeed);
        if((robotSpeed == -10000)){
          tft.fillScreen(ST77XX_BLACK);
          
          drawBattery(chargeLvl);
          drawCalibrationScreen(ERROR_ICON, "SPOJENIE NEUSPESNE",RED);
          delay(2000);
          state = 0x30;
          break;
        }
        else{
          tft.fillScreen(BLACK);
          tft.setCursor(0, 15);
          tft.setTextSize(2);
          char buff1[10];
          char buff2[10];
          tft.println("Rychlost:");
          dtostrf(robotDistance,2,2,buff2);
          tft.print(buff2);
          tft.println("[rad/s]");
          tft.println("------------");
          dtostrf(robotSpeed,2,2,buff1);
          tft.println("Vzdialenost:");
          tft.print(buff1);
          tft.println("[m]");
          tft.println("------------");
          
          delay(500);
        }
        break;  
      }
      case 0x40:
      {
        if(state != oldState){
          tft.fillScreen(ST77XX_BLACK);
          drawBattery(chargeLvl);
          if(stopped>0)drawMenuScreen(STOP_ON_ICON, "SLED. POLOHY", 0);
          else drawMenuScreen(STOP_OFF_ICON, "SLED. POLOHY", 0);
          oldState = state; 
        }
        if(rMove){            //neccessary to zero out used up move token
          rMove = 0;
          state=0x20;
        }
        else if(lMove){       //neccessary to zero out used up move token
          lMove = 0;
          state=0x30;
        }   
      else if(SEL_BUTTON){
        stopped*=-1;
        sendRobotCommand(TOGGLE_STOP);
        tft.fillScreen(ST77XX_BLACK);
        if(stopped>0)drawMenuScreen(STOP_ON_ICON, "SLED. POLOHY", 0);
        else drawMenuScreen(STOP_OFF_ICON, "SLED. POLOHY", 0);      
      }
      break; 
      }
      case 0x60:
        float tmp;
        while(1){
          //while(BTSerial.available()==0){};
          tmp = BTSerial.parseFloat();
          if(tmp == '\0')break;
          Serial.print(tmp);
        }
        Serial.print('\n');
     }
}

    
  

void MENU(){
  if(state == 0x00){
    _delay_ms(1);
    if(MENU_BUTTON){
      state = 0x20;
    }
  }
  else{
    _delay_ms(1);
    if(MENU_BUTTON){
      state = 0x00;
    }
  }
  
}
