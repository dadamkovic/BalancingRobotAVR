/*
 * uart.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */


#include "uart.h"
#define USE_2X 0        //twice the speed is unnecessary

/*
 * initUART : sets baudrate, no parity,8-bit data, 1 stop-bit, asynchronous mode and enables RX, TX pins
 * TODO : Check taht USE_2X thing
*/
void initControllerUART(){
    UBRR3H = UBRRH_VALUE;
    UBRR3L = UBRRL_VALUE;
    #if USE_2X
    UCSR3A |= (1 << U2X3);
    #else
    UCSR3A &= ~(1 << U2X3);
    #endif


/*
    UCSR3C &= ~((_BV(UMSEL30) | _BV(UMSEL31)));       //asynchronous mode
    UCSR3C &= ~(_BV(UPM30) | _BV(UPM31));           //no parity mode
    UCSR3C &= ~(_BV(USBS3));                      //1 stop-bit
    UCSR3C |= (_BV(UCSZ31) | _BV(UCSZ30));           //8-bits of data
    UCSR3B &= ~(_BV(UCSZ32));
    */
    UCSR3C &= ~(_BV(UMSEL30) | _BV(UMSEL31));       //asynchronous mode
    UCSR3C &= ~(_BV(UPM30) | _BV(UPM31));           //no parity mode
    UCSR3C &= ~(_BV(USBS3));                      //1 stop-bit
    UCSR3C |= (_BV(UCSZ31) | _BV(UCSZ30));           //8-bits otaf da
    UCSR3B &= ~(_BV(UCSZ32));


    UCSR3B |= _BV(RXEN3) | _BV(TXEN3);   //enables receiver and transmitter

}


/*
 * TODO : Check whether it won't be necessary to clear some flags after the interrupt
 */
void initBLEModul(){
    //#if USE_INTERRUPT
    //UCSR3B |= _BV(RXCIE3);                  //enables interrupt on receive
    //UCSR3B |= _BV(TXCIE3);                  //enables interrupt of transfer complete
    //#endif
    DDRJ |= _BV(PJ1);                       //PJ1(TX3) as output
    DDRJ &= ~(_BV(PJ0));                    //PJ0(RX3) as input
    //UCSR3B |= _BV(RXEN3) | _BV(TXEN3);   //enables receiver and transmitter

    //initInterfaceUART();
    //char buffer[25] = {'\0'};
    /*controllerSendString("AT+UUIDFFE2\r\n");    //UUID
    //controllerReceiveString(buffer);
    //interfaceSendString(buffer);*/
    //_delay_ms(1000);
    //char buffer[25] = {'\0'};
    /*controllerSendString("AT+CHARFFE3\r\n");
    //controllerReceiveString(buffer);
      //  interfaceSendString(buffer);
    _delay_ms(1000);*/
    //buffer[25] = {'\0'};
    /*initInterfaceUART();
    controllerSendString("AT\r\n");
    controllerSendString("AT+PIN000000\r\n");
    _delay_ms(5000);
    controllerSendString("AT+ROLE1\r\n");   //central
    _delay_ms(5000);
    //controllerReceiveString(buffer);
    //interfaceSendString(buffer);
    //buffer[25] = {'\0'};
    controllerSendString("AT+DISC?\r\n");
        //controllerReceiveString(buffer);
            //interfaceSendString("ADDRESSS IS:\n");
            //interfaceSendString(buffer);
        //buffer[25] = {'\0'};
        controllerSendString("AT+CONN0\r\n");
                //controllerReceiveString(buffer);
                    //interfaceSendString(buffer);
    //controllerSendString("AT+CON \r\n");*/

    /*controllerSendString("AT+IMME1\r\n");
    _delay_ms(1000);
    controllerSendString("AT+IMME0\r\n");
        _delay_ms(1000);
    controllerSendString("AT+RESET\r\n");*/

     controllerSendString("AT+RENEW\r\n");
     _delay_ms(1000);
     controllerSendString("AT+RESET\r\n");
          _delay_ms(1000);
          controllerSendString("AT+NAME=Controller\r\n");
          _delay_ms(1000);
          controllerSendString("AT+ROLE1\r\n");
          _delay_ms(1000);
        //controllerSendString("AT+INQ\r\n");
        //controlllerSendString("AT+ROLE0\n");
        //_delay_ms(1000);
        //controllerSendString("AT+CONN1\r\n");
        //_delay_ms(1000);
        controllerSendString("AT+TYPE1");
                _delay_ms(1000);
        //controllerSendString(buffer);
/*
        controllerSendString("AT+CHARFFE3\r\n");
        _delay_ms(1000);
        controllerSendString("AT+NAMEbalancingRobot\r\n");
        _delay_ms(1000);*/
        //controllerSendString("AT+CON8FA288F77C0F\n\r");
        //_delay_ms(1000);
        //controllerSendString("AT+CON8FA288F77C0F\n\r");

        _delay_ms(1000);
}

ISR(USART3_RX_vect){
    char receivedByte = UDR3;
    if(!receivingCommandFlag){
        for(int i=0;i<3;i++)controllerData[i] = 0;      //clears previous data from controller
        receivingCommandFlag++;                         //declared in main.h, set to 1 if we are currently reading command from controller and if so which
        controllerData[0] = receivedByte;
    }
    else{
        controllerData[receivingCommandFlag]=receivedByte;     //no data signals end of the read
        if(receivingCommandFlag == 2)receivingCommandFlag = 0;     //clearing the flag, no longer receiving command
        else receivingCommandFlag++;
    }
    initInterfaceUART();
    interfaceSendString("SOM TU");
}





//***********************************************************************************************************
/*
 * initUART : sets baudrate, no parity,8-bit data, 1 stop-bit, asynchronous mode and enables RX, TX pins
 * TODO : Check taht USE_2X thing
*/
void initInterfaceUART(){
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    #if USE_2X
    UCSR0A |= (1 << U2X0);
    #else
    UCSR0A &= ~(1 << U2X0);
    #endif



    UCSR0C &= ~(_BV(UMSEL00) | _BV(UMSEL01));       //asynchronous mode
    UCSR0C &= ~(_BV(UPM00) | _BV(UPM01));           //no parity mode
    UCSR0C &= ~(_BV(USBS0));                      //1 stop-bit
    UCSR0C |= (_BV(UCSZ01) | _BV(UCSZ00));           //8-bits otaf da
    UCSR0B &= ~(_BV(UCSZ02));


    UCSR0B |= _BV(RXEN0) | _BV(TXEN0);   //enables receiver and transmitter


}

/*
 *avaliableChar : returns true when there is something ready to be read in buffer
 *note : this isn't necessary as program will be handling received data though interrupts
 *but it might come in handy later when we need communication with RC
*/
bool interfaceAvaliableSerial(){
    if(UCSR0A & _BV(RXC0)){       //specifies that UART buffer s filled but hasn't been read yet
        return true;
    };
    return false;
};


/*
 * receiveChar : returns the contents of UDR register
 * ! has to be tested by avaliableSerial() from code !
 * will probably never use this
 * TODO : remove later if not necessary
 */


char interfaceReceiveChar(){
    while ( !(UCSR0A & (1<<RXC0)) );
    //while(!avaliableSerial());
    return  UDR0;
}

char controllerReceiveChar(){
    while ( !(UCSR3A & (1<<RXC3)) );
    //while(!avaliableSerial());
    return  UDR3;
}

/*
 * ***********************
 */
/*
void interfaceReceiveString(char buffer[]){
    uint8_t i=0;
    do{
    loop_until_bit_is_set(UCSR0A,RXC0);
    buffer[i] = UDR0;
    i++;
    }while((buffer[i-1]!='\n') & (i < 255));
}*/
void interfaceReceiveString(char myString[]) {
  char response;
  uint8_t i;
  i = 0;
  while (i < (255 - 1)) {                   /* prevent over-runs */
    response = interfaceReceiveChar();
    interfaceSendChar(response);                                    /* echo */
    if (response == '\r') {                     /* enter marks the end */
      break;
    }
    else {
      myString[i] = response;                       /* add in a letter */
      i++;
    }
  }
  myString[i] = 0;                          /* terminal NULL character */
}

void controllerReceiveString(char  buffer[]){
    uint8_t i=0;
    do{
    loop_until_bit_is_set(UCSR3A,RXC3);
    buffer[i] = UDR3;
    i++;
    }while((buffer[i-1]!='\n') & (i < 255));
}
/*
 * *********************
 */
void interfaceSendChar(uint8_t ch){
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = ch;
}

void controllerSendChar(uint8_t ch){
    loop_until_bit_is_set(UCSR3A, UDRE3);
    UDR3 = ch;
}
/*
 * **********************
 */

void interfaceSendString(const char string2send[]){
    uint8_t i=0;
    while(string2send[i]){
        interfaceSendChar(string2send[i]);
        i++;
    }
}

void controllerSendString(const char string2send[]){
    uint8_t i=0;
    while(string2send[i]){
        controllerSendChar(string2send[i]);
        i++;
    }
}
/*
 * ***********************
 */

void interfaceSendFloat(float float2send){
    char output[10];
    dtostrf(float2send, 5, 4, output);
    interfaceSendString(output);
}



