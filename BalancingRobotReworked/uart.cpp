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



    UCSR3C &= ~(_BV(UMSEL30) | _BV(UMSEL31));       //asynchronous mode
    UCSR3C &= ~(_BV(UPM30) | _BV(UPM31));           //no parity mode
    UCSR3C &= ~(_BV(USBS3));                      //1 stop-bit
    UCSR3C |= (_BV(UCSZ31) | _BV(UCSZ30));           //8-bits of data
    UCSR3B &= ~(_BV(UCSZ32));
    #if USE_INTERRUPT
    UCSR3B |= _BV(RXCIE3);                  //enables interrupt on receive
    UCSR3B |= _BV(TXCIE3);                  //enables interrupt of transfer complete
    #endif
    DDRJ |= _BV(PJ1);                       //PJ1(TX3) as output
    DDRJ &= ~(_BV(PJ0));                    //PJ0(RX3) as input
    UCSR3B |= _BV(RXEN3) | _BV(TXEN3);   //enables receiver and transmitter
}


/*
 * TODO : Check whether it won't be necessary to clear some flags after the interrupt
 */

ISR(USART3_RX_vect){
    char receivedByte = UDR3;
    if(!receiveCommandFlag){
        receiveCommandFlag = 1;                         //declared in main.h, set to 1 if we are currently reading command from controller
        for(int i=0;i<3;i++)controllerData[i] = 0;      //clears previous data from controller
        controllerData[0] = receivedByte;
    }
    else{
        if(receivedByte == '\n'){
            controllerData[2]='\0';     //no data signals end of the read
            receiveCommandFlag = 0;     //clearing the flag, no longer receiving command
        }
        else{
            controllerData[1] = receivedByte;          //if there is data and we didn't just get started reading it means we are at 2nd byte
        }
    }




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
    UCSR0C |= (_BV(UCSZ01) | _BV(UCSZ00));           //8-bits of data
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

void interfaceReceiveString(char* buffer){
    uint8_t i=0;
    do{
    loop_until_bit_is_set(UCSR0A,RXC0);
    buffer[i] = UDR0;
    i++;
    }while((buffer[i-1]!='\n') & (i < 255));
}

void interfaceSendChar(uint8_t ch){
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = ch;
}

void interfaceSendString(const char string2send[]){
    uint8_t i=0;
    while(string2send[i]){
        interfaceSendChar(string2send[i]);
        i++;
    }
}

void interfaceSendFloat(float float2send){
    char output[10];
    dtostrf(float2send, 5, 4, output);
    interfaceSendString(output);
}



