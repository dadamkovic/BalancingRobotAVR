/*
 * uart.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: daniel
 */


#include "uart.h"


/*
 * initUART : sets baudrate, no parity,8-bit data, 1 stop-bit, asynchronous mode and enables RX, TX pins
 * TODO : Check taht USE_2X thing
*/
void initUART(){
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    #if USE_2X
    UCSR0A |= (1 << U2X0);
    #else
    UCSR0A &= ~(1 << U2X0);
    #endif
    //UBRRH = 0;              //setup for 9600 baudrate UBRRH_VALUE always wanted to use 2x mode
    //UBRRL = 6;


    UCSR0B |= _BV(RXEN0) | _BV(TXEN0);   //enables receiver and transmitter

    UCSR0C &= ~(_BV(UMSEL0) | _BV(UMSEL1));       //asynchronous mode
    UCSR0C &= ~(_BV(UPM00) | _BV(UPM01));           //no parity mode
    UCSR0C &= ~(_BV(USBS0));                      //1 stop-bit
    UCSR0C |= (_BV(UCSZ01) | _BV(UCSZ00));           //8-bits of data
    UCSR0C &= ~(_BV(UCSZ02));
    #if USE_INTERRUPT
    UCSR0B |= _BV(TXCIE0);                  //enables interrupt on receive
    #endif
}
/*
 *avaliableChar : returns true when there is something ready to be read in buffer
*/
bool avaliableSerial(){
    if(UCSR0A & _BV(RXC0)){       //specifies that UART buffer s filled but hasn't been read yet
        return true;
    };
    return false;
};
/*
 * receiveChar : returns the contents of UDR register
 * ! has to be tested by avaliableSerial() from code !
 */

char receiveChar(){
    while ( !(UCSR0A & (1<<RXC0)) );
    //while(!avaliableSerial());
    return  UDR0;
}

void receiveString(char* buffer){
    uint8_t i=0;
    do{
    loop_until_bit_is_set(UCSR0A,RXC0);
    buffer[i] = UDR0;
    i++;
    }while((buffer[i-1]!='\n') & (i < 255));
}

void sendChar(uint8_t ch){
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = ch;
}

void sendString(const char string2send[]){
    uint8_t i=0;
    while(string2send[i]){
        sendChar(string2send[i]);
        i++;
    }
}

/*
 * TODO : Check whether it won't be necessary to clear some flags after the interrupt
 */

ISR(USART_RX_vect){
    char receivedByte = UDR0;
    if((tracking < 2) && (receivedByte != '\n')){
        direction[tracking] = receivedByte;
        directionReceived = 0;
        tracking++;
        }
    else{
        tracking = 0;
        directionReceived = 1;
    }



}


