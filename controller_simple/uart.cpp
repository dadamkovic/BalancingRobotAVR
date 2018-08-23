

#include "uart.h"
#define U2X 0

/*
 * initUART : sets baudrate, no parity,8-bit data, 1 stop-bit, asynchronous mode and enables RX, TX pins
*/
void initUART(){
    //UBRRH = UBRRH_VALUE;
    UBRRH = 0;              //setup for 9600 baudrate UBRRH_VALUE always wanted to use 2x mode
    UBRRL = 6;
    //UBRRL = UBRRL_VALUE;

    UCSRB |= _BV(RXEN) | _BV(TXEN);   //enables receiver and transmitter

    //UCSRC &= ~(_BV(UMSEL0) | _BV(UMSEL1));       //asynchronous mode
    //UCSRC &= ~(_BV(UPM0) | _BV(UPM1));           //no parity mode
    //UCSRC &= ~(_BV(USBS));                      //1 stop-bit
    UCSRC |= (_BV(UCSZ1) | _BV(UCSZ0));           //8-bits of data
    //UCSRC &= ~(_BV(UCSZ2));
}
/*
 *avaliableChar : returns true when there is something ready to be read in buffer
*/
bool avaliableSerial(){
    if(UCSRA & _BV(RXC)){       //specifies that UART buffer s filled but hasn't been read yet
        return true;
    };
    return false;
};
/*
 * receiveChar : returns the contents of UDR register
 * ! has to be tested by avaliableSerial() from code !
 */

char receiveChar(){
    while ( !(UCSRA & (1<<RXC)) );
    //while(!avaliableSerial());
    return  UDR;
}

void receiveString(char* buffer){
    uint8_t i=0;
    do{
    loop_until_bit_is_set(UCSRA,RXC);
    buffer[i] = UDR;
    i++;
    }while((buffer[i-1]!='\n') & (i < 255));
}

void sendChar(uint8_t ch){
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = ch;
}

void sendString(const char string2send[]){
    uint8_t i=0;
    while(string2send[i]){
        sendChar(string2send[i]);
        i++;
    }
}
