#include "timeTracking.h"

void clockInit(){
    TCCR1A &= ~(_BV(WGM10) | _BV(WGM11) | _BV(WGM12) | _BV(WGM13));    //normal mode

}

void clockStart(){
    TCCR1B |= _BV(CS11);                                            //prescaler == 8
}

double clockTime(){
    return TCNT1/65535*0.065535;
}

void clockReset(){
    TCNT1 = 0;
}

void clockShutdown(){
    TCCR1B &= ~(_BV(CS11));
}
