/*
 * utility.cpp
 *
 *  Created on: Feb 21, 2019
 *      Author: daniel
 */
#include "utility.h"


float constrain(float x, float minValue, float maxValue){
    if(x < minValue)return minValue;
    else if(x > maxValue)return maxValue;
    else return x;
}

float map(float num2map, float botInit, float topInit, float mapLow, float mapHigh){
    float range = (topInit - botInit);
    float newRange = mapHigh - mapLow;

    return (mapLow + ((num2map-botInit)/range)*newRange);
}



void FILOBuffer::add(int32_t item){
    if(buffer_head<10){
        buffer_items[buffer_head++] = item;
    }
    else{
        buff_filled = 1;
        buffer_head = 0;
        buffer_items[buffer_head++] = item;
    }
};
int32_t FILOBuffer::pop(){
    if(buff_filled){
        if(buffer_head<10){
            return buffer_items[buffer_head];
        }
        else{
            return buffer_items[0];
        }
    }
    return buffer_items[0];
};

uint8_t FILOBuffer::filled(){
    return buff_filled;
};
