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


/**
 * \brief Adds item to position head, increments head or wraps it around
 */
void FIFOBuffer::add(int32_t item){
    if(bufferHead<10){
        bufferItems[bufferHead++] = item;
    }
    else{
        buffFilled = 1;
        bufferHead = 0;
        bufferItems[bufferHead++] = item;
    }
};

/**
 * \brief Returns item from position head (oldest item)
 */
int32_t FIFOBuffer::pop(){
    if(buffFilled){
        if(bufferHead<10){
            return bufferItems[bufferHead];
        }
    }
    return bufferItems[0];
};

uint8_t FIFOBuffer::filled(){
    return buffFilled;
};
