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
    float range = topInit - botInit;
    float newRange = mapHigh - mapLow;
    return (mapLow + ((num2map-botInit)/range)*newRange);
}

