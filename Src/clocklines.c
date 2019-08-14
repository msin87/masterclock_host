/*
 * clocklines.c
 *
 *  Created on: Aug 13, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#include "clocklines.h"
uint16_t clockLines_pulseWidth=1000;
uint8_t clockLines_isCountersEmpty=0;
LinesGPIO_TypeDef LinesGPIO={.Positive=GPIOE,.Negative=GPIOD};
void sendPulse(ClockLine* clockLines,int8_t* linesToPulse, uint8_t length, LinesGPIO_TypeDef* LinesGPIO){
    uint16_t outBufferPos=0, outBufferNeg=0;
    for (uint8_t i=0; i<length; i++){
        if (linesToPulse[i]<0) break;
        if (clockLines[linesToPulse[i]].polarity==POSITIVE)
        {
            outBufferPos|=1<<linesToPulse[i];
        }
        else
        {
            outBufferNeg|=1<<linesToPulse[i];
        }
    }
    LinesGPIO->Positive->ODR|=outBufferPos;
    LinesGPIO->Negative->ODR|=outBufferNeg;
}
void stopPulse(LinesGPIO_TypeDef* LinesGPIO)
{
    LinesGPIO->Positive->ODR&=0xF000;
    LinesGPIO->Negative->ODR&=0xF000;
}
void resetLinesToPulse(int8_t* linesToPulse){
    for (uint8_t i=0; i<CLOCKLINES_TOTAL; i++){
        linesToPulse[i]=-1;
    }
}
