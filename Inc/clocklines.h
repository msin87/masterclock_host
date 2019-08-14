/*
 * clocklines.h
 *
 *  Created on: Aug 13, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#ifndef CLOCKLINES_H_
#define CLOCKLINES_H_
#include "stm32f4xx_hal.h"
#define CLOCKLINES_TOTAL 12
#define POSITIVE 1
#define DEATH_TIME 30
typedef struct {
    uint8_t measuredCurrent;
    int8_t polarity;
    uint16_t counter;
} ClockLine;
typedef struct
{
    GPIO_TypeDef* Positive;
    GPIO_TypeDef* Negative;
}LinesGPIO_TypeDef;
ClockLine clockLines[CLOCKLINES_TOTAL];
uint8_t clockLines_isCountersEmpty;
uint16_t clockLines_pulseWidth;
LinesGPIO_TypeDef LinesGPIO;
void sendPulse(ClockLine*,int8_t*,uint8_t, LinesGPIO_TypeDef*);
void stopPulse(LinesGPIO_TypeDef*);
void resetLinesToPulse(int8_t*);
#endif /* CLOCKLINES_H_ */
