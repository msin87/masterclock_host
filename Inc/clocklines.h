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
#define POSITIVE 1
#define DEATH_TIME 30
#define PULSE_OFF 0
#define PULSE_ON 1
typedef struct {
    int8_t polarity;
    uint16_t counter;
} ClockLine;
typedef struct
{
    GPIO_TypeDef* Positive;
    GPIO_TypeDef* Negative;
}LinesGPIO_TypeDef;
ClockLine clockLines[12];
uint8_t clockLines_isCountersEmpty;
uint8_t CLOCKLINES_TOTAL;
uint16_t clockLines_pulseWidth;
LinesGPIO_TypeDef LinesGPIO;
void sendPulse(ClockLine*,int8_t*,uint8_t, LinesGPIO_TypeDef*);
void stopPulse(LinesGPIO_TypeDef*);
void resetLinesId(int8_t* linesId);
void sendCountersToUART(UART_HandleTypeDef* huart, ClockLine* clockLines, int8_t* idToSend);
void setCounters(int8_t* idArray, uint16_t* counters);
void increaseCounters(int8_t* idArray);
#endif /* CLOCKLINES_H_ */
