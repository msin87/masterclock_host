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
typedef struct {
	uint8_t measuredCurrent[CLOCKLINES_TOTAL];
	uint16_t counters[CLOCKLINES_TOTAL];
	uint16_t polarity;
	uint16_t width[CLOCKLINES_TOTAL];
} ClockLines;
void clockLinesInit(ClockLines* lines);
#endif /* CLOCKLINES_H_ */
