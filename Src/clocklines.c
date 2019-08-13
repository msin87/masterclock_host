/*
 * clocklines.c
 *
 *  Created on: Aug 13, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#include "clocklines.h"

void clockLinesInit(ClockLines* lines){
	for (uint8_t i=0;i<CLOCKLINES_TOTAL;i++){
		lines->measuredCurrent[i]=0;
		lines->counters[i]=0;
		lines->width[i]=0;
	}
	lines->polarity=0;
}
