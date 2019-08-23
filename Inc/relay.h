/*
 * relay.h
 *
 *  Created on: 23 авг. 2019 г.
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#ifndef RELAY_H_
#define RELAY_H_
#include "stm32f4xx_hal.h"

#define RELAY_TOTAL 12

typedef struct
{
	uint8_t isOn;
}Relay;
Relay relay[RELAY_TOTAL];
void setRelay(int8_t* idArray, uint16_t* counters);


#endif /* RELAY_H_ */
