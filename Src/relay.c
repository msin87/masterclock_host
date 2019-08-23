/*
 * relay.c
 *
 *  Created on: 23 авг. 2019 г.
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#include "relay.h"
#include "message.h"
#define RELAY_GPIO_Port GPIOD
const uint8_t RELAY_PINS[RELAY_TOTAL] = {12,13,14,15};
void setRelay(int8_t* idArray, uint16_t* state)
{
	for (uint8_t i = 0; i < RELAY_TOTAL; i++)
	{
		if (idArray[i] < 0)
			break;
		if (RELAY_PINS[idArray[i]]>15)
			continue;
		if (state[i])
		{
			RELAY_GPIO_Port->BSRR|=1<<RELAY_PINS[idArray[i]];
		}
		else
		{
			RELAY_GPIO_Port->BSRR|=(1<<RELAY_PINS[idArray[i]])<<16;
		}
		relay[idArray[i]].isOn = state[i];
	}
}
