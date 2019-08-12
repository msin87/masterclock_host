/*
 * message.c
 *
 *  Created on: 12 авг. 2019 г.
 *      Author: Mikhail Sinelnikov
 *      E-Mail: msin87@yandex.ru
 */
#include "message.h"

void messageToFrame(uint8_t cmd, uint8_t* idArray, uint8_t* dataArray,
		uint8_t* frame)
{
	frame[0] = cmd;
	frame[1] = idArray[1];
	frame[2] = idArray[0];
	for (uint8_t i = 0; i < 24; i++)
	{
		frame[3 + i] = dataArray[i];
	}

}

void getFrameData16(uint8_t* frame, uint16_t* data)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		data[i] = frame[3 + i * 2] | frame[3 + i * 2 + 1] << 8;
	}
}
void getFrameData8(uint8_t* frame, uint8_t* data)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		data[i] = frame[3 + i];
	}
}
