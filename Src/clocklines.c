/*
 * clocklines.c
 *
 *  Created on: Aug 13, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#include "clocklines.h"
#include "message.h"

uint16_t clockLines_pulseWidth = 1000;
uint8_t clockLines_isCountersEmpty = 1;
LinesGPIO_TypeDef LinesGPIO =
{ .Positive = GPIOE, .Negative = GPIOD };
void sendPulse(ClockLine* clockLines, int8_t* linesToPulse, uint8_t length,
		LinesGPIO_TypeDef* LinesGPIO)
{
	uint16_t outBufferPos = 0, outBufferNeg = 0;
	for (uint8_t i = 0; i < length; i++)
	{
		if (linesToPulse[i] < 0)
			break;
		if (clockLines[linesToPulse[i]].polarity == POSITIVE)
		{
			outBufferPos |= 1 << linesToPulse[i];
		}
		else
		{
			outBufferNeg |= 1 << linesToPulse[i];
		}
	}
	LinesGPIO->Positive->ODR |= outBufferPos;
	LinesGPIO->Negative->ODR |= outBufferNeg;
}
void stopPulse(LinesGPIO_TypeDef* LinesGPIO)
{
	LinesGPIO->Positive->ODR &= 0xF000;
	LinesGPIO->Negative->ODR &= 0xF000;
}
void resetLinesId(int8_t* linesId)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		linesId[i] = -1;
	}
}
void setCounters(int8_t* idArray, uint16_t* counters)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idArray[i] < 0)
			break;
		clockLines[idArray[i]].counter = counters[i];
	}
	clockLines_isCountersEmpty = 0;
}
void increaseCounters(int8_t* idArray)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idArray[i] < 0)
			break;
		clockLines[idArray[i]].counter++;
	}
	clockLines_isCountersEmpty = 0;
}
void resetCounters(int8_t* idArray)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idArray[i] < 0)
			break;
		clockLines[idArray[i]].counter = 0;
	}
	clockLines_isCountersEmpty = 1;
}
void suspendCounters(int8_t* idArray)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idArray[i] < 0)
			break;
		clockLines[idArray[i]].isOn = 0;
	}

}
void resumeCounters(int8_t* idArray)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idArray[i] < 0)
			break;
		clockLines[idArray[i]].isOn = 1;
	}
}
void setPulseWidth(uint16_t* width)
{
	clockLines_pulseWidth = width[0];
}
void setPulsePolarity(int8_t* idArray, uint16_t* polarity)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idArray[i] < 0)
			break;
		clockLines[idArray[i]].polarity = polarity[i];
	}
}
void sendCountersToUART(UART_HandleTypeDef* huart, ClockLine* clockLines,
		int8_t* idToSend)
{
	Message message;
	uint8_t frame[32] =
	{ 0 };
	uint8_t j = 0;
	messageInit(&message);
	message.cmd = RESP_CNT;
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idToSend[i] < 0)
			break;
		message.dataArray[j] = clockLines[idToSend[i]].counter;
		j++;
		message.idArray[i] = idToSend[i];
	}
	messageToFrame(&message, frame);
	HAL_UART_Transmit(huart, frame, 32, 100);
}
void sendPolarityToUART(UART_HandleTypeDef* huart, ClockLine* clockLines,
		int8_t* idToSend)
{
	Message message;
	uint8_t frame[32] =
	{ 0 };
	uint8_t j = 0;
	messageInit(&message);
	message.cmd = RESP_LINES_POL;
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idToSend[i] < 0)
			break;
		message.dataArray[j] = clockLines[idToSend[i]].polarity;
		j++;
		message.idArray[i] = idToSend[i];
	}
	messageToFrame(&message, frame);
	HAL_UART_Transmit(huart, frame, 32, 100);
}
