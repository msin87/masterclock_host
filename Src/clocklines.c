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
	for (uint8_t i = 0; i < CLOCKLINES_TOTAL; i++)
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
	clockLines_isCountersEmpty=0;
}
void increaseCounters(int8_t* idArray)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idArray[i] < 0)
			break;
		clockLines[idArray[i]].counter++;
	}
	clockLines_isCountersEmpty=0;
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
