/*
 * message.c
 *
 *  Created on: 12 авг. 2019 г.
 *      Author: Mikhail Sinelnikov
 *      E-Mail: msin87@yandex.ru
 */
#include "message.h"
#include "crc.h"
uint8_t uartDataFrame[32] =
{ 0 };
uint16_t idArrayToU16(int8_t* idArray)
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idArray[i] < 0)
			continue;
		result |= 1 << idArray[i];
	}
	return result;
}
void u16ArrayToU8(uint16_t* u16Array, uint8_t size, uint8_t* u8Array)
{
	for (uint8_t i = 0; i < size; i++)
	{

		u8Array[2 * i] = u16Array[i] / 0x0100;
		u8Array[2 * i + 1] = u16Array[i] % 0x0100;
	}
}
void messageInit(Message* message)
{
	message->cmd = 0;
	for (uint8_t i = 0; i < 12; i++)
	{
		message->idArray[i] = -1;
		message->dataArray[i] = 0;
	}

}
void messageToFrame(Message* message, uint8_t* frame)
{
	uint32_t crc32;
	uint16_t id = idArrayToU16(message->idArray);
	frame[0] = message->cmd / 0x0100;
	frame[1] = message->cmd % 0x0100;
	frame[2] = id / 0x0100;
	frame[3] = id % 0x0100;
	for (uint8_t i = 0; i < 12; i++)
	{
		frame[4 + i * 2] = message->dataArray[i] / 0x100;
		frame[4 + i * 2 + 1] = message->dataArray[i] % 0x100;
	}
	crc32 = crc32_zlib((uint32_t*) frame, 28);
	frame[28] = (crc32 & 0xFF000000) >> 24;
	frame[29] = (crc32 & 0x00FF0000) >> 16;
	frame[30] = (crc32 & 0x0000FF00) >> 8;
	frame[31] = (crc32 & 0x000000FF);
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

uint8_t isFrameCrcOk(uint8_t* frame)
{
	uint32_t crc32Computed = crc32_zlib((uint32_t*) frame, 28);
	uint32_t crc32Income = frame[28] << 24 | frame[29] << 16 | frame[30] << 8
			| frame[31];
	return crc32Computed == crc32Income;
}
void frameToMessage(uint8_t* frame, Message* message)
{
	uint16_t idU16 = frame[2] << 8 | frame[3]; //Concatenate ID bytes in frame to uint16_t
	message->cmd = frame[0] << 8 | frame[1]; //Concatenate CMD bytes in frame to uint16_t
	for (uint8_t i = 0, j = 0; i < 16; i++)	//Parse ID in uint16_t to idArray
	{
		if ((1 << i) & idU16)
		{
			message->idArray[j] = i;			//Store after last added element
			j++;
		}
	}
	for (uint8_t i = 0; i < 12; i++)//Parse data bytes in frame to uint16_t dataArray
	{
		message->dataArray[i] = frame[4 + i * 2] << 8 | frame[5 + i * 2];
	}
}
void sendCurrentSensorsToUART(UART_HandleTypeDef* huart, float* data, uint16_t CMD,
		int8_t* idToSend)
{
	Message message;
	uint8_t frame[32] =
	{ 0 };
	uint8_t j = 0;
	messageInit(&message);
	message.cmd = CMD;
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idToSend[i] < 0)
			break;
		message.dataArray[j] = (uint16_t)data[i];
		j++;
		message.idArray[i]= idToSend[i];
	}

	messageToFrame(&message, frame);
	HAL_UART_Transmit(huart, frame, 32, 100);
}
