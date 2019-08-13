/*
 * message.c
 *
 *  Created on: 12 авг. 2019 г.
 *      Author: Mikhail Sinelnikov
 *      E-Mail: msin87@yandex.ru
 */
#include "message.h"
#include "crc.h"
uint16_t idArrayToU16(uint8_t* idArray, uint8_t size)
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < size; i++)
	{
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
void messageToFrame(uint16_t cmd, uint16_t id, uint16_t* dataArray,
		uint8_t* frame)
{
	uint32_t crc32 = 0;
	frame[0] = cmd / 0x0100;
	frame[1] = cmd % 0x0100;
	frame[2] = id / 0x0100;
	frame[3] = id % 0x0100;
	for (uint8_t i = 0; i < 12; i++)
	{
		frame[4 + i * 2] = dataArray[i] / 0x100;
		frame[4 + i * 2 + 1] = dataArray[i] % 0x100;
	}
	crc32 = crc32_zlib((uint32_t*)frame, 28);
	frame[28]=(crc32& 0xFF000000) >> 24;
	frame[29]=(crc32& 0x00FF0000) >> 16;
	frame[30]=(crc32& 0x0000FF00) >> 8;
	frame[31]=(crc32& 0x000000FF);
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
