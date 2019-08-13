#ifndef MESSAGE_H
#define MESSAGE_H
#include "stm32f4xx_hal.h"
typedef struct
{
	uint8_t frame[33];
	uint8_t cmd;
	uint8_t idArray[12];
	uint16_t dataArray[12];
	uint32_t crc32;
} Message;

void messageToFrame(uint16_t cmd, uint16_t id, uint16_t* dataArray,
		uint8_t* frame);
void getFrameData16(uint8_t* frame, uint16_t* data);
void getFrameData8(uint8_t* frame, uint8_t* data);
uint16_t idArrayToU16(uint8_t* idArray, uint8_t size);
#endif
