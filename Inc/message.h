#ifndef MESSAGE_H
#define MESSAGE_H
#include "stm32f4xx_hal.h"
typedef struct
{
	uint16_t cmd;
	uint8_t idArray[12];
	uint16_t dataArray[12];

} Message;
void messageToFrame(Message* message, uint8_t* frame);
void frameToMessage(uint8_t* frame, Message* message);
void getFrameData16(uint8_t* frame, uint16_t* data);
void getFrameData8(uint8_t* frame, uint8_t* data);
uint16_t idArrayToU16(uint8_t* idArray, uint8_t size);
void messageInit(Message* msg);
uint8_t isFrameCrcOk(uint8_t* frame);
uint8_t uartDataFrame[32];

#endif
