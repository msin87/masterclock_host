#ifndef MESSAGE_H
#define MESSAGE_H
#include "stm32f4xx_hal.h"
#define CMD_START	 		0x0000
#define CMD_CNT_SET 		0x0001
#define CMD_CNT_INCREASE 	0x0002
#define CMD_CNT_RESET 		0x0003
#define CMD_CNT_SUSPEND 	0x0004
#define CMD_CNT_RESUME 		0x0005
#define CMD_PULSE_WIDTH 	0x0006
#define RESP_CNT			0x0001
#define SENSOR_LINES		0x0002
typedef struct
{
	uint16_t cmd;
	int8_t idArray[12];
	uint16_t dataArray[12];

} Message;
void messageToFrame(Message* message, uint8_t* frame);
void frameToMessage(uint8_t* frame, Message* message);
void getFrameData16(uint8_t* frame, uint16_t* data);
void getFrameData8(uint8_t* frame, uint8_t* data);
uint16_t idArrayToU16(int8_t* idArray);
void messageInit(Message* msg);
uint8_t isFrameCrcOk(uint8_t* frame);
uint8_t uartDataFrame[32];
void sendMessageToUART(UART_HandleTypeDef* huart, uint16_t* data, uint16_t CMD,
		int8_t* idToSend);
#endif
