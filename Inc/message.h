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
#define CMD_PULSE_POL	 	0x0007
#define CMD_RELAY_SET	 	0x0008
#define CMD_FM_POWER		0x0F00
#define CMD_FM_SET_FREQ		0x0F01
#define CMD_FM_SEEK			0x0F02
#define RESP_CNT			0x0001
#define RESP_ADC_LINES		0x0002
#define RESP_LINES_POL		0x0007
#define RESP_FM_TEXTA0		0x0F00
#define RESP_FM_TEXTA1		0x0F01
#define RESP_FM_TEXTA2		0x0F02
#define RESP_FM_TEXTB0		0x0F10
#define RESP_FM_TEXTB1		0x0F11
#define RESP_FM_TIME	 	0x0F03
#define RESP_FM_TUNE_STATUS 0x0F04
typedef struct
{
	uint16_t cmd;
	int8_t idArray[12];
	uint16_t dataArray[12];

} Message;
void messageToFrame(Message* message, uint8_t* frame);
void rdsTextToFrame (Message* message, uint8_t* frame);
void frameToMessage(uint8_t* frame, Message* message);
void getFrameData16(uint8_t* frame, uint16_t* data);
void getFrameData8(uint8_t* frame, uint8_t* data);
uint16_t idArrayToU16(int8_t* idArray);
void messageInit(Message* msg);
uint8_t isFrameCrcOk(uint8_t* frame);
uint8_t uartDataFrame[32];
#endif
