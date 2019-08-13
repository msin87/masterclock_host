/*
 * uartcmd.c
 *
 *  Created on: Aug 13, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */
#include "uartcmd.h"
#include "message.h"
#define CMD_START	 		0x0000
#define CMD_CNT_SET 		0x0001
#define CMD_CNT_INCREASE 	0x0002
#define CMD_CNT_RESET 		0x0003
#define CMD_CNT_SUSPEND 	0x0004
#define CMD_CNT_RESUME 		0x0005
#define CMD_PULSE_WIDTH 	0x0006
extern void messageInit(Message* message);
extern void messageToFrame(Message* message, uint8_t* frame);
extern void frameToMessage(uint8_t* frame, Message* message);
extern uint8_t isFrameCrcOk(uint8_t* frame);

void uartCmdParse(uint8_t* frame)
{
	Message message;
	messageInit(&message);
	if (isFrameCrcOk(frame))
	{
		frameToMessage(frame, &message);
		switch (message.cmd)
		{
		case CMD_START:
			NVIC_SystemReset();
			break;
		case CMD_CNT_SET:
			break;
		case CMD_CNT_INCREASE:
			break;
		case CMD_CNT_RESET:
			break;
		case CMD_CNT_SUSPEND:
			break;
		case CMD_CNT_RESUME:
			break;
		case CMD_PULSE_WIDTH:
			break;
		default:
			break;
		}
	}
}
