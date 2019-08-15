/*
 * uartcmd.c
 *
 *  Created on: Aug 13, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */
#include "uartcmd.h"
#include "message.h"
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
