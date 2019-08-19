/*
 * uartcmd.c
 *
 *  Created on: Aug 13, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */
#include "uartcmd.h"
#include "message.h"
#include "clocklines.h"

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
			setCounters(message.idArray,message.dataArray);
			break;
		case CMD_CNT_INCREASE:
			increaseCounters(message.idArray);
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
