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
#include "relay.h"
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

			break;
		case CMD_CNT_SET:
			setCounters(message.idArray,message.dataArray);
			break;
		case CMD_CNT_INCREASE:
			increaseCounters(message.idArray);
			break;
		case CMD_CNT_RESET:
			resetCounters(message.idArray);
			break;
		case CMD_CNT_SUSPEND:
			suspendCounters(message.idArray);
			break;
		case CMD_CNT_RESUME:
			resumeCounters(message.idArray);
			break;
		case CMD_PULSE_WIDTH:
			setPulseWidth(message.dataArray);
			break;
		case CMD_PULSE_POL:
			setPulsePolarity(message.idArray,message.dataArray);
			break;
		case CMD_RELAY_SET:
			setRelay(message.idArray,message.dataArray);
			break;
		default:
			break;
		}
	}
}
