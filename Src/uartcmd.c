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
#include "si4703.h"
#include "cmsis_os.h"
extern volatile SemaphoreHandle_t Si4703MutexHandle;
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
		case CMD_FM_SET_FREQ:
			xSemaphoreTake(Si4703MutexHandle,portMAX_DELAY);
			Si4703_SetChannel(message.dataArray[0]);
			xSemaphoreGive(Si4703MutexHandle);
			break;
		case CMD_FM_SEEK:
			xSemaphoreTake(Si4703MutexHandle,portMAX_DELAY);
			switch (message.dataArray[0])
			{
			case 0x00:
				Si4703_Seek_Cancel();
				break;
			case 0x01:
				Si4703_SetChannel(875);
				Si4703_Seek(Si4703_SEEK_UP, Si4703_WRAP_OFF);
				break;
			case 0x02:
				Si4703_Seek(Si4703_SEEK_UP, Si4703_WRAP_OFF);
				break;
			}
			xSemaphoreGive(Si4703MutexHandle);
			break;
		default:
			break;
		}
	}
}
