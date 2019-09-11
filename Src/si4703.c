/*
 * si4703.c
 *
 *  Created on: Aug 27, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */
#include "si4703.h"
#include "i2c.h"
#include "cmsis_os.h"
#include "message.h"
#include <string.h>
#define GROUP_B 1

extern I2C_HandleTypeDef hi2c1;
uint16_t Si4703_REGs[Si4703_TOTAL_REGS];
RDS_Struct rdsStruct;
void Error(void);

void Si4703_I2C_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = Si4703_SDIO_Pin;					//config GPIOB for SDIO operation
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Si4703_SDIO_GPIO_Port, &GPIO_InitStruct);	//init GPIOB
	GPIO_InitStruct.Pin = Si4703_nReset_Pin;					//config GPIOB for SDIO operation
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Si4703_nReset_GPIO_Port, &GPIO_InitStruct);
	Si4703_nReset_GPIO_Port->BSRR|=Si4703_nReset_Pin<<16;	//set nReset to 0 (start Si4703)
	Si4703_SDIO_GPIO_Port->BSRR|=Si4703_SDIO_Pin<<16;		//set SDIO to 0
	osDelay(5);
	Si4703_nReset_GPIO_Port->BSRR|=Si4703_nReset_Pin;		//set nReset to 1 (start Si4703)
	osDelay(5);
}


void Si4703_Read(uint16_t* _Si4703_REGs) {
	uint8_t i;
	uint8_t buffer[32]; // 16 of 16-bit registers

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
	{
		osDelay(1); // Wait for EV5
	}
	I2C_Send7bitAddress(I2C_PORT,Si4703_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)==ERROR)
	{
		osDelay(1); // Wait for EV6
	}
	// Si4703 read start from r0Ah register
	for (i = 0x14; ; i++) {					//20
		if (i == 0x20) i = 0x00;			//32
		while (I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR)
		{
			osDelay(1); // Wait for EV7 (Byte received from slave)
		}
		buffer[i] = I2C_ReceiveData(I2C_PORT); // Receive byte
		if (i == 0x12) break;
	}
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)==ERROR)
		{
			osDelay(1); // Wait for EV7 (Byte received from slave)
		}
	buffer[i++] = I2C_ReceiveData(I2C_PORT); // Receive last byte

	for (i = 0; i < Si4703_TOTAL_REGS; i++) {
		_Si4703_REGs[i] = (buffer[i<<1] << 8) | buffer[(i<<1)+1];
	}
	osDelay(1);
}
void Si4703_Write(uint16_t* _Si4703_REGs) {
	uint8_t i;

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)==ERROR)
		{
			osDelay(1); // Wait for EV5
		}
	I2C_Send7bitAddress(I2C_PORT,Si4703_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR)
		{
			osDelay(1); // Wait for EV6
		}
	for (i = 2; i < 8; i++) {
		I2C_SendData(I2C_PORT,_Si4703_REGs[i] >> 8); // MSB
		while (I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR)
			{
				osDelay(1); // Wait for EV8
			}
		I2C_SendData(I2C_PORT,_Si4703_REGs[i] & 0x00ff); // LSB
		while (I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)==ERROR)
		{
				osDelay(1); // Wait for EV8
		}
	}
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	osDelay(1);
}

void Si4703_Init(void) {
	// Si4703 powerup configuration sequence
	Si4703_Read(Si4703_REGs);
	Si4703_REGs[Si4703_TEST1] = (1 << Si4703_T1_XOSCEN)|(1 << Si4703_T1_WTF); // power up crystall
	Si4703_Write(Si4703_REGs);
	osDelay(500); // wait for crystal to power up (by AN230 v0.61)

	// Configure tuner beginning conditions
	Si4703_Read(Si4703_REGs);
	Si4703_REGs[Si4703_POWERCFG] = (1<<Si4703_PWR_DMUTE)|(1<<Si4703_PWR_ENABLE);
    Si4703_REGs[Si4703_SYSCONFIG1] |= (1<<Si4703_SC1_RDS); // Enable RDS
    Si4703_REGs[Si4703_SYSCONFIG1] |= (1<<Si4703_SC1_RDSIEN); // Enable RDS
    Si4703_REGs[Si4703_SYSCONFIG1] |= (1<<Si4703_SC1_GPIO2); // set GPIO2 as RDS interrupt output (active low)
    Si4703_REGs[Si4703_SYSCONFIG1] |= (1<<Si4703_SC1_DE); // 50us de-emphasis (must be on for Europe, Australia and Japan)
    Si4703_REGs[Si4703_SYSCONFIG2] &= ~(1<<Si4703_SC2_BAND1)|(1<<Si4703_SC2_BAND0); // 87.5-108MHz (USA,Europe)
    //Si4703_REGs[Si4703_SYSCONFIG2] |= (1<<Si4703_SC2_BAND1)|(1<<Si4703_SC2_BAND0); // 76-108MHz (Japan wide band)
    Si4703_REGs[Si4703_SYSCONFIG2] |= (1<<Si4703_SC2_SPACE0); // 100kHz spacing (Europe)
	Si4703_REGs[Si4703_SYSCONFIG2] &= 0xfff0;
	//Si4703_REGs[Si4703_SYSCONFIG2] |= 0x0001; // minimum volume
	Si4703_REGs[Si4703_SYSCONFIG2] |= 0x0007; // medium volume
	//Si4703_REGs[Si4703_SYSCONFIG2] |= 0x000f; // maximum volume
	//Si4703_REGs[Si4703_SYSCONFIG3] |= (1<<Si4703_SC3_VOLEXT); // Decrease the volume by 28dB
	Si4703_Write(Si4703_REGs);
	osDelay(150); // wait for device powerup (110ms from datasheet?)
}
void Si4703_SetChannel(int32_t Channel) {
	Si4703_Read(Si4703_REGs);

	Channel *= 10;
	Channel -= ((Si4703_REGs[Si4703_SYSCONFIG2] & ((1<<Si4703_SC2_BAND1) | (1<<Si4703_SC2_BAND0))) == 0) ? 8750 : 7600;
	Channel /= 10;

	Si4703_REGs[Si4703_CHANNEL] &= 0xfe00; // Clear channel frequency from register
	Si4703_REGs[Si4703_CHANNEL] |= Channel; // Set new channel frequency
	Si4703_REGs[Si4703_CHANNEL] |= (1<<Si4703_CH_TUNE); // Set TUNE flag
	Si4703_Write(Si4703_REGs);

	osDelay(50); // Gime some time for the Si4703 to tune up

	// Wait for the Si4703 to set STC flag
	while(1) {
		Si4703_Read(Si4703_REGs);
		if ((Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_STC)) != 0) break; // Tuning complete
		osDelay(1);
	}
	Si4703_Read(Si4703_REGs);
	Si4703_REGs[Si4703_CHANNEL] &= ~(1<<Si4703_CH_TUNE); // Clear TUNE flag
	Si4703_Write(Si4703_REGs);
	osDelay(1); // <---------------------- Is this necessary?

	// Wait for the Si4703 to clear STC flag
	while(1) {
		Si4703_Read(Si4703_REGs);
		if ((Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_STC)) == 0) break; // Tuning complete
		osDelay(1);
	}
}
void Si4703_Seek_Cancel(void){
	Si4703_Read(Si4703_REGs);
	Si4703_REGs[Si4703_POWERCFG] &= ~(1<<Si4703_PWR_SEEK);
	Si4703_Write(Si4703_REGs);
	osDelay(1);
}
uint32_t Si4703_Seek(uint8_t SeekDirection, uint8_t Wrap) {
	uint32_t freq;
	uint32_t _sfbl;

	Si4703_Read(Si4703_REGs);

	if (Wrap) {
		Si4703_REGs[Si4703_POWERCFG] |=  (1<<Si4703_PWR_SKMODE); // Band wrap on
	} else {
		Si4703_REGs[Si4703_POWERCFG] &= ~(1<<Si4703_PWR_SKMODE); // Band wrap off
	}
	if (SeekDirection) {
		Si4703_REGs[Si4703_POWERCFG] |= (1<<Si4703_PWR_SEEKUP); // Seek up
	} else {
		Si4703_REGs[Si4703_POWERCFG] &= ~(1<<Si4703_PWR_SEEKUP); // Seek down
	}
	Si4703_REGs[Si4703_POWERCFG] |= (1<<Si4703_PWR_SEEK); // Set seek start bit

	Si4703_Write(Si4703_REGs); // Start seek

	// Wait for the Si4703 to set STC flag
	while(1) {
		Si4703_Read(Si4703_REGs);
		if ((Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_STC)) != 0) break; // Seek complete
		freq = Si4703_GetChannel();
	    osDelay(50); // <-- Fancy delay, in real this unnecessary
	}

	Si4703_Read(Si4703_REGs);

	_sfbl = Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_SFBL); // Store value of SFBL bit
	Si4703_REGs[Si4703_POWERCFG] &= ~(1<<Si4703_PWR_SEEK); // Reset seek bit (it must be done after seek)
	Si4703_Write(Si4703_REGs);
	osDelay(1); // <---------------------- Is this necessary?

	// Wait for the Si4703 to clear STC flag
	while(1) {
		Si4703_Read(Si4703_REGs);
		if ((Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_STC)) == 0) break; // Tuning complete
		osDelay(1);
	}

	if (_sfbl) {
		return 1;
	}

	return 0;
}
uint32_t Si4703_GetChannel(void) {
	uint32_t Channel;

	Si4703_Read(Si4703_REGs);
	Channel = Si4703_REGs[Si4703_READCHANNEL] & 0x03ff;
	Channel += ((Si4703_REGs[Si4703_SYSCONFIG2] & ((1<<Si4703_SC2_BAND1) | (1<<Si4703_SC2_BAND0))) == 0) ? 875 : 760;

	return Channel;
}
void Si4703_RDS_Reset(void){
	rdsStruct.day=0;
	rdsStruct.hours=0;
	rdsStruct.minutes=0;
	rdsStruct.month=0;
	rdsStruct.radiotext_AB_flag=0;
	rdsStruct.year=0;
	memset(rdsStruct.radiotextA, 0x00, sizeof(rdsStruct.radiotextA));
	memset(rdsStruct.radiotextB, 0x00, sizeof(rdsStruct.radiotextB));
	memset(rdsStruct.program_service_name, 0x00, sizeof(rdsStruct.program_service_name));
}

uint8_t Si4703_RDS_Decode(uint16_t* group){
	uint8_t group_type = (uint8_t)((group[1] >> 12) & 0xf);
	uint8_t ab = (group[1] >> 11 ) & 0x1;	//a=0,b=1
	uint8_t returnCode=RDS_NO_UPDATE;
	char newSymbol[4]={0};
	uint8_t text_segment_address_code;
	switch (group_type)
	{
	case 0:

		newSymbol[0]=(group[3] >> 8) & 0xff;
		newSymbol[1]= group[3]       & 0xff;
		uint8_t segment_address =  group[1] & 0x03;
		if ((rdsStruct.program_service_name[segment_address * 2]!=newSymbol[0])||(rdsStruct.program_service_name[segment_address * 2 + 1] != newSymbol[1]))
		{
			returnCode=RDS_PS_UPDATED;
		}
		rdsStruct.program_service_name[segment_address * 2]     = newSymbol[0];
		rdsStruct.program_service_name[segment_address * 2 + 1] = newSymbol[1];

		break;
	case 2:
		text_segment_address_code = group[1] & 0x0f;
		if(rdsStruct.radiotext_AB_flag != ((group[1] >> 4) & 0x01))
		{
			memset(rdsStruct.radiotextA, 0x00, sizeof(rdsStruct.radiotextA));
			memset(rdsStruct.radiotextB, 0x00, sizeof(rdsStruct.radiotextB));
		}
		rdsStruct.radiotext_AB_flag = (group[1] >> 4) & 0x01;
		if (ab==GROUP_B)
		{
			newSymbol[0] = (group[3] >> 8) & 0xff;
			newSymbol[1] =  group[3]       & 0xff;
			for (uint8_t i=0; i<2; i++)
			{
				if (rdsStruct.radiotextA[text_segment_address_code *4 + i] != newSymbol[i]) //check if the character has changed
					returnCode=RDS_RT_UPDATED;
				rdsStruct.radiotextB[text_segment_address_code * 2 + i] = newSymbol[i];
				if (newSymbol[i]==0x0D) 										//carriage return
				rdsStruct.RT_lengthB = text_segment_address_code*4+i;			//store text length
			}
			for (uint8_t i=0; i<rdsStruct.RT_lengthB; i++)						//checking for zeros in rt array (check if the text is complete)
			{
				if (rdsStruct.radiotextB[i]==0x00) continue;					//if text not completed do next step of cycle
				returnCode = RDS_RT_COMPLETE;									//if text completed say RDS_RT_COMPLETE
				break;															//exit from for() cycle
			}

		}
		else
		{
			newSymbol[0] = (group[2] >> 8) & 0xff;
			newSymbol[1] =  group[2]       & 0xff;
			newSymbol[2] = (group[3] >> 8) & 0xff;
			newSymbol[3] =  group[3]       & 0xff;
			for (uint8_t i=0; i<4; i++)
			{
				if (rdsStruct.radiotextA[text_segment_address_code *4 + i] != newSymbol[i]) //check if the character has changed
					returnCode=RDS_RT_UPDATED;
				rdsStruct.radiotextA[text_segment_address_code * 4 + i] = newSymbol[i];
				if (returnCode != RDS_RT_COMPLETE)
				{
					for (uint8_t j=0; j<sizeof(rdsStruct.radiotextA); j++)				//checking for zeros in rt array (check if the text is complete)
					{
						if (rdsStruct.radiotextA[j]==0x00) break;				//if text not completed do exit from cycle
						// Two cases when the text is considered complete separated for ease of reading code
						if ((j==sizeof(rdsStruct.radiotextA)-1)&&returnCode==RDS_RT_UPDATED)
						{
							returnCode = RDS_RT_COMPLETE;						//if text completed say RDS_RT_COMPLETE
							rdsStruct.RT_lengthA=0;
							break;
						}
						if (rdsStruct.radiotextA[j]==0x0D && returnCode==RDS_RT_UPDATED)
						{
							returnCode = RDS_RT_COMPLETE;						//if text completed say RDS_RT_COMPLETE
							rdsStruct.RT_lengthA=0;
							break;
						}
					}
				}
			}
		}


		break;
	case 4:
		if (ab==GROUP_B){
			break;
		}
		returnCode=RDS_TIME_DECODED;
		rdsStruct.hours   = ((group[2] & 0x1) << 4) | ((group[3] >> 12) & 0x0f);
		rdsStruct.minutes =  (group[3] >> 6) & 0x3f;
		double local_time_offset = .5 * (group[3] & 0x1f);

		if((group[3] >> 5) & 0x1) {
			local_time_offset *= -1;
		}
		double modified_julian_date = ((group[1] & 0x03) << 15) | ((group[2] >> 1) & 0x7fff);

		rdsStruct.year  = (uint16_t)((modified_julian_date - 15078.2) / 365.25);
		rdsStruct.month = (uint8_t)((modified_julian_date - 14956.1 - (int)(rdsStruct.year * 365.25)) / 30.6001);
		rdsStruct.day   = (uint8_t) (modified_julian_date - 14956 - (int)(rdsStruct.year * 365.25) - (int)(rdsStruct.month * 30.6001));
		uint8_t K = ((rdsStruct.month == 14) || (rdsStruct.month == 15)) ? 1 : 0;
		rdsStruct.year += K;
		rdsStruct.month -= 1 + K * 12;
		break;
	}

	return returnCode;
}
void Si4703_SendRDSToUART(UART_HandleTypeDef* huart, uint16_t* group){

	Message message;
	uint8_t frame[32] =	{ 0 };
	messageInit(&message);
	message.cmd=RESP_FM_RDS;
	message.idArray[0]=group[0] / 0x100;    //High byte of PI Code
	message.idArray[1]=group[0] % 0x100;	//Low Byte of PI Code

	messageToFrame(&message, frame);
	HAL_UART_Transmit(huart, &frame[4], 8, 100);
}
void Error(void) {
	while (1){

	}
};
