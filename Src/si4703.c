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


extern I2C_HandleTypeDef hi2c1;
uint16_t Si4703_REGs[Si4703_TOTAL_REGS];
void Error(void);

void Si4703_I2C_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	HAL_I2C_DeInit(&hi2c1);									//turn off I2C1
//	HAL_GPIO_DeInit(GPIOB, Si4703_SDIO_Pin);				//deinit GPIO for
	GPIO_InitStruct.Pin = Si4703_SDIO_Pin;					//config GPIOB for SDIO operation
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Si4703_SDIO_GPIO_Port, &GPIO_InitStruct);	//init GPIOB
	Si4703_nReset_GPIO_Port->BSRR|=Si4703_nReset_Pin<<16;	//set nReset to 0 (start Si4703)
	Si4703_SDIO_GPIO_Port->BSRR|=Si4703_SDIO_Pin<<16;		//set SDIO to 0
	osDelay(1);
	Si4703_nReset_GPIO_Port->BSRR|=Si4703_nReset_Pin;		//set nReset to 1 (start Si4703)
//	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;			//config GPIOB for I2C1
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
//	//HAL_GPIO_DeInit(GPIOB, Si4703_SDIO_Pin);				//deinit GPIO SDIO
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);					//init GPIOB 8,9 as I2C1
//	hi2c1.Instance = I2C1;									//config I2C1
//	hi2c1.Init.ClockSpeed = 400000;
//	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//	hi2c1.Init.OwnAddress1 = 1;
//	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//	hi2c1.Init.OwnAddress2 = 0;
//	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//	if (HAL_I2C_Init(&hi2c1) != HAL_OK)						//init I2C1
//	{
//	  Error();
//	}
}


void Si4703_Read(uint16_t* _Si4703_REGs) {
	uint8_t i;
	uint8_t buffer[32]; // 16 of 16-bit registers


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
	Si4703_REGs[Si4703_SYSCONFIG2] |= 0x0007; // minimum volume
	//Si4703_REGs[Si4703_SYSCONFIG2] |= 0x0007; // medium volume
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
		Si4703_REGs[Si4703_POWERCFG] &= ~(1<<Si4703_PWR_SEEKUP); // Seek up
	} else {
		Si4703_REGs[Si4703_POWERCFG] |=  (1<<Si4703_PWR_SEEKUP); // Seek down
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
void Error(void) {

};
