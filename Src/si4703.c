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
#define Si4702_nReset_Pin GPIO_PIN_6
#define Si4702_nReset_GPIO_Port GPIOC
#define Si4702_nSen_Pin GPIO_PIN_7
#define Si4702_nSen_GPIO_Port GPIOC
#define Si4702_GPIO1_Pin GPIO_PIN_8
#define Si4702_GPIO1_GPIO_Port GPIOC
#define Si4702_GPIO2_Pin GPIO_PIN_9
#define Si4702_GPIO2_GPIO_Port GPIOC
#define Si4702_SDIO_Pin GPIO_PIN_9
#define Si4702_SDIO_GPIO_Port GPIOB
/* Si4703 defines */
#define Si4703_ADDR                     0x20 // I2C address (0b0010000 shifted to left for one bit)
/* Si4703 registers */
#define Si4703_DEVICEID                 0x00 // Device ID
#define Si4703_CHIPID                   0x01 // Chip ID
#define Si4703_POWERCFG                 0x02 // Power configuration
#define Si4703_CHANNEL                  0x03 // Channel
#define Si4703_SYSCONFIG1               0x04 // System configuration #1
#define Si4703_SYSCONFIG2               0x05 // System configuration #2
#define Si4703_SYSCONFIG3               0x06 // System configuration #3
#define Si4703_TEST1                    0x07 // Test 1
#define Si4703_TEST2                    0x08 // Test 2
#define Si4703_BOOT                     0x09 // Boot configuration
#define Si4703_RSSI                     0x0a // Status RSSI
#define Si4703_READCHANNEL              0x0b // Read channel
#define Si4703_RDSA                     0x0c // RDSA
#define Si4703_RDSB                     0x0d // RDSB
#define Si4703_RDSC                     0x0e // RDSC
#define Si4703_RDSD                     0x0f // RDSD
/* Power configuration */
#define Si4703_PWR_DSMUTE               15 // Softmute disable (0 = enable (default); 1 = disable)
#define Si4703_PWR_DMUTE                14 // Mute disable (0 = enable (default); 1 = disable)
#define Si4703_PWR_MONO                 13 // Mono select (0 = stereo (default); 1 = force mono)
#define Si4703_PWR_RDSM                 11 // RDS mode (0 = standard (default); 1 = verbose)
#define Si4703_PWR_SKMODE               10 // Seek mode (0 = wrap band limits and continue (default); 1 = stop at band limit)
#define Si4703_PWR_SEEKUP                9 // Seek direction (0 = down (default); 1 = up)
#define Si4703_PWR_SEEK                  8 // Seek (0 = disable (default); 1 = enable)
#define Si4703_PWR_DISABLE               6 // Powerup disable (0 = enable (default))
#define Si4703_PWR_ENABLE                0 // Powerup enable (0 = enable (default))
/* Channel */
#define Si4703_CH_TUNE                  15 // Tune (0 = disable (default); 1 = enable)
/* System configuration #1 */
#define Si4703_SC1_RDSIEN               15 // RDS interrupt enable (0 = disable (default); 1 = enable)
#define Si4703_SC1_STCIEN               14 // Seek/Tune complete interrupt enable (0 = disable (default); 1 = enable)
#define Si4703_SC1_RDS                  12 // RDS enable (0 = disable (default); 1 = enable)
#define Si4703_SC1_DE                   11 // De-emphasis (0 = 75us, USA (default); 1 = 50us, Europe, Australia, Japan)
#define Si4703_SC1_AGCD                 10 // AGC disable (0 = AGC enable (default); 1 = AGC disable)
/* System configuration #2 */
#define Si4703_SC2_BAND0                 6 // Band select
#define Si4703_SC2_BAND1                 7
#define Si4703_SC2_SPACE0                4 // Channel spacing
#define Si4703_SC2_SPACE1                5
/* System configuration #3 */
#define Si4703_SC3_VOLEXT                8 // Extended volume range (0 = disabled (Default); 1 = enabled (decrease the volume by 28dB))
/* Test 1 */
#define Si4703_T1_XOSCEN                15 // Crystal oscillator enable (0 = disable (Default); 1 = enable)
#define Si4703_T1_WTF                    8 // Datasheet aren't say anything about this, but it's necessary on powerup.
/* Status RSSI */
#define Si4703_RSSI_RDSR                15 // RDSR is ready (0 = No RDS group ready; 1 = New RDS group ready)
#define Si4703_RSSI_STC                 14 // Seek/Tune complete (0 = not complete; 1 = complete)
#define Si4703_RSSI_SFBL                13 // Seek fail/Band limit (0 = Seek successful; 1 = Seek failure/Band limit reached)
#define Si4703_RSSI_AFCRL               12 // AFC fail (0 = AFC not railed; 1 = AFC railed)
#define Si4703_RSSI_RDSS                11 // RDS sync (0 = not synchronized; 1 = decoder synchronized)
#define Si4703_RSSI_ST                   8 // Stereo indicator (0 = mono; 1 = stereo)
/* Some additional constants */
#define Si4703_SEEK_UP                   0 // Seek up (default)
#define Si4703_SEEK_DOWN                 1 // Seek down
#define Si4703_WRAP_ON                   0 // Wrap around band limit enabled (default)
#define Si4703_WRAP_OFF                  1 // Wrap around band limit disabled
#define TOTAL_REGS 16
#define I2C_PORT &hi2c1
extern I2C_HandleTypeDef hi2c1;
uint16_t Si4703_REGs[TOTAL_REGS];
void Error(void);

void Si4703_I2C_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_I2C_DeInit(&hi2c1);									//turn off I2C1
	HAL_GPIO_DeInit(GPIOB, Si4702_SDIO_Pin);				//deinit GPIO for
	GPIO_InitStruct.Pin = Si4702_SDIO_Pin;					//config GPIOB for SDIO operation
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Si4702_SDIO_GPIO_Port, &GPIO_InitStruct);	//init GPIOB
	Si4702_nReset_GPIO_Port->BSRR|=Si4702_nReset_Pin<<16;	//set nReset to 0 (start si4702)
	Si4702_SDIO_GPIO_Port->BSRR|=Si4702_SDIO_Pin<<16;		//set SDIO to 0
	osDelay(1);
	Si4702_nReset_GPIO_Port->BSRR|=Si4702_nReset_Pin;		//set nReset to 1 (start si4702)
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;			//config GPIOB for I2C1
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_DeInit(GPIOB, Si4702_SDIO_Pin);				//deinit GPIO SDIO
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);					//init GPIOB 8,9 as I2C1
	hi2c1.Instance = I2C1;									//config I2C1
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 1;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)						//init I2C1
	{
	  Error();
	}
}


void Si4703_Read(uint16_t* _Si4703_REGs) {
	uint8_t i;
	uint8_t buffer[32]; // 16 of 16-bit registers

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT))
	{
		osDelay(1); // Wait for EV5
	}
	I2C_Send7bitAddress(I2C_PORT,Si4703_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		osDelay(1); // Wait for EV6
	}
	// Si4703 read start from r0Ah register
	for (i = 0x14; ; i++) {					//20
		if (i == 0x20) i = 0x00;			//32
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			osDelay(1); // Wait for EV7 (Byte received from slave)
		}
		buffer[i] = I2C_ReceiveData(I2C_PORT); // Receive byte
		if (i == 0x12) break;
	}
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			osDelay(1); // Wait for EV7 (Byte received from slave)
		}
	buffer[i++] = I2C_ReceiveData(I2C_PORT); // Receive last byte

	for (i = 0; i < TOTAL_REGS; i++) {
		_Si4703_REGs[i] = (buffer[i<<1] << 8) | buffer[(i<<1)+1];
	}
}
void Si4703_Write(void) {
	uint8_t i;

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT))
		{
			osDelay(1); // Wait for EV5
		}
	I2C_Send7bitAddress(I2C_PORT,Si4703_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
			osDelay(1); // Wait for EV6
		}
	for (i = 2; i < 8; i++) {
		I2C_SendData(I2C_PORT,Si4703_REGs[i] >> 8); // MSB
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			{
				osDelay(1); // Wait for EV8
			}
		I2C_SendData(I2C_PORT,Si4703_REGs[i] & 0x00ff); // LSB
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
				osDelay(1); // Wait for EV8
		}
	}
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
}
void Error(void) {

};
