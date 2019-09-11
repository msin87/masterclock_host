/*
 * si4703.h
 *
 *  Created on: Aug 27, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#ifndef SI4703_H_
#define SI4703_H_

#include "stm32f4xx_hal.h"
#define Si4703_nReset_Pin GPIO_PIN_6
#define Si4703_nReset_GPIO_Port GPIOC
#define Si4703_nSen_Pin GPIO_PIN_7
#define Si4703_nSen_GPIO_Port GPIOC
#define Si4703_GPIO1_Pin GPIO_PIN_8
#define Si4703_GPIO1_GPIO_Port GPIOC
#define Si4703_GPIO2_Pin GPIO_PIN_9
#define Si4703_GPIO2_GPIO_Port GPIOC
#define Si4703_SDIO_Pin GPIO_PIN_9
#define Si4703_SDIO_GPIO_Port GPIOB
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
#define Si4703_SC1_GPIO2                2  // GPIO2 [1:0] (01 = RDS interrupt (active low level), 10 = low, 11 = gigh)

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
#define Si4703_SEEK_UP                   1 // Seek up (default)
#define Si4703_SEEK_DOWN                 0 // Seek down
#define Si4703_WRAP_ON                   0 // Wrap around band limit enabled (default)
#define Si4703_WRAP_OFF                  1 // Wrap around band limit disabled
#define Si4703_TOTAL_REGS 16
#define I2C_PORT &hi2c1

#define RDS_NO_UPDATE 		0
#define RDS_RT_UPDATED 		1
#define RDS_TIME_DECODED 	2
#define RDS_PS_UPDATED 		3
#define RDS_RT_COMPLETE 	4

typedef struct {
	char program_service_name[8];
	char radiotextA[64];
	char radiotextB[32];
	uint8_t radiotext_AB_flag;
	uint8_t month;
	uint8_t day;
	uint8_t hours;
	uint8_t minutes;
	uint8_t RT_lengthA;
	uint8_t RT_lengthB;
	uint16_t year;
} RDS_Struct;
void Si4703_I2C_Init(void);
void Si4703_Init(void);
void Si4703_Read(uint16_t* _Si4703_REGs);
void Si4703_SetChannel(int32_t Channel);
uint32_t Si4703_Seek(uint8_t SeekDirection, uint8_t Wrap);
uint32_t Si4703_GetChannel(void);
void Si4703_SendRDSToUART(UART_HandleTypeDef* huart, uint16_t* rdsData);
void Si4703_RDS_Reset(void);
uint8_t Si4703_RDS_Decode(uint16_t* group);
void Si4703_Seek_Cancel(void);
uint16_t Si4703_REGs[Si4703_TOTAL_REGS];
RDS_Struct rdsStruct;
#endif /* SI4703_H_ */
