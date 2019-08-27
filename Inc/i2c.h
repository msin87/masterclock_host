/*
 * i2c.h
 *
 *  Created on: Aug 27, 2019
 *      Author: Megavolos
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define  I2C_Direction_Transmitter							((uint8_t)0x00)
#define  I2C_Direction_Receiver         					((uint8_t)0x01)
#define  I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED            ((uint32_t)0x00030002)  /* BUSY, MSL and ADDR flags */
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED         ((uint32_t)0x00070082)  /* BUSY, MSL, ADDR, TXE and TRA flags */
#define  I2C_EVENT_MASTER_BYTE_TRANSMITTED                  ((uint32_t)0x00070084)  /* TRA, BUSY, MSL, TXE and BTF flags */
#define  I2C_EVENT_MASTER_BYTE_RECEIVED                     ((uint32_t)0x00030040)  /* BUSY, MSL and RXNE flags */
#define  I2C_EVENT_MASTER_MODE_SELECT						((uint32_t)0x00030001)  /* BUSY, MSL and SB flag */

void I2C_GenerateSTART(I2C_HandleTypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_HandleTypeDef* I2Cx, FunctionalState NewState);
void I2C_AcknowledgeConfig(I2C_HandleTypeDef* I2Cx, FunctionalState NewState);
ErrorStatus I2C_CheckEvent(I2C_HandleTypeDef* I2Cx, uint32_t I2C_EVENT);
void I2C_Send7bitAddress(I2C_HandleTypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
uint8_t I2C_ReceiveData(I2C_HandleTypeDef* I2Cx);
#endif /* I2C_H_ */
