/*
 * i2c.c
 *
 *  Created on: Aug 27, 2019
 *      Author: Megavolos
 */

#include "i2c.h"
#define FLAG_Mask               ((uint32_t)0x00FFFFFF)
void I2C_GenerateSTART(I2C_HandleTypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  if (NewState != DISABLE)
  {
    /* Generate a START condition */
    I2Cx->Instance->CR1 |= I2C_CR1_START;
  }
  else
  {
    /* Disable the START condition generation */
	I2Cx->Instance->CR1 &= ~I2C_CR1_START;
  }
}
void I2C_GenerateSTOP(I2C_HandleTypeDef* I2Cx, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Generate a STOP condition */
	  I2Cx->Instance->CR1 |= I2C_CR1_STOP;
  }
  else
  {
    /* Disable the STOP condition generation */
	  I2Cx->Instance->CR1 &= I2C_CR1_STOP;
  }
}
void I2C_AcknowledgeConfig(I2C_HandleTypeDef* I2Cx, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the acknowledgement */
	  I2Cx->Instance->CR1 |= I2C_CR1_ACK;
  }
  else
  {
    /* Disable the acknowledgement */
	  I2Cx->Instance->CR1 &= ~I2C_CR1_ACK;
  }
}
ErrorStatus I2C_CheckEvent(I2C_HandleTypeDef* I2Cx, uint32_t I2C_EVENT)
{
  uint32_t lastevent = 0;
  uint32_t flag1 = 0, flag2 = 0;
  ErrorStatus status = ERROR;

  /* Read the I2Cx status register */
  flag1 = I2Cx->Instance->SR1;
  flag2 = I2Cx->Instance->SR2;
  flag2 = flag2 << 16;

  /* Get the last event value from I2C status register */
  lastevent = (flag1 | flag2) & FLAG_Mask;

  /* Check whether the last event contains the I2C_EVENT */
  if ((lastevent & I2C_EVENT) == I2C_EVENT)
  {
    /* SUCCESS: last event is equal to I2C_EVENT */
    status = SUCCESS;
  }
  else
  {
    /* ERROR: last event is different from I2C_EVENT */
    status = ERROR;
  }
  /* Return status */
  return status;
}
void I2C_Send7bitAddress(I2C_HandleTypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction)
{

  /* Test on the direction to set/reset the read/write bit */
  if (I2C_Direction != I2C_Direction_Transmitter)
  {
    /* Set the address bit0 for read */
    Address |= I2C_OAR1_ADD0;
  }
  else
  {
    /* Reset the address bit0 for write */
    Address &= ~I2C_OAR1_ADD0;
  }
  /* Send the address */
  I2Cx->Instance->DR = Address;
}
uint8_t I2C_ReceiveData(I2C_HandleTypeDef* I2Cx)
{
  /* Return the data in the DR register */
  return (uint8_t)I2Cx->Instance->DR;
}
void I2C_SendData(I2C_HandleTypeDef* I2Cx, uint8_t Data)
{

  /* Write in the DR register the data to be sent */
	I2Cx->Instance->DR = Data;
}
