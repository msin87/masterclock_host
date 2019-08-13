/*
 * crc.c
 *
 *  Created on: 12 авг. 2019 г.
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */
#include "stm32f4xx_hal.h"
#include "crc.h"
uint32_t crc32_zlib(const uint32_t *data, size_t cnt) {

  uint32_t i;

  CRC->CR = CRC_CR_RESET;

  for (i = 0; i < (cnt / 4); i++) CRC->DR = __RBIT(data[i]);

  uint32_t result = __RBIT(CRC->DR);
  cnt = (cnt % 4) * 8;
  if (cnt) {
    CRC->DR = CRC->DR;
    CRC->DR = __RBIT((data[i] & (0xFFFFFFFF >> (32 - cnt))) ^ result) >> (32 - cnt);
    result = (result >> cnt) ^ __RBIT(CRC->DR);
  }
  return ~result;
}
