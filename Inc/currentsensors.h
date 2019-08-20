/*
 * currentsensors.h
 *
 *  Created on: Aug 19, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#ifndef CURRENTSENSORS_H_
#define CURRENTSENSORS_H_
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"
typedef struct
{
	volatile uint8_t adc_data[12];
	int8_t storedLinesId[12];
	uint8_t totalLines;
	float filterCoeff;
	float filteredData[12];
} ClockLineCurrentSensor;
ClockLineCurrentSensor clockLineCurrentSensor;
void startClockLinesADC(ADC_HandleTypeDef* hadc, ClockLineCurrentSensor* sensor,
		int8_t* linesId);
void resetClockLineCurrentSensor(ClockLineCurrentSensor* sensor);
void sendCurrentSensorsToUART(UART_HandleTypeDef* huart, float* data, uint16_t CMD,
		int8_t* idToSend);
void filter(volatile uint8_t* adc_data, uint8_t size, float coeff, float* out);
#endif /* CURRENTSENSORS_H_ */
