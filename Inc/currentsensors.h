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




#define START_TIM_DMA_ADC() \
			TIM2->CR1 |= TIM_CR1_CEN;				/*turn on TIM2*/\
			DMA2_Stream0->CR |= DMA_SxCR_TCIE;		/*turn on DMA2 stream0 transfer complete interrupt*/\
			DMA2_Stream0->CR |= DMA_SxCR_EN;		/*turn off DMA2 Stream0 (ADC1)*/\
			ADC1->CR2 |= ADC_CR2_ADON				/*turn on ADC1*/

#define STOP_TIM_DMA_ADC() \
			ADC1->CR2 &= ~ADC_CR2_ADON; 			/*turn off ADC*/\
			TIM2->CR1 &= ~TIM_CR1_CEN;  			/*turn off TIM2*/\
			DMA2_Stream0->CR &= ~DMA_SxCR_EN; 		/*turn off DMA2 Stream0 (ADC1)*/\
			DMA2->LIFCR |= 0x3F;					/*clear interrupt flags for stream 0*/\
			TIM2->CNT = 0							/*reset TIM2 counter;*/
typedef struct
{
	volatile uint16_t adc_data[12];
	int8_t storedLinesId[12];
	uint8_t totalLines;
	float filterCoeff;
	float filteredData[12];
} ClockLineCurrentSensor;
ClockLineCurrentSensor clockLineCurrentSensor;
void startClockLinesADC(ADC_HandleTypeDef* hadc, ClockLineCurrentSensor* sensor,
		int8_t* linesId);
void stopClockLinesADC(ADC_HandleTypeDef* hadc);
void resetClockLineCurrentSensor(ClockLineCurrentSensor* sensor);
void sendCurrentSensorsToUART(UART_HandleTypeDef* huart, float* data, uint16_t CMD,
		int8_t* idToSend);
void filter(volatile uint16_t* adc_data, uint8_t size, float coeff, float* out);
#endif /* CURRENTSENSORS_H_ */
