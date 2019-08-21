/*
 * currentsensors.c
 *
 *  Created on: Aug 19, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#include "currentsensors.h"
#include "clocklines.h"
#include "message.h"
static const uint8_t linesADCChannels[12] =
{
0,
1,
2,
3,
5,
6,
7,
8,
9,
10,
11,
12 };
void reinitADC(ADC_HandleTypeDef* hadc, int8_t* linesId, uint8_t totalLines)
{

	ADC1->CR2 &= ~ADC_CR2_ADON; 			//turn off ADC
	TIM2->CR1 &= ~TIM_CR1_CEN;  			//turn off TIM2
	DMA2_Stream0->CR &= ~DMA_SxCR_EN; 		//turn off DMA2 Stream0 (ADC1)
	DMA2->LIFCR |= 0x3F						//clear interrupt flags for stream 0
	TIM2->CNT = 0;							//reset TIM2 counter;
	ADC1->SQR1 ^=ADC1->SQR1;	  			//reset SQR1 register
	ADC1->SQR2 ^=ADC1->SQR2;	  			//reset SQR2 register
	ADC1->SQR3 ^=ADC1->SQR3;	  			//reset SQR3 register
	ADC1->SQR1 |= totalLines << 20;			//set regular channel sequence length to {totalLines}
	for (uint8_t i=0; i < totalLines; i++)
	{
		if (i<6){
			ADC1->SQR3 |= linesADCChannels[linesId[i]]<<(5*i);		//writing ADC channel to SQR3
		}
		else if (i<12){
			ADC1->SQR2 |= linesADCChannels[linesId[i]]<<(5*(i-6));  //writing ADC channel to SQR2
		}
		else
		{
			ADC1->SQR1 |= linesADCChannels[linesId[i]]<<(5*(i-12)); //writing ADC channel to SQR1
		}
	}
	DMA2_Stream0->NDTR = totalLines*2;								//set bytes to transmit by DMA. ADC channel has a resolution of 12 bits = 2 byte per channel

	for (uint8_t i = 0; i < totalLines; i++)
	{
		if (linesId[i] < 0)
			break;
		sConfig.Channel = linesADCChannels[linesId[i]];
		sConfig.Rank = i + 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
		{
			for (;;)
			{

			}
		}
	}
}
uint8_t renewId(ClockLineCurrentSensor* sensor, int8_t* linesId)
{
	uint8_t i = 0;
	for (i = 0; i < 12; i++)
	{
		if (linesId[i] < 0)
			break;
		if (linesId[i] != sensor->storedLinesId[i])
		{
			sensor->storedLinesId[i] = linesId[i];
		}
	}
	return i;
}
void startClockLinesADC(ADC_HandleTypeDef* hadc, ClockLineCurrentSensor* sensor,
		int8_t* linesId)
{
	uint8_t totalLines = renewId(sensor, linesId);
	if (totalLines)
	{
		reinitADC(hadc, sensor->storedLinesId, totalLines);
		sensor->totalLines = totalLines;
	}
	HAL_ADC_Start(hadc);
	HAL_ADC_Start_DMA(hadc, (uint32_t*) sensor->adc_data, sensor->totalLines);
}

void filter(volatile uint16_t* adc_data, uint8_t size, float coeff, float* out)
{

	float a = 1 - coeff;
	for (int i = 0; i < size; i++)
	{
		if (out[i] == 0)
		{
			out[i] = adc_data[i];
		}
		else
		{
			out[i] = out[i] + a * (adc_data[i] - out[i]);
		}
	}
}
void resetClockLineCurrentSensor(ClockLineCurrentSensor* sensor)
{
	for (uint8_t i = 0; i < 12; i++)
	{
		sensor->filteredData[i] = 0;
	}
}


