/*
 * currentsensors.c
 *
 *  Created on: Aug 19, 2019
 *      Author: Mikhail Sinelinikov 
 *      E-Mail: msin87@yandex.ru
 */

#include "currentsensors.h"
#include "message.h"
const uint32_t linesADCChannels[12] =
{
ADC_CHANNEL_0,
ADC_CHANNEL_1,
ADC_CHANNEL_2,
ADC_CHANNEL_3,
ADC_CHANNEL_5,
ADC_CHANNEL_6,
ADC_CHANNEL_7,
ADC_CHANNEL_8,
ADC_CHANNEL_9,
ADC_CHANNEL_10,
ADC_CHANNEL_11,
ADC_CHANNEL_12 };
void reinitADC(ADC_HandleTypeDef* hadc, int8_t* linesId, uint8_t totalLines)
{
	ADC_ChannelConfTypeDef sConfig =
	{ 0 };
	ADC_HandleTypeDef hadc1;
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 12;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		for (;;)
		{

		}
	}
	for (uint8_t i = 0; i < totalLines; i++)
	{
		if (linesId[i] < 0)
			break;
		sConfig.Channel = linesADCChannels[linesId[i]];
		sConfig.Rank = i++;
		sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
	HAL_ADC_Start_DMA(hadc, (uint32_t*)sensor->adc_data, sensor->totalLines);

}

void filter(volatile uint8_t* adc_data, uint8_t size, float coeff, float* out)
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	filter(clockLineCurrentSensor.adc_data, clockLineCurrentSensor.totalLines,
			0.01, clockLineCurrentSensor.filteredData);
}

