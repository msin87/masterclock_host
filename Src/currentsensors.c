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
#include "cmsis_os.h"
//static const uint8_t linesADCChannels[12] =
//{
//0,
//1,
//2,
//3,
//5,
//6,
//7,
//8,
//9,
//10,
//11,
//12 };
static const uint32_t linesADCChannels[12] =
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
void stopClockLinesADC(ADC_HandleTypeDef* hadc){
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0	;
	HAL_ADC_Stop_DMA(hadc);
}
void reinitADC(ADC_HandleTypeDef* hadc, int8_t* linesId, uint8_t totalLines)
{


//	ADC1->SQR1 ^=ADC1->SQR1;	  			//reset SQR1 register
//	ADC1->SQR2 ^=ADC1->SQR2;	  			//reset SQR2 register
//	ADC1->SQR3 ^=ADC1->SQR3;	  			//reset SQR3 register
//	ADC1->CR2 |= ADC_CR2_DMA; 			 	//enable ADC1 DMA mode
//	ADC1->CR1 |= ADC_CR1_EOCIE;				//enable ADC1 EOC interrupt
//	ADC1->SQR1 |= totalLines << 20;			//set regular channel sequence length to {totalLines}
//	for (uint8_t i=0; i < totalLines; i++)
//	{
//		if (i<6){
//			ADC1->SQR3 |= linesADCChannels[linesId[i]]<<(5*i);		//writing ADC channel to SQR3
//		}
//		else if (i<12){
//			ADC1->SQR2 |= linesADCChannels[linesId[i]]<<(5*(i-6));  //writing ADC channel to SQR2
//		}
//		else
//		{
//			ADC1->SQR1 |= linesADCChannels[linesId[i]]<<(5*(i-12)); //writing ADC channel to SQR1
//		}
//	}
//	DMA2_Stream0->M0AR = (uint32_t) clockLineCurrentSensor.adc_data;
//	DMA2_Stream0->NDTR = totalLines*2;								//set bytes to transmit by DMA. ADC channel has a resolution of 12 bits = 2 byte per channel
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0	;
	ADC_ChannelConfTypeDef sConfig =
		{ 0 };

		HAL_ADC_Stop(hadc);
		HAL_ADC_Stop_DMA(hadc);
		HAL_ADC_DeInit(hadc);
		hadc->Instance = ADC1;
		hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
		hadc->Init.Resolution = ADC_RESOLUTION_12B;
		hadc->Init.ScanConvMode = ENABLE;
		hadc->Init.ContinuousConvMode = ENABLE;
		hadc->Init.DiscontinuousConvMode = DISABLE;
		hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
		hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
		hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc->Init.NbrOfConversion = totalLines;
		hadc->Init.DMAContinuousRequests = ENABLE;
		hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		if (HAL_ADC_Init(hadc) != HAL_OK)
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
			sConfig.Rank = i + 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;;
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
	clockLineCurrentSensor.filterCoeff=0.001;
	if (totalLines)
	{
		//STOP_TIM_DMA_ADC();
		reinitADC(hadc, sensor->storedLinesId, totalLines);
		sensor->totalLines = totalLines;
	}
	//START_TIM_DMA_ADC();
	HAL_ADC_Start_DMA(hadc, (uint32_t*) sensor->adc_data, sensor->totalLines);
	TIM2->CR1 |= TIM_CR1_CEN;
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
void sendCurrentSensorsToUART(UART_HandleTypeDef* huart, float* data, uint16_t CMD,
		int8_t* idToSend)
{
	Message message;
	uint8_t frame[32] =
	{ 0 };
	uint8_t j = 0;
	messageInit(&message);
	message.cmd = CMD;
	for (uint8_t i = 0; i < 12; i++)
	{
		if (idToSend[i] < 0)
			break;
		if (clockLines[idToSend[i]].counter==0) continue;
		message.dataArray[j] = (uint16_t)data[i];
		j++;
		message.idArray[i]= idToSend[i];
	}

	messageToFrame(&message, frame);
	while (HAL_UART_GetState(huart)==HAL_UART_STATE_BUSY_TX)
	{
		osDelay(1);
	}
	HAL_UART_Transmit(huart, frame, 32, 100);
}


