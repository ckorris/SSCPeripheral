/*

#include "ADCHelper.h"
#include "stm32f7xx_hal.h"

int isInit = 0;

ADC_HandleTypeDef* hadc1;
ADC_HandleTypeDef* hadc2;
ADC_HandleTypeDef* hadc3;

int currentCycle_ADC1 = 0;
int currentCycle_ADC2 = 0;
int currentCycle_ADC3 = 0;


void InitADCHelper(ADC_HandleTypeDef* adc1, ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3)
{
	hadc1 = adc1;
	hadc2 = adc2;
	hadc3 = adc3;

	isInit = 1;
}

enum ADCNumber GetADCEnumVal(ADC_HandleTypeDef* hadc)
{
	if(hadc == hadc1)
	{
		return ADC_1;
	}
	else if (hadc == hadc2)
	{
		return ADC_2;
	}
	else if (hadc == hadc3)
	{
		return ADC_3;
	}
	else
	{
		return BadValue;
	}
}

int* CurrentCycle(enum ADCNumber adcNum)
{
	switch(adcNum)
	{
	case ADC_1:
		return &currentCycle_ADC1;
	case ADC_2:
		return &currentCycle_ADC2;
	case ADC_3:
		return &currentCycle_ADC3;
	default:
		return 0;
	}
}
*/
