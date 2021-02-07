

#include "ADCHelper.h"
#include "stm32f7xx_hal.h"

int isInit = 0;

ADC_HandleTypeDef* _hadc1;
ADC_HandleTypeDef* _hadc2;
ADC_HandleTypeDef* _hadc3;

int currentCycleADC1 = 0;
int currentCycleADC2 = 0;
int currentCycleADC3 = 0;


void InitADCHelper(ADC_HandleTypeDef* adc1, ADC_HandleTypeDef* adc2, ADC_HandleTypeDef* adc3)
{
	_hadc1 = adc1;
	_hadc2 = adc2;
	_hadc3 = adc3;

	isInit = 1;
}

enum ADCNumber GetADCEnumVal(ADC_HandleTypeDef* hadc)
{
	if(hadc == _hadc1)
	{
		return ADC_1;
	}
	else if (hadc == _hadc2)
	{
		return ADC_2;
	}
	else if (hadc == _hadc3)
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
		return &currentCycleADC1;
	case ADC_2:
		return &currentCycleADC2;
	case ADC_3:
		return &currentCycleADC3;
	default:
		return 0;
	}
}

