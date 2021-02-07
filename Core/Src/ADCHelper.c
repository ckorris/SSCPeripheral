

#include "ADCHelper.h"
#include "stm32f7xx_hal.h"

int isInit = 0;

ADC_HandleTypeDef* _hadc1;
ADC_HandleTypeDef* _hadc2;
ADC_HandleTypeDef* _hadc3;

int currentCycleADC1 = 0;
int currentCycleADC2 = 0;
int currentCycleADC3 = 0;

int hasFinishedSampling_ADC1 = 0;
int hasFinishedSampling_ADC2 = 0;
int hasFinishedSampling_ADC3 = 0;

uint32_t firstCycleStartTicks_ADC1; //Clock at the moment we started the ADCs. May be different than startTicks if a delay was requested, plus setup time.
uint32_t firstCycleStartTicks_ADC2;
uint32_t firstCycleStartTicks_ADC3;

uint32_t* cycleEndTimes_ADC1;
uint32_t* cycleEndTimes_ADC2;
uint32_t* cycleEndTimes_ADC3;

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

int* HasFinishedSampling(enum ADCNumber adcNum)
{
	switch(adcNum)
	{
	case ADC_1:
		return &hasFinishedSampling_ADC1;
	case ADC_2:
		return &hasFinishedSampling_ADC2;
	case ADC_3:
		return &hasFinishedSampling_ADC3;
	default:
		return 0;
	}
}

uint32_t* FirstCycleStartTicks(enum ADCNumber adcNum)
{
	switch(adcNum)
	{
	case ADC_1:
		return &firstCycleStartTicks_ADC1;
	case ADC_2:
		return &firstCycleStartTicks_ADC2;
	case ADC_3:
		return &firstCycleStartTicks_ADC3;
	default:
		return 0;
	}
}

uint32_t** CycleEndTimes(enum ADCNumber adcNum)
{
	switch(adcNum)
	{
	case ADC_1:
		return &cycleEndTimes_ADC1;
	case ADC_2:
		return &cycleEndTimes_ADC2;
	case ADC_3:
		return &cycleEndTimes_ADC3;
	default:
		return 0;
	}
}

