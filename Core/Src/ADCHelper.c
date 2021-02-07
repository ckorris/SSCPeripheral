

#include "ADCHelper.h"
#include "stm32f7xx_hal.h"

#include <stdlib.h>

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

uint16_t** transmitBuffers_ADC1; //Filled with data from adc1_buf as it completes, and remains until cleared.
uint16_t** transmitBuffers_ADC2;
uint16_t** transmitBuffers_ADC3;



//Not accessible.
int hasAllocatedBuffersAndEndTimes = 0;
int previousCycleCount = 0;

//Forward declaration.
void AllocateBuffersAndEndTimes(enum ADCNumber adcNum, int cycleCount);
void DeallocateBuffersAndEndTimes(enum ADCNumber adcNum, int cycleCount);

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

int DeviceCount(enum ADCNumber adcNum)
{
	switch(adcNum)
	{
	case ADC_1:
		return DEVICE_COUNT_ADC1;
	case ADC_2:
		return DEVICE_COUNT_ADC2;
	case ADC_3:
		return DEVICE_COUNT_ADC3;
	default:
		return 0;
	}
}

int BufferSize(enum ADCNumber adcNum)
{
	switch(adcNum)
		{
		case ADC_1:
			return ADC1_BUFFER_LENGTH;
		case ADC_2:
			return ADC2_BUFFER_LENGTH;
		case ADC_3:
			return ADC3_BUFFER_LENGTH;
		default:
			return 0;
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

int AreAllADCsFinishedSampling()
{
	if(DEVICE_COUNT_ADC1 > 0 && hasFinishedSampling_ADC1 == 0)
	{
		return 0;
	}
	if(DEVICE_COUNT_ADC2 > 0 && hasFinishedSampling_ADC2 == 0)
	{
		return 0;
	}
	if(DEVICE_COUNT_ADC3 > 0 && hasFinishedSampling_ADC3 == 0)
	{
		return 0;
	}

	return 1;
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

uint16_t*** TransmitBuffer(enum ADCNumber adcNum)
{
	switch(adcNum)
	{
	case ADC_1:
		return &transmitBuffers_ADC1;
	case ADC_2:
		return &transmitBuffers_ADC2;
	case ADC_3:
		return &transmitBuffers_ADC3;
	default:
		return 0;
	}
}


void AllocateAllBuffersAndEndTimes(int cycleCount)
{
	if(hasAllocatedBuffersAndEndTimes == 1)
	{
		DeallocateBuffersAndEndTimes(ADC_1, previousCycleCount);
		DeallocateBuffersAndEndTimes(ADC_2, previousCycleCount);
		DeallocateBuffersAndEndTimes(ADC_3, previousCycleCount);
	}

	AllocateBuffersAndEndTimes(ADC_1, cycleCount);
	AllocateBuffersAndEndTimes(ADC_2, cycleCount);
	AllocateBuffersAndEndTimes(ADC_3, cycleCount);

	hasAllocatedBuffersAndEndTimes = 1;
	previousCycleCount = cycleCount;
}

//Privates.

void AllocateBuffersAndEndTimes(enum ADCNumber adcNum, int cycleCount)
{
	*TransmitBuffer(adcNum) = calloc(cycleCount, sizeof(uint16_t*));
	for(int i = 0; i < cycleCount; i++)
	{
		(*TransmitBuffer(adcNum))[i] = calloc(BufferSize(adcNum), sizeof(uint16_t));
	}
	*CycleEndTimes(adcNum) = calloc(cycleCount, sizeof(uint32_t));
}

void DeallocateBuffersAndEndTimes(enum ADCNumber adcNum, int cycleCount)
{
	for(int i = 0; i < cycleCount; i++)
	{
		free((*TransmitBuffer(adcNum))[i]);
	}

	free(*TransmitBuffer(adcNum));
	free(*CycleEndTimes(adcNum));
}







