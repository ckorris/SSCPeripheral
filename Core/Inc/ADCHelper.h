

#include "stm32f7xx_hal.h"

#define DEVICE_COUNT_ADC1 8
#define DEVICE_COUNT_ADC2 8
#define DEVICE_COUNT_ADC3 8

#define BUFFER_SIZE_PER_DEVICE 1024 //The buffer size is this times the total number of devices.



#define TOTAL_DEVICE_COUNT (DEVICE_COUNT_ADC1 + DEVICE_COUNT_ADC2 + DEVICE_COUNT_ADC3)

//How much space is allocated in the transfer buffers for ADC1, which tells ADC3 where to copy its values.
#define ADC1_BUFFER_LENGTH (DEVICE_COUNT_ADC1 * BUFFER_SIZE_PER_DEVICE)
#define ADC2_BUFFER_LENGTH (DEVICE_COUNT_ADC2 * BUFFER_SIZE_PER_DEVICE)
#define ADC3_BUFFER_LENGTH (DEVICE_COUNT_ADC3 * BUFFER_SIZE_PER_DEVICE)

enum ADCNumber{BadValue = -1, ADC_1 = 0, ADC_2 = 1, ADC_3 = 2};

void InitADCHelper(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3);

enum ADCNumber GetADCEnumVal(ADC_HandleTypeDef* hadc);

int DeviceCount(enum ADCNumber adcNum);

int BufferSize(enum ADCNumber adcNum);

int* CurrentCycle(enum ADCNumber adcNum);

int* HasFinishedSampling(enum ADCNumber adcNum);

int AreAllADCsFinishedSampling();

uint32_t* FirstCycleStartTicks(enum ADCNumber adcNum);

uint32_t** CycleEndTimes(enum ADCNumber adcNum);

uint16_t*** TransmitBuffer(enum ADCNumber adcNum); //IMMA THREE STAR GENERAL BOIIII

void AllocateAllBuffersAndEndTimes(int cycleCount);

void StartSampling();
