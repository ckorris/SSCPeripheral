

#include "stm32f7xx_hal.h"

enum ADCNumber{BadValue = -1, ADC_1 = 0, ADC_2 = 1, ADC_3 = 2};

void InitADCHelper(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3);

enum ADCNumber GetADCEnumVal(ADC_HandleTypeDef* hadc);

int* CurrentCycle(enum ADCNumber adcNum);

int* HasFinishedSampling(enum ADCNumber adcNum);
