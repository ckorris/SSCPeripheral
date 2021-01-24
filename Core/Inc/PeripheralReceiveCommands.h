#pragma once

#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "I2CNetworkCommon.h"

void ReceiveSampleParamsCommand(I2C_HandleTypeDef *hi2c, sampleParams *outParams, int *outSetState); //Outs are for setting main file values.

void ReceiveBeginSamplingCommand(ADC_HandleTypeDef* hadc, uint32_t* adcBuffer, sampleParams params, uint16_t** transferBuffers, int *startedFlag, int *currentCycleCount); //TODO: Will need more params for DMA and whatnot.

void ReceiveCheckFinishedCommand(I2C_HandleTypeDef *hi2c, int isFinished);

void ReceiveRequestSampleHeaderCommand(I2C_HandleTypeDef *hi2c, sampleParams params, TIM_HandleTypeDef htim, uint32_t startOffsetTicks, uint32_t* cycleEndTimes);

void ReceiveRequestDataCommand(I2C_HandleTypeDef *hi2c, sampleParams params, uint16_t** transmitBuffers);


;
