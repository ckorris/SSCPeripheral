#pragma once

#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "I2CNetworkCommon.h"

void ReceiveSampleParamsCommand(I2C_HandleTypeDef *hi2c, sampleParams *outParams, int *outSetState); //Outs are for setting main file values.

void ReceiveBeginSamplingCommand(ADC_HandleTypeDef* hadc, uint32_t* adcBuffer, uint32_t bufferLength, int *finishedFlag, int *currentCycleCount);

void ReceiveCheckFinishedCommand(I2C_HandleTypeDef *hi2c, int isFinished);

void ReceiveRequestTotalPacketCount(I2C_HandleTypeDef *hi2c, uint16_t count);

void ReceiveRequestSampleHeaderCommand(I2C_HandleTypeDef *hi2c, sampleParams params, int deviceCount, int samplesPerDevice, TIM_HandleTypeDef htim, uint32_t startOffsetTicks, uint32_t* cycleEndTimes);

void ReceiveRequestDataCommand(I2C_HandleTypeDef *hi2c, sampleParams params, int deviceCount, int samplesPerDevice, uint16_t** transmitBuffers);


;
