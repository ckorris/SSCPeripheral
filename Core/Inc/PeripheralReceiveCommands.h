#pragma once

#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "I2CNetworkCommon.h"

void ReceiveSampleParamsCommand(I2C_HandleTypeDef *hi2c, sampleParams *outParams, int *outSetState); //Outs are for setting main file values.

void ReceiveBeginSamplingCommand(ADC_HandleTypeDef* hadc, uint16_t** transmitBuffers, uint32_t bufferLength, int *finishedFlag, int *currentCycleCount);

void ReceiveCheckFinishedCommand(I2C_HandleTypeDef *hi2c, int isFinished);

void ReceiveRequestTotalPacketCount(I2C_HandleTypeDef *hi2c, uint16_t count);

void ReceiveRequestSampleHeaderCommand(I2C_HandleTypeDef *hi2c, samplePacketHeader** processedHeaders);

void ReceiveRequestDataCommand(I2C_HandleTypeDef *hi2c, uint16_t** processedSamples, int samplesPerDevice);


;
