#pragma once

#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "I2CNetworkCommon.h"

void ReceiveSampleParamsCommand(I2C_HandleTypeDef *hi2c, sampleParams *outParams, int *outSetState); //Outs are for setting main file values.

void ReceiveBeginSamplingCommand(ADC_HandleTypeDef* hadc, uint32_t* adcBuffer, sampleParams params, uint16_t** transferBuffers, int *startedFlag, int *currentCycleCount); //TODO: Will need more params for DMA and whatnot.

void ReceiveCheckFinishedCommand(I2C_HandleTypeDef *hi2c); //TODO: Will need more params for flags.

void ReceiveRequestDataCommand(I2C_HandleTypeDef *hi2c, sampleParams params); //TODO: Will need flags to make sure it's ready.

void ReceiveResetCommand(); //TODO: Will need more params for flags.
;
