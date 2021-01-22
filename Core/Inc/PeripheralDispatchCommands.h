#pragma once

#include<stdio.h>
#include "stm32f7xx_hal.h"
#include "I2CNetworkCommon.h"


void SendFinishedStatusCommand(I2C_HandleTypeDef *hi2c, int isFinished);

void SendSampleDataCommand(I2C_HandleTypeDef *hi2c, sampleParams *params, uint16_t** transmitBuffers);
