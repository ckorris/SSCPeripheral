#pragma once

#include<stdio.h>
#include "stm32f7xx_hal.h"
#include "I2CNetworkCommon.h"


void SendFinishedStatusCommand(I2C_HandleTypeDef *hi2c, int isFinished);

void SendSampleHeaderCommand(I2C_HandleTypeDef *hi2c, samplePacketHeader *packetHeader);

void SendSampleDataCommand(I2C_HandleTypeDef *hi2c, uint16_t* data, uint16_t bufferSize);
