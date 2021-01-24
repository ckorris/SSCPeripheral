#include<stdio.h>

#include "PeripheralDispatchCommands.h"
#include "I2CNetworkCommon.h"


void SendFinishedStatusCommand(I2C_HandleTypeDef *hi2c, int isFinished)
{
	uint8_t finishedStateBuffer[1] = { (uint8_t)isFinished };
	HAL_I2C_Slave_Transmit(hi2c, finishedStateBuffer, 1, 50); //Timeout val arbitrary.
}

void SendSampleHeaderCommand(I2C_HandleTypeDef *hi2c, samplePacketHeader *packetHeader)
{
	HAL_I2C_Slave_Transmit(hi2c, (uint8_t*)packetHeader, sizeof(samplePacketHeader), 50); //Timeout val arbitrary.
}

void SendSampleDataCommand(I2C_HandleTypeDef *hi2c, uint16_t* data, uint16_t bufferSize)
{
	HAL_I2C_Slave_Transmit(hi2c, (uint8_t*)data, bufferSize, 50);
}
