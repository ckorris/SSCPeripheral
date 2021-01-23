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

	HAL_I2C_Slave_Transmit(hi2c, packetHeader, sizeof(samplePacketHeader), 10); //Timeout val arbitrary.
}
