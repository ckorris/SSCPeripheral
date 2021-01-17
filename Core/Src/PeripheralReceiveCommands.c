#include <stdio.h>
#include "PeripheralReceiveCommands.h"


void ReceiveSampleParamsCommand(I2C_HandleTypeDef *hi2c, sampleParams *outParams, int *outSetState)
{
	uint16_t packetSize = CommandTypeBufferSize(SendSampleParams);
	uint8_t packetBuf[packetSize];

	//Start a blocking listen for the packet.
	HAL_I2C_Slave_Receive(hi2c, packetBuf, packetSize, 5); //1000 is arbitrary.

	//Build a packet out of what we received.
	//sampleParams* packet = (sampleParams*)&packetBuf;
	outParams = (sampleParams*)&packetBuf;
	outSetState = 1;
}

void ReceiveBeginSamplingCommand(sampleParams *params) //TODO: Will need more params for DMA and whatnot.
{

}

void ReceiveCheckFinishedCommand(I2C_HandleTypeDef *hi2c); //TODO: Will need more params for flags.
{

}

void ReceiveRequestDataCommand(I2C_HandleTypeDef *hi2c, sampleParams *params); //TODO: Will need flags to make sure it's ready.
{

}

void ReceiveResetCommand(); //TODO: Will need more params for flags.
{

}



