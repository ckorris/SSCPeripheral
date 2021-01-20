#include <stdio.h>
#include "PeripheralReceiveCommands.h"
#include "I2CNetworkCommon.h"


void ReceiveSampleParamsCommand(I2C_HandleTypeDef *hi2c, sampleParams *outParams, int *outSetState)
{
	uint16_t packetSize = CommandTypeBufferSize(SendSampleParams);
	uint8_t packetBuf[packetSize];

	//Start a blocking listen for the packet.
	HAL_I2C_Slave_Receive(hi2c, packetBuf, packetSize, 5); //1000 is arbitrary.

	//Build a packet out of what we received.
	//sampleParams* packet = (sampleParams*)&packetBuf;
	//outParams = (sampleParams*)&packetBuf;
	sampleParams newParams = *((sampleParams*)&packetBuf);
	*outParams = newParams;
	*outSetState = 1;
}

void ReceiveBeginSamplingCommand(ADC_HandleTypeDef* hadc, uint32_t* adcBuffer, sampleParams params, uint16_t** transferBuffers, int *startedFlag, int *currentCycleCount)
{
	/*
	if(startedFlag == 1)
	{
		return; //TODO: Throw error.
	}
	*/

	//TEST: Commenting to do elsewhere.
	//uint32_t newADCBuffer[params.BufferSize];
	//adcBuffer = newADCBuffer;

	*currentCycleCount = 0;

	for(int i = 0; i < params.CycleCount; i++)
	{
		uint16_t newTransferBuffer[params.BufferSize];
		//uint16_t* newTransferBuffer =
		transferBuffers[i] = (uint16_t*)newTransferBuffer;
	}

	//HAL_ADC_Start_DMA(hadc, adcBuffer, params->BufferSize); //Gonna try moving to while loop.

	*startedFlag = 1;
}

void ReceiveCheckFinishedCommand(I2C_HandleTypeDef *hi2c) //TODO: Will need more params for flags.
{

}

void ReceiveRequestDataCommand(I2C_HandleTypeDef *hi2c, sampleParams params) //TODO: Will need flags to make sure it's ready.
{

}

void ReceiveResetCommand() //TODO: Will need more params for flags.
{

}



