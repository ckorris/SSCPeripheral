#include <stdio.h>
#include <math.h>
#include "PeripheralReceiveCommands.h"
#include "I2CNetworkCommon.h"
#include "TimeHelpers.h"


void ReceiveSampleParamsCommand(I2C_HandleTypeDef *hi2c, sampleParams *outParams, int *outSetState)
{
	//uint16_t packetSize = CommandTypeBufferSize(SendSampleParams);
	//uint8_t packetBuf[packetSize];

	uint16_t packetSize = sizeof(sampleParams);
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
	//Currently doing most logic in main file for debugging. TODO: Fix that.
	*currentCycleCount = 0;

	*startedFlag = 1;
}

void ReceiveCheckFinishedCommand(I2C_HandleTypeDef *hi2c, int isFinished)
{
	SendFinishedStatusCommand(hi2c, isFinished);
}

//startOffset is nanoseconds since being told to sample that it actually started sampling (to factor in a delay).
//cycleEndTimes should point to an array that gives the times (since being told to start) that each cycle ended.
void ReceiveRequestSampleHeaderCommand(I2C_HandleTypeDef *hi2c, sampleParams params, TIM_HandleTypeDef htim, uint32_t startOffsetTicks, uint32_t* cycleEndTimes)
{
	uint8_t idBuf[1];
	HAL_I2C_Slave_Receive(hi2c, idBuf, 1, 5); //5 is arbitrary.
	int sampleID = (int)idBuf[0];

	//Make a header out of known values to send back to the host.
	int cycle = floor(sampleID / (float)params.DeviceCount);
	int deviceID = sampleID % params.DeviceCount;

	int samplesPerDevice = params.BufferSize / params.DeviceCount; //Will be slightly off if the buffer size isn't a multiple of the device count for some reason.

	uint32_t startTimeTicks = cycle == 0 ? startOffsetTicks : cycleEndTimes[cycle - 1];
	uint32_t startTimeUs = TicksToSubSecond(htim, startTimeTicks, NANOSECOND_DIVIDER);
	uint32_t endTimeUs = TicksToSubSecond(htim, cycleEndTimes[cycle], NANOSECOND_DIVIDER);

	samplePacketHeader header;
	header.DeviceID = deviceID;
	header.SampleID = sampleID;
	header.startTimeUs = startTimeUs;
	header.endTimeUs = endTimeUs;
	header.AnalogInPinCount = params.DeviceCount;
	header.SampleCount = samplesPerDevice;

	SendSampleHeaderCommand(hi2c, &header);
}

void ReceiveRequestSampleDataCommand(I2C_HandleTypeDef *hi2c, sampleParams params) //TODO: Will need flags to make sure it's ready.
{

}

void ReceiveResetCommand() //TODO: Will need more params for flags.
{

}



