#include <stdio.h>
#include <math.h>
#include "PeripheralReceiveCommands.h"

#include <stdlib.h>
#include "PeripheralDispatchCommands.h"
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

void ReceiveBeginSamplingCommand(ADC_HandleTypeDef* hadc, uint32_t* adcBuffer, uint32_t bufferLength, int *finishedFlag, int *currentCycleCount)
{
	/*
	if(startedFlag == 1)
	{
		return; //TODO: Throw error.
	}
	*/

	*currentCycleCount = 0;
	*finishedFlag = 0;

	HAL_ADC_Start_DMA(hadc, adcBuffer, bufferLength);
}

void ReceiveCheckFinishedCommand(I2C_HandleTypeDef *hi2c, int isFinished)
{
	SendFinishedStatusCommand(hi2c, isFinished);
}

void ReceiveRequestTotalPacketCount(I2C_HandleTypeDef *hi2c, uint16_t count)
{
	SendTotalPacketCount(hi2c, count);
}

//startOffset is nanoseconds since being told to sample that it actually started sampling (to factor in a delay).
//cycleEndTimes should point to an array that gives the times (since being told to start) that each cycle ended.
void ReceiveRequestSampleHeaderCommand(I2C_HandleTypeDef *hi2c, sampleParams params, int deviceCount, int samplesPerDevice, TIM_HandleTypeDef htim, uint32_t startOffsetTicks, uint32_t* cycleEndTimes)
{
	//Wait to be told what sample ID that's being requested.
	uint8_t idBuf[1];
	HAL_I2C_Slave_Receive(hi2c, idBuf, 1, 50); //50 is arbitrary.
	int sampleID = (int)idBuf[0];

	//Make a header out of known values to send back to the host.
	int cycle = floor(sampleID / (float)deviceCount);
	int deviceID = sampleID % deviceCount;

	//int samplesPerDevice = sampleCount / deviceCount; //Will be slightly off if the buffer size isn't a multiple of the device count for some reason.

	uint32_t startTimeTicks = cycle == 0 ? startOffsetTicks : cycleEndTimes[cycle - 1];
	uint32_t startTimeUs = TicksToSubSecond(htim, startTimeTicks, MICROSECOND_DIVIDER);
	uint32_t endTimeUs = TicksToSubSecond(htim, cycleEndTimes[cycle], MICROSECOND_DIVIDER);


	samplePacketHeader header;
	header.DeviceID = deviceID;
	header.SampleID = sampleID;
	header.startTimeUs = startTimeUs;
	header.endTimeUs = endTimeUs;
	header.AnalogInPinCount = deviceCount;
	header.SampleCount = samplesPerDevice;

	SendSampleHeaderCommand(hi2c, &header);

	int x = 0; //For debugging breakpoints.
	x++;

}

void ReceiveRequestDataCommand(I2C_HandleTypeDef *hi2c, sampleParams params, int deviceCount, int samplesPerDevice, uint16_t** transmitBuffers)
{
	//Wait to be told what sample ID that's being requested.
	uint8_t idBuf[1];
	HAL_I2C_Slave_Receive(hi2c, idBuf, 1, 5); //5 is arbitrary.
	int sampleID = (int)idBuf[0];

	//Calculate the cycle number and device id, given the sample nuumber and knowing how they're indexed.
	int cycle = floor(sampleID / (float)deviceCount);
	int deviceID = sampleID % deviceCount;


	//Grab all the samples for the given ID from that index.
	uint16_t* cycleSamples = transmitBuffers[cycle];
	//uint16_t deviceSamples[samplesPerDevice];
	uint16_t* deviceSamples = malloc(sizeof(uint16_t) * samplesPerDevice);

	for(int i = 0; i < samplesPerDevice; i ++)
	{
		//deviceSamples[i] = cycleSamples[i * deviceCount + deviceID];
		//deviceSamples[i] = 3400; //Just testing OOB.

		deviceSamples[i] = deviceID * 100; //TEST identifying ID. Can identify IDs on graph by values this way.

	}

	//SendSampleDataCommand(hi2c, &deviceSamples, samplesPerDevice);
	SendSampleDataCommand(hi2c, deviceSamples, sizeof(uint16_t) * samplesPerDevice);

	int x = 0;
	x++;

	free(deviceSamples);
}





