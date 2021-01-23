#pragma once

#include<stdio.h>

#include "TimeHelpers.h"

#define NANOSECOND_DIVIDER 1000000000


uint32_t TicksToSubSecond(TIM_HandleTypeDef htim, uint32_t ticks, double fractionOfSecond)
{
	//First get how many times per second one tick happens.
	//This could be calculated once at start, but putting it here for clarity's sake.
	uint32_t ticksPerSecond = (HAL_RCC_GetPCLK1Freq() * 2) / (htim.Instance->PSC + 1);
	double incrementsPerTick = fractionOfSecond / (double)ticksPerSecond;

	return ticks * incrementsPerTick;
}

uint32_t ReadCurrentTicks(TIM_HandleTypeDef htim)
{
	return htim.Instance->CNT;
}

void ResetClock(TIM_HandleTypeDef htim)
{
	htim.Instance->CNT = 0;
}