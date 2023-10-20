#pragma once

#include "stm32g4xx.h"
#include <algorithm>

extern volatile uint32_t SysTickVal;

#define SYSTICK 1000						// 1ms
#define ADC_BUFFER_LENGTH 4

void InitClocks();
void InitSysTick();
void InitDAC();
void InitIO();
void InitOutputTimer();
void InitADC(volatile uint16_t* ADC_array);
void InitCordic();
void InitPWMTimer();

