#include "initialisation.h"
#include "USB.h"
#include <cmath>
#include "lfo.h"

volatile uint32_t SysTickVal;
volatile ADC adc;

extern "C" {
#include "interrupts.h"
}

Config config{&lfos.configSaver};		// Initialise configuration to handle saving and restoring lfo settings

extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();						// Activates floating point coprocessor and resets clock
	InitClocks();						// Configure the clock and PLL
	InitSysTick();
	InitDAC();
	InitIO();
	InitOutputTimer();
	InitADC(reinterpret_cast<volatile uint16_t*>(&adc));
	InitCordic();
	config.RestoreConfig();
	usb.InitUSB();

	while (1) {
		usb.cdc.ProcessCommand();		// Check for incoming CDC commands
	}
}

