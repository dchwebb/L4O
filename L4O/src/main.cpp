#include "initialisation.h"
#include "usb.h"
#include "configManager.h"
#include "SerialHandler.h"
#include <cmath>
#include "lfo.h"

volatile uint32_t SysTickVal;
volatile ADC adc;

USBHandler usb;
SerialHandler serial(usb);

extern "C" {
#include "interrupts.h"
}


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();							// Activates floating point coprocessor and resets clock
	SystemClock_Config();					// Configure the clock and PLL
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitSysTick();
	InitDAC();
	InitIO();
	InitEnvTimer();
	InitADC(reinterpret_cast<volatile uint16_t*>(&adc));
	InitCordic();
	configManager.RestoreConfig();
	usb.InitUSB();

	while (1) {
		serial.Command();					// Check for incoming CDC commands
	}
}

