#pragma once

#include "initialisation.h"
#include "configManager.h"

struct ADC {
	uint16_t speed;
	uint16_t spread;
	uint16_t fadeIn;
	uint16_t level;
};

extern volatile ADC adc;

struct LFO {
public:
	LFO(volatile uint32_t* outputChn, GPIO_TypeDef* gatePort, uint8_t gatePin)
	 : outputChn{outputChn}, gatePort{gatePort}, gatePin{gatePin} {}

	void calcLFO(uint32_t spread);				// Sets the DAC level for LFO

private:
	float CordicCos(uint32_t pos);
	float CordicExp(float x);

	float              currentLevel = 0.0f;		// The current level of the envelope (held as a float for accuracy of calulculation)
	uint32_t           lfoCosPos = 0;			// Position of cordic cosine wave in q1.31 format

	// Hardware settings for each envelope (DAC Output, GPIO gate input)
	volatile uint32_t* outputChn;
	GPIO_TypeDef*      gatePort;
	uint8_t            gatePin;
};


struct LFOs {

public:
	void calcLFOs();							// Calls calculation on all contained envelopes
	static void VerifyConfig();					// Check config is valid (must be static in order to store pointer)

	struct {
		float fadeInRate = 0.9996f;
		bool fadeInSpeed = false;				// True when speed fade in activated
	} settings;

	float fadeInScale = (1.0f - settings.fadeInRate) / 4096.0f;

	ConfigSaver configSaver = {
		.settingsAddress = &settings,
		.settingsSize = sizeof(settings),
		.validateSettings = &VerifyConfig
	};

private:
	void CheckFadeInBtn();

	LFO lfo[4] = {
			{&(DAC1->DHR12R1), GPIOB, 6},		// PA4 Env1
			{&(DAC1->DHR12R2), GPIOB, 5},		// PA5 Env2
			{&(DAC3->DHR12R2), GPIOB, 4},		// PB1 Env3
			{&(DAC3->DHR12R1), GPIOB, 3} 		// PA2 Env4
	};

	bool fadeInBtnDown = false;					// To manage debouncing
	uint32_t fadeInBtnUp = 0;					// Store systick time button released for debouncing
};

extern LFOs lfos;
