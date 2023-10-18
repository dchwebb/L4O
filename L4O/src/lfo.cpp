#include "lfo.h"

#include <cmath>
#include <cstring>

LFOs lfos;


void LFOs::CheckFadeInBtn()
{
	// check if fade-in button has been pressed with debounce
	if ((GPIOF->IDR & GPIO_IDR_ID0) == 0) {
		if (SysTickVal > fadeInBtnUp + 100 && !fadeInBtnDown) {
			fadeInBtnDown = true;
			settings.fadeInSpeed = !settings.fadeInSpeed;
			if (settings.fadeInSpeed) {
				GPIOC->ODR |= GPIO_ODR_OD13;
			} else {
				GPIOC->ODR &= ~GPIO_ODR_OD13;
			}
		}
	} else if (fadeInBtnDown) {
		fadeInBtnUp = SysTickVal;
		fadeInBtnDown = false;
	}
}


void LFOs::calcLFOs()
{
	CheckFadeInBtn();				// check if fade-in speed button has been pressed

	for (uint8_t i = 0; i < 4; ++i) {
		// Calculate lfo speed spread
		uint32_t lfoSpread = static_cast<uint32_t>(static_cast<float>(adc.spread) * i * 3);
		lfo[i].calcLFO(lfoSpread);
	}
}


void LFO::calcLFO(uint32_t spread)
{
	uint16_t speed = adc.speed;

	if (adc.fadeIn > 10) {
		if ((gatePort->IDR & (1 << gatePin)) != 0) {			// Gate on

			currentLevel = 1.0f - (1.0f - currentLevel) * (lfos.settings.fadeInRate + (float)adc.fadeIn * lfos.fadeInScale);
		} else {
			currentLevel = 0.0f;
		}
		if (lfos.settings.fadeInSpeed) {
			speed *= currentLevel;
		}

	} else {
		currentLevel = 1.0f;
	}

	lfoCosPos += (speed + 20) * 200 + spread;
	*outputChn = static_cast<uint32_t>(CordicCos(lfoCosPos) * adc.level * currentLevel);			// Will output value from 0 - 4095
}


float LFO::CordicCos(uint32_t pos)
{

	CORDIC->CSR = (0 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cosine, 1: Sine, 2: Phase, 3: Modulus, 4: Arctangent, 5: Hyperbolic cosine, 6: Hyperbolic sine, 7: Arctanh, 8: Natural logarithm, 9: Square Root
			(5 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 5 (gives 5 * 4 = 20 iterations in 5 clock cycles)

	CORDIC->WDATA = pos;		// This should be a value between -1 and 1 in q1.31 format, relating to -pi to +pi

	return ((static_cast<float>(static_cast<int32_t>(CORDIC->RDATA)) / 4294967296.0f) + 0.5f);
}


float LFO::CordicExp(float x)
{
	// use CORDIC sinh function and generate e^x = sinh(x) + cosh(x)
	CORDIC->CSR = (6 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cos, 1: Sin, 2: Phase, 3: Modulus, 4: Arctan, 5: cosh, 6: sinh, 7: Arctanh, 8: ln, 9: Square Root
			CORDIC_CSR_SCALE_0 |					// Must be 1 for sinh
			CORDIC_CSR_NRES |						// 2 Results as we need both sinh and cosh
			(6 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 6 (gives 6 * 4 = 24 iterations in 6 clock cycles)

	// convert float to q1_31 format scaling x by 1/2 at the same time
	int q31;
	if (x < -1.118f) {
		q31 = (int)((x + 1.0f) * 1073741824.0f);	// as range of x is limited to -1.118 to +1.118 reduce exponent by e^-1 (note that only values from around -1.75 to 0 used in this mechanism)
	} else {
		q31 = (int)(x * 1073741824.0f);
	}

	//volatile float etest = std::exp(x);

	CORDIC->WDATA = q31;

	// convert values back to floats scaling by * 2 at the same time
	float sinh = (float)((int)CORDIC->RDATA) / 1073741824.0f;	// command will block until RDATA is ready - no need to poll RRDY flag
	float cosh = (float)((int)CORDIC->RDATA) / 1073741824.0f;
	float res = sinh + cosh;
	if (x < -1.118f) {
		return res * 0.3678794411714f;				// multiply by e^-1 to correct range offset
	} else {
		return res;
	}
}


void LFOs::VerifyConfig()
{
	// Verify settings and update as required

	if (lfos.settings.fadeInRate < 0.1f || lfos.settings.fadeInRate > 0.99999999f) {
		lfos.settings.fadeInRate = 0.9996f;
	}
	lfos.fadeInScale = (1.0f - lfos.settings.fadeInRate) / 4096.0f;

	if (lfos.settings.fadeInSpeed) {
		GPIOC->ODR |= GPIO_ODR_OD13;
	} else {
		GPIOC->ODR &= ~GPIO_ODR_OD13;
	}
}
