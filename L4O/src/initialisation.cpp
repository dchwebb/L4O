#include "initialisation.h"

// Settings for HSI at 16MHz
// 16MHz / 4(M) * 85(N) / 2(R) = 170MHz
#define PLL_M 0b011		// 0000 = 1; 0001 = 2; 0010 = 3; 0011 = 4; 0100 = 5; 0101 = 6; 0110 = 7; 0111 = 8; 1000 = 9
#define PLL_N 85
#define PLL_R 0			//  00: PLLR = 2, 01: PLLR = 4, 10: PLLR = 6, 11: PLLR = 8
#define PLL_P 2


void SystemClock_Config(void) {
	// See page 236 for clock configuration
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		// SYSCFG + COMP + VREFBUF + OPAMP clock enable
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;		// Enable Power Control clock
	PWR->CR5 &= ~PWR_CR5_R1MODE;				// Select the Range 1 boost mode

	// Configure PLL
	RCC->PLLCFGR = (PLL_M << RCC_PLLCFGR_PLLM_Pos) |
				   (PLL_N << RCC_PLLCFGR_PLLN_Pos) |
				   (PLL_P << RCC_PLLCFGR_PLLPDIV_Pos) |
				   (PLL_R << RCC_PLLCFGR_PLLR_Pos) |
				   RCC_PLLCFGR_PLLSRC_HSI;
	RCC->CR |= RCC_CR_PLLON;					// Enable the main PLL
	RCC->PLLCFGR = RCC_PLLCFGR_PLLREN;			// Enable PLL R (drives AHB clock)
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);		// Wait till the main PLL is ready

	// Configure Flash prefetch and wait state. NB STM32G431 is a category 2 device (128KB flash in 1 bank)
	FLASH->ACR |= FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN;
	FLASH->ACR &= ~FLASH_ACR_LATENCY_1WS;

	// The system clock must be divided by 2 using the AHB prescaler before switching to a higher system frequency.
	RCC->CFGR |= RCC_CFGR_HPRE_DIV2;			// HCLK = SYSCLK / 2
	RCC->CFGR |= RCC_CFGR_SW_PLL;				// Select the main PLL as system clock source

	// Wait till the main PLL is used as system clock source
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);

	// Reset the AHB clock (previously divided by 2) and set APB clocks
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;			// PCLK1 = HCLK / 1 (APB1)
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;			// PCLK2 = HCLK / 1 (APB2)
}


void InitSysTick()
{
	SysTick_Config(SystemCoreClock / SYSTICK);		// gives 1ms
	NVIC_SetPriority(SysTick_IRQn, 0);
}

void InitDAC()
{
	// Configure 4 DAC outputs PA4 and PA5 are regular DAC1 buffered outputs; PA2 and PB1 are DAC3 via OpAmp1 and OpAmp3 (Manual p.789)

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;			// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;			// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;				// Enable DAC Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_DAC3EN;				// Enable DAC Clock

	DAC1->MCR &= ~DAC_MCR_MODE1_Msk;				// Set to normal mode: DAC channel1 is connected to external pin with Buffer enabled
	DAC1->CR |= DAC_CR_EN1;							// Enable DAC using PA4 (DAC_OUT1)

	DAC1->MCR &= ~DAC_MCR_MODE2_Msk;				// Set to normal mode: DAC channel2 is connected to external pin with Buffer enabled
	DAC1->CR |= DAC_CR_EN2;							// Enable DAC using PA5 (DAC_OUT2)

	// output triggered with DAC->DHR12R1 = x;

	// Opamp for DAC3 Channel 1: Follower configuration mode - output on PA2
	DAC3->MCR |= DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1;	// 011: DAC channel1 is connected to on chip peripherals with Buffer disabled
	DAC3->CR |= DAC_CR_EN1;							// Enable DAC

	OPAMP1->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP1->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC3_CH1  connected to OPAMP1 VINP input
	OPAMP1->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PA2)

	// Opamp for DAC3 Channel 2: Follower configuration mode - output on PB1
	// NB According to manual p.790 this should only work on category 3/4 devices and STM32G431 is category 2, but tested working
	DAC3->MCR |= DAC_MCR_MODE2_0 | DAC_MCR_MODE2_1;	// 011: DAC channel2 is connected to on chip peripherals with Buffer disabled
	DAC3->CR |= DAC_CR_EN2;							// Enable DAC

	OPAMP3->CSR |= OPAMP_CSR_VMSEL;					// 11: Opamp_out connected to OPAMPx_VINM input
	OPAMP3->CSR |= OPAMP_CSR_VPSEL;					// 11: DAC3_CH2  connected to OPAMP1 VINP input
	OPAMP3->CSR |= OPAMP_CSR_OPAMPxEN;				// Enable OpAmp: voltage on pin OPAMPx_VINP is buffered to pin OPAMPx_VOUT (PB1)

}


void InitIO()
{
	// MODER 00: Input mode, 01: General purpose output mode, 10: Alternate function mode, 11: Analog mode (reset state)

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;			// Reset and clock control - GPIO port A
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;			// Reset and clock control - GPIO port B
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;			// Reset and clock control - GPIO port C
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;			// Reset and clock control - GPIO port C

	// NB PB6 is used in USB Power delivery and by default has a pull down to ground - disable in the PWR register (datasheet p.60 note 5)
	PWR->CR3 |= PWR_CR3_UCPD_DBDIS;

	GPIOB->MODER &= ~GPIO_MODER_MODER6;				// PB6 gate 1 input
	GPIOB->MODER &= ~GPIO_MODER_MODER5;				// PB5 gate 2 input
	GPIOB->MODER &= ~GPIO_MODER_MODER4;				// PB4 gate 3 input
	GPIOB->MODER &= ~GPIO_MODER_MODER3;				// PB3 gate 4 input

	GPIOF->MODER &= ~GPIO_MODER_MODER0;				// PF0 Fade-in Button input
	GPIOF->PUPDR |= GPIO_PUPDR_PUPD0_0;				// 00: None; *01: Pull-up; 10: Pull-down

	GPIOC->MODER &= ~GPIO_MODER_MODE13_1;			// PC13 LED Out
}


//	Setup Timer 3 on an interrupt to trigger sample output
void InitEnvTimer()
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;			// Enable Timer 3
	TIM3->PSC = 34;									// Set prescaler
	TIM3->ARR = 103; 								// Set auto reload register - 170Mhz / 33 / 103 = ~50kHz

	TIM3->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 0);					// Lower is higher priority

	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->EGR |= TIM_EGR_UG;						//  Re-initializes counter and generates update of registers
}


void InitAdcPins(ADC_TypeDef* ADC_No, std::initializer_list<uint8_t> channels)
{
	uint8_t sequence = 1;

	for (auto channel: channels) {
		// Set conversion sequence to order ADC channels are passed to this function
		if (sequence < 5) {
			ADC_No->SQR1 |= channel << ((sequence) * 6);
		} else if (sequence < 10) {
			ADC_No->SQR2 |= channel << ((sequence - 5) * 6);
		} else if (sequence < 15) {
			ADC_No->SQR3 |= channel << ((sequence - 10) * 6);
		} else {
			ADC_No->SQR4 |= channel << ((sequence - 15) * 6);
		}

		// 000: 3 cycles, 001: 15 cycles, 010: 28 cycles, 011: 56 cycles, 100: 84 cycles, 101: 112 cycles, 110: 144 cycles, 111: 480 cycles
		if (channel < 10)
			ADC_No->SMPR1 |= 0b010 << (3 * channel);
		else
			ADC_No->SMPR2 |= 0b010 << (3 * (channel - 10));

		sequence++;
	}
}


void InitADC(volatile uint16_t* ADC_array)
{
	// Initialize Clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
	RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;				// 00: pll2_p_ck (default), 01: pll3_r_ck clock, 10: per_ck clock

	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Channel1->CCR |= DMA_CCR_MINC;				// Memory in increment mode
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Channel1->CCR |= DMA_CCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF1_Pos;		// clear all five interrupts for this stream

	DMAMUX1_Channel0->CCR |= 5; 					// DMA request MUX input 5 = ADC1 (See p.427)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF0; // Channel 1 Clear synchronization overrun event flag

	ADC1->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC1->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC1->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC12_COMMON->CCR |= ADC_CCR_CKMODE;			// adc_hclk/4 (Synchronous clock mode)
	ADC1->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC1->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC1->CFGR |= ADC_CFGR_DMACFG;					// 0: DMA One Shot Mode selected, 1: DMA Circular Mode selected
	ADC1->CFGR |= ADC_CFGR_DMAEN;					// Enable ADC DMA

	// For scan mode: set number of channels to be converted
	ADC1->SQR1 |= (ADC_BUFFER_LENGTH - 1);

	// Start calibration
	ADC1->CR &= ~ADC_CR_ADCALDIF;					// Calibration in single ended mode
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};


	/*--------------------------------------------------------------------------------------------
	Configure ADC Channels to be converted:
	0	PA0 ADC12_IN1		Env Attack
	1	PA1 ADC12_IN2		Env Decay
	2	PB0 ADC1_IN15		Env Sustain
	3	PA3 ADC1_IN4		Env Release
	*/

	InitAdcPins(ADC1, {1, 2, 15, 4});

	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF0; // Channel 1 Clear synchronization overrun event flag
	DMA1->IFCR = 0x3F << DMA_IFCR_CGIF1_Pos;		// clear all five interrupts for this stream

	DMA1_Channel1->CNDTR |= ADC_BUFFER_LENGTH;		// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Channel1->CMAR = (uint32_t)(ADC_array);	// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Channel1->CCR |= DMA_CCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}

	ADC1->CR |= ADC_CR_ADSTART;						// Start ADC
}


void InitCordic()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
}


/*
//	Setup Timer 9 to count clock cycles for coverage profiling
void InitCoverageTimer() {
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;				// Enable Timer
	TIM9->PSC = 100;
	TIM9->ARR = 65535;

	TIM9->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 2);		// Lower is higher priority

}

//	Setup Timer 5 to count time between bounces
void InitDebounceTimer() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;				// Enable Timer
	TIM5->PSC = 10000;
	TIM5->ARR = 65535;
}

*/


void InitPWMTimer()
{
	// TIM3: Channel 1: PA0 (AF1)
	// 		 Channel 2: PB3 (AF1)
	// 		 Channel 3: PB10 (AF1)
	// 		 Channel 4: PB11 (AF1)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;			// reset and clock control - advanced high performance bus - GPIO port A
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;			// reset and clock control - advanced high performance bus - GPIO port B
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// Enable channel 1, 2, 3 PWM output pins on PA0
	// 00: Input mode; 01: General purpose output mode; 10: Alternate function mode; 11: Analog mode (default)
	GPIOA->MODER &= ~GPIO_MODER_MODE0_0;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL0_0;			// AF1


	// Enable channel 4 PWM output pin on PB3, PB10, PB11
	GPIOB->MODER &= ~(GPIO_MODER_MODE3_0 | GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0);
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL3_0;			// AF1
	GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL10_0 | GPIO_AFRH_AFSEL11_0);					// AF1


	// Timing calculations: Clock = 64MHz / (PSC + 1) = 32m counts per second
	// ARR = number of counts per PWM tick = 4096
	// 32m / ARR = 7.812kHz of PWM square wave with 4096 levels of output
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;					// Output compare 1 preload enable
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;					// Output compare 2 preload enable
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE;					// Output compare 3 preload enable
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE;					// Output compare 4 preload enable

	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);	// 0110: PWM mode 1 - In upcounting, channel 1 active if TIMx_CNT<TIMx_CCR1
	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);	// 0110: PWM mode 1 - In upcounting, channel 2 active if TIMx_CNT<TIMx_CCR2
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3
	TIM2->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);	// 0110: PWM mode 1 - In upcounting, channel 3 active if TIMx_CNT<TIMx_CCR3

	TIM2->CCR1 = 0;									// Initialise PWM level to 0
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;

	TIM2->ARR = 2047;								// Total number of PWM ticks = 512
	TIM2->PSC = 0;									// Should give ~93.7kHz
	TIM2->CR1 |= TIM_CR1_ARPE;						// 1: TIMx_ARR register is buffered
	TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);		// Capture mode enabled / OC1 signal is output on the corresponding output pin
	TIM2->EGR |= TIM_EGR_UG;						// 1: Re-initialize the counter and generates an update of the registers

	// Generate interrupt on Update
//	TIM2->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
//	NVIC_EnableIRQ(TIM2_IRQn);
//	NVIC_SetPriority(TIM2_IRQn, 0);					// Lower is higher priority

	TIM2->CR1 |= TIM_CR1_CEN;						// Enable counter
}


