#include "ticker.h"

#define CPU_FREQ 22118400

volatile uint32_t _tick = 0;

void TMR0_IRQHandler(void)
{
	_tick++;
	// clear timer interrupt flag
	TIMER0->TISR = TIMER_TISR_TIF_Msk;
}

void TMR1_IRQHandler(void)
{
	_tick++;
	// clear timer interrupt flag
	TIMER1->TISR = TIMER_TISR_TIF_Msk;
}

/**
 * @brief Starts Ticker Interrupts
 * 
 * @param timer Hardware Timer Struct (TIMER0, TIMER1)
 */
void startTicker(TIMER_T *timer)
{

	if (!TIMER_IS_ACTIVE(timer))
	{
		if (timer == TIMER0)
			CLK->APBCLK |= CLK_APBCLK_TMR0_EN_Msk;
		if (timer == TIMER1)
			CLK->APBCLK |= CLK_APBCLK_TMR1_EN_Msk;
	}

	uint32_t compareDivider, prescaler;

	// (pre+1)/comp = tick interval
	prescaler = 1;
	compareDivider = 2000;
	// Tick every 1 ms

	timer->TCMPR = (CPU_FREQ / compareDivider);
	timer->TCSR = TIMER_TCSR_CEN_Msk | TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE | (prescaler);

	// Enable ADC interrupt
	if (timer == TIMER0)
		NVIC_EnableIRQ(TMR0_IRQn);
	if (timer == TIMER1)
		NVIC_EnableIRQ(TMR1_IRQn);
}

/**
 * @brief Stops Ticker Interrupts
 * 
 * @param timer Hardware Timer Struct (TIMER0, TIMER1)
 */
void stopTicker(TIMER_T *timer)
{
	if (timer == TIMER0)
		NVIC_DisableIRQ(TMR0_IRQn);
	if (timer == TIMER1)
		NVIC_DisableIRQ(TMR1_IRQn);
}

/**
 * @brief Get the Current Tick Value
 * 
 * @return uint32_t tick
 */
uint32_t getTick()
{
	return _tick;
}

/**
 * @brief Halts CPU for a desired amount of time (Deprecated)
 * 
 * @param delayMs 
 */
void delay(uint32_t delayMs)
{
	static uint32_t prevTick = 0;

	while (_tick - prevTick < delayMs)
		__NOP();

	prevTick = _tick;
}
