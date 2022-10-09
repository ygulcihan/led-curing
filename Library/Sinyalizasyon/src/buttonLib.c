#include <stdio.h>
#include <NUC029FAE.h>
#include <gpio.h>
#include <stdbool.h>
#include "buttonLib.h"
#include "ticker.h"

/**
 * @brief       Constructs Button Object
 *
 * @param[in]   port            	GPIO port. It could be P0, P1, P2, P3, P4 or P5.
 * @param[in]   bit             	The pin of specified GPIO port. It could be BIT0 ~ BIT7.
 * @param[in]   pinPointer      	The pointer to the specified pin. An example is P12 meaning PORT1, BIT2.
 * @param[in]	ppMode				Existing Push-Pull mode configuration for the button (pushPullMode_t)
 * @param[in]	debounceTimeInMs	Desired debounce time for the button in milliseconds
 *
 * @return      Button Struct
 *
 * @details     This function is used to construct a button object.
 */
Button *attachButton(Button *me, GPIO_T *port, uint32_t bit, volatile uint32_t *pinPointer, pushPullMode_t ppMode, uint32_t debounceTimeInMs)
{
	me->_port = port;
	me->_bit = bit;
	me->_pinPointer = pinPointer;
	me->_ppMode = ppMode;
	me->_timePressed = 0;
	me->_initialPressTime = 0;
	me->_longPressed = false;
	me->_longPressedOnPrevCall = false;
	me->_sampled = false;
	me->_debounceTimeInMs = debounceTimeInMs;
	me->readPin = &_readPin;
	me->isPressed = &_isPressed;
	me->getButtonState = &_getButtonState;
	GPIO_SetMode(port, bit, GPIO_PMD_INPUT);

	return me;
}

/**
 * @brief Returns Current State of the Button Pin (0 or 1)
 *
 * @return uint32_t
 */
uint32_t _readPin(Button *me)
{
	me->_state = *me->_pinPointer;
	return *me->_pinPointer;
}

/**
 * @brief Returns true if the Button is Pressed
 *
 * @return true
 * @return false
 *
 * @details Checks if the Button is Pressed Using the pushPullMode Setting.
 */
bool _isPressed(Button *me)
{
	me->_state = *me->_pinPointer;

	return (me->_state != me->_ppMode);
}

/**
 * @brief Returns a buttonState enum after reading the button pin
 * 
 * @param me Button object to read
 * @return buttonState enum
 */
buttonState _getButtonState(Button *me)
{
	me->_sampled = false;
	me->_longPressed = false;

	while (_isPressed(me))
	{
		if (!me->_sampled)
		{
			me->_initialPressTime = getTick();
			me->_sampled = true;
		}

		me->_timePressed = getTick() - me->_initialPressTime;

		if (me->_timePressed >= 3000)
		{
			me->_longPressed = true;
			break;
		}
	}

	if (me->_longPressed)
	{
		me->_timePressed = 0;
		//printf("Button Long Pressed\n");
		me->_longPressedOnPrevCall = true;
		return BUTTON_LONG_PRESS;
	}

	else if (me->_timePressed <= me->_debounceTimeInMs)
		return BUTTON_DEFAULT_STATE;

	else
	{
		me->_timePressed = 0;

		if (me->_longPressedOnPrevCall)
		{
			me->_longPressedOnPrevCall = false;
			return BUTTON_DEFAULT_STATE;
		}
		else
		{
			//printf("Button Clicked\n");
			return BUTTON_CLICK;
		}
	}
}
