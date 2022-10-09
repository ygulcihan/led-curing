#ifndef __BUTTONLIB_H__
#define __BUTTONLIB_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <NUC029FAE.h>
#include <stdbool.h>

#define HIGH true
#define LOW false

	typedef enum
	{
		PULL_DOWN,
		PULL_UP,
	} pushPullMode_t;

	typedef enum
	{
		BUTTON_DEFAULT_STATE,
		BUTTON_CLICK,
		BUTTON_LONG_PRESS,
	} buttonState;

	typedef struct _button Button;
	struct _button
	{
		GPIO_T *_port;
		uint32_t _bit, _timePressed, _initialPressTime;
		volatile uint32_t *_pinPointer;
		bool _state, _longPressed, _sampled, _debounceTimeInMs, _longPressedOnPrevCall;
		uint8_t _ppMode;
		uint32_t (*readPin)(Button *);
		bool (*isPressed)(Button *);
		buttonState (*getButtonState)(Button *);
	};

	buttonState _getButtonState(Button *me);
	Button *attachButton(Button *me, GPIO_T *port, uint32_t bit, volatile uint32_t *pinPointer, pushPullMode_t ppMode, uint32_t debounceTimeInMs);
	uint32_t _readPin(Button *me);
	bool _isPressed(Button *me);

#ifdef __cplusplus
}
#endif
#endif
