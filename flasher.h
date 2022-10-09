#ifndef __FLASHER_H__
#define __FLASHER_H__

#define CONFIG_MODE false /*Enable configuration of flash duration*/
#define DEBUG_MODE false  /*Enable printf commands for debugging purposes*/

#ifdef __cplusplus
extern "C"
{
#endif

#include <NUC029FAE.h>
#include <fmc.h>
#include "buttonLib.h"
#include "ticker.h"

#define CONFIG_STORAGE_ADDR 0X3FFC

#define toggleLed() P04 ^= 1
#define ledOn() P04 = 1
#define ledOff() P04 = 0

#ifndef HIGH
#define HIGH true
#endif
#ifndef LOW
#define LOW false
#endif

	void flasherInit(void), stateHandler(void), blinkLed(uint32_t blinkIntervalinMs), buttonStateHandler(buttonState bState), configHandler(void);
	void stopFlash(void), flashLed(uint32_t flashDurationInSeconds), _readConfig(void), _clickHandler(void), _saveConfig(void);
	static int _set_data_flash_base(uint32_t u32DFBA);

	typedef enum _deviceState
	{
		MAIN,
		CONFIG,
	} DeviceState;

	typedef enum
	{
		DURATION_20S,
		DURATION_30S,
		DURATION_60S,
	} flashDuration_t;

#ifdef __cplusplus
}
#endif
#endif
