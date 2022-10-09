#include "flasher.h"

const uint16_t flashDuration[] = {20000, 30000, 60000};
const uint8_t nrOfElements = sizeof(flashDuration) / sizeof(flashDuration[0]);

/*Default Flash Duration Settings (Change Both)*/
uint8_t flashDurationIndex = DURATION_30S, prevFlashDurationIndex = DURATION_30S;
bool flash = false, flashed = false;

Button button;
DeviceState deviceState = MAIN;

/**
 * @brief Initialize Flasher Device
 *
 */
void flasherInit()
{

	#if CONFIG_MODE
		_readConfig();
	#endif

	attachButton(&button, P1, BIT2, &P12, PULL_DOWN, 150);
	GPIO_SetMode(P0, BIT4, GPIO_PMD_OUTPUT);
}

/**
 * @brief Main Handler Function
 *
 */
void stateHandler()
{
	switch (deviceState)
	{
	case MAIN:
		buttonStateHandler(button.getButtonState(&button));
		break;

	case CONFIG:
		configHandler();
		break;
	}
}

/**
 * @brief Blinks an Led by calling the toggleLed() function in intervals
 *
 * @param blinkIntervalinMs Blink Interval in Milliseconds
 */
void blinkLed(uint32_t blinkIntervalinMs)
{
	static uint32_t ledPrevTick = 0;

	if (getTick() - ledPrevTick >= blinkIntervalinMs)
	{
		toggleLed();
		ledPrevTick = getTick();
	}
}

/**
 * @brief Button State Handler Function
 *
 * @param bState Current Button State (using buttonLib)
 */
void buttonStateHandler(buttonState bState)
{
	switch (bState)
	{
	case BUTTON_CLICK:
		flash = true;
		break;

	case BUTTON_LONG_PRESS:
		#if (CONFIG_MODE)
			deviceState = CONFIG;
		#else
			flash = true;
		#endif
		break;

	default:
		break;
	}

	if (flash)
		flashLed(flashDuration[flashDurationIndex]);
	else
		stopFlash();
}

/**
 * @brief Handler Function for Config State
 *
 */
void configHandler()
{
	ledOn();

	switch (button.getButtonState(&button))
	{
	case BUTTON_CLICK:
		_clickHandler();
		break;

	case BUTTON_LONG_PRESS:
		_saveConfig();
		break;

	default:
		break;
	}

	if (prevFlashDurationIndex != flashDurationIndex)
	{
		for (uint8_t i = 0; i < (flashDurationIndex + 1); i++)
		{
			ledOff();
			CLK_SysTickDelay(1000000);
			ledOn();
			CLK_SysTickDelay(1000000);
		}
		prevFlashDurationIndex = flashDurationIndex;
	}
}

/**
 * @brief Turns the Led on for a defined time
 *
 * @param flashDurationInSeconds
 */
void flashLed(uint32_t flashDurationInSeconds)
{
	static uint32_t initialFlashTime = 0;

	if (!flashed)
	{
		ledOn();
		initialFlashTime = getTick();
		flashed = true;
	}

	if (getTick() - initialFlashTime >= flashDurationInSeconds)
	{
		stopFlash();
	}
}

/**
 * @brief Turns the Led Off and Resets Necessary Variables
 *
 */
void stopFlash()
{
	ledOff();
	flash = false;
	flashed = false;
}

void _clickHandler()
{
	if (++flashDurationIndex == 3)
		flashDurationIndex = 0;
	#if DEBUG_MODE
		printf("Flash Duration = %d ms\n", flashDuration[flashDurationIndex]);
	#endif
}

void _readConfig()
{
	SYS_UnlockReg();
	FMC_Open();
	_set_data_flash_base(CONFIG_STORAGE_ADDR);
	uint32_t temp = FMC_Read(CONFIG_STORAGE_ADDR);

	if (temp <= nrOfElements)
	{
		#if DEBUG_MODE
			printf("Config Read Successfully, %d \n", (temp + 1));
		#endif
		flashDurationIndex = temp;
		prevFlashDurationIndex = flashDurationIndex;
	}
	else
	{
		#if DEBUG_MODE
			printf("Invalid Config, %d\n", (temp + 1));
		#endif
	}
	FMC_Close();
	SYS_LockReg();
}

static int _set_data_flash_base(uint32_t u32DFBA)
{
	uint32_t au32Config[2];

	if (FMC_ReadConfig(au32Config, 2) < 0)
	{
		#if DEBUG_MODE
			printf("\nRead User Config failed!\n");
		#endif
		return -1;
	}

	if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
		return 0;

	FMC_EnableConfigUpdate();

	au32Config[0] &= ~0x1;
	au32Config[1] = u32DFBA;

	if (FMC_WriteConfig(au32Config, 2) < 0)
		return -1;

	#if DEBUG_MODE
		printf("\nSet Data Flash base as 0x%x.  Please Reset CPU\n", u32DFBA);
	#endif
	// Perform chip reset to make new User Config take effect
	SYS->IPRSTC1 = SYS_IPRSTC1_CHIP_RST_Msk;
	return 0;
}

void _saveConfig()
{
	stopFlash();
	SYS_UnlockReg();
	FMC_Open();
	for (uint32_t currentAddr = CONFIG_STORAGE_ADDR; currentAddr < (CONFIG_STORAGE_ADDR + 4); currentAddr += 4)
	{
		FMC_Erase(CONFIG_STORAGE_ADDR);
		FMC_Write(CONFIG_STORAGE_ADDR, flashDurationIndex);
	}
	#if DEBUG_MODE
		printf("Saved Intended:%d, Read:%d\n", flashDurationIndex + 1, FMC_Read(CONFIG_STORAGE_ADDR) + 1);
	#endif
	FMC_Close();
	SYS_LockReg();
	deviceState = MAIN;
}
