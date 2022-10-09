#include "main.h"

int main()
{
	SYS_Init();
	flasherInit();

	while (1)
	{
		stateHandler();
	}
}
