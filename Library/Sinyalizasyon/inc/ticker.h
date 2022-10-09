#ifndef __TICKER_H__
#define __TICKER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <NUC029FAE.h>
#include <stdio.h>
#include <stdlib.h>

    void startTicker(TIMER_T *timer);
    void stopTicker(TIMER_T *timer);
    uint32_t getTick(void);
    void tickerDelay(uint32_t delayMs);

#ifdef __cplusplus
}
#endif
#endif
