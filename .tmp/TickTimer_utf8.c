#include "TickTimer.h"
#include <stddef.h>

static volatile uint64_t s_tickCount = 0;
static uint64_t s_lastCallTick = 0;

// 鍒濆嬪寲婊寸瓟瀹氭椂鍣
void tickTimer_Init(void)
{
    s_tickCount = 0;
    s_lastCallTick = tickTimer_GetCount(); 
}

// Systick.c 涓娣诲姞鍑芥暟瀹炵幇
uint64_t tickTimer_GetRawTick(void) {
    uint64_t tick;
    //__disable_irq();  // 鍏充腑鏂锛岄伩鍏嶈诲彇鏃惰涓鏂淇鏀癸紙淇濊瘉鏁版嵁瀹屾暣鎬э級
    tick = s_tickCount;
    //__enable_irq();
    return tick;
}

// 鑾峰彇褰撳墠婊寸瓟鏁
uint64_t tickTimer_GetCount(void)
{
    uint64_t tick;
    //__disable_irq();
    tick = s_tickCount;
    //__enable_irq();
    return tick;
}

// 闃诲炲紡寤舵椂
void tickTimer_DelayMs(uint64_t ms)
{
    uint64_t startTick = tickTimer_GetCount();
    while ((tickTimer_GetCount() - startTick) < ms);
}

// 瀹氭椂鍣ㄤ腑鏂涓璋冪敤
__attribute__((section(".ramfunc"))) void tickTimer_Update(void)
{
    s_tickCount++;
}

// 闈為樆濉炲欢鏃跺嚱鏁板疄鐜

void nbDelay_Init(NonBlockingDelay_t* delayObj, uint64_t delayMs) {
    // 鏂板烇細鍙傛暟鍚堟硶鎬ф鏌ワ紙閬垮厤绌烘寚閽堝拰鏃犳晥寤舵椂锛
    if (delayObj == NULL) {
        return;  // 鎴栨柇瑷锛歛ssert(delayObj != NULL);锛堥渶鍖呭惈assert.h锛
    }

    
    delayObj->startTick = 0;
    delayObj->delayMs = delayMs;
    delayObj->isRunning = false;
}

void nbDelay_Start(NonBlockingDelay_t* delayObj)
{
    delayObj->startTick = tickTimer_GetCount();
    delayObj->isRunning = true;
}

bool nbDelay_IsComplete(NonBlockingDelay_t* delayObj)
{
    if (!delayObj->isRunning) {
        return false;
    }
    
    uint64_t currentTick = tickTimer_GetCount();
    uint64_t elapsed = currentTick - delayObj->startTick;
    
    if (elapsed >= delayObj->delayMs) {
        delayObj->isRunning = false;
        return true;
    }
    
    return false;
}

// 鏂板炲嚱鏁帮細妫鏌ュ欢鏃舵槸鍚﹀畬鎴愪絾涓嶇粨鏉熷欢鏃
bool nbDelay_IsComplete_noclose(NonBlockingDelay_t* delayObj)
{
    if (!delayObj->isRunning) {
        return false;
    }
    
    uint64_t currentTick = tickTimer_GetCount();
    uint64_t elapsed = currentTick - delayObj->startTick;
    
    // 鍙妫鏌ユ槸鍚﹀畬鎴愶紝涓嶄慨鏀筰sRunning鐘舵
    return (elapsed >= delayObj->delayMs);
}

void nbDelay_Stop(NonBlockingDelay_t* delayObj)
{
    delayObj->isRunning = false;
}

void nbDelay_SetTime(NonBlockingDelay_t* delayObj, uint64_t delayMs) {
    if (delayObj == NULL) {
        return;
    }

    delayObj->delayMs = delayMs;
}

// 鑾峰彇璺濈讳笂娆¤皟鐢ㄧ粡杩囩殑婊寸瓟鏁
uint64_t tickTimer_GetElapsedSinceLastCall(void)
{
    uint64_t currentTick;
    uint64_t elapsed;
    
    //__disable_irq();
    currentTick = s_tickCount;  // 鐩存帴璁块棶锛岄伩鍏嶅嚱鏁拌皟鐢
    elapsed = currentTick - s_lastCallTick;
    s_lastCallTick = currentTick;
    //__enable_irq();
    
    return elapsed;
}
