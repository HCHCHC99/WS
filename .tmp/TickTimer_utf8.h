#ifndef TICKTIMER_H
#define TICKTIMER_H

#include "stdint.h"
#include "stdbool.h"
// #include "stdlib.h"

// 闈為樆濉炲欢鏃跺櫒缁撴瀯浣
typedef struct {
    uint64_t startTick;   // 寤舵椂鍚鍔ㄦ椂鐨勬淮绛旇℃暟锛堣皟鐢╪bDelay_Start鏃惰板綍锛
    uint64_t delayMs;     // 鐩鏍囧欢鏃舵椂闂达紙鍗曚綅锛歮s锛岄氳繃nbDelay_Init/nbDelay_SetTime璁剧疆锛
    bool isRunning;       // 寤舵椂鍣ㄨ繍琛岀姸鎬侊細true=姝ｅ湪寤舵椂锛宖alse=鏈鍚鍔/宸插畬鎴
} NonBlockingDelay_t;

// 鍒濆嬪寲婊寸瓟瀹氭椂鍣
void tickTimer_Init(void);

// 鑾峰彇褰撳墠婊寸瓟鏁帮紙姣绉掞級
uint64_t tickTimer_GetCount(void);

// 闃诲炲紡寤舵椂锛堝崟浣嶏細姣绉掞級
void tickTimer_DelayMs(uint64_t ms);

// 涓鏂鏈嶅姟鍑芥暟涓璋冪敤鐨勬淮绛旀洿鏂板嚱鏁
// 1ms 璋冪敤涓娆
void tickTimer_Update(void);

// 闈為樆濉炲欢鏃跺嚱鏁扮粍

// 鍒濆嬪寲闈為樆濉炲欢鏃跺櫒
void nbDelay_Init(NonBlockingDelay_t* delayObj, uint64_t delayMs);

// 鍚鍔ㄩ潪闃诲炲欢鏃
void nbDelay_Start(NonBlockingDelay_t* delayObj);


// 妫鏌ラ潪闃诲炲欢鏃舵槸鍚﹀畬鎴愶紙瀹屾垚鍚庝細缁撴潫寤舵椂锛
bool nbDelay_IsComplete(NonBlockingDelay_t* delayObj);

// 妫鏌ラ潪闃诲炲欢鏃舵槸鍚﹀畬鎴愶紙瀹屾垚鍚庝笉浼氱粨鏉熷欢鏃讹級
bool nbDelay_IsComplete_noclose(NonBlockingDelay_t* delayObj);

// 鍋滄㈤潪闃诲炲欢鏃
void nbDelay_Stop(NonBlockingDelay_t* delayObj);

// 璁剧疆鏂扮殑寤舵椂鏃堕棿锛堜笉鍚鍔锛
void nbDelay_SetTime(NonBlockingDelay_t* delayObj, uint64_t delayMs);

// 鑾峰彇璺濈讳笂娆¤皟鐢ㄧ粡杩囩殑婊寸瓟鏁
uint64_t tickTimer_GetElapsedSinceLastCall(void);

uint64_t tickTimer_GetRawTick(void);

#endif // TICKTIMER_H
