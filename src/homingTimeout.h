// homingTimeout.h
// Timeout test of the homing steps (goUntil, releaseSw), without hardware access, so that
// it can be tested on the host.
//
// loop() takes millis() once at the start of the slot (nowMs), but a homing step can start
// later in the same slot: checkStatus() sends releaseSw when the home switch is found, and
// sets the start time with a new millis(). If millis() ticked in between, the start time is
// later than nowMs, and the unsigned difference nowMs - startMs is about 0xFFFFFFFF: the
// timeout fired at once. A start time later than nowMs (up to 2^31 ms) counts as 0 ms
// elapsed (millisElapsed(), elapsedMs.h). Times are millis() values; the test is correct
// across the wrap of millis().

#ifndef _HOMINGTIMEOUT_h
#define _HOMINGTIMEOUT_h

#include <stdint.h>

// Whether timeoutMs or more have elapsed from startMs to nowMs. A timeout of 0 is disabled
// (never elapsed), and so is a timeout of 2^31 ms or more (e.g. a negative OSC argument).
bool homingTimeoutElapsed(uint32_t startMs, uint32_t nowMs, uint32_t timeoutMs);

#endif
