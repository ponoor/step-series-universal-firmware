// elapsedMs.h
// Elapsed time test of the timers in milliseconds, without hardware access, so that it can
// be tested on the host.
//
// loop() takes millis() once at the start of the slot (nowMs) and passes it to the functions
// of the slot, but a timer can be started later in the same slot with a new millis() (e.g.
// releaseSw sent by checkStatus(), a UDP send that starts the TX LED blink). If millis()
// ticked in between, the start time is later than nowMs, and the unsigned difference
// nowMs - startMs is about 0xFFFFFFFF: the timer would elapse at once. A start time later
// than nowMs (up to 2^31 ms) counts as 0 ms elapsed. Times are millis() values; the test is
// correct across the wrap of millis().

#ifndef _ELAPSEDMS_h
#define _ELAPSEDMS_h

#include <stdint.h>

// Whether durationMs or more have elapsed from startMs to nowMs. A start time later than
// nowMs counts as 0 ms elapsed: true for a duration of 0 only if startMs is not later than
// nowMs. A duration of 2^31 ms or more never elapses.
bool millisElapsed(uint32_t startMs, uint32_t nowMs, uint32_t durationMs);

#endif
