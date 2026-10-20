// goToDirection.h
// Direction of a GoTo / GoHome / GoMark, without hardware access so that it can be tested
// on the host. ABS_POS is a 22-bit two's complement counter and the driver takes the
// shortest way to the target (a move of 2^21 steps or more goes the other way round),
// so the direction is the sign of the 22-bit difference. Used for the start conditions
// of the motion (the home switch and the limit switch prohibit one direction).

#ifndef _GOTODIRECTION_h
#define _GOTODIRECTION_h

#include <stdint.h>

// The low 22 bits of v as a signed number (-2^21 .. 2^21 - 1).
inline int32_t signExtend22(int32_t v)
{
    uint32_t u = (uint32_t)v & 0x3FFFFFu;
    if (u & 0x200000u)
        u |= ~0x3FFFFFu;
    return (int32_t)u;
}

// true = FWD (the driver's direction), for a motor at ABS_POS pos going to target (any
// int32: only its low 22 bits count, as in the driver). No move (target == pos) gives
// false (REV), as does a move of exactly -2^21 steps.
inline bool goToDirection(int32_t pos, int32_t target)
{
    return signExtend22((int32_t)((uint32_t)target - (uint32_t)pos)) > 0;
}

#endif
