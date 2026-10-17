// rawSpeed.h
// The value to keep in the variable (step/s, step/s^2) of a setting made with a Raw
// command (/setAccRaw, ...), so that /saveConfig and resetMotorDriver() give the driver
// the same register value again. No hardware access, so that it can be tested on the host.
//
// The variables are applied with the driver library's setAcc() etc. (accCalc() ...), which
// are private there and not an exact inverse of the Parse functions: accCalc() rounds to
// the nearest, maxSpdCalc() takes the ceiling, but minSpdCalc() and FSCalc() truncate, so
// calc(parse(n)) is n - 1 for n >= 1077 (MIN_SPEED, 486 values) and n >= 538 (FS_SPD, 243
// values). The values here are the middle of the interval that the calc functions map to
// n, so they come back as n. They differ from the Parse functions (what /getMinSpeed
// returns for the register) by half an LSB, which is below what the register resolves.
// Exceptions: ACC / DEC 0 comes back as 1 and ACC 0xFFF as 0xFFE (the calc functions clamp
// to the valid range of the register).

#ifndef _RAWSPEED_h
#define _RAWSPEED_h

#include <stdint.h>

inline float accFromRaw(uint16_t raw) { return (float)(raw & 0x0FFF) * 14.551915F; }
inline float decFromRaw(uint16_t raw) { return (float)(raw & 0x0FFF) * 14.551915F; }
inline float maxSpeedFromRaw(uint16_t raw) { return (float)(raw & 0x03FF) * 15.258789F; }
// MIN_SPEED without LSPD_OPT (bit 12)
inline float minSpeedFromRaw(uint16_t raw) { return ((float)(raw & 0x0FFF) + 0.5F) * 0.23841858F; }
// FS_SPD without BOOST_MODE (bit 10, PowerSTEP01)
inline float fullStepSpeedFromRaw(uint16_t raw) { return ((float)(raw & 0x03FF) + 1.0F) * 15.258789F; }

#endif
