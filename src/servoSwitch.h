// servoSwitch.h
// Whether the home switch or the limit switch stops a motor in the servo mode, without
// hardware access, so that it can be tested on the host.
//
// The two switches are judged separately: one pressed switch does not hide the other.
// - Home switch pressed: no motion towards homingDirection when the motion on the home
//   switch is prohibited (/setProhibitMotionOnHomeSw) or the switch hard-stops the driver
//   (homeSwMode false = SW_HARD_STOP).
// - Limit switch pressed: no motion away from homingDirection when the motion on the
//   limit switch is prohibited (/setProhibitMotionOnLimitSw) or limitSwMode is false
//   (the hard stop mode, as for the home switch).
// Directions are the driver's (true = FWD).

#ifndef _SERVOSWITCH_h
#define _SERVOSWITCH_h

// True if the servo update must not move the motor in the direction dir.
bool servoSwitchBlocks(bool homeSw, bool limitSw, bool prohibitHome, bool prohibitLimit,
                       bool homeSwMode, bool limitSwMode, bool homingDirection, bool dir);

#endif
