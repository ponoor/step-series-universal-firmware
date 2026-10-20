// oscArgs.h
// Checks the arguments of an OSC message against the argument list of its route
// (oscRoutes.h). It does not depend on Arduino so that it can be tested on the host.
//
// A route's args is one character per required argument:
//   m  motor ID: a number, 0..255 after the conversion to int (1..NUM_OF_MOTOR or 255 are
//      valid IDs; the handler tells those, this check only keeps the value from being
//      truncated to uint8_t)
//   n  number: OSC int or float, not NaN / infinity
//   b  boolean-like: a number, or OSC true / false
//   s  string
//   L  one number per motor (motorCount of them)
// Arguments after the last one in the list are ignored.

#ifndef _OSCARGS_h
#define _OSCARGS_h

#include <stdint.h>
#include "oscParser.h"

enum OscArgStatus
{
    OSC_ARGS_OK = 0,
    OSC_ARGS_MISSING,    // fewer arguments than the route needs
    OSC_ARGS_WRONG_TYPE, // an argument of another type, or NaN / infinity
    OSC_ARGS_BAD_MOTOR_ID // a motor ID outside 0..255; badMotorId is the value
};

// Returns the first problem found, checking the count first and then the arguments in
// order. badMotorId (may be null) is set only with OSC_ARGS_BAD_MOTOR_ID.
OscArgStatus oscCheckArgs(OscMessage &msg, const char *args, uint8_t motorCount, int32_t *badMotorId);

#endif
