// oscArgs.cpp

#include "oscArgs.h"
#include <math.h>
#include <string.h>

static bool isFiniteFloat(float f)
{
    return !isnan(f) && !isinf(f);
}

OscArgStatus oscCheckArgs(OscMessage &msg, const char *args, uint8_t motorCount, int32_t *badMotorId)
{
    // 'L' stands for motorCount arguments
    unsigned required = 0;
    for (const char *p = args; *p; p++)
        required += (*p == 'L') ? motorCount : 1;
    if ((unsigned)msg.size() < required)
        return OSC_ARGS_MISSING;

    int position = 0;
    for (const char *p = args; *p; p++)
    {
        const int repeat = (*p == 'L') ? motorCount : 1;
        for (int r = 0; r < repeat; r++, position++)
        {
            switch (*p)
            {
            case 's':
                if (!msg.isString(position))
                    return OSC_ARGS_WRONG_TYPE;
                break;
            case 'b':
                if (msg.isBoolean(position))
                    break;
                // fall through: a number
            case 'n':
            case 'L':
                if (msg.isFloat(position))
                {
                    if (!isFiniteFloat(msg.getFloat(position)))
                        return OSC_ARGS_WRONG_TYPE;
                }
                else if (!msg.isInt(position))
                {
                    return OSC_ARGS_WRONG_TYPE;
                }
                break;
            case 'm':
                if (msg.isFloat(position))
                {
                    const float f = msg.getFloat(position);
                    if (!isFiniteFloat(f))
                        return OSC_ARGS_WRONG_TYPE;
                    if (!(f > -1.0f && f < 256.0f))
                    {
                        if (badMotorId)
                            *badMotorId = (f <= -2147483648.0f) ? INT32_MIN : (f >= 2147483648.0f) ? INT32_MAX : (int32_t)f;
                        return OSC_ARGS_BAD_MOTOR_ID;
                    }
                }
                else if (msg.isInt(position))
                {
                    const int32_t v = msg.getInt(position);
                    if (v < 0 || v > 255)
                    {
                        if (badMotorId)
                            *badMotorId = v;
                        return OSC_ARGS_BAD_MOTOR_ID;
                    }
                }
                else
                {
                    return OSC_ARGS_WRONG_TYPE;
                }
                break;
            default:
                return OSC_ARGS_WRONG_TYPE; // a mistake in the table
            }
        }
    }
    return OSC_ARGS_OK;
}
