// servoSwitch.cpp

#include "servoSwitch.h"

bool servoSwitchBlocks(bool homeSw, bool limitSw, bool prohibitHome, bool prohibitLimit,
                       bool homeSwMode, bool limitSwMode, bool homingDirection, bool dir)
{
    if (homeSw && (prohibitHome || !homeSwMode) && (dir == homingDirection))
        return true;
    if (limitSw && (prohibitLimit || !limitSwMode) && (dir != homingDirection))
        return true;
    return false;
}
