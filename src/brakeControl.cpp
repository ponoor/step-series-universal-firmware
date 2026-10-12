// brakeControl.cpp
// See brakeControl.h.

#include "brakeControl.h"

bool brakeOutputReleased(BrakeState state)
{
    return (state == BRAKE_RELEASED) || (state == BRAKE_STOPPING) || (state == BRAKE_FREE);
}

static BrakeAction stay(BrakeState state)
{
    BrakeAction a;
    a.next = state;
    a.driver = BRAKE_DRIVER_NONE;
    a.error = BRAKE_ERROR_NONE;
    a.execute = false;
    a.startTimer = false;
    a.cancelPending = false;
    a.warn = false;
    return a;
}

static BrakeAction go(BrakeState next, BrakeDriverCommand driver = BRAKE_DRIVER_NONE)
{
    BrakeAction a = stay(next);
    a.driver = driver;
    // Entering a timed state starts the transition time.
    a.startTimer = (next == BRAKE_RELEASING) || (next == BRAKE_ENGAGING);
    return a;
}

static BrakeAction execute(BrakeState next)
{
    BrakeAction a = stay(next);
    a.execute = true;
    return a;
}

static BrakeAction reject(BrakeState state, BrakeError error)
{
    BrakeAction a = stay(state);
    a.error = error;
    return a;
}

BrakeAction brakeOnCommand(BrakeState s, BrakeCommand c)
{
    switch (c)
    {
    case BRAKE_CMD_ACTIVATE:
        switch (s)
        {
        case BRAKE_ENGAGED:
            return go(BRAKE_RELEASING, BRAKE_DRIVER_HARD_STOP);
        case BRAKE_STOPPING:
            // Cancels the engaging plan; the deceleration goes on.
            return go(BRAKE_RELEASED);
        case BRAKE_ENGAGING:
            return go(BRAKE_RELEASING);
        case BRAKE_FREE:
            return go(BRAKE_RELEASED, BRAKE_DRIVER_HARD_STOP);
        default: // RELEASING, RELEASED
            return stay(s);
        }
    case BRAKE_CMD_DEACTIVATE:
    case BRAKE_CMD_SOFT_HIZ:
        switch (s)
        {
        case BRAKE_RELEASING:
            return go(BRAKE_ENGAGING);
        case BRAKE_RELEASED:
            return go(BRAKE_STOPPING, BRAKE_DRIVER_SOFT_STOP);
        case BRAKE_FREE:
            return go(BRAKE_ENGAGED);
        default: // ENGAGED, STOPPING, ENGAGING
            return stay(s);
        }
    case BRAKE_CMD_HARD_HIZ:
        switch (s)
        {
        case BRAKE_RELEASING:
            return go(BRAKE_ENGAGING);
        case BRAKE_RELEASED:
        case BRAKE_STOPPING:
            return go(BRAKE_ENGAGING, BRAKE_DRIVER_HARD_STOP);
        case BRAKE_FREE:
            return go(BRAKE_ENGAGED);
        default: // ENGAGED, ENGAGING
            return stay(s);
        }
    case BRAKE_CMD_FREE:
        if (s == BRAKE_FREE)
            return stay(s);
        return go(BRAKE_FREE, BRAKE_DRIVER_HARD_HIZ);
    case BRAKE_CMD_MOTION:
        switch (s)
        {
        case BRAKE_RELEASED:
            return execute(s);
        case BRAKE_STOPPING:
        case BRAKE_FREE:
            return execute(BRAKE_RELEASED);
        default: // ENGAGED, RELEASING, ENGAGING
            return reject(s, BRAKE_ERROR_ENGAGED);
        }
    case BRAKE_CMD_STOP:
        switch (s)
        {
        case BRAKE_RELEASING:
        case BRAKE_RELEASED:
        case BRAKE_ENGAGING:
            return execute(s);
        case BRAKE_STOPPING:
            return execute(BRAKE_RELEASED);
        default: // ENGAGED, FREE: the driver is HiZ, a stop command would energize it
            return stay(s);
        }
    case BRAKE_CMD_RESET_DRIVER:
        if ((s == BRAKE_ENGAGED) || (s == BRAKE_FREE))
            return execute(s);
        return reject(s, BRAKE_ERROR_IGNORED);
    default:
        return stay(s);
    }
}

BrakeAction brakeOnPoll(BrakeState s, const BrakePollInput &in)
{
    BrakeAction a = stay(s);
    switch (s)
    {
    case BRAKE_RELEASING:
        if (!in.statusFresh)
            break;
        if (in.hiz)
        {
            a.next = BRAKE_ENGAGED;
            a.error = BRAKE_ERROR_RELEASE_FAILED;
        }
        else if (in.elapsed)
        {
            a.next = BRAKE_RELEASED;
        }
        break;
    case BRAKE_RELEASED:
    case BRAKE_STOPPING:
        if (!in.statusFresh)
            break;
        if (in.hiz)
        {
            a.next = BRAKE_ENGAGED;
            a.cancelPending = true;
            a.error = BRAKE_ERROR_ENGAGED_ON_HIZ;
        }
        else if ((s == BRAKE_STOPPING) && in.stopped)
        {
            a = go(BRAKE_ENGAGING);
        }
        break;
    case BRAKE_ENGAGING:
        if (in.statusFresh && in.hiz)
            a.next = BRAKE_ENGAGED;
        else if (in.elapsed)
            a = go(BRAKE_ENGAGED, BRAKE_DRIVER_HARD_HIZ);
        break;
    case BRAKE_ENGAGED:
        if (in.statusFresh && !in.hiz)
        {
            a.driver = BRAKE_DRIVER_HARD_HIZ;
            a.warn = true;
        }
        break;
    case BRAKE_FREE:
        if (in.statusFresh && !in.hiz)
        {
            a.next = BRAKE_RELEASED;
            a.warn = true;
        }
        break;
    default:
        break;
    }
    return a;
}
