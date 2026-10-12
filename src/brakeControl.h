// brakeControl.h
// State machine of the electromagnetic brake of one motor, without hardware access, so
// that it can be tested on the host. The firmware feeds it the commands and the polled
// driver status, and carries out the returned action.
//
// Brake output: engaged (holding) or released. Driver: HiZ or energized.
// The state numbers are part of the OSC specification (/brakeStatus). Never change the
// meaning of an existing number; add new states as 6, 7, ...
//
// Invariants:
// 1. While the output is engaged (ENGAGED, RELEASING, ENGAGING), no command makes the
//    motor turn. In ENGAGED and FREE, no command energizes the driver (HardStop and
//    SoftStop do in HiZ), except /activate 1 and a motion command from FREE.
// 2. The brake is released only after a status read after the release trigger shows
//    HiZ = 0.
// 3. HiZ in RELEASED or STOPPING engages the brake at once.
// 4. The brake is not engaged on a turning motor.
// The brake is not used for a motor with the brake disabled (output kept engaged).

#ifndef _BRAKECONTROL_h
#define _BRAKECONTROL_h

#include <stdint.h>

enum BrakeState : uint8_t
{
    BRAKE_ENGAGED = 0,   // output engaged, driver HiZ
    BRAKE_RELEASING = 1, // output engaged, driver energized, waiting the transition time
    BRAKE_RELEASED = 2,  // output released, driver energized
    BRAKE_STOPPING = 3,  // output released, decelerating; engages when stopped
    BRAKE_ENGAGING = 4,  // output engaged, driver energized, waiting the transition time
    BRAKE_FREE = 5,      // output released, driver HiZ (/free)
    BRAKE_STATE_COUNT
};

enum BrakeCommand : uint8_t
{
    BRAKE_CMD_ACTIVATE,     // /activate 1
    BRAKE_CMD_DEACTIVATE,   // /activate 0
    BRAKE_CMD_SOFT_HIZ,     // /softHiZ
    BRAKE_CMD_HARD_HIZ,     // /hardHiZ
    BRAKE_CMD_FREE,         // /free
    BRAKE_CMD_MOTION,       // /run, /move, /goTo, ..., /homing, /enableServoMode 1
    BRAKE_CMD_STOP,         // /softStop, /hardStop, /enableServoMode 0
    BRAKE_CMD_RESET_DRIVER, // /resetMotorDriver
    BRAKE_CMD_COUNT
};

// Driver command sent by the brake layer (before the command itself, if executed).
enum BrakeDriverCommand : uint8_t
{
    BRAKE_DRIVER_NONE,
    BRAKE_DRIVER_HARD_STOP,
    BRAKE_DRIVER_SOFT_STOP,
    BRAKE_DRIVER_HARD_HIZ
};

enum BrakeError : uint8_t
{
    BRAKE_ERROR_NONE,
    BRAKE_ERROR_ENGAGED,         // BrakeEngaged: the command is not executed
    BRAKE_ERROR_IGNORED,         // CommandIgnored: the command is not executed
    BRAKE_ERROR_RELEASE_FAILED,  // BrakeReleaseFailed (poll)
    BRAKE_ERROR_ENGAGED_ON_HIZ   // BrakeEngagedOnHiZ (poll)
};

struct BrakeAction
{
    BrakeState next;
    BrakeDriverCommand driver;
    BrakeError error;
    bool execute;       // carry out the command itself (motion, stop, driver reset)
    bool startTimer;    // record the start of the transition time
    bool cancelPending; // cancel the pending motion (poll: HiZ detected)
    bool warn;          // unexpected driver state (poll)
};

// Driver status read by the poll.
struct BrakePollInput
{
    bool hiz;         // HiZ flag
    bool stopped;     // MOT_STATUS == 0
    bool statusFresh; // the status was read after the last action of the brake layer
    bool elapsed;     // brakeTransitionDuration elapsed since the timer start
};

// true: the brake output is released in this state.
bool brakeOutputReleased(BrakeState state);

// Action for a command. For BRAKE_CMD_MOTION, the error tells whether the command may
// start; apply next only when the command is actually sent (other start conditions
// can still reject it).
BrakeAction brakeOnCommand(BrakeState state, BrakeCommand command);

// Action for a status poll (every 10 ms).
BrakeAction brakeOnPoll(BrakeState state, const BrakePollInput &in);

#endif
