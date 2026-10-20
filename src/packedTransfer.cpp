// packedTransfer.cpp

#include "packedTransfer.h"
#include "utils.h"

bool performPreparedTransfer()
{
    static bool warned = false;
    if (MotorDriver::performPrepared())
        return true;
    if (!warned)
    {
        warned = true;
        dbgPrint("Warning: packed transfer failed, using one transfer per motor\n");
    }
    return false;
}

void readAllPositions(int32_t pos[NUM_OF_MOTOR])
{
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        stepper[i].prepareGetPos();
    if (performPreparedTransfer())
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
            pos[i] = stepper[i].preparedPos();
    }
    else
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
            pos[i] = stepper[i].getPos();
    }
}

// GetStatus clears the latched flags in the same way as getStatus().
void readAllStatus(uint16_t status[NUM_OF_MOTOR])
{
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        stepper[i].prepareGetStatus();
    if (performPreparedTransfer())
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
            status[i] = stepper[i].preparedStatus();
    }
    else
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
            status[i] = stepper[i].getStatus();
    }
}

void readAllParams(uint8_t param, uint32_t values[NUM_OF_MOTOR])
{
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        stepper[i].prepareGetParam(param);
    if (performPreparedTransfer())
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
            values[i] = stepper[i].preparedResult();
    }
    else
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
            values[i] = stepper[i].getParam(param);
    }
}
