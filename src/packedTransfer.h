// packedTransfer.h
// Reads all motor drivers of the daisy chain with one packed transfer
// (prepare*() / performPrepared() of the driver library).
// Falls back to one transfer per motor if the packed transfer is not possible.

#ifndef _PACKEDTRANSFER_h
#define _PACKEDTRANSFER_h

#include <stdint.h>
#include "globals.h"

// Sends the staged commands of all motors. On failure, prints a warning to USB serial
// once and returns false (nothing was sent, the caller must fall back).
bool performPreparedTransfer();

void readAllPositions(int32_t pos[NUM_OF_MOTOR]);
void readAllStatus(uint16_t status[NUM_OF_MOTOR]);
void readAllParams(uint8_t param, uint32_t values[NUM_OF_MOTOR]);

#endif
