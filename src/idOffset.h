// idOffset.h
// The DIP switch ID added to the IP address and the send port (isMyIpAddId,
// isOutPortAddId), without hardware access, so that it can be tested on the host.
//
// The ID is added only if the result stays in range: 1-254 for the last byte of the IP
// address (255 is the broadcast address of a /24 network, and more would wrap around),
// 0-65535 for the port. Otherwise the configured value is used as is. With the default
// address 10.0.0.100 the largest ID is therefore 154 (STEP800: the DIP switch goes to 255).

#ifndef _IDOFFSET_h
#define _IDOFFSET_h

#include <stdint.h>

// Sets out to configured + id and returns true if that is 254 or less; otherwise sets out
// to configured and returns false. (0 only results from 0 + 0, configured as is.)
bool addIdToIpLastByte(uint8_t configured, uint8_t id, uint8_t &out);

// Sets out to configured + id and returns true if that is 65535 or less; otherwise sets
// out to configured and returns false.
bool addIdToPort(uint16_t configured, uint8_t id, uint16_t &out);

#endif
