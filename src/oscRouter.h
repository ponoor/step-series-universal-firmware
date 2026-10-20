// oscRouter.h
// Table-driven OSC dispatch. It does not depend on Arduino so that it can be tested
// on the host.
//
// Fast path: the received address is looked up by exact match (binary search over
// an index sorted at startup) and only that handler is called.
// Slow path: if there is no exact match, every table entry is tested in table order
// with the rule of CNMAT OSCMessage::route() (6cea108) and all matching handlers are
// called. This keeps two undocumented behaviors of the former implementation:
//  - the received address is used as an OSC pattern (e.g. /enable*Report, /set{Acc,Dec})
//  - an address followed by '/...' matches (e.g. /run/foo calls /run)
// An exact match can only call one handler because no table address contains pattern
// characters, is duplicated, or is a '/'-prefix of another (checked by the host tests).

#ifndef _OSCROUTER_h
#define _OSCROUTER_h

#include <stdint.h>
#include "oscParser.h"

typedef void (*OscHandler)(OscMessage &msg);
// Checks the arguments of msg against the route's args (oscArgs.h) and reports the error.
// Returns false if the handler must not be called.
typedef bool (*OscArgValidator)(OscMessage &msg, const char *args);

struct OscRoute
{
    const char *address;
    OscHandler handler;
    const char *args; // see oscRoutes.h
};

// Bytes reserved before and after the packet in the receive buffer.
// osc_match() scans past the end of the received pattern when a '[' or '{' is not
// closed (and then scans back for '[', '{'). The guards keep these scans inside
// the buffer. See oscSetRxGuards().
constexpr uint8_t OSC_RX_GUARD_BEFORE = 2;
constexpr uint8_t OSC_RX_GUARD_AFTER = 5;
// Writes the guard bytes around packet[0..length-1]. packet must have
// OSC_RX_GUARD_BEFORE bytes before it and OSC_RX_GUARD_AFTER bytes after length.
void oscSetRxGuards(uint8_t *packet, uint16_t length);

// Same rule as CNMAT OSCMessage::match() > 0 (used by route()).
bool oscRouteMatches(const char *receivedAddress, const char *routeAddress);

enum OscDispatchResult
{
    OSC_DISPATCH_NOT_MATCHED = 0,
    OSC_DISPATCH_FAST,
    OSC_DISPATCH_SLOW
};

class OscRouter
{
public:
    // index must have room for count entries. Sorts the index by address. With a
    // validator, a handler is called only if the validator returns true for its args.
    void begin(const OscRoute *routes, uint8_t count, uint8_t *index,
               OscArgValidator validator = nullptr);
    // Calls the matching handler(s) of a message without errors.
    OscDispatchResult dispatch(OscMessage &msg) const;
    // Exact-match lookup, or -1.
    int find(const char *address) const;

private:
    const OscRoute *routes = nullptr;
    uint8_t count = 0;
    uint8_t *index = nullptr;
    OscArgValidator validator = nullptr;
    void call(const OscRoute &route, OscMessage &msg) const;
};

#endif
