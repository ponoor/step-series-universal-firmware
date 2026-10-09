// oscRouter.cpp

#include <string.h>
#include "oscRouter.h"
#include "OSCMatch.h"

void oscSetRxGuards(uint8_t *packet, uint16_t length)
{
    // Backward scans stop at '[' or '{'.
    packet[-2] = '[';
    packet[-1] = '{';
    // Forward scans stop at NUL, ']' or '}'; each closing bracket is followed by a NUL.
    uint8_t *p = packet + length;
    p[0] = '\0';
    p[1] = ']';
    p[2] = '\0';
    p[3] = '}';
    p[4] = '\0';
}

bool oscRouteMatches(const char *receivedAddress, const char *routeAddress)
{
    int patternOffset, addressOffset;
    int ret = osc_match(receivedAddress, routeAddress, &patternOffset, &addressOffset);
    if (ret == (OSC_MATCH_ADDRESS_COMPLETE | OSC_MATCH_PATTERN_COMPLETE))
        return true;
    return (patternOffset > 0) && (receivedAddress[patternOffset] == '/');
}

void OscRouter::begin(const OscRoute *routes_, uint8_t count_, uint8_t *index_)
{
    routes = routes_;
    count = count_;
    index = index_;
    // insertion sort, once at startup
    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t j = i;
        while (j > 0 && strcmp(routes[index[j - 1]].address, routes[i].address) > 0)
        {
            index[j] = index[j - 1];
            j--;
        }
        index[j] = i;
    }
}

int OscRouter::find(const char *address) const
{
    int lo = 0, hi = (int)count - 1;
    while (lo <= hi)
    {
        const int mid = (lo + hi) / 2;
        const int c = strcmp(address, routes[index[mid]].address);
        if (c == 0)
            return index[mid];
        if (c < 0)
            hi = mid - 1;
        else
            lo = mid + 1;
    }
    return -1;
}

OscDispatchResult OscRouter::dispatch(OscMessage &msg) const
{
    const char *address = msg.getAddress();
    if (address == nullptr)
        return OSC_DISPATCH_NOT_MATCHED;
    const int i = find(address);
    if (i >= 0)
    {
        routes[i].handler(msg);
        return OSC_DISPATCH_FAST;
    }
    bool matched = false;
    for (uint8_t k = 0; k < count; k++)
    {
        if (oscRouteMatches(address, routes[k].address))
        {
            routes[k].handler(msg);
            matched = true;
        }
    }
    return matched ? OSC_DISPATCH_SLOW : OSC_DISPATCH_NOT_MATCHED;
}
