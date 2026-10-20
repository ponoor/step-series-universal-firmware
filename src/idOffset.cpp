// idOffset.cpp

#include "idOffset.h"

bool addIdToIpLastByte(uint8_t configured, uint8_t id, uint8_t &out)
{
    const uint16_t sum = (uint16_t)configured + id;
    if (sum > 254)
    {
        out = configured;
        return false;
    }
    out = (uint8_t)sum;
    return true;
}

bool addIdToPort(uint16_t configured, uint8_t id, uint16_t &out)
{
    const uint32_t sum = (uint32_t)configured + id;
    if (sum > 0xFFFFu)
    {
        out = configured;
        return false;
    }
    out = (uint16_t)sum;
    return true;
}
