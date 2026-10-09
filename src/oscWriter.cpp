// oscWriter.cpp

#include <string.h>
#include "oscWriter.h"

static inline uint16_t padSize(uint16_t bytes) { return (4 - (bytes & 3)) & 3; }

OscWriter::Arg *OscWriter::next()
{
    if (count >= OSC_TX_MAX_ARGS)
    {
        overflow = true;
        return nullptr;
    }
    return &args[count++];
}

OscWriter &OscWriter::addInt(int32_t value)
{
    Arg *arg = next();
    if (arg)
    {
        arg->type = 'i';
        arg->i = value;
    }
    return *this;
}

OscWriter &OscWriter::addFloat(float value)
{
    Arg *arg = next();
    if (arg)
    {
        arg->type = 'f';
        arg->f = value;
    }
    return *this;
}

OscWriter &OscWriter::addString(const char *value)
{
    Arg *arg = next();
    if (arg)
    {
        arg->type = 's';
        arg->s = value;
    }
    return *this;
}

OscWriter &OscWriter::addBool(bool value)
{
    Arg *arg = next();
    if (arg)
        arg->type = value ? 'T' : 'F';
    return *this;
}

namespace
{
    struct Output
    {
        uint8_t *buffer;
        uint16_t capacity;
        uint16_t length;
        bool ok;

        void write(const void *data, uint16_t n)
        {
            if (!ok || n > capacity - length)
            {
                ok = false;
                return;
            }
            memcpy(buffer + length, data, n);
            length += n;
        }
        void zeros(uint16_t n)
        {
            if (!ok || n > capacity - length)
            {
                ok = false;
                return;
            }
            memset(buffer + length, 0, n);
            length += n;
        }
        void bigEndian32(uint32_t v)
        {
            const uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};
            write(b, 4);
        }
        // NUL-terminated string padded to 4 bytes
        void paddedString(const char *s)
        {
            const size_t len = strlen(s) + 1;
            if (len > 0xFFFF)
            {
                ok = false;
                return;
            }
            write(s, (uint16_t)len);
            zeros(padSize((uint16_t)len));
        }
    };
}

uint16_t OscWriter::encode(uint8_t *buffer, uint16_t capacity) const
{
    if (overflow || address == nullptr)
        return 0;
    Output out = {buffer, capacity, 0, true};
    out.paddedString(address);
    out.write(",", 1);
    for (uint8_t i = 0; i < count; i++)
        out.write(&args[i].type, 1);
    uint16_t typePad = padSize(count + 1);
    if (typePad == 0)
        typePad = 4; // the type tag string must be NUL-terminated
    out.zeros(typePad);
    for (uint8_t i = 0; i < count; i++)
    {
        const Arg &arg = args[i];
        switch (arg.type)
        {
        case 'i':
            out.bigEndian32((uint32_t)arg.i);
            break;
        case 'f':
        {
            uint32_t bits;
            memcpy(&bits, &arg.f, sizeof(bits));
            out.bigEndian32(bits);
            break;
        }
        case 's':
            out.paddedString(arg.s);
            break;
        default: // 'T', 'F': no data
            break;
        }
    }
    return out.ok ? out.length : 0;
}
