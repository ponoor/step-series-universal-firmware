// oscWriter.h
// Builds an outgoing OSC message without heap allocation.
// It does not depend on Arduino so that it can be tested on the host.
//
// The encoding is byte-identical to CNMAT OSCMessage::send() (6cea108):
// address + NUL padded to 4 bytes, ',' + type tags + NUL padding (4 NULs when
// the tag string is already aligned), then the arguments (big endian, strings
// NUL-terminated and padded to 4 bytes).

#ifndef _OSCWRITER_h
#define _OSCWRITER_h

#include <stdint.h>
#include <stddef.h>
#include <type_traits>

// Maximum number of arguments of an outgoing message (/positionList: NUM_OF_MOTOR).
constexpr uint8_t OSC_TX_MAX_ARGS = 10;

class OscWriter
{
public:
    // address must stay valid until encode().
    explicit OscWriter(const char *address) : address(address), count(0), overflow(false) {}

    // Integers of up to 4 bytes (including enums) are sent as 'i'.
    template <typename T>
    typename std::enable_if<(std::is_integral<T>::value || std::is_enum<T>::value) &&
                                !std::is_same<T, bool>::value && (sizeof(T) <= 4),
                            OscWriter &>::type
    add(T value)
    {
        return addInt((int32_t)value);
    }
    OscWriter &add(bool value) { return addBool(value); }
    OscWriter &add(float value) { return addFloat(value); }
    // The string must stay valid until encode().
    OscWriter &add(const char *value) { return addString(value); }

    OscWriter &addInt(int32_t value);
    OscWriter &addFloat(float value);
    OscWriter &addString(const char *value);
    OscWriter &addBool(bool value);

    uint8_t size() const { return count; }
    // Writes the message to buffer. Returns its length, or 0 if it does not fit
    // (or has more than OSC_TX_MAX_ARGS arguments).
    uint16_t encode(uint8_t *buffer, uint16_t capacity) const;

private:
    struct Arg
    {
        char type;
        union
        {
            int32_t i;
            float f;
            const char *s;
        };
    };
    Arg *next();

    const char *address;
    uint8_t count;
    bool overflow;
    Arg args[OSC_TX_MAX_ARGS];
};

#endif
