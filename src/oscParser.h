// oscParser.h
// Zero-copy OSC message parser working on a caller-owned receive buffer.
// It does not depend on Arduino so that it can be tested on the host.
//
// Compatibility: decode() follows the byte-by-byte decoder of CNMAT OSC
// (OSCMessage::decode() at commit 6cea108, v3.5.8) so that well-formed and
// malformed packets give the same address, arguments and error state as before.
// The getters keep the CNMAT names and semantics used by the handlers, including
// the sticky INDEX_OUT_OF_BOUNDS error: once an index past the last argument is
// accessed, hasError() becomes true and every getter returns its default value.

#ifndef _OSCPARSER_h
#define _OSCPARSER_h

#include <stdint.h>
#include <stddef.h>

// Largest packet accepted. The longest standard message, /setTargetPositionList with
// 8 int arguments, is 68 bytes; the margin is for /setConfigName strings.
constexpr uint16_t OSC_RX_BUFFER_SIZE = 512;
// Maximum number of arguments (must cover /setTargetPositionList).
constexpr uint8_t OSC_MAX_ARGS = 16;

class OscMessage
{
public:
    OscMessage() { clear(); }

    // Decodes a packet. data must stay valid and unchanged while the message is used.
    void decode(const uint8_t *data, uint16_t length);

    bool hasError() const;
    // NUL-terminated address, or nullptr if no address was decoded.
    const char *getAddress() const { return addressValid ? address : nullptr; }
    int size() const { return argCount; }

    bool isInt(int position);
    bool isFloat(int position);
    bool isString(int position);
    bool isBoolean(int position);
    char getType(int position);

    int32_t getInt(int position);
    float getFloat(int position);
    bool getBoolean(int position);
    // Copies the string with its NUL. Returns the copied length including the NUL,
    // 0 if the argument is not a string or does not fit, -1 on error.
    int getString(int position, char *buffer, int bufferSize);
    // Size of the argument in bytes (string: length including the NUL).
    int getDataLength(int position);

private:
    struct Arg
    {
        char type;
        bool valid;      // false while the data has not been decoded yet (INVALID_OSC)
        uint16_t offset; // position of the data in the packet
        uint16_t bytes;  // CNMAT OSCData::bytes
    };

    void clear();
    const Arg *getArg(int position);
    bool testType(int position, char type);
    void addType(char type);
    void decodeData(uint8_t incomingByte, uint16_t pos);

    const uint8_t *packet;
    const char *address;
    bool addressValid;
    bool indexError;    // sticky, see the header comment
    bool tooManyArgs;   // more than OSC_MAX_ARGS type tags
    uint8_t argCount;
    Arg args[OSC_MAX_ARGS];

    // decoder state
    enum DecodeState
    {
        STANDBY,
        ADDRESS,
        ADDRESS_PADDING,
        TYPES,
        TYPES_PADDING,
        DATA,
        DATA_PADDING
    } decodeState;
    uint16_t bufferStart; // start of the CNMAT "incomingBuffer" in the packet
};

#endif
