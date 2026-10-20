// oscParser.cpp

#include <string.h>
#include "oscParser.h"

static inline int padSize(int bytes) { return (4 - (bytes & 3)) & 3; }

static uint32_t readBigEndian32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

void OscMessage::clear()
{
    packet = nullptr;
    address = nullptr;
    addressValid = false;
    indexError = false;
    tooManyArgs = false;
    argCount = 0;
    decodeState = STANDBY;
    bufferStart = 0;
}

// Same state machine as CNMAT OSCMessage::decode(). The CNMAT incomingBuffer always
// holds the bytes received since it was last cleared, i.e. packet[bufferStart..pos].
void OscMessage::decode(const uint8_t *data, uint16_t length)
{
    clear();
    packet = data;
    for (uint16_t pos = 0; pos < length; pos++)
    {
        const uint8_t incomingByte = data[pos];
        const int bufferSize = pos - bufferStart + 1; // including this byte
        switch (decodeState)
        {
        case STANDBY:
            if (incomingByte == '/')
                decodeState = ADDRESS;
            break;
        case ADDRESS:
            if (incomingByte == 0)
            {
                // The address is the buffer as a C string. It may end before this byte
                // if bytes before the '/' contained a NUL (e.g. "#bundle").
                address = (const char *)(data + bufferStart);
                addressValid = true;
                bufferStart = pos + 1;
                decodeState = ADDRESS_PADDING;
            }
            break;
        case ADDRESS_PADDING:
            if (incomingByte == ',')
            {
                decodeState = TYPES;
                bufferStart = pos + 1;
            }
            break;
        case TYPES:
            if (incomingByte != 0)
                addType((char)incomingByte);
            else
                decodeState = TYPES_PADDING;
            // fall through
        case TYPES_PADDING:
        {
            int typePad = padSize(argCount + 1); // 1 is the comma
            if (typePad == 0)
                typePad = 4;
            if (bufferSize == typePad + argCount)
            {
                bufferStart = pos + 1;
                decodeState = DATA;
            }
            break;
        }
        case DATA:
            decodeData(incomingByte, pos);
            break;
        case DATA_PADDING:
            // padding of the last decoded argument
            for (int i = argCount - 1; i >= 0; i--)
            {
                if (args[i].valid)
                {
                    const int dataPad = padSize(args[i].bytes);
                    if (dataPad == 0)
                    {
                        // CNMAT keeps the buffer: this byte belongs to the next argument.
                        decodeState = DATA;
                    }
                    else if (bufferSize == dataPad)
                    {
                        bufferStart = pos + 1;
                        decodeState = DATA;
                    }
                    break;
                }
            }
            break;
        }
    }
}

// CNMAT OSCMessage::add(char): T and F carry no data and are valid at once.
void OscMessage::addType(char type)
{
    if (argCount >= OSC_MAX_ARGS)
    {
        tooManyArgs = true;
        return;
    }
    Arg &arg = args[argCount++];
    arg.type = type;
    arg.valid = (type == 'T' || type == 'F');
    arg.offset = 0;
    arg.bytes = 0;
}

// CNMAT OSCMessage::decodeData(): fills the first argument that has no data yet.
// Types without a case (e.g. 'c', 'N', 'I') never become valid.
void OscMessage::decodeData(uint8_t incomingByte, uint16_t pos)
{
    const int bufferSize = pos - bufferStart + 1;
    for (uint8_t i = 0; i < argCount; i++)
    {
        Arg &arg = args[i];
        if (arg.valid)
            continue;
        int dataBytes = 0;
        switch (arg.type)
        {
        case 'i':
        case 'f':
        case 'r':
        case 'm':
            if (bufferSize == 4)
                dataBytes = 4;
            break;
        case 'h':
        case 'd':
        case 't':
            if (bufferSize == 8)
                dataBytes = 8;
            break;
        case 's':
            if (incomingByte == 0)
            {
                // strlen() of the buffer, which may stop before this byte
                const char *str = (const char *)(packet + bufferStart);
                dataBytes = (int)strnlen(str, bufferSize) + 1;
                decodeState = DATA_PADDING;
            }
            break;
        case 'b':
            if (bufferSize > 4)
            {
                const uint32_t blobLength = readBigEndian32(packet + bufferStart);
                if (bufferSize == (int)(blobLength + 4))
                {
                    dataBytes = blobLength + 4;
                    decodeState = DATA_PADDING;
                }
            }
            break;
        default:
            break;
        }
        if (dataBytes > 0)
        {
            arg.valid = true;
            arg.offset = bufferStart;
            arg.bytes = dataBytes;
            bufferStart = pos + 1;
        }
        break; // only the first argument without data
    }
}

bool OscMessage::hasError() const
{
    if (!addressValid || indexError || tooManyArgs)
        return true;
    for (uint8_t i = 0; i < argCount; i++)
    {
        if (!args[i].valid)
            return true;
    }
    return false;
}

// CNMAT OSCMessage::getOSCData(): an index past the end sets a sticky error.
const OscMessage::Arg *OscMessage::getArg(int position)
{
    if (position >= 0 && position < argCount)
        return &args[position];
    indexError = true;
    return nullptr;
}

bool OscMessage::testType(int position, char type)
{
    const Arg *arg = getArg(position);
    if (hasError())
        return false;
    return arg->type == type;
}

bool OscMessage::isInt(int position) { return testType(position, 'i'); }
bool OscMessage::isFloat(int position) { return testType(position, 'f'); }
bool OscMessage::isString(int position) { return testType(position, 's'); }
bool OscMessage::isBoolean(int position) { return testType(position, 'T') || testType(position, 'F'); }

char OscMessage::getType(int position)
{
    const Arg *arg = getArg(position);
    if (hasError())
        return '\0';
    return arg->type;
}

int32_t OscMessage::getInt(int position)
{
    const Arg *arg = getArg(position);
    if (hasError() || arg->type != 'i')
        return 0;
    return (int32_t)readBigEndian32(packet + arg->offset);
}

float OscMessage::getFloat(int position)
{
    const Arg *arg = getArg(position);
    if (hasError() || arg->type != 'f')
        return 0.0f;
    const uint32_t bits = readBigEndian32(packet + arg->offset);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

bool OscMessage::getBoolean(int position)
{
    const Arg *arg = getArg(position);
    if (hasError())
        return false;
    return arg->type == 'T';
}

int OscMessage::getString(int position, char *buffer, int bufferSize)
{
    const Arg *arg = getArg(position);
    if (hasError())
        return -1;
    // CNMAT: the copy size is min(bufferSize, bytes), and nothing is copied
    // unless the whole string with its NUL fits.
    const int copyBytes = bufferSize < arg->bytes ? bufferSize : arg->bytes;
    if (arg->type != 's' || arg->bytes > copyBytes)
        return 0;
    strncpy(buffer, (const char *)(packet + arg->offset), arg->bytes);
    return arg->bytes;
}

int OscMessage::getDataLength(int position)
{
    const Arg *arg = getArg(position);
    if (hasError())
        return 0;
    return arg->bytes;
}
