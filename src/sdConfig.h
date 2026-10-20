// sdConfig.h
// SdFat build options. Include this before <SdFat.h>.
//
// PlatformIO passes the same two values as build flags (platformio.ini, [samd_base]).
// The Arduino IDE has no build flags, so SdFat would use its defaults: SDFAT_FILE_TYPE 3
// (FAT and exFAT), where `SdFat sd` is an SdFs and sd.begin() accepts an exFAT card
// although the code opens its files as File32. Saving to such a card then fails after
// sd.remove() has deleted the file. With type 1 an exFAT card is "No SD card".

#ifndef _SDCONFIG_h
#define _SDCONFIG_h
#pragma once

#ifndef SDFAT_FILE_TYPE
#define SDFAT_FILE_TYPE 1
#endif
#ifndef USE_LONG_FILE_NAMES
#define USE_LONG_FILE_NAMES 1
#endif

#endif
