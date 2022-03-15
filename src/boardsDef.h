// boards definitions

// other board dependended functions:
// - getId
// 
#ifndef _BOARDSDEF_h
#define _BOARDSDEF_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <SPI.h>

// Products
#define STEP400_R1
// #define STEP800_R1

// X-Nucleos
// #define STEP100_R1 // X-NUCLEO-IHM03A1, PowerSTEP01 x1
// #define STEP200_R1 // X-NUCLEO-IHM02A1, L6470 x2 

// Prototypes
// #define STEP400_PROTO_R4 // rev.2020-04-06
// #define STEP400_PROTO_R3 // rev.2020-01-07
// #define STEP800_PROTO_R3
// #define STEP800_PROTO_R1

// Uncomment this line if the board is M0 with Zero bootloader (Swap D2 and D4) 
// Only work for STEP100/STEP200
// #define MZERO_WITH_ZERO_BOOTLOADER

// Common version number
const uint8_t firmwareVersion[3] = {1,0,0};

#ifdef STEP400_R1
    #define PRODUCT_NAME    "STEP400"
    #define FIRMWARE_NAME   "STEP400_R1_UNIVERSAL"
    const uint8_t applicableConfigVersion[2] = {1,2};
    #define NUM_OF_MOTOR (4)
    #define HAVE_SD
    #define HAVE_BRAKE
    #define HAVE_LIMIT_ADC
    #define HAVE_DIP_SW
    #define DIP_SW_DIGITS   8
    #define DRIVER_POWERSTEP01
    #define DRIVER_OSC EXT_16MHZ_OSCOUT_INVERT
    #define DRIVER_RESET_PIN_OPERATION_STATE  HIGH
    #define W5500_RESET_PIN_OPERATION_STATE    HIGH // HIGH: Operation, LOW: RESET

    // pins
    #define ledPin 13u
    const uint8_t dipSwPin[8] = { 7u,30u,A5,31u,2u,9u,3u,0u };
    #define SD_CS_PIN	4u
    const uint8_t brakePin[4] = { A1,8u,5u,1u };
    #define SD_DETECT_PIN   A4
    #define PIN_DRIVER_MISO 6u	// D6 /SERCOM3/PAD[2] miso
    #define PIN_DRIVER_MOSI 11u	// D11/SERCOM3/PAD[0] mosi
    #define PIN_DRIVER_SCK	12u	// D12/SERCOM3/PAD[3] sck
    #define EPIO_DRIVER_MISO    PIO_SERCOM_ALT
    #define EPIO_DRIVER_MOSI    PIO_SERCOM_ALT
    #define EPIO_DRIVER_SCK PIO_SERCOM_ALT
    #define PAD_DRIVER_SPI  SPI_PAD_0_SCK_3
    #define PAD_DRIVER_RX   SERCOM_RX_PAD_2
    #define PIN_DRIVER_CS   A0
    #define PIN_DRIVER_RESET A2
    #define W5500_RESET_PIN A3
    #define DRIVER_SERCOM sercom3

#elif defined(STEP400_PROTO_R4)
    #define PRODUCT_NAME    "STEP400"
    #define FIRMWARE_NAME   "STEP400_PROTO_R4_UNIVERSAL"
    const uint8_t applicableConfigVersion[2] = {1,2};
    #define NUM_OF_MOTOR (4)
    #define HAVE_SD
    #define HAVE_LIMIT_GPIO
    #define HAVE_DIP_SW
    #define DIP_SW_DIGITS   (8)
    #define DRIVER_POWERSTEP01
    #define DRIVER_OSC EXT_16MHZ_OSCOUT_INVERT
    #define DRIVER_RESET_PIN_OPERATION_STATE  HIGH
    #define W5500_RESET_PIN_OPERATION_STATE    HIGH // HIGH: Operation, LOW: RESET
    // pins
    #define ledPin 13u
    const uint8_t dipSwPin[8] = { A5,SCL,7u,SDA,2u,9u,3u,0u }; // Prototype_r4
    const uint8_t limitSwPin[4] = { 1u,5u,8u,A1 };
    #define SD_CS_PIN	4u
    #define SD_DETECT_PIN   A4
    #define PIN_DRIVER_MISO 6u	// D6 /SERCOM3/PAD[2] miso
    #define PIN_DRIVER_MOSI 11u	// D11/SERCOM3/PAD[0] mosi
    #define PIN_DRIVER_SCK	12u	// D12/SERCOM3/PAD[3] sck
    #define EPIO_DRIVER_MISO    PIO_SERCOM_ALT
    #define EPIO_DRIVER_MOSI    PIO_SERCOM_ALT
    #define EPIO_DRIVER_SCK PIO_SERCOM_ALT
    #define PAD_DRIVER_SPI  SPI_PAD_0_SCK_3
    #define PAD_DRIVER_RX   SERCOM_RX_PAD_2
    #define PIN_DRIVER_CS   A0
    #define PIN_DRIVER_RESET    A2
    #define W5500_RESET_PIN A3

    #define DRIVER_SERCOM sercom3
#elif defined(STEP400_PROTO_R3)
    #define PRODUCT_NAME    "STEP400"
    #define FIRMWARE_NAME   "STEP400_PROTO_R3_UNIVERSAL"
    const uint8_t applicableConfigVersion[2] = {1,2};
    #define NUM_OF_MOTOR (4)
    #define HAVE_SD
    // #define HAVE_LIMIT_GPIO
    #define HAVE_DIP_SW
    #define DIP_SW_DIGITS   (8)
    #define DRIVER_POWERSTEP01
    #define DRIVER_OSC EXT_24MHZ_OSCOUT_INVERT
    #define DRIVER_RESET_PIN_OPERATION_STATE  HIGH
    #define W5500_RESET_PIN_OPERATION_STATE    HIGH // HIGH: Operation, LOW: RESET
    // pins
    #define ledPin 13u
    const uint8_t dipSwPin[8] = { A5,SCL,7u,SDA,2u,9u,3u,0u }; // Prototype_r4
    const uint8_t limitSwPin[4] = { 1u,5u,8u,A1 };
    #define SD_CS_PIN	4u
    #define SD_DETECT_PIN   A4
    #define PIN_DRIVER_MISO 6u	// D6 /SERCOM3/PAD[2] miso
    #define PIN_DRIVER_MOSI 11u	// D11/SERCOM3/PAD[0] mosi
    #define PIN_DRIVER_SCK	12u	// D12/SERCOM3/PAD[3] sck
    #define EPIO_DRIVER_MISO    PIO_SERCOM_ALT
    #define EPIO_DRIVER_MOSI    PIO_SERCOM_ALT
    #define EPIO_DRIVER_SCK PIO_SERCOM_ALT
    #define PAD_DRIVER_SPI  SPI_PAD_0_SCK_3
    #define PAD_DRIVER_RX   SERCOM_RX_PAD_2
    #define PIN_DRIVER_CS   A0
    #define PIN_DRIVER_RESET    A2
    #define W5500_RESET_PIN A3

    #define DRIVER_SERCOM sercom3
#elif defined(STEP800_R1)
    #define PRODUCT_NAME    "STEP800"
    #define FIRMWARE_NAME   "STEP800_R1_UNIVERSAL"
    const uint8_t applicableConfigVersion[2] = {1,2};
    #define NUM_OF_MOTOR (8)
    #define HAVE_SD
    #define HAVE_BRAKE
    #define HAVE_DIP_SW_SPI
    #define DRIVER_L6470
    #define DRIVER_OSC EXT_16MHZ_OSCOUT_INVERT
    #define DRIVER_RESET_PIN_OPERATION_STATE   HIGH
    #define W5500_RESET_PIN_OPERATION_STATE    HIGH // HIGH: Operation, LOW: RESET

    #define ledPin 13u
    #define SD_CS_PIN	4u
    #define SD_DETECT_PIN   A4
    #define PIN_DRIVER_MISO 6u	// D6 /SERCOM3/PAD[2] miso
    #define PIN_DRIVER_MOSI 11u	// D11/SERCOM3/PAD[0] mosi
    #define PIN_DRIVER_SCK	12u	// D12/SERCOM3/PAD[3] sck
    #define EPIO_DRIVER_MISO    PIO_SERCOM_ALT
    #define EPIO_DRIVER_MOSI    PIO_SERCOM_ALT
    #define EPIO_DRIVER_SCK PIO_SERCOM_ALT
    #define PAD_DRIVER_SPI  SPI_PAD_0_SCK_3
    #define PAD_DRIVER_RX   SERCOM_RX_PAD_2
    #define PIN_DRIVER_CS   A0
    #define PIN_DRIVER_RESET    A2
    #define W5500_RESET_PIN A3

    #define DRIVER_SERCOM   sercom3

    #define PIN_DIPSW_MISO  3u
    #define PIN_DIPSW_MOSI  2u
    #define PIN_DIPSW_SCK   0u
    #define EPIO_DIPSW_MISO PIO_SERCOM_ALT
    #define EPIO_DIPSW_MOSI PIO_SERCOM
    #define EPIO_DIPSW_SCK  PIO_SERCOM_ALT
    #define PAD_DIPSW_SPI   SPI_PAD_2_SCK_3
    #define PAD_DIPSW_RX    SERCOM_RX_PAD_1
    #define PIN_DIPSW_LATCH A5

    #define DIPSW_SERCOM    sercom2
    
    #define PIN_BRAKE_SHIFTOUT_ENABLE    5u

#elif defined(STEP800_PROTO_R3)
    #define PRODUCT_NAME    "STEP800"
    #define FIRMWARE_NAME   "STEP800_PROTO_R3_UNIVERSAL"
    const uint8_t applicableConfigVersion[2] = {1,2};
    #define NUM_OF_MOTOR (8)
    #define HAVE_SD
    #define HAVE_DIP_SW_SPI
    #define DRIVER_L6470
    #define DRIVER_OSC EXT_16MHZ_OSCOUT_INVERT
    
    #define DRIVER_RESET_PIN_OPERATION_STATE   HIGH
    #define W5500_RESET_PIN_OPERATION_STATE    HIGH // HIGH: Operation, LOW: RESET

    #define ledPin 13u
    #define SD_CS_PIN	4u
    #define SD_DETECT_PIN   A4
    #define PIN_DRIVER_MISO 6u	// D6 /SERCOM3/PAD[2] miso
    #define PIN_DRIVER_MOSI 11u	// D11/SERCOM3/PAD[0] mosi
    #define PIN_DRIVER_SCK	12u	// D12/SERCOM3/PAD[3] sck
    #define EPIO_DRIVER_MISO    PIO_SERCOM_ALT
    #define EPIO_DRIVER_MOSI    PIO_SERCOM_ALT
    #define EPIO_DRIVER_SCK PIO_SERCOM_ALT
    #define PAD_DRIVER_SPI  SPI_PAD_0_SCK_3
    #define PAD_DRIVER_RX   SERCOM_RX_PAD_2
    #define PIN_DRIVER_CS   A0
    #define PIN_DRIVER_RESET    A2
    #define W5500_RESET_PIN A3

    #define DRIVER_SERCOM   sercom3

    #define PIN_DIPSW_MISO  3u
    #define PIN_DIPSW_MOSI  2u
    #define PIN_DIPSW_SCK   0u
    #define EPIO_DIPSW_MISO PIO_SERCOM_ALT
    #define EPIO_DIPSW_MOSI PIO_SERCOM
    #define EPIO_DIPSW_SCK  PIO_SERCOM_ALT
    #define PAD_DIPSW_SPI   SPI_PAD_2_SCK_3
    #define PAD_DIPSW_RX    SERCOM_RX_PAD_1
    #define PIN_DIPSW_LATCH A5

    #define DIPSW_SERCOM    sercom2

    #define SETUP_SW_PIN	5u

#elif defined(STEP800_PROTO_R1)
    #define PRODUCT_NAME    "STEP800"
    #define FIRMWARE_NAME   "STEP800_PROTO_R1_UNIVERSAL"
    const uint8_t applicableConfigVersion[2] = {1,2};
    #define NUM_OF_MOTOR (8)
    #define HAVE_DIP_SW_SPI
    #define DRIVER_L6470
    #define DRIVER_OSC EXT_24MHZ_OSCOUT_INVERT
    #define DRIVER_RESET_PIN_OPERATION_STATE   LOW
    #define W5500_RESET_PIN_OPERATION_STATE    LOW

    #define ledPin 4u
    #define PIN_DRIVER_MISO 12u	// D6 /SERCOM3/PAD[2] miso
    #define PIN_DRIVER_MOSI 11u	// D11/SERCOM3/PAD[0] mosi
    #define PIN_DRIVER_SCK	13u	// D12/SERCOM3/PAD[3] sck
    #define EPIO_DRIVER_MISO    PIO_SERCOM
    #define EPIO_DRIVER_MOSI    PIO_SERCOM
    #define EPIO_DRIVER_SCK     PIO_SERCOM
    #define PAD_DRIVER_SPI  SPI_PAD_0_SCK_1
    #define PAD_DRIVER_RX   SERCOM_RX_PAD_3
    #define PIN_DRIVER_CS   9u
    #define PIN_DRIVER_RESET    A5
    #define W5500_RESET_PIN 8u

    #define DRIVER_SERCOM   sercom1

    #define PIN_DIPSW_MISO  A3
    #define PIN_DIPSW_SCK   A2
    #define PIN_DIPSW_LATCH A1

#elif defined(STEP100_R1)
    #define PRODUCT_NAME    "STEP100"
    #define FIRMWARE_NAME   "STEP100_R1_UNIVERSAL"
    const uint8_t applicableConfigVersion[2] = {1,2};
    #define NUM_OF_MOTOR (1)
    #define HAVE_SD
    // #define HAVE_LIMIT_GPIO
    // #define HAVE_DIP_SW
    #define DRIVER_POWERSTEP01
    #define DRIVER_OSC INT_16MHZ
    #define DRIVER_RESET_PIN_OPERATION_STATE   HIGH
    // #define W5500_RESET_PIN_OPERATION_STATE    HIGH
    // pins
    #define ledPin 3u
#ifdef MZERO_WITH_ZERO_BOOTLOADER
    #define SD_CS_PIN   2u
#else
    #define SD_CS_PIN	4u
#endif
    // #define SD_DETECT_PIN   A4
    #define PIN_DRIVER_MISO 12u	// 
    #define PIN_DRIVER_MOSI 11u	// 
    #define PIN_DRIVER_SCK	13u	// 
    #define EPIO_DRIVER_MISO    PIO_SERCOM
    #define EPIO_DRIVER_MOSI    PIO_SERCOM
    #define EPIO_DRIVER_SCK     PIO_SERCOM
    #define PAD_DRIVER_SPI  SPI_PAD_0_SCK_1
    #define PAD_DRIVER_RX   SERCOM_RX_PAD_3
    #define PIN_DRIVER_CS   A2
    #define PIN_DRIVER_RESET    8u
    // #define W5500_RESET_PIN A3

    #define DRIVER_SERCOM sercom1

    #define ID  1

#elif defined(STEP200_R1)
    #define PRODUCT_NAME    "STEP200"
    #define FIRMWARE_NAME   "STEP200_R1_UNIVERSAL"
    const uint8_t applicableConfigVersion[2] = {1,2};
    #define NUM_OF_MOTOR (2)
    #define HAVE_SD
    // #define HAVE_LIMIT_GPIO
    // #define HAVE_DIP_SW
    #define DRIVER_L6470
    #define DRIVER_OSC INT_16MHZ
    #define DRIVER_RESET_PIN_OPERATION_STATE   HIGH
    // #define W5500_RESET_PIN_OPERATION_STATE    HIGH // ENABLE: Operation
    // pins
    #define ledPin 3u
#ifdef MZERO_WITH_ZERO_BOOTLOADER
    #define SD_CS_PIN   2u
#else
    #define SD_CS_PIN	4u
#endif
    // #define SD_DETECT_PIN   A4
    #define PIN_DRIVER_MISO 12u	// 
    #define PIN_DRIVER_MOSI 11u	// 
    #define PIN_DRIVER_SCK	13u	// 
    #define EPIO_DRIVER_MISO    PIO_SERCOM
    #define EPIO_DRIVER_MOSI    PIO_SERCOM
    #define EPIO_DRIVER_SCK     PIO_SERCOM
    #define PAD_DRIVER_SPI  SPI_PAD_0_SCK_1
    #define PAD_DRIVER_RX   SERCOM_RX_PAD_3
    #define PIN_DRIVER_CS   A2
    #define PIN_DRIVER_RESET    8u
    // #define W5500_RESET_PIN A3

    #define DRIVER_SERCOM sercom1

    #define ID  1
#endif

#endif