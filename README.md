# step-series-universal-firmware
The two hardware from Ponoor Experiments, STEP400 and STEP800, most of their source codes are common or similar. This project aims to compile the each firmware from the same source codes. 

Also trying to cover some variants and X-Nucleos.

**Still work in progress**

## Target
- [STEP400](https://github.com/ponoor/STEP400/)
- [STEP800](https://github.com/ponoor/STEP800/)
- Arduino Zero + [X-NUCLEO-IHM02A1](https://www.st.com/en/ecosystems/x-nucleo-ihm02a1.html) + Ethernet shield
- Arduino Zero + [X-NUCLEO-IHM03A1](https://www.st.com/en/ecosystems/x-nucleo-ihm03a1.html) + Ethernet shield

## Compile
This project is based on PlatformIO with Arduino platform. Uncomment one which you want to compile at the top of `boardsDef.h`.

```
// Products
// #define STEP400_R1
// #define STEP800_R1

// Prototypes
// #define STEP400_PROTO_R4
// #define STEP800_PROTO_R3
// #define STEP800_PROTO_R1

// X-Nucleos
// #define STEP100_R1 // PowerSTEP01 x1
// #define STEP200_R1 // L6470 x2 
```

