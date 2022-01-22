# step-series-universal-firmware
The two hardware from Ponoor Experiments, STEP400 and STEP800, most of their source codes are common or similar. This project aims to compile the each firmware from the same source codes. 

Also trying to cover some variants and X-Nucleos.

**Still work in progress**

## Target
- [STEP400](https://github.com/ponoor/STEP400/)
- [STEP800](https://github.com/ponoor/STEP800/)
- Arduino Zero + [X-NUCLEO-IHM02A1](https://www.st.com/en/ecosystems/x-nucleo-ihm02a1.html) + Ethernet shield
- Arduino Zero + [X-NUCLEO-IHM03A1](https://www.st.com/en/ecosystems/x-nucleo-ihm03a1.html) + Ethernet shield

<img src="https://ponoor.com/cms/wp-content/uploads/2021/03/IMG_4219-1600x1067.jpg" width="50%" /><img src="https://ponoor.com/cms/wp-content/uploads/2020/10/step800_r1_1-1600x1067.jpg" width="50%" />

<img src="https://www.st.com/bin/ecommerce/api/image.PF262214.en.feature-description-include-personalized-no-cpn-large.jpg" width="50%" /><img src="https://www.st.com/bin/ecommerce/api/image.PF261319.en.feature-description-include-personalized-no-cpn-large.jpg" width="50%" />

## Compile
This project is based on PlatformIO with Arduino framework. There are a board definitions list in `boardsDef.h` and uncomment one of them.

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

