Ponoor L6470 Library
==========
Arduino library support for STMicroelectronics [L6470](https://www.st.com/en/motor-drivers/l6470.html) chip.

This library is a modification of the L6470-based SparkFun [AutoDriver library](https://github.com/sparkfun/SparkFun_AutoDriver_Arduino_Library).

Repository Contents
-------------------
* **src** - Contains the source for the Arduino library.
* **Examples** - Example sketches demonstrating the use of the library
* **keywords.txt** - List of words to be highlighted by the Arduino IDE
* **library.properties** - Used by the Arduino package manager

Differences from the original library
-------------------
- Added `getSpeed()` function
- Disable interrupts during `getStatus` and `xferParam` to avoid return value collapse for ATSAMD
- Changed some constants name(`CMD_GET_STATUS`, `REG_STATUS`) to avoid conflicts with other libraries.
- Added functions related with `EL_POS`

License Information
-------------------
This product is open source! 
The code is beerware; if you see any SparkFun employee at the local, and you've found their code helpful, please buy them a round! 
Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release anything derivative under the same license.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

