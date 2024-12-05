# OTA sender test

Sender must be a ESP32 board with a SD card slot. Currently using this one from 
Freenove: [Freenove_ESP32_WROVER_Board](https://github.com/Freenove/Freenove_ESP32_WROVER_Board/).
You can also use a SD card module as suggested by the painlessMesh [exmaple](https://gitlab.com/painlessMesh/painlessMesh/-/tree/develop/examples/otaSender).

Instead of using the SD.h library in the painlessmesh example, you will need to
use SD_MMC.h library for this board. Follow the [SDMMC](https://github.com/Freenove/Freenove_ESP32_WROVER_Board/tree/main/C/Sketches/Sketch_03.1_SDMMC_Test) example
for accessing the firmware on the SD card.


Firmware Naming Rule: "firmware_ESP32_**role**.bin"


Specify **role** when uploading the firmware for the first time.