# ESP_HX8357K
Optimised HX8357 Library for the ESP32

Based heavily on the work of Paul Stoffenregen: https://github.com/PaulStoffregen/ILI9341_t3 and Bodmer: https://github.com/Bodmer/TFT_HX8357

This library uses the DMA capabilities of the ESP32 to deliver a faster rendering speed, however writing individual pixels is still slow. (fills are relatively fast).

Hope this helps somebody. Enjoy!
