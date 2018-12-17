
# ESPBatterySense
This project was inspired by my desire to track the humidity, and to a much lesser degree, the temperature inside the storage solution for my 3D printing filament. To goal was to have a device that is battery operated, lasts several months (if not longer) before needing to recharge or replace batteries, and capable of transmitting to my MQTT broker for display and graphing in HASS.IO. I've been working a lot with ESP-based projects and love the the simplicity of the ESP. In this project, which is a varition of the the original that made use of a BME280 sensor, I've moved to a SI7021-A20 IC that is not a seprate daughter-board which I like better. The SI7021-A20 is a great solution for me as it's easily and affordably sourced, well documented, easy to reflow solder in my homemade solution, and has great accuracy.

The firmware is designed to put the ESP in deep sleep for 1 hour and upon reset, take a sample of humidity, temperature, voltage of battery, and then transmit that to my MQTT broker (which happens to be part of my HASS.IO setup). I originally had been using a LiPo battery from Adafruit, and while it worked, it was providing > 3.6V to the board directly. I never saw any negative issues with such a high voltage from the BME280 sensor or the ESP, but it's not a good design practice and it did cause the ESP to consume far more power in deep sleep than voltages closer to 3.3V. For these reasons, I've switched to (2) AA alkaline batteries and have had great longevity thus far with them. Running the prototype design for well over 3 weeks with a tenth of a volt drop on the batteries.

Unfortunately, this design requires a reflow solution as the SI7021 is not something I would classify as easily hand soldered, it's also the reason I'm keeping the other design files available which use components which can all easily be hand soldered. 

**BOM**
----------
1. [ESP-12S](https://www.aliexpress.com/item/ESP8266-ESP-12E-serial-WIFI-wireless-module-wireless-transceiver-Complete-circuit-impedance-matching-better-signal/32324777806.html)
2. [SI7021-A20](https://www.aliexpress.com/item/100-brand-new-original-Si7021-A20-GM1R-Si7021-A20/32834030103.html) (with protective cover and A20 version not A10)
3. [10K OHM 0805 Resistor](https://www.digikey.com/product-detail/en/yageo/RC0805FR-0710KL/311-10.0KCRCT-ND/730482) (QTY: 5)
4. [0.1UF Ceramic Capacitor X7R 16V 0603](https://www.digikey.com/product-detail/en/samsung-electro-mechanics/CL10B104KO8NNNC/1276-1005-1-ND/3889091) (QTY: 1)
6. [Custom PCB](https://oshpark.com/shared_projects/NEJbJ0QY) from OSHPark.com
7. [JST connector](https://www.digikey.com/product-detail/en/jst-sales-america-inc/S2B-PH-K-S%28LF%29%28SN%29/455-1719-ND/926626) for battery
7. Battery (any ~3.3V battery source)
8. Optional header for programming and debug pins
9. Enclosure of some sort (once I finalize my battery size, I'll upload a 3D printable enclosure)
#


[![Travis](https://img.shields.io/travis/jaycollett/ESPBatterySense.svg?style=for-the-badge)](https://travis-ci.org/jaycollett/ESPBatterySense) [![license](https://img.shields.io/github/license/jaycollett/ESPBatterySense.svg?style=for-the-badge)](https://github.com/jaycollett/ESPBatterySense/blob/master/LICENSE)
