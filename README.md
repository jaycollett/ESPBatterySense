
# ESPBatterySense
This project was inspired by my desire to track the humidity, and to a much lesser degree, the temperature inside the storage solution for my 3D printing filament. To goal was to have a device that is battery operated, lasts several months before needing recharging, and capable of transmitting to my MQTT broker for display and graphing in HASS.IO. I've been working a lot with ESP-based projects that used the BME 280 and thought it might work well in a battery operated solution (granted, it's not totally ideal components).

The firmware is designed to put the ESP in deep sleep for 1 hour and upon reset, take a sample of humidity, temperature, voltage of battery, and then transmit that to my MQTT broker (which happens to be part of my HASS.IO setup). Given a 2700mAH lipo battery, I'm thinking I'll get several months of use before needing to recharge the battery based on my rough math. Ultimately transmitting less frequently may be required to increase time btween recharges but even transmitting once every 4 hours would be more than enough for the intended goal.

The design if quite easy to hand solder, which is an added bonus in my book.

**BOM**
----------
1. [ESP-12F](https://www.aliexpress.com/item/ESP-12S-ESP-12F-upgrade-ESP8266-remote-serial-Port-WIFI-wireless-module-2016-New-version/32715475579.html)
2. [BME 280 Module](https://www.aliexpress.com/item/GY-BME280-3-3-precision-altimeter-atmospheric-pressure-BME280-sensor-module/32688607454.html) (tried and failed multiple times to oven reflow the IC alone)
3. [10K OHM 0805 Resistor](https://www.digikey.com/product-detail/en/yageo/RC0805FR-0710KL/311-10.0KCRCT-ND/730482) (QTY: 3)
4. [Custom PCB](https://oshpark.com/shared_projects/Ion0FFkC) from OSHPark.com
5. [JST connector](https://www.digikey.com/product-detail/en/jst-sales-america-inc/S2B-PH-K-S%28LF%29%28SN%29/455-1719-ND/926626) for battery
6. Battery (any ~3.3V battery source with a JST connector, [I'm using this battery](https://www.adafruit.com/product/328))
7. Optional header for programming and debug pins
8. Enclosure of some sort (once I finalize my battery size, I'll upload a 3D printable enclosure)
#


[![Travis](https://img.shields.io/travis/jaycollett/ESPBatterySense.svg?style=for-the-badge)](https://travis-ci.org/jaycollett/ESPBatterySense) [![license](https://img.shields.io/github/license/jaycollett/ESPBatterySense.svg?style=for-the-badge)](https://github.com/jaycollett/ESPBatterySense/blob/master/LICENSE)
