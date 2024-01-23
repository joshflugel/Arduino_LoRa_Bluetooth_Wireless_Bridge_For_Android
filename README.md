# Arduino LoRa-Bluetooth Wireless Bridge for Android
Arduino Bluetooth-LoRa Wireless bridge for Android

Arduino Wireless Bluetooth-LoRa bridge by Josué Galindo

                             |      LoRa| <- Low Power Area Network -> |LoRa     |
                             | Arduino 1|                              |Arduino 2|
    |Android Device "A"| <-> | Bluetooth|                              |Bluetooth| <-> |Android Device "B"| 

In the schematic above, this code runs in Arduino Microcontrollers 1 and 2, 
which are connected to Android devices A and B respectively via Low Eenrgy Bluettooth (BLE).
The Arduinos acts as a bridge that allows both Android devices to communicate with each other
via LoRa WAN (Low Power Wide Area Network), in use cases where cellular network 
or WiFi connectivity is not available.

This Arduino Bluetooth-LoRa bridge, is the wireless communication hardware counterpart to the
main Android app presented by Josué Galindo as the final project for the Architect Coders course 
issued by DevExpert.io (2024).

References:
https://architectcoders.com/
https://lora-alliance.org/about-lorawan/
