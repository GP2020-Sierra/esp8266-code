# ESP8266-data

This code allows the ESP8266 to gather wifi packets and to get data from the 

For information about our project, see our project's [README](https://gp2020-sierra.github.io/README/)

[License](/LICENSE)

## Introduction

This code has a wifi handling loop that gathers packets. It ignores all control frames (Ack, etc.) as they only have 1 address, so are typically useless for device recognition. It then looks at address 2 in the packet; which is the packet's source device, and stores it in a linked list.

Due to the "parallelism" involved in this packet handling (I.e. interupts occuring mid packet), typically devices will be detected twice. To avoid this we half the result after the fact.

The DHT11 code suspends wifi packet handling to ensure timing constraints are met. To communicate, we use the Adafruit DHT11 library

## Code  

The code is a single file in src.

You could probably build it manually, but I used PlatformIO. The provided PlatformIO also supplies the library requirements.

ArduinoIDE can be used, but is harder to use.

## Programming

To program the ESP8266 you need an USB to UART converter. You plug your ESP8266 into the USB to UART converter, and then ensure you have pulled GPIO0 to ground (we used a jumper wire to do this).

## Atttribution

The majority of the wifi code is derived from [kalanda/esp8266-sniffer](https://github.com/kalanda/esp8266-sniffer).
The DHT11 capture is powered by the Adafruit sensor libraries
