# Firmware

[![Build Status](https://travis-ci.org/tribesat/firmware.svg?branch=master)](https://travis-ci.org/tribesat/firmware)

Onboard software for TribeSat

## Comm setup

The communications sketches in the `src/TribeSatSerialPayload` and `src/TribeSatSerialModem` are an initial version of how to do communciations.

To develop this further, get two Arduino Uno's where pins 10 and 11 are cross-connected (10 to 11 for RX1/TX2, 11 to 10 for TX1/RX2). We use SoftwareSerial to connect using screen /dev/ttyACM0 57600 on the host computer end. Any screen input on the Payload side is turned into TribeSat ICD packets (1 char per packet) and transmitted to the Modem side where the entire packet is printed. The Modem side sends an ACK or NAK depending on whether the packet preamble is indeed HEX 50 50 50.
