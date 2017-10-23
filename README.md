# Firmware

[![Build Status](https://travis-ci.org/tribesat/firmware.svg?branch=master)](https://travis-ci.org/tribesat/firmware)

Onboard software for TribeSat

## Developing

Running `make develop` symlinks this repository to your Arduino libraries directory. Make sure to use the absolute path in the command below.

```sh
make develop ARDUINO_LIB_DIR=/Users/kelvin/Arduino/libraries
```

Now you can include the various components of the firmware in your Arduino code:

```C++
#include "Comms.h"
#include "Packet.h"
...
```

To set up hardware for developing and debugging, you can use 2 Arduinos; one acting as our payload and the other as the modem. One Uno pins 10 and 11 are cross-connected (10 to 11 for RX1/TX2, 11 to 10 for TX1/RX2) for serial communication. You can monitor the boards using the Arduino IDE serial monitor.

## Tools

The `/tools` directory contains an Arduino sketch `ModemEmulator` that makes turns an Arduino into an emulator of the onboard modem. It reads packets on serial and ACKs/NAKs them as appropriate.

