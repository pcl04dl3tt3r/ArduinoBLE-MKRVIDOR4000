# ArduinoBLE - with MKR Vidor 4000 support and the ability to initiate pairing and bonding with a peripheral such as the Google daydream controller.

[![Compile Examples Status](https://github.com/arduino-libraries/ArduinoBLE/workflows/Compile%20Examples/badge.svg)](https://github.com/arduino-libraries/ArduinoBLE/actions?workflow=Compile+Examples) [![Spell Check Status](https://github.com/arduino-libraries/ArduinoBLE/workflows/Spell%20Check/badge.svg)](https://github.com/arduino-libraries/ArduinoBLE/actions?workflow=Spell+Check)

Enables Bluetooth® Low Energy connectivity on the Arduino MKR Vidor 4000.

This library supports initiating and bonding a Bluetooth® Low Energy central mode connection to a peripheral that supports JustWorks pairing. In particular this was created to add Google daydream controller support to the Arduino SAMD boards. This contoller provides a 9 axis orientation/gryro/accelerometer, plus 2 axis touch pad, and 5 buttons.

For the Arduino MKR WiFi 1010, Arduino UNO WiFi Rev.2, and Arduino Nano 33 IoT boards, it requires the NINA module to be running [Arduino NINA-W102 firmware](https://github.com/arduino/nina-fw) v1.2.0 or later.

For the MKR Vidor 4000 it requires pcl04dl3tt3r/nina-fw-MKRVIDOR4000 (https://github.com/pcl04dl3tt3r/nina-fw-MKRVIDOR4000) v1.5.0 or later.

For more information about this library please visit us at:
https://www.arduino.cc/en/Reference/ArduinoBLE

## License

```
Copyright (c) 2019 Arduino SA. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
```
