# Flash ESP8266 using Arduino with native USB
## EspSerialPassthrough v1.0

ESP8266 / ESP-01 Serial Passthrough for serial console or flashing, using Arduino with SAMD MCU.

This Arduino sketch allows you to send AT commands, watch bootloader messages or flash an ESP-01/ESP8266 through a SAMD21/SAMD51 or similar MCU. 
The MCU used should be operating at 3.3V, if not, voltage level shifters must be used on the connections to ESP.
The MCU should preferably also have native USB.
The program might also work on other ESPs, but have only been tested with ESP8266 in the form of a ESP-01 board.

Developed and tested with a Arduino MKR board, SAMD21-based, with native USB connection to PC.
The program might work on MCUs without native USB, but flashing must then be done at a fixed baudrate of 115200. 

Sending AT commands has been tested using the Arduino serial monitor, Visual Micro serial monitor and PuTTY in serial mode.
The same goes for watching bootloader output.

Firmware upgrade/flashing has been done with the official ESP8266 Flash Download Tool from https://www.espressif.com/en/support/download/other-tools, 
with firmware downloaded from https://www.espressif.com/en/support/download/at.

See guide in [Wiki](https://github.com/ksmith3036/EspSerialPassthrough/wiki/)

&nbsp;

### Wiring

- Arduino GND is connected to ESP GND
- Arduino VCC (3.3V) is connected to ESP8266 3V3
- Arduino TX is connected to ESP8266 RX
- Arduino RX is connected to ESP8266 TX
- ESP8266 GPIO2 is left unconnected

Select the EN, RESET and BOOT pins using the config header file:

- CH_PD_PIN connected to ESP EN
- GPIO0_PIN connected to ESP BOOT
- RESET_PIN connected to ESP RST

### AT test

```
// List version information
AT+GMR
AT version:1.1.0.0(May 11 2016 18:09:56)
SDK version:1.5.4(baaeaebb)
compile time:May 20 2016 15:08:19
OK

// After upgrade:
AT+GMR
AT version:1.7.4.0(May 11 2020 19:13:04)
SDK version:3.0.4(9532ceb)
compile time:May 27 2020 10:12:17
Bin version(Wroom 02):1.7.4
OK


// List mode: 1 = Station mode, 2 = Access point mode, 3 = Both station and access point
AT+CWMODE?
+CWMODE:2

OK

// Set combined accesspoint and station (client) mode
AT+CWMODE=3

OK
// List networks
AT+CWLAP

AT+CWLAP
+CWLAP:(3,"Get-2G-A4948D",-87,"98:1e:19:a4:94:92",1,-26,0)
+CWLAP:(4,"Lars_Anette",-84,"70:b1:4e:7d:26:d9",1,-46,0)
+CWLAP:(3,"DIRECT-53-HP M281 LaserJet",-42,"2e:6f:c9:46:3d:53",6,-32,0)
+CWLAP:(3,"HP-Print-7C-Officejet Pro 6830",-85,"64:51:06:ba:ad:7c",6,-21,0)
+CWLAP:(3,"SMITH",-36,"58:cb:52:b9:7f:bc",6,0,0)
+CWLAP:(3,"SMITHX",-35,"5a:cb:52:b9:7f:bc",6,0,0)
+CWLAP:(3,"Ryums vei",-86,"f0:72:ea:4f:40:42",6,0,0)
+CWLAP:(3,"skynet",-80,"88:41:fc:ab:88:d7",6,-17,0)
+CWLAP:(3,"MaxVirus 2",-83,"98:1e:19:a3:9c:58",11,-17,0)
+CWLAP:(3,"NTGR_VMB_3127449426",-88,"a4:11:62:5a:cb:d2",11,-12,0)
+CWLAP:(3,"SMITH",-63,"58:cb:52:b9:77:66",11,-4,0)
+CWLAP:(3,"SMITHX",-63,"5a:cb:52:b9:77:64",11,-4,0)
+CWLAP:(3,"skynet",-85,"88:41:fc:ab:88:d3",11,-29,0)


//Bootloader messages (will vary with firmware version and ESP-variant):
 ets Jan  8 2013,rst cause:4, boot mode:(3,7)

wdt reset
load 0x40100000, len 27728, room 16
tail 0
chksum 0x2a
load 0x3ffe8000, len 2124, room 8
tail 4
chksum 0x07
load 0x3ffe8850, len 9276, room 4
tail 8
chksum 0xba
csum 0xba
�rlL�
ready

```

### License

Copyright 2021 Kåre Smith (Kaare Smith)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

&nbsp;&nbsp;&nbsp;http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
