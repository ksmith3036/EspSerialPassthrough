/*
   EspSerialPassthrough v1.0
   ESP8266 / ESP-01 Serial Passthrough for flashing using Arduino SAMD MCU

   This Arduino sketch allows you to send AT commands, watch bootloader messages, or flash an ESP-01/ESP8266 through a SAMD21/SAMD51 or similar MCU. 
   The MCU should be operating at 3.3V, if not, voltage level shifters must be used on the connections to ESP.
   The program might also work on other ESPs, but have only been tested with ESP8266 in the form of a ESP-01 board.
   
   Developed and tested with a Arduino MKR board, SAMD21-based.
   The program might work on MCUs without native USB, but no guarantees! :-)

   Sending AT commands has been tested using the Arduino serial monitor, Visual Micro serial monitor and Putty in serial mode.
   The same goes for watching bootloader output.
   Firmware upgrade/flashing has been done with the official ESP8266 Flash Download Tool from https://www.espressif.com/en/support/download/other-tools, 
   with firmware downloaded from https://www.espressif.com/en/support/download/at

  Copyright 2021 Kåre Smith (Kaare Smith)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

// Arduino SAMD MCUs have native USB, but others might have native USB as well. If so: define HAVE_NATIVEUSB manually
#ifdef SERIAL_PORT_USBVIRTUAL
#define HAVE_NATIVEUSB
#define SERIAL_WRITE_BUFFER_SIZE 16384
#else
#define SERIAL_WRITE_BUFFER_SIZE 64
#endif

// Baud rates used for ESP8266 Serial port
#define BOOTLOADER_BAUD 74880 // Used when ESP8266 is in bootloader, for instance crashing after a failed flash of firmware, and in the first second or two after boot
#define AT_MODE_BAUD 115200 // Used when in AT command mode (I have read that some ESP8266 use 9600 baud in AT mode...)
#define FLASH_MODE_BAUD 115200 // Used when in firmware flashing mode 

// Set PASSTHROUGH_MODE to AutoMode, AtCommandsMode, BootloaderMode or FlashingMode
//#define PASSTHROUGH_MODE AtCommandsMode   
//#define PASSTHROUGH_MODE BootloaderMode
//#define PASSTHROUGH_MODE FlashingMode
// This controls baudrate and how the pins on the ESP8266 is set
#ifndef PASSTHROUGH_MODE
  #ifdef HAVE_NATIVEUSB
    #define PASSTHROUGH_MODE AutoMode  // Automatic switch between modes
  #else
  // If not native USB: default to AtCommandsMode
    #define PASSTHROUGH_MODE AtCommandsMode
  #endif
#endif

// AutoMode is only for devices with native USB, like Arduino Zero, Arduino MKR and compatibles (SAMD21/SAMD51 MCU), since DTR and RTS signals is vital.
// In AutoMode, baud rate switching is supported using the ESP flash tool, hence on a SAMD21, flashing can be done at speeds up to 1500000 (1.5Mbit/s).
// In FlashingMode, a fixed baudrate is used to the ESP, and the ESP flash download tool MUTS be set to the same baud rate.

// If serial port on MCU used to connect to PC (even if connecting through a USB serial adapter):
//   It is best that USB baud towards PC matches the baudrate of the ESP8266, but when using it to see bootloader messages or test AT commands this isn't important
//   since the serial buffers will not overflow. Having a fixed baudrate to the PC, makes it easier by not having to change baudrate in Putty (or other terminal emulator).
//   When flashing ESP8266 firmware, it could be more important that the baudrate from PC to MCU is equal to the baudrate used from the MCU to the ESP8266. 
// Native USB connection to PC: 
//   If using a Arduino board with native USB, like Arduino Zero or MKR series, the baud rate isn't important, since virtual serial transfers is at the native USB bus speed. 
unsigned long pcUsbBaud = 115200;  


enum PassthroughMode {
   AtCommandsMode // Locked to AT mode
  ,BootloaderMode // Locked to bootloader mode
  ,FlashingMode   // Locked to flashing mode
#ifdef HAVE_NATIVEUSB
  ,AutoMode       // Start in AT command mode, but enter flash mode, if esptool/espressif flash download tool connects.
#endif
};
// In Auto mode, USB serial DTR and RTS is controlling GPIO0 and CH_PD (EN) pins om ESP.

// Set Passthrough mode from defined mode
PassthroughMode passthroughMode = PASSTHROUGH_MODE;

// Sets ESP baudrate to AT_MODE_BAUD, BOOTLOADER_BAUD or FLASH_MODE_BAUD according to mode.
uint32_t espBaud = passthroughMode == AtCommandsMode ? AT_MODE_BAUD : passthroughMode == BootloaderMode ? BOOTLOADER_BAUD : FLASH_MODE_BAUD;

// Serial1 is the physical serial port on the Arduino MKR series boards (and compatible boards)
#define esp8266 Serial1

// The pin used on the Arduino board. Important: must be 3.3V GPIO pins, or use voltage level shifter
#define CH_PD_PIN 8  // Arduino pin connected to CH_PD (sometimes marked EN) pin on ESP8266
#define GPIO0_PIN 9  // Arduino pin connected to GPIO0 pin on ESP8266
#define RESET_PIN 10 // Arduino pin connected to RESET pin on ESP8266
// Arduino GND is connected to ESP GND
// Arduino VCC (3.3V) is connected to ESP8266 3V3
// Arduino TX is connected to ESP8266 RX
// Arduino RX is connected to ESP8266 TX
// ESP8266 GPIO2 is left unconnected

// Definitions from ESP flashing tool esptool.py
#define SLIP_END 0xC0
#define ESP_SYNC 0x08
#define ESP_CHANGE_BAUDRATE 0x0F

static bool gpio0Pin = true;
static bool gpio0PinBefore = true;
static bool chPdPin = true;
static bool chPdPinBefore = true;
static unsigned long switchBaudRateMillis = 0;

static byte fromEspBuffer[2048]; // Must be at least the size of serial buffers in Arduino, but preferably bigger, to pick up bootloader messages. Must of course be set according to available RAM. The SAMD21G has 32 KBytes

void setup() {
  Serial.begin(pcUsbBaud);
  esp8266_begin(espBaud);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CH_PD_PIN, OUTPUT);
  pinMode(GPIO0_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  delay(1000);

  if (passthroughMode == FlashingMode) {
    enterFlashMode();

    // Wait for serial to PC is connected
    while (!isSerialOpen());
  }
  else {
    // Wait for serial to PC is connected
    if (passthroughMode == BootloaderMode) {
      while (!isSerialOpen());
      Serial.println("Bootloader mode");
    }
    else if (passthroughMode == AtCommandsMode) {
      while (!isSerialOpen());
      Serial.println("AT mode");
    }

    // Bootloader mode and AT mode is similar

    // Start in bootloader baudrate, to pickup bootloader messages
    esp8266_begin(BOOTLOADER_BAUD);
    enterAtMode();
    // Switch from bootloader baudrate after two seconds (2000 millis)
    switchBaudRateMillis = millis() + 2000; 

    if (passthroughMode == FlashingMode) {
      // Wait for serial port to PC opens
      while (!isSerialOpen());
    }
    else {
      // Wait for serial connection, while buffering up bootloader messages
      size_t fromEspBufferIndex = 0;
      while (!isSerialOpen()) {
        switchBaudRateAfterTimeout(isSerialOpen());
        int availableBufferSpace = sizeof(fromEspBuffer) - fromEspBufferIndex;
        if (availableBufferSpace > 0) {
          int bytesAvailableFromEsp = esp8266.available();
          if (bytesAvailableFromEsp) {
            size_t bytesFromEsp = esp8266.readBytes(fromEspBuffer + fromEspBufferIndex, bytesAvailableFromEsp < availableBufferSpace ? bytesAvailableFromEsp : availableBufferSpace);
            fromEspBufferIndex += bytesFromEsp;
          }
        }
      }

      // Serial port is open, flush received buffer
      writeEspBufferToSerial(fromEspBuffer, fromEspBufferIndex, true);
    }
  }

  gpio0Pin = gpio0PinBefore = true;
  chPdPin = chPdPinBefore = true;
}

// Might have to break up writes into several writes, because of serial buffer size
inline void writeEspBufferToSerial(byte *fromEspBuffer, size_t bytesToWrite, bool delayed) {
  size_t writeIndex = 0;
  if (bytesToWrite > 0 && delayed) {
    delay(500);
  }
  while (bytesToWrite > 0) {
    // Have buffered some data, write to Serial
    size_t writtenBytes = Serial.write(fromEspBuffer + writeIndex, bytesToWrite > SERIAL_WRITE_BUFFER_SIZE ? SERIAL_WRITE_BUFFER_SIZE : bytesToWrite);
    bytesToWrite -= writtenBytes;
    writeIndex += writtenBytes;
  }
}

static bool enterFlashingMode = false;
static int enterFlashingModeState = 0;

static size_t bytesFromEsp;
static int bytesAvailableFromEsp;
static int bytesAvailableFromPc;

static uint32_t espBaudrateSet = 0;

void esp8266_begin(uint32_t baud) {
  if (baud != espBaudrateSet) {
    esp8266.begin(baud);
    espBaudrateSet = baud;
  }
}

// Two seconds after serial connect, switch to normal (115200) baud, if started in bootloader baud
inline void switchBaudRateAfterTimeout(bool writeSerial) {
  if (!enterFlashingMode && espBaudrateSet == BOOTLOADER_BAUD && espBaudrateSet != espBaud && millis() > switchBaudRateMillis) {
    if (writeSerial) {
      Serial.println();
      Serial.print("Switches to baud: ");
      Serial.println(espBaud);
    }
    esp8266_begin(espBaud);
  }
}

// Main loop
void loop() {
  bytesAvailableFromEsp = esp8266.available();
  if (bytesAvailableFromEsp) {
    bytesFromEsp = esp8266.readBytes(fromEspBuffer, bytesAvailableFromEsp < sizeof(fromEspBuffer) ? bytesAvailableFromEsp : sizeof(fromEspBuffer));
    // Might have to break up writes into several writes, because of serial buffer size
    writeEspBufferToSerial(fromEspBuffer, bytesFromEsp, false);
  }

  // Two seconds after serial connect, switch to normal (115200) baud, if started in bootloader baud
  switchBaudRateAfterTimeout(true);

#ifdef HAVE_NATIVEUSB
  lookForEspFlashReset();
#endif

  bytesAvailableFromPc = Serial.available();
  if (bytesAvailableFromPc) {
#ifdef HAVE_NATIVEUSB
    lookForEspFlashCommandsWhilePassingTraffic(bytesAvailableFromPc);
#else
    esp8266.write(Serial.read());
#endif
  }
}

static bool _serialPortOpen = false;
// SAMD SerialUSB operator bool has a builtin 10 millisecond delay, so cache the result of it
inline bool isSerialOpen() {
  if (_serialPortOpen) {
    return _serialPortOpen;
  }
  _serialPortOpen = Serial;
  return _serialPortOpen;
}

#ifdef HAVE_NATIVEUSB
void lookForEspFlashReset() {
  if (passthroughMode == AutoMode) {
    bool dtr = SerialUSB.dtr();
    bool rts = SerialUSB.rts();
    // From esptool.py:
    //    self._setDTR(False)  # IO0 = HIGH
    //    self._setRTS(True)   # EN = LOW, chip in reset
    if (!dtr && rts && enterFlashingModeState == 0) {
      gpio0Pin = true;
      chPdPin = false;
      enterFlashingModeState = 1;
    }
    //    self._setDTR(True)   # IO0 = LOW
    //    self._setRTS(False)  # EN = HIGH, chip out of reset
    if (dtr && !rts && enterFlashingModeState == 1) {
      gpio0Pin = false;
      chPdPin = true;
      enterFlashingModeState = 2;
    }
    //    self._setDTR(False)  # IO0=HIGH, done
    //    and RTS still False
    if (!rts && !dtr && enterFlashingModeState != 0) {
      gpio0Pin = true;
      chPdPin = true;
      enterFlashingMode = true;
      enterFlashingModeState = 0;
    }

    // Should GPIO0 pin be changed?
    if (gpio0Pin != gpio0PinBefore) {
      digitalWrite(GPIO0_PIN, gpio0Pin ? HIGH : LOW);
      gpio0PinBefore = gpio0Pin;
    }

    // Should CH_PD(EN) pin be changed?
    if (chPdPin != chPdPinBefore) {
      digitalWrite(CH_PD_PIN, chPdPin ? HIGH : LOW);
      //digitalWrite(RESET_PIN, chPdPin ? HIGH : LOW);
      chPdPinBefore = chPdPin;
      Serial.clear();
      if (espBaudrateSet != FLASH_MODE_BAUD && !chPdPin) {
        esp8266_begin(FLASH_MODE_BAUD);
        Serial.clear();
      }
    }

  }
}

static byte flashCommandInspectBuffer[32];
static byte baudBuffer[4];

void lookForEspFlashCommandsWhilePassingTraffic(int bytesAvailableFromPc) {
  if (enterFlashingMode && passthroughMode != FlashingMode) {
    // Serial over USB, so we get packets of data, not single bytes.
    // In any case: Just read a part, which is large enough to inspect for command, but small enough to fit in Serial send buffer (and receive buffer on ESP)
    int bytesRead = Serial.readBytes((char*)flashCommandInspectBuffer, bytesAvailableFromPc < sizeof(flashCommandInspectBuffer) ? bytesAvailableFromPc : sizeof(flashCommandInspectBuffer));
    if (bytesRead > 0) {
      // Commands start and ends with 0xc0 - it is SLIP encoded (Serial Line IP)
      // Command: 0xc0 0x00 opcode(1 byte) length_of_data(2 bytes) checksum(4 bytes) + data(x bytes) + 0xc0
      // SLIP frames encodes 0xc0 in content as 0xdb + 0xdc 
      //         and encodes 0xdb in content as 0xdb + 0xdd
      // So 0xc0 is never found in the data
      if (flashCommandInspectBuffer[0] == SLIP_END && flashCommandInspectBuffer[1] == 0x00) {
        if (flashCommandInspectBuffer[2] == ESP_SYNC && bytesRead > 8) {
          // self.command(self.ESP_SYNC, b'\x07\x07\x12\x20' + 32 * b'\x55',

          setLed(true);
        }
        else if (flashCommandInspectBuffer[2] == ESP_CHANGE_BAUDRATE && bytesRead > 12) {
          // self.command(self.ESP_CHANGE_BAUDRATE, struct.pack('<II', baud, second_arg))
          // If baudrate contains SLIP escaping, it must be unescaped first: 0xdb,0xdc=>0xc0 0xdb,0xdd=>0xdb
          for (int bbi = 0, fcib = 9; bbi < 4; bbi++, fcib++) {
            byte b = flashCommandInspectBuffer[fcib];
            baudBuffer[bbi] = (b == 0xdb ? (flashCommandInspectBuffer[++fcib] == 0xdc ? 0xc0 : 0xdb) : b);
          }
          uint32_t baudrate = baudBuffer[0] | (baudBuffer[1] << 8) | (baudBuffer[2] << 16) | (baudBuffer[3] << 24);
          esp8266.write(flashCommandInspectBuffer, bytesRead);
          int idx = 0;
          int slipEndCount = 0;
          byte b = 0;
          unsigned long millisWait = millis() + 300;
          // Buffer response from ESP
          while (slipEndCount < 2 && millis() < millisWait) {
            while (esp8266.available() && idx < sizeof(fromEspBuffer)) {
              b = esp8266.read();
              fromEspBuffer[idx++] = b;
              if (b == SLIP_END) slipEndCount++;
            }
          }
          // Switch baud rate
          esp8266_begin(baudrate);
          // Give the ESP serial port time to settle
          delay(500);
          // Return response to PC, so the flash downloader continues
          Serial.write(fromEspBuffer, idx);
          return;
        }
      }

      esp8266.write(flashCommandInspectBuffer, bytesRead);
    }

  }
  else {
    esp8266.write(Serial.read());
  }
}
#endif

static bool ledOn = false;
void setLed(bool state) {
  ledOn = state;
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
}

void flashLed(int flashes) {
  for (int i = 1; i <= flashes; i++) {
    digitalWrite(LED_BUILTIN, !ledOn ? HIGH : LOW);
    delay(250);
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    if (i < flashes) {
      delay(250);
    }
  }
}

/*
esptool.py _connect_attempt:
        # issue reset-to-bootloader:
        # RTS = either CH_PD/EN or nRESET (both active low = chip in reset
        # DTR = GPIO0 (active low = boot to flasher)
        #
        # DTR & RTS are active low signals,
        # ie True = pin @ 0V, False = pin @ VCC.
        if mode != 'no_reset':
            self._setDTR(False)  # IO0=HIGH
            self._setRTS(True)   # EN=LOW, chip in reset
            time.sleep(0.1)
            if esp32r0_delay:
                # Some chips are more likely to trigger the esp32r0
                # watchdog reset silicon bug if they're held with EN=LOW
                # for a longer period
                time.sleep(1.2)
            self._setDTR(True)   # IO0=LOW
            self._setRTS(False)  # EN=HIGH, chip out of reset
            if esp32r0_delay:
                # Sleep longer after reset.
                # This workaround only works on revision 0 ESP32 chips,
                # it exploits a silicon bug spurious watchdog reset.
                time.sleep(0.4)  # allow watchdog reset to occur
            time.sleep(0.05)
            self._setDTR(False)  # IO0=HIGH, done

    ESP_SYNC: 
  //  c0 00 08 24 00    00 00 00 00
  //  07 07 12 20 55 55 55 55 55 55 55 55
  //  55 55 55 55 55 55 55 55 55 55 55 55 55 55 55 55
  //  55 55 55 55 55 55 55 55 c0

*/


// Enter flashing mode
void enterFlashMode() {
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(CH_PD_PIN, HIGH);
  digitalWrite(GPIO0_PIN, HIGH);
  delay(300);
  
  // Reset ESP8266, to restart the ESP8266 on restart of the Arduino board
  digitalWrite(RESET_PIN, LOW);
  delay(1000);
  digitalWrite(RESET_PIN, HIGH);

  // Set GPIO0 low, to signal that on next boot, the ESP8266 should enter flashing mode
  digitalWrite(GPIO0_PIN, LOW);
  delay(3000);

  // Reset ESP8266 again
  digitalWrite(RESET_PIN, LOW);
  delay(1000);
  digitalWrite(RESET_PIN, HIGH);
  delay(200);

  // Now the ESP8266 should be in flashing mode
}

// AT mode
void enterAtMode() {
  digitalWrite(GPIO0_PIN, HIGH);
  digitalWrite(CH_PD_PIN, HIGH);
  digitalWrite(RESET_PIN, HIGH);
  delay(300);

  // Reset ESP8266, to restart the ESP8266 on restart of the Arduino board
  digitalWrite(RESET_PIN, LOW);
  delay(500);
  digitalWrite(RESET_PIN, HIGH);
  delay(500);

  // On some ESP8266 the CH_PD/EN-pin seems to need a pulse before it will start communication
  digitalWrite(CH_PD_PIN, LOW);
  delay(300);
  digitalWrite(CH_PD_PIN, HIGH);
}

/*
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

*/