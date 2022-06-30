/***********************************************************************
  
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
  
 This code is licensed under the Apache License, Version 2.0 (the "License");

***********************************************************************/

#include "config.h"

// Native USB connection to PC: 
//   If using a Arduino board with native USB, like Arduino Zero or MKR series, the baud rate isn't important, since virtual serial transfers is at the native USB bus speed. 
unsigned long pcUsbBaud = 115200;  


// Set Passthrough mode from defined mode
PassthroughMode passthroughMode = PASSTHROUGH_MODE;

// Sets ESP baudrate to AT_MODE_BAUD, BOOTLOADER_BAUD or FLASH_MODE_BAUD according to mode.
uint32_t espBaud = passthroughMode == AtCommandsMode ? AT_MODE_BAUD : passthroughMode == BootloaderMode ? BOOTLOADER_BAUD : FLASH_MODE_BAUD;

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