#ifdef SERIAL_PORT_USBVIRTUAL
#define HAVE_NATIVEUSB
#define SERIAL_WRITE_BUFFER_SIZE 16384
#else
#define SERIAL_WRITE_BUFFER_SIZE 64
#endif

/***************** Set ESP Baud Rate ********************/

// Baud rates used for ESP8266 Serial port
#define BOOTLOADER_BAUD 74880 // Used when ESP8266 is in bootloader, for instance crashing after a failed flash of firmware, and in the first second or two after boot
#define AT_MODE_BAUD 115200 // Used when in AT command mode (I have read that some ESP8266 use 9600 baud in AT mode...)
#define FLASH_MODE_BAUD 115200 // Used when in firmware flashing mode

/********************************************************/ 


/***************** Set PASSTHROUGH MODE *****************/

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

/*****************************************************/ 


// Definitions from ESP flashing tool esptool.py
#define SLIP_END 0xC0
#define ESP_SYNC 0x08
#define ESP_CHANGE_BAUDRATE 0x0F


/***************** Set Arduino Ports *****************/

// Serial1 is the physical serial port on the Arduino MKR series boards (and compatible boards)
#define esp8266 Serial1

// The pin used on the Arduino board. Important: must be 3.3V GPIO pins, or use voltage level shifter
#define CH_PD_PIN 27  // Arduino pin connected to CH_PD (sometimes marked EN) pin on ESP8266
#define GPIO0_PIN 4  // Arduino pin connected to GPIO0 pin on ESP8266
#define RESET_PIN 7 // Arduino pin connected to RESET pin on ESP8266

/****************************************************/ 

enum PassthroughMode {
   AtCommandsMode // Locked to AT mode
  ,BootloaderMode // Locked to bootloader mode
  ,FlashingMode   // Locked to flashing mode
#ifdef HAVE_NATIVEUSB
  ,AutoMode       // Start in AT command mode, but enter flash mode, if esptool/espressif flash download tool connects.
#endif
};
// In Auto mode, USB serial DTR and RTS is controlling GPIO0 and CH_PD (EN) pins om ESP.
  

// Native USB connection to PC: 
// If using a Arduino board with native USB, like Arduino Zero or MKR series, the baud rate isn't important, since virtual serial transfers is at the native USB bus speed. 
#ifdef DEFINE_PCUSBBAUD
unsigned long pcUsbBaud = 115200;
#else
extern unsigned long pcUsbBaud;
#endif