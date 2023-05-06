// ============================================================================
// ===
// === Name:      Exploration Computer
// ===
// === Abstract:  Logs position + altitude data using GPS, IMU, and pressure
// ===            sensors and displays data on a 240x320 pixel display
// ===
// === Author:    Raymond Yang
// ===
// === Date:      05/04/2023
// ===
// ============================================================================

// ----------------------------------------------------------------------------
// Compile Settings
// ----------------------------------------------------------------------------
#define EN_GPS_MOD    1   // Ultimate GPS v3
#define EN_IMU_MOD    1   // BNO055 9-dof imu
#define EN_TPS_MOD    1   // BMP388 temp pressure sensor
#define EN_SD_MOD     0   // External SD card reader
#define EN_DISP_MOD   1   // Embedded display module


// ----------------------------------------------------------------------------
// Libraries
// ----------------------------------------------------------------------------
#include <Arduino.h>

#if EN_DISP_MOD
  #include <ILI9341_T4.h>
#endif


// ----------------------------------------------------------------------------
// Macro
// ----------------------------------------------------------------------------

// *** general macros ***
#define DEBUG_verbose   1   // print info for debugging


// *** display settings ***
#if EN_DISP_MOD
  #define PIN_DISP_SCK    13
  #define PIN_DISP_MISO   12
  #define PIN_DISP_MOSI   11
  #define PIN_DISP_DC     10    // SPI data/command selector pin
  #define PIN_DISP_CS     9
  #define PIN_DISP_RESET  6
  #define PIN_BACKLIGHT 255

  #define DISP_SPI_SPEED  30000000  // 30MHz SPI speed
  #define LX  240   // screen size x
  #define LY  320   // screen size y
#endif


// ----------------------------------------------------------------------------
// Global Variables/Instances
// ----------------------------------------------------------------------------

// *** display ***
#if EN_DISP_MOD
  // display driver
  ILI9341_T4::ILI9341Driver disp(PIN_DISP_CS, PIN_DISP_DC, PIN_DISP_SCK, PIN_DISP_MOSI, PIN_DISP_MISO, PIN_DISP_RESET);

  // diff buffers in bytes (8 bits)
  ILI9341_T4::DiffBuffStatic<8192> diff1;
  ILI9341_T4::DiffBuffStatic<8192> diff2;

  // framebuffers
  uint16_t internal_fb[LX * LY];     // used by the library for buffering
  uint16_t fb[LX * LY];              // the main framebuffer we draw onto.
#endif

// ----------------------------------------------------------------------------
// Function Definitions
// ----------------------------------------------------------------------------

//
// display related ************************************************************
//

#if EN_DISP_MOD
  /*
    set the background color
  */
  void draw_background(uint16_t* fb, uint16_t color = 255) {
    for(int i = 0; i < LX*LY; i++) {
      fb[i] = color;
    }
  }
#endif

// ----------------------------------------------------------------------------
// MCU Entry Point
// ----------------------------------------------------------------------------=
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("serial ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  exit(0);
}
