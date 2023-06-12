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

#define EN_DISP_TEST  1   // Enable display tests

// basic checks to add collaterals
#if (EN_DISP_TEST && !EN_DISP_MOD)
  #undef EN_DISP_MOD
  #define EN_DISP_MOD 1
#endif

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
  #define PIN_BACKLIGHT   255     // control backlight brightness?

  #define DISP_SPI_SPEED  30000000  // 30MHz SPI speed
  #define DISP_LX  240   // screen size x
  #define DISP_LY  320   // screen size y
  #define DISP_PIX 76800 // total pixels
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
  uint16_t internal_fb[DISP_PIX];     // used by the library for buffering
  uint16_t fb[DISP_PIX];              // the main framebuffer we draw onto.
  uint8_t zbuf[DISP_PIX] DMAMEM;     // z-buffer
#endif

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

  /*
    Point on a display
  */
  struct Point {
    int x, y;
    Point(int x, int y) : x(x), y(y) {} 
  };

// ----------------------------------------------------------------------------
// Function Definitions
// ----------------------------------------------------------------------------

// *** display ***
#if EN_DISP_MOD
  
  /*
    set priority
  */
  void set_priority(Point ll, Point ur, uint8_t priority) {
    int xlow = (ll.x >= 0) ? ll.x : 0;
    int ylow = (ll.y >= 0) ? ll.y : 0;
    int xhigh = (ur.x < DISP_LX) ? ur.x : DISP_LX - 1;
    int yhigh = (ur.y < DISP_LY) ? ur.y : DISP_LY - 1;
    for(int x = xlow; x <= xhigh; x++) {
      for(int y = ylow; y <= yhigh; y++) {
        zbuf[x + DISP_LX*y] = priority;
      }
    }
  }

  /*
    clear and draw background color, reset priority to 0 for pixels with priority >= the limit
  */
  void clear(Point ll={0, 0}, Point ur={DISP_LX - 1, DISP_LY - 1}, uint16_t color=0, uint8_t priority_limit=0) {
    int xlow = (ll.x >= 0) ? ll.x : 0;
    int ylow = (ll.y >= 0) ? ll.y : 0;
    int xhigh = (ur.x < DISP_LX) ? ur.x : DISP_LX - 1;
    int yhigh = (ur.y < DISP_LY) ? ur.y : DISP_LY - 1;
    for(int x = xlow; x <= xhigh; x++) {
      for(int y = ylow; y <= yhigh; y++) {
        int pix = x + DISP_LX*y;
        if(zbuf[pix] >= priority_limit) {
          fb[pix] = color;
          zbuf[pix] = 0;
        }
      }
    }
  }

  /*
    draw a point with customizable size, default red color
  */
  void draw_point(Point pt1, int size=2, uint16_t color=0xF800, uint8_t priority=3) {
    int xlow = (pt1.x > 0) ? pt1.x - 1 : 0;
    int xhigh = (pt1.x < DISP_LX - 1) ? pt1.x + 1 : DISP_LX - 1;
    int ylow = (pt1.y > 0) ? pt1.y - 1 : 0;
    int yhigh = (pt1.y < DISP_LY - 1) ? pt1.y + 1 : DISP_LY - 1;
    for(int x = xlow; x <= xhigh; x++) {
      for(int y = ylow; y <= yhigh; y++) {
        int pix = x + DISP_LX*y;
        if(priority >= zbuf[pix]) {
          fb[pix] = color;
          zbuf[pix] = priority;
        }
      }
    }
  }

  /*
    draw line between 2 points, uses Bresenham's line algorithm
  */
  void draw_line(Point pt1, Point pt2, uint16_t color=65535, uint8_t priority=2) {
    int x0 = pt1.x;
    int x1 = pt2.x;
    int y0 = pt1.y;
    int y1 = pt2.y;

    int dx = abs(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int error = dx + dy;
    
    int e2;
    while(1) {
      int pix = x0 + DISP_LX*y0;
      if(priority >= zbuf[pix]) {
        fb[pix] = color;
        zbuf[pix] = priority;
      }
      if (x0 == x1 && y0 == y1) {
        break;
      }
      e2 = 2 * error;
      if (e2 >= dy) {
          if (x0 == x1 || x0 >= DISP_LX) {
            break;
          } 
          error = error + dy;
          x0 = x0 + sx;
      }
      if (e2 <= dx) {
          if (y0 == y1 || y0 >= DISP_LY) {
            break;
          }
          error = error + dx;
          y0 = y0 + sy;
      }
    }
  }

  /*
    fill area with color
  */
  void draw_fill(Point ll, Point ur, uint16_t color=0, uint8_t priority=1) {
    int xlow = (ll.x >= 0) ? ll.x : 0;
    int ylow = (ll.y >= 0) ? ll.y : 0;
    int xhigh = (ur.x < DISP_LX) ? ur.x : DISP_LX - 1;
    int yhigh = (ur.y < DISP_LY) ? ur.y : DISP_LY - 1;
    for(int x = xlow; x <= xhigh; x++) {
      for(int y = ylow; y <= yhigh; y++) {
        int pix = x + DISP_LX*y;
        if(priority >= zbuf[pix]) {
          fb[pix] = color;
          zbuf[pix] = priority;
        }
      }
    }
  }

  /*
    draw a box
  */
  void draw_box(Point ll, Point ur, bool fill=false, uint16_t color=65535, uint8_t priority=4) {
    int xlow = (ll.x >= 0) ? ll.x : 0;
    int ylow = (ll.y >= 0) ? ll.y : 0;
    int xhigh = (ur.x < DISP_LX) ? ur.x : DISP_LX - 1;
    int yhigh = (ur.y < DISP_LY) ? ur.y : DISP_LY - 1;
    if(fill) {
      draw_fill(ll, ur, color, 1);
    }
    draw_line((Point){xlow, ylow}, (Point){xlow, yhigh}, color, priority);
    draw_line((Point){xlow, ylow}, (Point){xhigh, ylow}, color, priority);
    draw_line((Point){xhigh, ylow}, (Point){xhigh, yhigh}, color, priority);
    draw_line((Point){xlow, yhigh}, (Point){xhigh, yhigh}, color, priority);
  }

#endif

// ----------------------------------------------------------------------------
// Tests Definitions
// ----------------------------------------------------------------------------

// *** display ***
#if EN_DISP_TEST

  /*
    test to draw lines on the display
  */
  void line_test(int num_lines = 1) {
    for(int i = 0; i < num_lines; i++) {
      draw_line((Point){(int)random(DISP_LX), (int)random(DISP_LY)}, (Point){(int)random(DISP_LX), (int)random(DISP_LY)});
    }
  }

  /*
    test to draw lines and points on the display
  */
  void line_point_test(int num_lines = 1) {
    for(int i = 0; i < num_lines; i++) {
      Point pt1 = {(int)random(DISP_LX), (int)random(DISP_LY)};
      Point pt2 = {(int)random(DISP_LX), (int)random(DISP_LY)};
      draw_line(pt1, pt2);
      draw_point(pt1);
      draw_point(pt2);
    }
  }

  /*
    test to draw a rectangle
  */
  void box_test() {
    draw_box((Point){0,0},(Point){DISP_LX-1,DISP_LX-1});
  }

  /*
    test to fill a rectangle
  */
  void fill_test() {
    draw_box((Point){0,0},(Point){DISP_LX-1,DISP_LX-1},true);
  }

  /*
    test to draw basic ui background
  */
  void ui_test_01() {
    draw_box((Point){0,0},(Point){DISP_LX-1,DISP_LX-1});
    draw_fill((Point){0,DISP_LX},(Point){DISP_LX-1,DISP_LY-1},65535);
  }

#endif

// ----------------------------------------------------------------------------
// MCU Entry Point
// ----------------------------------------------------------------------------=
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(millis());  // remove

  disp.output(&Serial);                // output debug infos to serial port.     
  while (!disp.begin(DISP_SPI_SPEED));      // init the display
  disp.setRotation(0);                 // portrait mode 240 x320
  disp.setFramebuffer(internal_fb);    // set the internal framebuffer (enables double buffering)
  disp.setDiffBuffers(&diff1, &diff2); // set the 2 diff buffers => activate differential updates. 
  disp.setDiffGap(4);                  // use a small gap for the diff buffers

  disp.setRefreshRate(120);            // around 120hz for the display refresh rate. 
  disp.setVSyncSpacing(2);             // set framerate = refreshrate/2 (and enable vsync at the same time). 
}

void loop() {
  // put your main code here, to run repeatedly:
  clear();
  fill_test();
  disp.update(fb);
  
  
  
  while(1);
}
