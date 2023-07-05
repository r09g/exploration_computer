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
#define EN_GPS_TEST   1   // Enable GPS tests
#define EN_TPS_TEST   1   // Enable temp pressure sensor tests
#define EN_IMU_TEST   1   // Enable BNO055 tests

// turn off test if module is not enabled
#if (EN_DISP_TEST && !EN_DISP_MOD)
  #undef EN_DISP_TEST
  #define EN_DISP_TEST 0
#endif
#if (EN_GPS_TEST && !EN_GPS_MOD)
  #undef EN_GPS_TEST
  #define EN_GPS_TEST 0
#endif
#if (EN_TPS_TEST && !EN_TPS_MOD)
  #undef EN_TPS_TEST
  #define EN_TPS_TEST 0
#endif
#if (EN_IMU_TEST && !EN_IMU_MOD)
  #undef EN_IMU_TEST
  #define EN_IMU_TEST 0
#endif


// ----------------------------------------------------------------------------
// Libraries
// ----------------------------------------------------------------------------
#include <Arduino.h>
#include <Bounce.h>

#if (EN_DISP_MOD || EN_GPS_MOD)
  #include <cppQueue.h>
#endif

#if EN_DISP_MOD
  #include <ILI9341_T4.h>
#endif

#if EN_GPS_MOD
  #include <Adafruit_GPS.h>
  #include <TinyGPSPlus.h>
  #include <TimeLib.h>
#endif

#if (EN_TPS_MOD || EN_IMU_MOD)
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
#endif

#if EN_TPS_MOD
  #include "Adafruit_BMP3XX.h"
#endif

#if EN_IMU_MOD
  #include <Adafruit_BNO055.h>
  #include <utility/imumaths.h>
#endif

#if EN_SD_MOD
  #include <SD.h>
  #include <SPI.h>
#endif


// ----------------------------------------------------------------------------
// Macro
// ----------------------------------------------------------------------------

#define DEBUG_verbose   0   // print info for debugging

// #define PIN_PB_LEFT   2
// #define PIN_PB_RIGHT  3
// #define PIN_PB_UP     4
// #define PIN_PB_DOWN   5
#define PIN_PB_ZI     6
#define PIN_PB_ZO     7
#define PIN_PB_CT     8
#define DEBOUNCE_TIME 10  // in ms

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

  #define DISP_GPS_LX 238  // pixels for GPS drawing area
  #define DISP_GPS_LY 238  // pixels for GPS drawing area
  #define DISP_ALT_LX 238  // pixels for altitude drawing area
  #define DISP_ALT_LY 59  // pixels for altitude drawing area

  #define DISP_MAX_LINES  23
  #define DISP_MAX_CHAR   34
#endif

#if EN_GPS_MOD
  #define GPSSerial Serial1  // RX=0, TX=1
#endif

#if EN_TPS_MOD
#endif

#if EN_IMU_MOD
#endif

#if EN_SD_MOD
  #define PIN_SD_SCK    13
  #define PIN_SD_MISO   12
  #define PIN_SD_MOSI   11
  #define PIN_SD_CS     14
#endif

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

#if EN_DISP_MOD
  /*
    Point on a display
  */
  struct Point {
    int x, y;
    Point() : x(0), y(0) {} 
    Point(int x, int y) : x(x), y(y) {} 
  };

#endif

#if EN_GPS_MOD
  /*
    location object
  */
  struct LocData {
    time_t time_data;
    float latdeg, londeg;
    float altitude;
    LocData() : time_data(0), latdeg(0), londeg(0), altitude(0) {} 
    LocData(time_t time_data, float latdeg, float londeg, float altitude) : time_data(time_data), latdeg(latdeg), londeg(londeg), altitude(altitude) {} 
  };  // 16 bytes

#endif

// ----------------------------------------------------------------------------
// Global Variables/Instances
// ----------------------------------------------------------------------------


// Bounce pb_left = Bounce(PIN_PB_LEFT, DEBOUNCE_TIME);
// Bounce pb_right = Bounce(PIN_PB_RIGHT, DEBOUNCE_TIME);
// Bounce pb_up = Bounce(PIN_PB_UP, DEBOUNCE_TIME);
// Bounce pb_down = Bounce(PIN_PB_DOWN, DEBOUNCE_TIME);
Bounce pb_zi = Bounce(PIN_PB_ZI, DEBOUNCE_TIME);
Bounce pb_zo = Bounce(PIN_PB_ZO, DEBOUNCE_TIME);
Bounce pb_ct = Bounce(PIN_PB_CT, DEBOUNCE_TIME);

time_t init_time;
unsigned long gps_timer;
float map_left_bound, map_right_bound, map_bottom_bound, map_top_bound, map_scale = 0.001;

#if EN_DISP_MOD
  // display driver
  ILI9341_T4::ILI9341Driver disp(PIN_DISP_CS, PIN_DISP_DC, PIN_DISP_SCK, PIN_DISP_MOSI, PIN_DISP_MISO, PIN_DISP_RESET);

  // diff buffers in bytes (8 bits)
  ILI9341_T4::DiffBuffStatic<8192> diff1;
  ILI9341_T4::DiffBuffStatic<8192> diff2;

  // framebuffers
  uint16_t internal_fb[DISP_PIX];     // used by the library for buffering
  uint16_t fb[DISP_PIX];              // the main framebuffer we draw onto.
  uint8_t zbuf[DISP_PIX] DMAMEM;      // z-buffer

  // for size 12 font on ILI9341 there can be 34 characters and 23 lines max
  char text_mem[DISP_MAX_LINES][DISP_MAX_CHAR] DMAMEM;
  cppQueue disp_q(DISP_MAX_CHAR, DISP_MAX_LINES, FIFO, true, text_mem, sizeof(text_mem));  // buffer for storing debug texts
#endif

#if EN_GPS_MOD
  Adafruit_GPS gps(&GPSSerial);

  LocData locdata_mem[10000] DMAMEM;
  cppQueue gps_q(sizeof(LocData), 10000, FIFO, true, locdata_mem, sizeof(locdata_mem));  // buffer for storing gps location data objects
#endif

#if EN_TPS_MOD
  Adafruit_BMP3XX bmp;
  float SEALEVELPRESSURE_HPA = 1015;
#endif

#if EN_IMU_MOD
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
#endif

// ----------------------------------------------------------------------------
// Function Definitions
// ----------------------------------------------------------------------------

#if EN_DISP_MOD
  /*
    wrapper to update display
  */
  void update_disp() {
    disp.update(fb);
  }

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
    draw a point with customizable drawing boundary, default red color
  */
  void draw_point(Point pt1, Point scrn_ll=(Point){0,0}, Point scrn_ur=(Point){DISP_LX-1,DISP_LY-1}, uint16_t color=0xF800, int rad=1, uint8_t priority=3) {
    // check if point is outside screen
    if(pt1.x < scrn_ll.x + rad || pt1.y < scrn_ll.y + rad || pt1.x > scrn_ur.x - rad || pt1.y > scrn_ur.y - rad) {
      return;
    }
    int xlow = pt1.x - rad;
    int xhigh = pt1.x + rad;
    int ylow = pt1.y - rad;
    int yhigh = pt1.y + rad;
    int rad2 = rad * rad;
    for(int x = xlow; x <= xhigh; x++) {
      int x2 = (x - pt1.x) * (x - pt1.x);
      for(int y = ylow; y <= yhigh; y++) {
        int y2 = (y - pt1.y) * (y - pt1.y);
        int pix = x + DISP_LX*y;
        if((x2 + y2) <= rad2) {
          if(priority >= zbuf[pix]) {
            fb[pix] = color;
            zbuf[pix] = priority;
          }
        }
      }
    }
  }

  /*
    draw line between 2 points, uses Bresenham's line algorithm
  */
  void draw_line(Point pt1, Point pt2, Point scrn_ll=(Point){0,0}, Point scrn_ur=(Point){DISP_LX-1,DISP_LY-1}, uint16_t color=65535, uint8_t priority=2) {
    // check if both points are off screen
    if((pt1.x < scrn_ll.x && pt2.x < scrn_ll.x) || (pt1.x > scrn_ur.x && pt2.x > scrn_ur.x) || (pt1.y < scrn_ll.y && pt2.y < scrn_ll.y) || (pt1.y > scrn_ur.y && pt2.y > scrn_ur.y)) {
      return;
    }
    
    int x0, x1, y0, y1;
    if(pt1.x < scrn_ll.x) {
      x0 = scrn_ll.x;
    } else if(pt1.x > scrn_ur.x) {
      x0 = scrn_ur.x;
    } else {
      x0 = pt1.x;
    }

    if(pt2.x < scrn_ll.x) {
      x1 = scrn_ll.x;
    } else if(pt2.x > scrn_ur.x) {
      x1 = scrn_ur.x;
    } else {
      x1 = pt2.x;
    }

    if(pt1.y < scrn_ll.y) {
      y0 = scrn_ll.y;
    } else if(pt1.y > scrn_ur.y) {
      y0 = scrn_ur.y;
    } else {
      y0 = pt1.y;
    }

    if(pt2.y < scrn_ll.y) {
      y1 = scrn_ll.y;
    } else if(pt2.y > scrn_ur.y) {
      y1 = scrn_ur.y;
    } else {
      y1 = pt2.y;
    }

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
          if (x0 == x1 || x0 > scrn_ur.x || x0 < scrn_ll.x) {
            break;
          } 
          error = error + dy;
          x0 = x0 + sx;
      }
      if (e2 <= dx) {
          if (y0 == y1 || y0 > scrn_ur.y || y0 < scrn_ll.y) {
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
    draw_line((Point){xlow, ylow}, (Point){xlow, yhigh}, (Point){0,0}, (Point){DISP_LX-1,DISP_LY-1}, color, priority);
    draw_line((Point){xlow, ylow}, (Point){xhigh, ylow}, (Point){0,0}, (Point){DISP_LX-1,DISP_LY-1}, color, priority);
    draw_line((Point){xhigh, ylow}, (Point){xhigh, yhigh}, (Point){0,0}, (Point){DISP_LX-1,DISP_LY-1}, color, priority);
    draw_line((Point){xlow, yhigh}, (Point){xhigh, yhigh}, (Point){0,0}, (Point){DISP_LX-1,DISP_LY-1}, color, priority);
  }

  /*
    print text
  */
  void disp_print(String str_in, bool same_line = false) {
    clear();
    if(same_line) {
      uint16_t idx = (disp_q.get_in_idx() == 0) ? DISP_MAX_LINES - 1 : disp_q.get_in_idx() - 1;
      str_in.toCharArray(text_mem[idx], DISP_MAX_CHAR);
    } else {
      char str[DISP_MAX_CHAR]; 
      str_in.toCharArray(str, DISP_MAX_CHAR);
      disp_q.push(str);
    }
    for(int i = 0; i < disp_q.getCount(); i++) {
      char tmp[DISP_MAX_CHAR];
      disp_q.peekIdx(tmp, i);
      disp.overlayText(fb, tmp, 3, i, 12);
    }
    update_disp();
  }

  /*
    draw the ui boxes
  */
  void draw_ui() {
    draw_box((Point){0,0}, (Point){DISP_GPS_LX+1,DISP_GPS_LY+1}, 0, 0x5ACB, 10);
    draw_box((Point){0,DISP_GPS_LY+1}, (Point){DISP_ALT_LX+1,DISP_GPS_LY+DISP_ALT_LY+2}, 0, 0x5ACB, 10);
  }

  /*
    draws gps points given bounds and scale
  */
  void draw_graph(bool draw_alt=false) {
    if(!gps_q.isEmpty()) {  // do nothing if empty
      // clear current map
      clear((Point){1,1}, (Point){DISP_GPS_LX,DISP_GPS_LY});

      String str_in = String(int(TinyGPSPlus::distanceBetween(0, 0, map_scale, 0)));
      char str[DISP_MAX_CHAR] = "                                 ";
      disp.overlayText(fb, str, 0, 0, 12, ILI9341_T4_COLOR_WHITE, 1.0f, ILI9341_T4_COLOR_BLACK, 1.0f);
      str_in.toCharArray(str, DISP_MAX_CHAR);
      disp.overlayText(fb, str, 0, 0, 12);

      LocData data, prev_data;
      int x, y, prev_x, prev_y;
      float min_alt, max_alt;
      // draw map
      for(int idx = 0; idx < gps_q.getCount(); idx++) {
        if(idx > 0) {
          prev_data = data;
          prev_x = x;
          prev_y = y;
        }
        gps_q.peekIdx(&data, idx);
        if(draw_alt) {
          if(idx == 0) {
            min_alt = data.altitude;
            max_alt = data.altitude;
          } else {
            if(data.altitude > max_alt) {
              max_alt = data.altitude;
            }
            if(data.altitude < min_alt) {
              min_alt = data.altitude;
            }
          }
        }
        x = 1 + int(DISP_GPS_LX*(data.latdeg - map_left_bound)/map_scale);
        y = 1 + int(DISP_GPS_LY*(data.londeg - map_bottom_bound)/map_scale);
        if(idx == gps_q.getCount() - 1) {
          draw_point((Point){x,y}, (Point){1,1}, (Point){DISP_GPS_LX,DISP_GPS_LY}, 0x659F, 3);
        } else {
          draw_point((Point){x,y}, (Point){1,1}, (Point){DISP_GPS_LX,DISP_GPS_LY});
        }
        if(idx > 0) {
          draw_line((Point){prev_x,prev_y}, (Point){x,y}, (Point){1,1}, (Point){DISP_GPS_LX,DISP_GPS_LY});
        }
      }
      // draw altitude plot
      if(draw_alt) {        
        gps_q.peek(&data);
        time_t min_time = data.time_data;
        gps_q.peekPrevious(&data);
        time_t max_time = data.time_data;
        time_t time_scale = max_time - min_time;
        float alt_scale = max_alt - min_alt;

        char str[DISP_MAX_CHAR] = "                                 ";
        disp.overlayText(fb, str, 0, 17, 12, ILI9341_T4_COLOR_WHITE, 1.0f, ILI9341_T4_COLOR_BLACK, 1.0f);
        String str_in = String(int(alt_scale));
        str_in.toCharArray(str, DISP_MAX_CHAR);
        disp.overlayText(fb, str, 0, 17, 12);
        
        for(int idx = 0; idx < gps_q.getCount(); idx++) {
          if(idx > 0) {
            prev_data = data;
            prev_x = x;
            prev_y = y;
          }
          gps_q.peekIdx(&data, idx);
          x = 1 + int(DISP_ALT_LX*(data.time_data - min_time)/time_scale);
          y = 2 + DISP_GPS_LY + int(DISP_ALT_LY*(data.altitude - min_alt)/alt_scale);
          // draw_point((Point){x,y}, (Point){1,2+DISP_GPS_LY}, (Point){DISP_ALT_LX,1+DISP_GPS_LY+DISP_ALT_LY});
          if(idx > 0) {
            draw_line((Point){prev_x,prev_y}, (Point){x,y}, (Point){1,2+DISP_GPS_LY}, (Point){DISP_ALT_LX,1+DISP_GPS_LY+DISP_ALT_LY});
          }
        }
      }
    }
    draw_ui();
  }

  void draw_bar() {
    time_t t_elapsed = now() - init_time;
    char str[34] = "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH";
    disp.overlayText(fb, str, 2, 0, 12, ILI9341_T4_COLOR_BLACK, 1.0f, ILI9341_T4_COLOR_BLACK, 1.0f);
    String time_str = "[";
    int hours = int(t_elapsed/3600);
    if(hours < 10) {
      time_str += "0";
    }
    time_str += String(hours);
    time_str += ":";
    int minutes = int((t_elapsed - hours*3600)/60);
    if(minutes < 10) {
      time_str += "0";
    }
    time_str += String(minutes);
    time_str += ":";
    int seconds = int(t_elapsed - hours*3600 - minutes*60);
    if(seconds < 10) {
      time_str += "0";
    }
    time_str += String(seconds);
    time_str += "] H[";
    sensors_event_t event; 
    uint8_t system, gyro, accel, mag;
    bno.getEvent(&event);
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if(event.orientation.x < 100) {
      time_str += "0";
    }
    if(event.orientation.x < 10) {
      time_str += "0";
    }
    time_str += String(event.orientation.x, 0);
    time_str += "] T[";
    int temp = int(bmp.temperature);
    if(temp < 0) {

    } else if(temp < 10) {
      time_str += "0";
    }
    time_str += String(temp);
    time_str += "] A[";
    int alt = int(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    if(alt < 0) {

    } else if(alt < 10) {
      time_str += "000";
    } else if(alt < 100) {
      time_str += "00";
    } else if (alt < 1000) {
      time_str += "0";
    }
    time_str += String(alt);
    time_str += "] S[";
    if(gps.satellites < 10) {
      time_str += "0";
    }
    time_str += String(gps.satellites);
    time_str += "]";
    time_str.toCharArray(str, 41);
    disp.overlayText(fb, str, 2, 0, 12);
  }


  /*
    centers ui to current location
  */
  void ui_center() {
    if(!gps_q.isEmpty()) {
      LocData data;
      gps_q.peekPrevious(&data);
      map_left_bound = data.latdeg - map_scale/2;
      map_right_bound = data.latdeg + map_scale/2;
      map_bottom_bound = data.londeg - map_scale/2;
      map_top_bound = data.londeg + map_scale/2;
    } else {
      map_left_bound = 0;
      map_right_bound = 0.001;
      map_bottom_bound = 0;
      map_top_bound = 0.001;
      map_scale = 0.001;
    }
  }

  /*
    scales ui
  */
  void ui_scale(float factor=2) {
    map_scale = map_scale * factor;
    if(factor < 1) {
      float shrink = map_scale * (1 - factor);
      map_left_bound += shrink/2;
      map_right_bound -= shrink/2;
      map_bottom_bound += shrink/2;
      map_top_bound -= shrink/2;
    } else {
      float expand = map_scale * (factor - 1);
      map_left_bound -= expand/2;
      map_right_bound += expand/2;
      map_bottom_bound -= expand/2;
      map_top_bound += expand/2;
    }
  }

#else

  // dummy function for use when display not enabled
  void disp_print(String str_in, bool same_line = false) {}

#endif


#if EN_GPS_MOD
  /*
    fetch new gps data
  */
  bool gps_fetch_data() {
    gps.read();
    if(gps.newNMEAreceived()) {
      if(!gps.parse(gps.lastNMEA()))
        return false;
      return true;
    }
    return false;
  }

  void gps_read() {
    while(!gps_fetch_data());
  }

  /*
    Wait for GPS fix
  */
  void gps_get_fix() {
    disp_print("[GPS] Getting GPS fix ...");
    int count = 0;
    while(!gps.fix) {
      count++;
      if(count == 4) { count = 0; }
      while(!gps_fetch_data());
      delay(500);
      if(count == 0) {
        disp_print("[GPS] Getting GPS fix", true);
      } else if(count == 1) {
        disp_print("[GPS] Getting GPS fix .", true);
      } else if (count == 2) {
        disp_print("[GPS] Getting GPS fix ..", true);
      } else {
        disp_print("[GPS] Getting GPS fix ...", true);
      }
    }
    disp_print("[GPS] Getting GPS fix ...", true);
    setTime(gps.hour, gps.minute, gps.seconds, gps.day, gps.month, gps.year);
    adjustTime((-7) * SECS_PER_HOUR);
    while(year() < 2000) {
      while(!gps_fetch_data());
    }
    disp_print("[GPS] Obtained GPS fix");
  };

  /*
    helper function to convert time to string
  */
  String get_time_pst() {
    String time_str = "";
    time_str = String(time_str + String(year()));
    time_str = String(time_str + "/");
    if(month() < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(month()));
    time_str = String(time_str + "/");
    if(day() < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(day()));
    time_str = String(time_str + " - ");
    if(hour() < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(hour()));
    time_str = String(time_str + ":");
    if(minute() < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(minute()));
    time_str = String(time_str + ":");
    if(second() < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(second()));
    return time_str;
  }

  /*
    helper function to convert time to string (overloaded)
  */
  String convert_time_str(tmElements_t &tm) {
    String time_str = "";
    time_str = String(time_str + String(tmYearToCalendar(tm.Year)));
    time_str = String(time_str + "/");
    if(tm.Month < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(tm.Month));
    time_str = String(time_str + "/");
    if(tm.Day < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(tm.Day));
    time_str = String(time_str + " - ");
    if(tm.Hour < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(tm.Hour));
    time_str = String(time_str + ":");
    if(tm.Minute < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(tm.Minute));
    time_str = String(time_str + ":");
    if(tm.Second < 10)
      time_str = String(time_str + "0");
    time_str = String(time_str + String(tm.Second));
    return time_str;
  }

#endif


#if EN_TPS_MOD
  void tps_read() {
    while(!bmp.performReading());
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


#if EN_GPS_TEST
  /*
    test read gps data and print to monitor
  */
  void gps_time_test() {
    while(!gps_fetch_data());
    if(!gps.fix) {
      return;
    }
    Serial.println(get_time_pst());
    delay(1000);
    time_t time_data = now();
    Serial.println(time_data);
    tmElements_t tm_data;
    breakTime(time_data, tm_data);
    Serial.println(convert_time_str(tm_data));
    time_t new_time_data = makeTime(tm_data);
    Serial.println(new_time_data);
    while(1);
  }

  // test gps compute location distance
  void gps_location_test() {
    while(!gps_fetch_data());
    if(!gps.fix) {
      return;
    }
    nmea_float_t latdeg = gps.latitudeDegrees;
    nmea_float_t londeg = gps.longitudeDegrees;
    Serial.println(latdeg);
    Serial.println(londeg);
    Serial.println(TinyGPSPlus::distanceBetween(latdeg, londeg, latdeg+0.01, londeg+0.01));
  }

  // read altitude from gps
  void gps_altitude_test() {
    while(!gps_fetch_data());
    if(!gps.fix) {
      return;
    }
    Serial.println(gps.altitude);
    delay(2000);
  }

  void gps_quality_test() {
    while(!gps_fetch_data());
    Serial.print(gps.fix);
    Serial.print(" ");
    Serial.print(gps.fixquality);
    Serial.print(" ");
    Serial.println(gps.satellites);
    delay(500);
  }

#endif


#if EN_TPS_TEST
  void tps_test_altitude() {
    if (!bmp.performReading()) {
      return;
    }
    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println();
    delay(2000);
  }
#endif


#if EN_IMU_TEST
  void compass_mode() {
    sensors_event_t event; 
    bno.getEvent(&event);
    Serial.println(event.orientation.x, 0);
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
#endif


#if (EN_GPS_TEST && EN_TPS_TEST)
  void locdata_test() {
    gps_read();
    tps_read();
    LocData data(now(), gps.latitudeDegrees, gps.longitudeDegrees, bmp.readAltitude(SEALEVELPRESSURE_HPA));
    gps_q.push(&data);

    LocData prev;
    gps_q.pop(&prev);
    Serial.println(prev.latdeg);
    Serial.println(prev.londeg);
    Serial.println(prev.altitude);
  }

#endif


// ----------------------------------------------------------------------------
// MCU Entry Point
// ----------------------------------------------------------------------------


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(millis());  // remove
  unsigned long last_time;

#if EN_DISP_MOD
  disp.output(&Serial);                // output debug infos to serial port.     
  while (!disp.begin(DISP_SPI_SPEED));      // init the display
  disp.setRotation(0);                 // portrait mode 240 x320
  disp.setFramebuffer(internal_fb);    // set the internal framebuffer (enables double buffering)
  disp.setDiffBuffers(&diff1, &diff2); // set the 2 diff buffers => activate differential updates. 
  disp.setDiffGap(4);                  // use a small gap for the diff buffers
  disp.setRefreshRate(120);            // around 120hz for the display refresh rate. 
  disp.setVSyncSpacing(2);             // set framerate = refreshrate/2 (and enable vsync at the same time).
  disp_print("[DISP] Connected");
#endif

#if EN_GPS_MOD
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  gps.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ
  disp_print("[GPS] Connected");
#endif

#if EN_TPS_MOD
  while(!bmp.begin_I2C());
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  disp_print("[TPS] Connected");
#endif

#if EN_IMU_MOD
  while(!bno.begin(OPERATION_MODE_COMPASS));
  // while(!bno.begin());
  bno.setExtCrystalUse(true);
  bno.setAxisRemap(bno.REMAP_CONFIG_P0);
  bno.setAxisSign(bno.REMAP_SIGN_P0);
  disp_print("[IMU] Connected");
#endif

  // pinMode(PIN_PB_LEFT, INPUT_PULLUP);
  // pinMode(PIN_PB_RIGHT, INPUT_PULLUP);
  // pinMode(PIN_PB_UP, INPUT_PULLUP);
  // pinMode(PIN_PB_DOWN, INPUT_PULLUP);
  pinMode(PIN_PB_ZI, INPUT_PULLUP);
  pinMode(PIN_PB_ZO, INPUT_PULLUP);
  pinMode(PIN_PB_CT, INPUT_PULLUP);
  disp_print("[SYS] Push buttons active");

#if EN_GPS_MOD
  // GPS get fix
  gps_get_fix();
  delay(1000);
  disp_print("[GPS] Time: " + get_time_pst());
  disp_print("[GPS] Loc: " + String(gps.latitudeDegrees, 6) + ", " + String(gps.longitudeDegrees, 6));
  disp_print("[GPS] FixQual=[" + String(gps.fixquality) + "," + String(gps.fixquality_3d) + "] #Sats=" + String(gps.satellites));
  last_time = millis();
  while(1) {
    if(millis() - last_time > 500) {
      disp_print("[GPS] FixQual=[" + String(gps.fixquality) + "," + String(gps.fixquality_3d) + "] #Sats=" + String(gps.satellites), 1);
      last_time = millis();
    }
    if(pb_ct.update()) {
      if(pb_ct.fallingEdge()) {
        break;
      }
    }
  }
#endif

#if EN_TPS_MOD
  // TPS report
  for(int i = 0; i < 10; i++) {
    tps_read();
    delay(100);
  }
  disp_print("[TPS] " + String(bmp.temperature) + "*C " + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "m " + String(SEALEVELPRESSURE_HPA, 1) + "hPa");

  last_time = millis();
  while(1) {
    if(millis() - last_time > 500) {
      disp_print("[TPS] " + String(bmp.temperature) + " *C " + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "m " + String(SEALEVELPRESSURE_HPA, 1) + "hPa", 1);
      last_time = millis();
    }
    if(pb_ct.update()) {
      if(pb_ct.fallingEdge()) {
        break;
      }
    }
    if(pb_zi.update()) {
      if(pb_zi.fallingEdge()) {
        SEALEVELPRESSURE_HPA -= 0.1; 
      }
    }
    if(pb_zo.update()) {
      if(pb_zo.fallingEdge()) {
        SEALEVELPRESSURE_HPA += 0.1;
      }
    }
  }
#endif

delay(500);

#if EN_IMU_MOD
  sensors_event_t event; 
  bno.getEvent(&event);
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  disp_print("[IMU] Cal=[" + String(system) + "," + String(gyro) + "," + String(accel) + "," + String(mag) + "] x=" + String(event.orientation.x, 0));
  last_time = millis();
  while(1) {
    if(millis() - last_time > BNO055_SAMPLERATE_DELAY_MS) {
      bno.getEvent(&event);
      bno.getCalibration(&system, &gyro, &accel, &mag);
      disp_print("[IMU] Cal=[" + String(system) + "," + String(gyro) + "," + String(accel) + "," + String(mag) + "] x=" + String(event.orientation.x, 0), 1);
    }
    if(pb_ct.update()) {
      if(pb_ct.fallingEdge()) {
        break;
      }
    }
  }

#endif

  delay(500);
  disp_print("[SYS] Setup complete ...");
  delay(1000);

  clear();

  init_time = now();
  gps_timer = millis();

  draw_ui();
  map_left_bound = gps.latitudeDegrees - map_scale/2;
  map_right_bound = gps.latitudeDegrees + map_scale/2;
  map_bottom_bound = gps.longitudeDegrees - map_scale/2;
  map_top_bound = gps.longitudeDegrees + map_scale/2;
  map_scale = 0.001;
  update_disp();
}

// actual program
void run() {
  // backend
  if(millis() - gps_timer > 1000) {
    gps_read();
    tps_read();
    if(gps.fix) {
      Serial.println(gps.satellites);
      LocData new_data(now(), gps.latitudeDegrees, gps.longitudeDegrees, bmp.readAltitude(SEALEVELPRESSURE_HPA));
      if(gps_q.isEmpty()) {
        gps_q.push(&new_data);
      } else {
        LocData prev;
        gps_q.peekPrevious(&prev);
        if((TinyGPSPlus::distanceBetween(new_data.latdeg, new_data.londeg, prev.latdeg, prev.londeg) > 10) || (now() - prev.time_data > 60)) {
          gps_q.push(&new_data);
        }
      }
      gps_timer = millis();
      draw_graph(0);
      draw_ui();
    }
  } 

  draw_bar();

  if(pb_ct.update()) {
    if(pb_ct.fallingEdge()) {
      ui_center();
    }
  }

  if(pb_zi.update()) {
    if(pb_zi.fallingEdge()) {
      ui_scale(0.5);
    }
  }

  if(pb_zo.update()) {
    if(pb_zo.fallingEdge()) {
      ui_scale(2);
    }
  }
  update_disp();
  return;
}

// for debugging
void run_debug() {
  LocData data1(10, 37.4798, -122.1554, 10);
  LocData data2(12, 37.4799, -122.1556, -5);
  LocData data3(16, 37.48, -122.1557, 0);
  LocData data4(17, 37.4801, -122.1552, 14);
  LocData data5(20, 37.4802, -122.1555, 22);

  gps_q.push(&data1);
  gps_q.push(&data2);
  gps_q.push(&data3);
  gps_q.push(&data4);
  gps_q.push(&data5);

  while(1) {
    draw_graph(1);
    draw_bar();
    if(pb_ct.update()) {
      if(pb_ct.fallingEdge()) {
        ui_center();
      }
    }

    if(pb_zi.update()) {
      if(pb_zi.fallingEdge()) {
        ui_scale(0.5);
      }
    }

    if(pb_zo.update()) {
      if(pb_zo.fallingEdge()) {
        ui_scale(2);
      }
    }
    update_disp();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // run();
  run_debug();
}
