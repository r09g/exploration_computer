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
#define EN_IMU_MOD    0   // BNO055 9-dof imu
#define EN_TPS_MOD    0   // BMP388 temp pressure sensor
#define EN_SD_MOD     0   // External SD card reader
#define EN_DISP_MOD   1   // Embedded display module
#define EN_DISP_TEST  1   // Enable display tests
#define EN_GPS_TEST   0   // Enable GPS tests
#define EN_TPS_TEST   0   // Enable temp pressure sensor tests
#define EN_IMU_TEST   0   // Enable BNO055 tests

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


// ----------------------------------------------------------------------------
// Macro
// ----------------------------------------------------------------------------

#define DEBUG_verbose   0   // print info for debugging

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

  #define DISP_MAX_LINES  23
  #define DISP_MAX_CHAR   34
#endif

#if EN_GPS_MOD
  #define GPSSerial Serial1  // RX=0, TX=1
#endif

#if EN_TPS_MOD
  #define SEALEVELPRESSURE_HPA (1013.25)
#endif

#if EN_IMU_MOD
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
  };  // 16 bytes

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
#endif

#if EN_IMU_MOD
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
#endif

// ----------------------------------------------------------------------------
// Function Definitions
// ----------------------------------------------------------------------------

// *** display ***
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

  /*
    print text
  */
  void disp_print(String str_in, bool same_line = false) {
    clear();
    if(same_line) {
      str_in.toCharArray(text_mem[disp_q.getCount() - 1], DISP_MAX_CHAR);
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
      if(count == 0 ) {
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
    disp_print("[GPS] Obtained GPS fix");
  }

  /*
    helper function to convert time to string
  */
  String convert_time_str() {
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

  /*
    get current gps time and convert to pst
  */
  String get_time_pst() {
    setTime(gps.hour, gps.minute, gps.seconds, gps.day, gps.month, gps.year);
    adjustTime((-7) * SECS_PER_HOUR);  // utc to pst
    return convert_time_str();
  }


#endif


#if EN_IMU_MOD
  

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
    uint8_t system, gyro, accel, mag = 0;
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

// ----------------------------------------------------------------------------
// MCU Entry Point
// ----------------------------------------------------------------------------=
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(millis());  // remove

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
  disp_print("[GPU] Connected");
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
  while(!bno.begin());
  bno.setExtCrystalUse(true);
  bno.setAxisRemap(bno.REMAP_CONFIG_P0);
  bno.setAxisSign(bno.REMAP_SIGN_P0);
  disp_print("[IMU] Connected");
#endif

  // GPS get fix
  gps_get_fix();
  disp_print("[GPS] FixQual=[" + String(gps.fixquality) + "," + String(gps.fixquality_3d) + "] #Sats=" + String(gps.antenna));
  disp_print("");

}

void loop() {
  // put your main code here, to run repeatedly:


  while(1);
}
