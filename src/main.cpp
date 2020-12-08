/*
(Modified) MIT License

Copyright (c) 2020 Gábor Ziegler and other contributors

Portions of this repo contains sourcecode either inspired by or copied from 
published code from Adafruit Industires, from thisisant.com and from 
Nordic Semiconductor ASA. The main inputs were:
* Adafruit_nRF52_Arduino repo and various public forks of that (LGPL License)
* The nRF5 SDK by Nordic Semiconductor (a mashup of licenses)
* Various ANT+ software from thisisant.com

The license conditions of particular files can be found in the top of the 
individual files. The TL/DR summary of the restrictions beyond the usual 
 MIT license:
* This software, with or without modification, must only be used with a
  Nordic Semiconductor ASA integrated circuit.
* The user if this software, with or without modification, must comply with
  the ANT licensing terms: https://www.thisisant.com/developer/ant/licensing.
  (Note particluarly that the said ANT license permits only non-commercial, 
  non revenue-generating usage without paying a yearly license fee.)

The rest of this library, which are original contributions or
derivative works falls under the MIT license. 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software. The notifications about the 
legal requirements of adhering to the Nordic Semiconductor ASA and the
thisiant.com licensing terms shall be included.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#define USE_BLEUART true
#define USE_BMP true
#define USE_SDC true
// Set GPSDEBUG to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSDEBUG true

// Set GPSLOG to 'false' to turn off echoing the GPS data NMEA sentences to the fitness file
// Set to 'true' if you want to debug and log the NMEA sentences
#define GPSFITLOG true

//ANT+ devices-------------------------------------------------------
#include "ANTProfile.h"
#include "profiles/HeartRateMonitor.h"
#include "profiles/Environment.hh"
#include "profiles/SpeedDistanceMonitor.hh"
#include "profiles/BicycleSpeedCadence.h"
#include "sdant.h"

HeartRateMonitor hrm(RX);
EnvironmentSensor env(RX);
SpeedDistanceMonitor sdm(RX);
BicycleSpeedCadence cyclespeed(Speed, RX);
BicycleSpeedCadence pedalrpm(Cadence, RX);

uint8_t currHR = 0;    //heart rate, bpm
float currELE = 0;     // elevation (altitude), m
float currATEMP = 0;   // ambient temp., Celsius
uint8_t currCAD = 0;   // feet cadence, spm
uint8_t currRpm = 0;   // bike cadence, rpm
uint8_t currSpeed = 0; //bike speed, kmph

void _OnComputedHeartRate(int rate)
{
  Serial.printf("HRM computed rate: %d\n", rate);
  //bleuart.printf("HRM: %d\n", rate);
  currHR = rate;
}
void _OnTemperature(int16_t temp)
{
  //Serial.printf("Current temperature: %f\n", temp / 100.0);
  //bleuart.print("Temp:");
  //bleuart.println(temp / 100.0);
  currATEMP = temp / 100.0;
}
void _OnSpeed(float speed)
{
  Serial.printf("Speed:  %f km/h\n", speed);

  //bleuart.print("Speed (km/h):");
  //bleuart.println(speed);
}
void _OnCadence(uint8_t cadence)
{
  Serial.printf("Cadence:  %d\n", cadence);
  //bleuart.print("Cadence:");
  //bleuart.println(cadence);
  currCAD = cadence;
}
void _OnStride(int16_t stride)
{
  Serial.printf("Stride count:  %d\n", stride);
  //bleuart.print("Stride:");
  //bleuart.println(stride);
}
void _OnPace(uint8_t min, uint8_t sec)
{
  Serial.printf("Pace:  %02d:%02d min/km\n", min, sec);
  //bleuart.printf("Pace: %02d:%02d min/km\n", min, sec);
}
void PrintUnhandledANTEvent(ant_evt_t *evt)
{
  Serial.printf("Channel #%d for %s: event %s\n", evt->channel, ANTplus.getAntProfileByChNum(evt->channel)->getName(), AntEventTypeDecode(evt));
  if (evt->event != EVENT_CHANNEL_COLLISION && evt->event != EVENT_RX_FAIL && evt->event != EVENT_CHANNEL_CLOSED)
    Serial.printf("  (%s)\n", AntEventType2LongDescription(evt));
}
void ReopenANTChannel(ant_evt_t *evt)
{
  if (evt->event == EVENT_CHANNEL_CLOSED)
  {
    Serial.printf("Channel #%d closed for %s\n", evt->channel, ANTplus.getAntProfileByChNum(evt->channel)->getName());
    Serial.printf("Reopening...");
    uint32_t ret = sd_ant_channel_open(evt->channel);
    if (ret == NRF_SUCCESS)
      Serial.println("success!");
    else
      Serial.printf("failed with code:%#x\n", ret);
  }
}

//---Display --------------------------------------------------------
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
#include "Adafruit_miniTFTWing.h"
#include <SPI.h>
//#include <Adafruit_PCD8544.h>
#include "DataField.hh"

//---- baro sensor-------------------------------
#if USE_BMP
#define SEALEVELPRESSURE_HPA (1031) //988
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
Adafruit_BMP3XX bmp; // I2C
float sealevelpressure = SEALEVELPRESSURE_HPA;
#endif

//----------------RTC-----------------------------
// Date and time functions using a PCF8523 RTC connected via I2C and Wire lib
#include <RTClib.h>   //by Adafruit
#include <Timezone.h> // https://github.com/JChristensen/Timezone

//from RTClib
RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//adapting example from timezonelib
//https://www.timeanddate.com/time/zone/sweden/stockholm
//  2019	Sun, 31 Mar, 02:00	CET → CEST	+1 hour (DST start)	UTC+2h
// 	      Sun, 27 Oct, 03:00	CEST → CET	-1 hour (DST end)	UTC+1h
//  2020	Sun, 29 Mar, 02:00	CET → CEST	+1 hour (DST start)	UTC+2h
// 	      Sun, 25 Oct, 03:00	CEST → CET	-1 hour (DST end)	UTC+1h
//  2021	Sun, 28 Mar, 02:00	CET → CEST	+1 hour (DST start)	UTC+2h
// 	      Sun, 31 Oct, 03:00	CEST → CET	-1 hour (DST end) | Preliminary date	UTC+1h
//  2022	Sun, 27 Mar, 02:00	CET → CEST	+1 hour (DST start) | Preliminary date	UTC+2h
// 	      Sun, 30 Oct, 03:00	CEST → CET	-1 hour (DST end) | Preliminary date	UTC+1h
//  2023	Sun, 26 Mar, 02:00	CET → CEST	+1 hour (DST start) | Preliminary date	UTC+2h
//       	Sun, 29 Oct, 03:00	CEST → CET	-1 hour (DST end) | Preliminary date	UTC+1h
// TimeChangeRule myRule = {abbrev, week, dow, month, hour, offset};
// week: First, Second, Third, Fourth, Last
// dow: Sun, Mon, Tue, Wed, Thu, Fri, Sat
// month: Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec
//
// For year 2020 in Sweden (se) for winter (CET) and summer (CEST) times
TimeChangeRule seCEST_2020 = {"CEST", Last, Sun, Mar, 29, +120}; //UTC + 2 hours
TimeChangeRule seCET_2020 = {"CET", Last, Sun, Oct, 25, +60};    //UTC + 1 hours
//Timezone(TimeChangeRule dstStart, TimeChangeRule stdStart);
Timezone myTZ(seCEST_2020, seCET_2020);
TimeChangeRule *tcr; //pointer to the time change rule, use to get TZ abbrev

//---- Data logging------------------------------------------------------
#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;
SDFile activityFile, activityFile2;
#define ACTIVITY_FILE_PREFIX "/GARMIN/ACTIVITY/"
uint32_t activityFileTimer = xTaskGetTickCount();

// Callback to set file dates properly
// https://forum.arduino.cc/index.php?topic=348562.0
void FATdateTime(uint16_t *date, uint16_t *time)
{
  DateTime now = rtc.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

//---GPX file format -------------------------------------------------
#include "GPX.h"

//---miniTFT ----------------------------------------------------------
Adafruit_miniTFTWing ss;
#define TFT_RST -1 // we use the seesaw for resetting to save a pin
#define VBATPIN A6
#define TEXT_SIZE 2
#define TFT_CS 5
#define TFT_DC 6
// 16 bit '565' color:              rrrrr gggggg bbbbb
#define MINITFT_DARKRED 0xC000    //11000 000000 00000
#define MINITFT_DARKBLUE 0x0018   //00000 000000 11000
#define MINITFT_DARKORANGE 0xCB80 //1100 1 011 100 00000
#define MINITFT_DARKGREEN 0x0380  //0000 0 011 100 0 0000

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
uint32_t buttontimer = xTaskGetTickCount();

float measuredvbat = 0.0, measuredvbat_old = 0.0;
char oldvoltage[5] = "0000", newvoltage[5] = "0000";
uint8_t batlevel = 0;
OneLineFieldDescriptor oneLineDesc;
OneLineDataField fieldVolt, fieldFix, fieldSats, fieldQual, fieldHRM, fieldCAD, fieldATEMP, fieldEleGPS, fieldEleBARO, fieldTempBARO, fieldPress;

uint32_t displayTimer = xTaskGetTickCount();

#define VBATPIN A6 //BAT battery voltage
#define VUSBPIN A0 //USB USB-voltage/2
//https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/nrf52-adc
#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER (0.5F)           // 150K + 150K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (2.0F)      // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
bool usbdisconnected = true;

float readVBAT(void)
{
  float raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  //analogReference(AR_INTERNAL_3_0); //set in setup

  // Set the resolution to 12-bit (0..4095)
  //analogReadResolution(12); // Can be 8, 10, 12 or 14 //set in the setup

  // Let the ADC settle
  //delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBATPIN);

  // Set the ADC back to the default settings
  //analogReference(AR_DEFAULT);
  //analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}
uint8_t mvToPercent(float mvolts)
{
  if (mvolts < 3300)
    return 0;

  if (mvolts < 3600)
  {
    mvolts -= 3300;
    return mvolts / 30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F); // thats mvolts /6.66666666
}

//float vusbmin = 0;
//float vusbmax = 0;

//----BLE UART -------------------------------------------------------------
#if USE_BLEUART
//ble-on
#include <bluefruit.h>
//#include <bluefruit52.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

// BLE Service
//BLEDfu  bledfu;  // OTA DFU service
BLEDis bledis;   // device information
BLEUart bleuart; // uart over ble
BLEBas blebas;   // battery

//fwd dcls
// callback invoked when central connects
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void startAdv(void);

//ble-off
#endif

//--- GPS -------------------------------------------------------------
#include <Adafruit_GPS.h>
// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

//----event handling---------------------------
boolean sealevelpressureRestore(float &pressure)
{
  //call begin again, just in case the card has been  ejected and reinserted
  if (!SD.begin(chipSelect))
  {
    Serial.println("ERROR: sealevelpressureRestore: couldn't reinit card!");
    return false;
  }
  File seapFile = SD.open("SEALEV.TXT", FILE_READ);
  if (!seapFile)
  {
    Serial.println("ERROR: sealevelpressureRestore: couldn't open SEALEV.txt");
    return false;
  }
  //assuming single line containing sealevel pressure as float.
  String line = "";

  if (seapFile.available())
    line = seapFile.readStringUntil('\n');
  else
    return false;
  //https://github.com/arduino/Arduino/issues/1719
  //Arduino can't do float scanning, so scan as string first.
  //if (sscanf(line.c_str(), "%s", &pressure) != 1)
  if (line.toFloat() == 0.0)
    return false;
  else
  {
    pressure = line.toFloat();
    Serial.print("sealevelpressureStore: read pressure from file as: ");
    Serial.println(pressure);
    return true;
  }
}

boolean sealevelpressureStore(float pressure)
{
  //call begin again, just in case the card has been  ejected and reinserted
  if (!SD.begin(chipSelect))
  {
    Serial.println("ERROR: sealevelpressureStore: couldn't reinit card!");
    return false;
  }
  File seapFile = SD.open("SEALEV.TXT", O_READ | O_WRITE | O_CREAT | O_TRUNC);
  if (!seapFile)
  {
    Serial.println("ERROR: sealevelpressureStore: couldn't open SEALEV.txt");
    return false;
  }
  seapFile.println(pressure);
  seapFile.close();
  Serial.printf("sealevelpressureStore: stored pressure as %f\r\n", pressure);
  return true;
}

boolean activityRecording = false;
boolean activityStart()
{
  //call begin again, just in case the card has been  ejected and reinserted
  if (!SD.begin(chipSelect))
  {
    Serial.println("ERROR: activityStart couldn't reinit card, activity NOT started.");
    return false;
  }
  DateTime now = rtc.now();
  String filename = String(ACTIVITY_FILE_PREFIX + String(now.unixtime(), HEX) + ".gpx");
  Serial.print("Activity starting. Filename:");
  Serial.println(filename);

  activityFile = SD.open(filename, FILE_WRITE);
  if (activityFile)
  {
    Serial.println("Activity started.");
    activityFile.println(gpx_intro_start());
    activityFile.println(gpx_time(now.timestamp()));
    activityFile.println(gpx_intro_end());
    activityFile.println(gpx_trk_start("nRF52840-track " + now.timestamp()));
    tft.fillCircle(159 - 6, 8, 5, MINITFT_DARKRED);
    activityRecording = true;
  }
  else
  {
    Serial.print("ERROR: Couldn't open file. Activity NOT started");
    activityRecording = false;
  }
#if SHADOWFILE
  //shadow file
  filename = String(ACTIVITY_FILE_PREFIX + String(now.unixtime(), HEX) + ".g2x");
  Serial.print("Shadow activity starting. Filename:");
  Serial.println(filename);

  activityFile2 = SD.open(filename, FILE_WRITE);
  if (activityFile2)
  {
    Serial.println("Activity started.");
    activityFile2.println(gpx_intro_start());
    activityFile2.println(gpx_time(now.timestamp()));
    activityFile2.println(gpx_intro_end());
    activityFile2.println(gpx_trk_start("nRF52840-track " + now.timestamp()));
    return true;
  }
  else
  {
    Serial.print("ERROR: Couldn't open file. Activity NOT started");
    return false;
  }
#endif
  return activityRecording;
}

boolean activityReStart()
{
  if (!activityRecording)
  {
    Serial.println("ERROR: activityReStart has found activityRecording flag was false! Aborting restart");
    return false;
  }
  String filename = String(ACTIVITY_FILE_PREFIX) + activityFile.name();
  Serial.print("Attempting restart for ");
  Serial.println(filename);
  //call begin again, just in case the card has been  ejected and reinserted
  activityFile.close();
  SD.end();
  if (!SD.begin(chipSelect))
  {
    Serial.println("ERROR: activityReStart couldn't reinit card, activity NOT restarted.");
    return false;
  }
  DateTime now = rtc.now();
  Serial.print("Activity restarting, using old filename:");
  Serial.println(filename);
  if (SD.exists(filename))
  {
    Serial.println("...file exists.");
  }
  else
  {
    Serial.println("...file doesn't exist.");
  }
  //the permissions here are different than those at activityStart: we need to append
  activityFile = SD.open(filename, O_APPEND | O_RDWR | O_WRITE | O_CREAT);
  if (activityFile)
  {
    //activityFile.println(gpx_intro_start());
    //activityFile.println(gpx_time(now.timestamp()));
    //activityFile.println(gpx_intro_end());
    //activityFile.println(gpx_trk_start("nRF52840-track " + now.timestamp()));
    if (activityFile.println("<file_restart time=\"" + String(now.timestamp() + "\"/>")) > 0)
    {
      Serial.println("Activity restarted for " + filename);
      tft.fillCircle(159 - 6, 8, 5, MINITFT_DARKRED);
      return true;
    }
    else
      Serial.println("File " + filename + " has been reopened, but writing still failed.");
    return false;
  }
  else
  {
    Serial.print("ERROR: Couldn't open file. Activity NOT restarted");
  }
  return false;
}

void activityStop()
{
  Serial.print("Activity stopped. Filename:");
  Serial.println(activityFile.name());
  activityFile.println(gpx_trk_end());
  activityFile.close();
  activityRecording = false;
#if SHADOWFILE
  Serial.print("Activity stopped. Filename:");
  Serial.println(activityFile2.name());
  activityFile2.println(gpx_trk_end());
  activityFile2.close();
#endif
  tft.fillCircle(159 - 6, 8, 5, ST77XX_WHITE);
}

//---- Screen handling routines --------------------------------------
typedef enum
{
  up,
  down,
  left,
  right,
  sel
} minitft_buttons_t;

//assuming 2px internal border, 12x16 fontbox, indices are x=0..11, y=0..15
#define LEFT_X 1    // 0+1
#define MIDDLE_X 5  // 0+12/6
#define RIGHT_X 10  // 11-1
#define TOP_Y 1     // 0+1
#define MIDDLE_Y 7  // 0+16/2
#define BOTTOM_Y 14 // 15-1
void draw_button(uint8_t tcol, uint8_t trow, minitft_buttons_t button, uint16_t color)
{
  //assuming TEXTSIZE=2, i.e., 12x16 font. Pixel origo is upper-left (rotation=3). Boxorigo is upper left.
  uint16_t box_x = tcol * 12;
  uint16_t box_y = trow * 16;
  switch (button)
  {
  case left:
    tft.fillTriangle(box_x + LEFT_X, box_y + MIDDLE_Y, box_x + RIGHT_X, box_y + TOP_Y, box_x + RIGHT_X, box_y + BOTTOM_Y, color);
    break;
  case right:
    tft.fillTriangle(box_x + LEFT_X, box_y + TOP_Y, box_x + LEFT_X, box_y + BOTTOM_Y, box_x + RIGHT_X, box_y + MIDDLE_Y, color);
    break;
  case up:
    tft.fillTriangle(box_x + LEFT_X, box_y + BOTTOM_Y, box_x + RIGHT_X, box_y + BOTTOM_Y, box_x + MIDDLE_X, box_y + TOP_Y, color);
    break;
  case down:
    tft.fillTriangle(box_x + LEFT_X, box_y + TOP_Y, box_x + RIGHT_X, box_y + TOP_Y, box_x + MIDDLE_X, box_y + BOTTOM_Y, color);
    break;
  default:
    break;
  }
}

bool inSubmenu = false;
uint32_t subMenuTicks = 0;
// handle buttons
void handleButtons(uint32_t buttons)
{
  //drawing buttons
  tft.setTextColor(MINITFT_DARKORANGE, ST77XX_WHITE);
  uint8_t tcol = 0;
  uint16_t color = ST77XX_WHITE;

  //left
  color = ST77XX_WHITE;
  if (!(buttons & TFTWING_BUTTON_LEFT))
  {
    //Serial.println("BUTTON: LEFT");
    color = MINITFT_DARKORANGE;
  }
  draw_button(tcol++, 4, left, color);

  //right
  color = ST77XX_WHITE;
  if (!(buttons & TFTWING_BUTTON_RIGHT))
  {
    //Serial.println("BUTTON: RIGHT");
    color = MINITFT_DARKORANGE;
  }
  draw_button(tcol++, 4, right, color);

  //up
  color = ST77XX_WHITE;
  if (!(buttons & TFTWING_BUTTON_UP))
  {
    //Serial.println("BUTTON: UP");
    color = MINITFT_DARKORANGE;
    if (inSubmenu)
    {
      inSubmenu = false;
      if (!activityFile)
        activityStart();
      Serial.print("Leaving Submenu: ");
      Serial.println(subMenuTicks);
      Serial.println((xTaskGetTickCount() - subMenuTicks));
    }
  }
  draw_button(tcol++, 4, up, color);

  //down
  color = ST77XX_WHITE;
  if (!(buttons & TFTWING_BUTTON_DOWN))
  {
    //Serial.println("BUTTON: DOWN");
    color = MINITFT_DARKORANGE;
    if (inSubmenu)
    {
      inSubmenu = false;
      if (activityFile)
        activityStop();
      Serial.print("Leaving Submenu: ");
      Serial.println(subMenuTicks);
      Serial.println((xTaskGetTickCount() - subMenuTicks));
    }
  }
  draw_button(tcol++, 4, down, color);

  //select
  tft.setCursor(12 * (tcol++), 4 * 16);
  if (!(buttons & TFTWING_BUTTON_SELECT))
  {
    //Serial.println("BUTTON: SELECT");
    tft.print("S");
    if (!inSubmenu)
    {
      inSubmenu = true;
      subMenuTicks = xTaskGetTickCount();
      Serial.print("inSubmenu: ");
      Serial.println(subMenuTicks);
      Serial.println((xTaskGetTickCount() - subMenuTicks));
    }
  }
  else
    tft.print(" ");

  //A
  tft.setCursor(12 * (tcol++), 4 * 16);
  if (!(buttons & TFTWING_BUTTON_A))
  {
    //Serial.println("BUTTON: A");
    tft.print("A");
    if (!activityFile || !activityRecording)
    {
      if (!activityStart())
        ;
    }
  }
  else
    tft.print(" ");

  //B
  tft.setCursor(12 * (tcol++), 4 * 16);
  if (!(buttons & TFTWING_BUTTON_B))
  {
    //Serial.println("BUTTON: B");
    tft.print("B");
    if (activityFile || activityRecording)
      activityStop();
  }
  else
    tft.print(" ");

  //Recording
  tft.setCursor(12 * (tcol++), 4 * 16);
  if (activityFile)
    tft.setTextColor(MINITFT_DARKRED, ST77XX_WHITE);
  else
    tft.setTextColor(ST77XX_WHITE, ST77XX_WHITE);
  tft.print("(R)");
  tcol += 2;

  //in submenu
  //Recording
  tft.setCursor(12 * (tcol++), 4 * 16);
  if (inSubmenu)
    tft.setTextColor(MINITFT_DARKRED, ST77XX_WHITE);
  else
    tft.setTextColor(ST77XX_WHITE, ST77XX_WHITE);
  tft.print("?");
}

// initially assume all buttons released
uint32_t prev_buttons = ~TFTWING_BUTTON_ALL;

//---- persistent configuration handling

float_t getSeaLevelPressureFromAltitude(float_t altitude)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  if (bmp.performReading())
  {
    double atmospheric = bmp.pressure / 100.0;
    float level = (float)(atmospheric / pow(((44330.0 - altitude) / 44330.0), 5.255));
    Serial.printf("getSeaLevelPressureFromAltitude: got altitude %f, measured pressure %f, returning press. value %f\n", altitude, atmospheric, level);
    return level;
  }
  else
  {
    Serial.println("getSeaLevelPressureFromAltitude: can't read pressure, returning builtin value " + String(SEALEVELPRESSURE_HPA));
    return (float)SEALEVELPRESSURE_HPA;
  }
}

// ---- BLE UART handling-------------------------------------------------------
#if USE_BLEUART
void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}
// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = {0};
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;

  Serial.println();
  Serial.println("Disconnected");
}

#endif //USE_BLEUART

void processCommand(String p_blecommand)
{
  bleuart.println("Received:" + p_blecommand);
  if (p_blecommand == "astop\n")
  {
    if (activityFile || activityRecording)
    {
      activityStop();
      bleuart.println("Activity stopped.");
      bleuart.println("File:" + String(activityFile.name()));
      handleButtons(prev_buttons);
    }
    return;
  }
  if (p_blecommand == "astart\n")
  {
    bleuart.println("Attempting astart...");
    if (!activityFile || !activityRecording)
    {
      bleuart.print("Activity start:");
      if (activityStart())
        bleuart.println("success");
      else
        bleuart.println("failure");
      bleuart.println("File:" + String(activityFile.name()));
      handleButtons(prev_buttons);
    }
    return;
  }
  if (p_blecommand.startsWith("seap="))
  {
    bleuart.println("Processing seap=...");
    int16_t press;
    int scanned = sscanf(p_blecommand.c_str(), "seap=%d", &press);
    if (scanned == 1)
    {
      sealevelpressure = press;
      bleuart.printf("Scanned seap=%d\r\n", press);
    }
    else
    {
      bleuart.printf("Scanning seap failed!\n");
      bleuart.printf("sealevel pressure remains %f\r\n", sealevelpressure);
    }
    if (sealevelpressureStore((float)press))
      bleuart.printf("Stored pressure as %f\r\n ", (float)press);
    else
      bleuart.println("Failed to store pressure");
    return;
  }
  if (p_blecommand.startsWith("elev="))
  {
    bleuart.println("Processing elev=...");
    uint16_t elev;
    int scanned = sscanf(p_blecommand.c_str(), "elev=%u", &elev);
    if (scanned == 1)
    {
      sealevelpressure = getSeaLevelPressureFromAltitude(elev);
      bleuart.printf("Scanned elev=%d\r\n", elev);
      bleuart.print("Calculated pressure:");
      bleuart.printf("%f\r\n", sealevelpressure);
    }
    else
    {
      bleuart.printf("Scanning elev. failed!\n");
    }
    if (sealevelpressureStore(sealevelpressure))
      bleuart.printf("Stored as %f\r\n", sealevelpressure);
    else
      bleuart.println("Failed to store pressure");
    return;
  }
  if (p_blecommand == "gstat\n")
  {
    bleuart.printf("F:%d S:%d Q:%d\r\n", GPS.fix, GPS.satellites, GPS.fixquality);
    bleuart.print("Lat:" + String(GPS.latitudeDegrees, 7));
    bleuart.println(" Lon:" + String(GPS.longitudeDegrees, 7));

    bleuart.print("Baro. elevation:");
    bleuart.println(String(bmp.readAltitude(sealevelpressure), 2));
    bleuart.print("Sealevel press.");
    bleuart.println("assumed as: " + String(sealevelpressure, 5));
    bleuart.print("GPS elevation:");
    bleuart.println(String(GPS.altitude));
    bleuart.println("RTC:" + rtc.now().timestamp());
    bleuart.printf("GPS:%d-%02d-%02dT", 2000 + GPS.year, GPS.month, GPS.day);
    bleuart.printf("%02d:%02d:%02d\n", GPS.hour, GPS.minute, GPS.seconds);
    bleuart.println("HRM:"+ String(currHR) + " ELE:" + String(currELE));
    bleuart.println("SPM:"+ String(currRpm) + " TMP:" + String(currATEMP));
    bleuart.println("Internal temp.:" + String(bmp.readTemperature()));

    if (activityRecording)
    {
      bleuart.printf("Recording activity:\n");
      if (activityFile)
        bleuart.printf("File:\n");
      else
        bleuart.printf("Failing:\n");
      bleuart.printf("%s\r\n", activityFile.name());
    }
    else
      bleuart.println("Not recording activity");
    return;
  }
}

//---- Setup ----------------------------------
void setup()
{
  bool ret = false;
  Serial.begin(115200);
  
  //while (!Serial); //cannot be blocked due to lack of USB connection out on the field
  if (!Serial)
    delay(5000);
  delay(2000);
  Serial.printf("Starting setup.\nSketch compiled: %s %s\n", F(__DATE__), F(__TIME__));

  //setting up LiPo measurement
  // https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/nrf52-adc
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0); //(0.6V Ref * 5 = 0..3.0V)
  // Set the ADC resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  //taking care of Datalogger (RTC and SD card)
  //setting up LiPo measurement
  // https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/nrf52-adc
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0); //(0.6V Ref * 5 = 0..3.0V)

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  //restore startup data from SD card, if any
#if USE_SDC
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    //while (1)
    //  ;
  }

  //Error: \.platformio\lib\RTClib_ID83\RTClib.cpp:975:30: error: call of overloaded 'write(int)' is ambiguous
  //needs fix at \.platformio\lib\RTClib_ID83\RTClib.h:37:31
  //#define DS3231_TIME           (uint8_t)0x00
  //see https://forum.arduino.cc/index.php?topic=174883.msg1299547#msg1299547
  if (!rtc.initialized())
  {
    Serial.println("RTC is NOT running!");
    Serial.println("Initializing to the the date & time this sketch was compiled!");
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  else
  {
    DateTime now = rtc.now();
    Serial.print("RTC is initialized: ");
    Serial.println(now.timestamp());
  }
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
    Serial.println("card failed, or not present!");
  else
    Serial.println("card initialized.");

  SdFile::dateTimeCallback(FATdateTime); //https://forum.arduino.cc/index.php?topic=348562.0

  //restore last sealevel pressure from SD card
  float press;
  if (sealevelpressureRestore(press))
  {
    sealevelpressure = press;
    Serial.printf("Successfully restored sealevelpressure from SD card as %f hPa\n", sealevelpressure);
  }
  else // use compiled default
  {
    //sealevelpressure = SEALEVELPRESSURE_HPA;
    Serial.printf("Couldn't restore sealevelpressure, so will use a predermined value of %f hPa\n", sealevelpressure);
  }
#endif //USE_SDC

  // ---- start miniTFT featherwing

  //--- tft
  if (!ss.begin())
  {
    Serial.println("seesaw couldn't be found!");
    while (1)
      ;
  }
  //lcd.init(); // initialize the lcd
  //lcd.backlight();
  //lcd.setCursor(0, 0);
  Serial.print("seesaw started!\tVersion: ");
  Serial.println(ss.getVersion(), HEX);

  ss.tftReset();   // reset the display
  ss.setBacklight( //TFTWING_BACKLIGHT_ON
      0x0000       //max
                   //0x7fff //50%
  );               // turn off the backlight

  tft.initR(INITR_MINI160x80); // initialize a ST7735S chip, mini display
  Serial.println("TFT initialized");
  delay(1000); //seesaw reset can be slot
  tft.setRotation(3);
  tft.setTextSize(TEXT_SIZE); // Set text 'magnification' size. Each increase in s makes 1 pixel that much bigger.
                              //Desired text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
  tft.fillScreen(ST77XX_WHITE);
  tft.setCursor(0, 0);
  //tft.setTextColor(MINITFT_DARKORANGE, ST77XX_WHITE);

  //commom  data field attributes for TFT
  oneLineDesc.descriptionColor = ST77XX_BLACK;
  oneLineDesc.bgColor = ST77XX_WHITE;
  //oneLineDesc.xScale = 2;        //< horizontal pixel stretching factor (ADafruit_GFX default font is 6x8 px)
  //oneLineDesc.yFieldScale = 2;   //< vertical pixel stretching factor for the data field part
  oneLineDesc.yDescScale = 2; //< vertical pixel stretching factor for the desc.  field part

  //first row
  oneLineDesc.displayRow = 0; //< display row# assuming identical fields everywhere on display
  //battery voltage
  oneLineDesc.displayCol = 0; //< dislay column# assuming identical fields everywhere on display
  oneLineDesc.dataColor = MINITFT_DARKORANGE;
  fieldVolt = OneLineDataField("V:", newvoltage, oneLineDesc, &tft);
  fieldVolt.reprint();
  //HRM comes after battery
  oneLineDesc.displayCol = String("V:4200").length(); //< dislay column# assuming identical fields everywhere on display
  oneLineDesc.dataColor = MINITFT_DARKRED;
  fieldHRM = OneLineDataField(" H:", "123", oneLineDesc, &tft);
  fieldHRM.reprint();

  //second row, GPS stats
  oneLineDesc.displayRow = 1; //< display row# assuming identical fields everywhere on display
  oneLineDesc.dataColor = MINITFT_DARKGREEN;
  oneLineDesc.descriptionColor = ST77XX_BLACK;
  //GPS fix
  oneLineDesc.displayCol = 0; //< dislay column# assuming identical fields everywhere on display
  fieldFix = OneLineDataField("F:", "0", oneLineDesc, &tft);
  fieldFix.reprint();
  //GPS nof. sats
  oneLineDesc.displayCol = String("F:0").length(); //< dislay column# assuming identical fields everywhere on display
  fieldSats = OneLineDataField(" S:", "00", oneLineDesc, &tft);
  fieldSats.reprint();
  //GPS fix. qual.
  oneLineDesc.displayCol = String("F:0 S:00").length(); //< dislay column# assuming identical fields everywhere on display
  fieldQual = OneLineDataField(" Q:", "0", oneLineDesc, &tft);
  fieldQual.reprint();

  //third row, cadence & temp
  oneLineDesc.displayRow = 2; //< display row# assuming identical fields everywhere on display
  oneLineDesc.dataColor = ST77XX_BLACK;
  oneLineDesc.descriptionColor = ST77XX_BLACK;
  //Cadence
  oneLineDesc.displayCol = 0; //< dislay column# assuming identical fields everywhere on display
  fieldCAD = OneLineDataField("C:", "172", oneLineDesc, &tft);
  fieldCAD.reprint();
  //Ambient temp.
  oneLineDesc.displayCol = String("C:172").length(); //< dislay column# assuming identical fields everywhere on display
  fieldATEMP = OneLineDataField(" T:", "+22.2", oneLineDesc, &tft);
  fieldATEMP.reprint();

  //fourth row, elevation
  oneLineDesc.displayRow = 3; //< display row# assuming identical fields everywhere on display
  oneLineDesc.dataColor = ST77XX_BLACK;
  //elevation
  oneLineDesc.displayCol = 0; //< dislay column# assuming identical fields everywhere on display
  fieldEleBARO = OneLineDataField("B", "100.2", oneLineDesc, &tft);
  fieldEleBARO.reprint();
  oneLineDesc.displayCol = String("B100.2").length();
  ; //< dislay column# assuming identical fields everywhere on display
  fieldEleGPS = OneLineDataField(" G", "100.2", oneLineDesc, &tft);
  fieldEleGPS.reprint();
  Serial.println("TFT initial screen printed");

  ///----GPS init---------------------------------------------------------------------------------
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GGA only lacks GPS date
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);               //1 Hz sampling
#define PMTK_API_SET_FIX_CTL_2HZ "$PMTK300,500,0,0,0,0*28" ///< 2 Hz
#define PMTK_API_SET_FIX_CTL_3HZ "$PMTK300,333,0,0,0,0*2E" ///< 3 Hz
  //https://nmeachecksum.eqth.net/
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_2HZ); //2 Hz sampling
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_3HZ); //3 Hz sampling
  GPS.sendCommand(PMTK_ENABLE_SBAS);      ///< Enable search for SBAS satellite (only works with 1Hz
                                          ///< output rate)
  GPS.sendCommand(PMTK_ENABLE_WAAS);      ///< Use WAAS for DGPS correction data
#define PMTK_DISABLE_SBAS "$PMTK313,1*2E" ///< Enable search for SBAS satellite (only works with 1Hz \
                                          ///< output rate)
#define PMTK_DISABLE_WAAS "$PMTK301,2*2E" ///< Use WAAS for DGPS correction data

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);
  Serial.println("GPS has been initialized");

//----baro sensor init ------------------------------------------------------------------------------
#if USE_BMP
  if (!bmp.begin_I2C())
  {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
  else
  {
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    bmp.performReading();
    currELE = bmp.readAltitude(sealevelpressure);
  }
  Serial.println("BMP3 sensor has been initiazed. Elevation measured as:" + String(currELE));
#endif

  //---- Initializing ANT+ sensors -------------------------------------
  Serial.println("Adding HRM profile");
  hrm.SetOnComputedHeartRate(_OnComputedHeartRate);
  hrm.setUnhandledEventListener(PrintUnhandledANTEvent);
  hrm.setAllEventListener(ReopenANTChannel);
  hrm.setName("HRM");
  ANTplus.AddProfile(&hrm);

  Serial.println("Adding ENV profile");
  env.SetOnTemperatureData(_OnTemperature);
  env.setUnhandledEventListener(PrintUnhandledANTEvent);
  env.setAllEventListener(ReopenANTChannel);
  env.setName("TEMPE");
  ANTplus.AddProfile(&env);

  Serial.println("Adding SDM profile");
  sdm.setOnCadenceData(_OnCadence);
  sdm.setOnSpeedData(_OnSpeed);
  sdm.setOnStrideCountData(_OnStride);
  sdm.setOnPaceData(_OnPace);
  sdm.setUnhandledEventListener(PrintUnhandledANTEvent);
  sdm.setAllEventListener(ReopenANTChannel);
  sdm.setName("Footpod");
  ANTplus.AddProfile(&sdm);

  Serial.println("Adding bike cadence profile");
  //hrm.SetOnComputedHeartRate(_OnComputedHeartRate);
  pedalrpm.setUnhandledEventListener(PrintUnhandledANTEvent);
  pedalrpm.setAllEventListener(ReopenANTChannel);
  pedalrpm.setName("PedalRpm");
  ANTplus.AddProfile(&pedalrpm);

  Serial.println("Adding bike speed  profile");
  //hrm.SetOnComputedHeartRate(_OnComputedHeartRate);
  cyclespeed.setUnhandledEventListener(PrintUnhandledANTEvent);
  cyclespeed.setAllEventListener(ReopenANTChannel);
  cyclespeed.setName("CycleSpeed");
  ANTplus.AddProfile(&cyclespeed);

#if USE_BLEUART
  //ble on
  Serial.println("BLEUART Startup");
  Serial.println("---------------------------\n");
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_NORMAL);

  Serial.print("Starting BLE stack. Expecting 'true':");
  ret = Bluefruit.begin(1, 0);
  Serial.println(ret);
  Serial.print("Starting ANT stack. Expecting 'true':");
  ret = ANTplus.begin(5); //5 channels: HRM, SDM, ENV, RPM, SPD
  Serial.println(ret);

  ANTProfile *profiles[] = {&hrm, &env, &sdm, &pedalrpm, &cyclespeed};
  for (auto i : profiles)
  {
    Serial.printf("Channel number for %s became %d\n", i->getName(), i->getChannelNumber());
  }
  Serial.println("ANT+ profile setup finished, ANT+ broadcast listening has started");

  //---- BLE -----------------------------------------------------------------
  Bluefruit.setTxPower(5); // Check bluefruit.h for supported values
  Bluefruit.setName("ArduinoSportsTracker");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  //bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  //blebas.write(100);

  // Set up and start advertising
  startAdv();
  Serial.println("BLE advertising has started, BLE init finished");
//ble off
#endif
}

bool RTC_checked = false;
bool fix_detected = false;
bool gpsUpdated = false;
String blecommand = "";
String ELE, ATEMP, HR, CAD;
nmea_float_t lastLongitude = 0.0, lastLatitude = 0.0;
String lastNMEAfix = "";
bool fixIsLost = false;

//---- process GPS update ----------------------------------------------
void processGpsUpdate()
{
  if (GPS.fix)
  {
    if (lastLatitude != GPS.latitudeDegrees || lastLongitude != GPS.longitudeDegrees)
    {
      gpsUpdated = true; //counting updates
      if (activityFile && GPSFITLOG)
        activityFile.println("<positionUpdated/>");

      lastLatitude = GPS.latitudeDegrees;
      lastLongitude = GPS.longitudeDegrees;
    }
    if (fixIsLost)
    {
      if (activityFile)
        activityFile.println("<fixRegained time='" + rtc.now().timestamp() + "'/>");
      fixIsLost = false;
    }
  }
  else //no fix
  { 
    if (fix_detected && !fixIsLost) //we had fix earlier but we just lost it
    {
      if (activityFile)
        activityFile.println("<fixLost time='" + rtc.now().timestamp() + "'/>");
      fixIsLost = true;
    }
  } //if no fix
} //processGpsUpdate

void checkDisplayButtons(void)
{

  //check miniTFT buttons
  uint32_t buttons = ss.readButtons();
  if (buttons != prev_buttons)
  {
    handleButtons(buttons);
    prev_buttons = buttons;
  }
  if (inSubmenu && (xTaskGetTickCount() - subMenuTicks > pdMS_TO_TICKS(2000)))
  {
    inSubmenu = false;
    Serial.println("Submenu timeout");
    handleButtons(prev_buttons);
  }
} //check display buttons

//---- refresh sensor fields---------------------------------------------
void refreshSensorDataFields()
{
  //######################### refreshing display and sensor data

  //----------------------------read sensors -----------
  ///elevation, ambient temp, heartrate, cadence
  //elevation
  //barom. elev.
  if (!bmp.performReading())
  {
    Serial.println("Failed to perform bmp reading :(");
    currELE = GPS.altitude;
  }
  else
  {
    currELE = bmp.readAltitude(sealevelpressure);
  }
  //hrm, temp, cadence is update via IRQ callbacks

  // preparign disp. values
  ELE = String(currELE);
  //uint32_t ticks = xTaskGetTickCount();

  //check ANT+ sensor data
  //consider sensor data being stale after 2*3=6 sec (assuming 0,5Hz Tempe )

  //Stringify values for fitness file
  if (displayTimer - env.newTicks <= pdMS_TO_TICKS(6000))
    ATEMP = String(currATEMP);
  else
    ATEMP = "";
  if (displayTimer - hrm.newTicks <= pdMS_TO_TICKS(6000))
    HR = String(currHR);
  else
    HR = "";
  if (displayTimer - sdm.newTicks <= pdMS_TO_TICKS(6000))
    CAD = String(currCAD);
  else
    CAD = "";

  //Serial.println(update++);
  //display.setCursor(0, 1 * 8);
  //display.setTextColor(BLACK, WHITE);
  if (hrm.newRxData)
  {
    fieldHRM.updateContentRight(String(currHR));
    //display.printf("%3d %f", currHR, bmp.temperature);
  }
  else
  {
    fieldHRM.updateContentRight("---");
    //display.printf("--- %f", bmp.temperature);
  }

  if (sdm.newRxData)
    fieldCAD.updateContentRight(String((int)currCAD));
  else
    fieldCAD.updateContentRight("", '-');
  if (env.newRxData)
    fieldATEMP.updateContentLeft(String(currATEMP));
  else
    fieldATEMP.updateContentLeft("", '-');

  //---- GPS fix data -------------------------------------
  fieldFix.updateContentRight(String((int)GPS.fix));
  fieldSats.updateContentRight(String((int)GPS.satellites));
  fieldQual.updateContentRight(String((int)GPS.fixquality));

  //---- elevation -----
  fieldEleBARO.updateContentLeft(String(bmp.readAltitude(sealevelpressure)));
  fieldEleGPS.updateContentRight(String(GPS.altitude));

  //mark stale sensor data as such
  if (displayTimer - hrm.newTicks >= pdMS_TO_TICKS(5000))
    hrm.newRxData = false;
  if (displayTimer - env.newTicks >= pdMS_TO_TICKS(5000))
    env.newRxData = false;
  if (displayTimer - sdm.newTicks >= pdMS_TO_TICKS(5000))
    sdm.newRxData = false;

} //refreshSensorDataFields

#if USE_SDC

//---- initialize RTC ---------------------------------------------------------------
void adjustRTC()
{

  /* Checking RTC*/
  //uint16_t milliseconds; ///< GMT milliseconds
  uint8_t seconds; ///< GMT seconds
  uint8_t minute;  ///< GMT minutes
  uint8_t hour;    ///< GMT hours
  uint8_t day;     ///< GMT day
  uint8_t month;   ///< GMT month
  uint16_t year;   ///< GMT year

  Serial.println("Adjusting RTC");
  DateTime now = rtc.now();

  seconds = GPS.seconds;
  minute = GPS.minute;
  hour = GPS.hour;
  day = GPS.day;
  month = GPS.month;
  year = GPS.year;

  if (now.second() != seconds || now.minute() != minute || now.hour() != hour || now.day() != day || now.month() != month || now.year() != year + 2000)
  {
    //Serial.println("now.second() != seconds: " + String(now.second() != seconds));
    //Serial.println("now.second(): " + String(now.second()));
    //Serial.println("seconds: " + String(seconds));

    //Serial.println("now.minute() != minute: " + String(now.minute() != minute));
    //Serial.println("now.hour() != hour: "+ String(now.hour() != hour));
    //Serial.println("now.day() != day: " + String(now.day() != day));
    //Serial.println("now.month() != month: "+ String(now.month() != month ));
    //Serial.println("now.year() != year +2000: " + String(now.year() != year+2000));

    //adjusting
    rtc.adjust(DateTime(year, month, day, hour, minute, seconds));
    Serial.println("RTC deviated more than a second, adjusted  RTC");
    Serial.print("old RTC: ");
    Serial.printf("%02d", now.year());
    Serial.print('-');
    Serial.printf("%02d", now.month());
    Serial.print('-');
    Serial.printf("%02d", now.day());
    //Serial.print(" (");
    //Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    //Serial.print(")");
    Serial.printf(" %02d", now.hour());
    Serial.print(':');
    Serial.printf("%02d", now.minute());
    Serial.print(':');
    Serial.printf("%02d", now.second());
    Serial.println(" UTC");

    Serial.print("GPS:     ");
    Serial.printf("  %02d", year);
    Serial.print('-');
    Serial.printf("%02d", month);
    Serial.print('-');
    Serial.printf("%02d", day);
    Serial.print(' ');
    Serial.printf("%02d", hour);
    Serial.print(':');
    Serial.printf("%02d", minute);
    Serial.print(':');
    Serial.printf("%02d", seconds);
    Serial.println(" UTC");

    now = rtc.now();
    Serial.print("new RTC: ");
    Serial.printf("%02d", now.year());
    Serial.print('-');
    Serial.printf("%02d", now.month());
    Serial.print('-');
    Serial.printf("%02d", now.day());
    //Serial.print(" (");
    //Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    //Serial.print(")");
    Serial.printf(" %02d", now.hour());
    Serial.print(':');
    Serial.printf("%02d", now.minute());
    Serial.print(':');
    Serial.printf("%02d", now.second());
    Serial.println(" UTC");
  }
  else // diff small
  {
    Serial.println("RTC deviation is less than a second from GPS time, not adjusted RTC");
  }
  RTC_checked = true; //either adjusted, or good anyway
} //adjustRTC
#endif //USE RTC

//---- Measure LiPo battery ---------------------------------------------------
#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER (0.5F)           // 150K + 150K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (2.0F)      // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
void measAndDisplayBattery()
{
  // Measuring LiPo  battery //https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/nrf52-adc
  measuredvbat = readVBAT(); //in mV
  sprintf(newvoltage, "%4d", (int)measuredvbat);
  batlevel = mvToPercent(measuredvbat);
  if (0 != strcmp(newvoltage, oldvoltage))
  {
    fieldVolt.updateContentLeft(newvoltage);
    strcpy(oldvoltage, newvoltage);
    //update blebas
    blebas.write(batlevel);
  } //newvoltage
}

//---- update recorded track points
void addTrackPoint()
{
  //#################writing trkpts #########################
  // at updated positionm or at-least approximately every 1.199 seconds or so, write trkpts
  //if (gpsUpdated || (xTaskGetTickCount() - activityFileTimer >= pdMS_TO_TICKS(5000)) || (xTaskGetTickCount() < activityFileTimer)) //also take care of overflow
  activityFileTimer = xTaskGetTickCount(); // reset the buttontimer
  lastNMEAfix=GPS.lastNMEA();
  String tstamp;

  //if (gpsUpdated) {
  char buffer[20];
  sprintf(buffer, "%d-%02d-%02dT%02d:%02d:%02d.%d", 2000 + GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, 0 /*GPS.milliseconds*/);
  tstamp = String(buffer);
  //} else {
  //  tstamp = rtc.now().timestamp();
  //}

  if (!activityFile)
  {
    //recording stopped somehow, attempting to restart
    Serial.println("Recording is active, but file is not ready. Attempting to restart");
    activityReStart();
  }
  if (activityFile)
  {
    //fixed=dec.degrees*10^7. I.e., it has a pattern: DDmmmmmmm
    double lonFix = GPS.longitude_fixed / (double)10000000;
    double latFix = GPS.latitude_fixed / (double)10000000;
    size_t written = activityFile.println(gpx_trkpt(String(GPS.secondsSinceFix()), String(latFix, 7), String(lonFix, 7), ELE, tstamp, ATEMP, HR, CAD, lastNMEAfix));
    //Serial.print("Recording is active, trkpt chars: "); Serial.println(written);
    //activityFile.println("<lastPosDeg lon='" + String(GPS.longitudeDegrees, 6) + "' lat='" + String(GPS.latitudeDegrees, 6) + "' />");
    //activityFile.println("<lastPos lon='" + String(GPS.longitude, 4) + "' lat='" + String(GPS.latitude, 4) + "' />");
    //activityFile.println("<lastPosFix  lon='" + String(lonFix, 6) + "' lat='" + String(latFix, 6) + "' fixlon='" + String(GPS.longitude_fixed) + "' fixlat='" + String(GPS.latitude_fixed) + "' />");
    activityFile.println("<gpsFixQuality Q='" + String(GPS.fixquality) + "' S='" + String(GPS.satellites) + "' />");
    if (written == 0)
    {
      //recording stopped somehow, attempting to restart
      //Serial.println("Recording is active, trkpt failed. Attempting to restart");
      if (activityReStart())
      {
        size_t written = activityFile.println(gpx_trkpt(String(GPS.secondsSinceFix()), String(latFix, 6), String(lonFix, 6), ELE, tstamp, ATEMP, HR, CAD, lastNMEAfix));
        //Serial.print("Restart successful, trkpt chars: "); Serial.println(written);
      }
    }
    activityFile.flush();
  } //if activity file
}

uint8_t dollars = 0; //how many dollars in NMEA
uint32_t loopStart=xTaskGetTickCount() * ( TickType_t ) 1000 / configTICK_RATE_HZ ;//(double) portTICK_PERIOD_MS;
uint32_t loopStartOld=loopStart;// (double) portTICK_PERIOD_MS;
uint16_t recchars=0; //how many chars read in a single loop
void loop()
{
  //--- reading GPS
  char c;
  // read data from the GPS in the 'main loop'
  // read as long as GPS has something to say
  // and not yet a full sentence. Process full
  // sentences if there is any
  bool continuousRead=true; 
  while ( continuousRead && ( c = GPS.read() ) ) 
  {
      loopStartOld = loopStart;
      loopStart=xTaskGetTickCount() * ( TickType_t ) 1000 / configTICK_RATE_HZ; //(double) portTICK_PERIOD_MS;
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences!
      // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    if (GPSDEBUG) Serial.print(c);
    if (c == '$') 
      dollars++;               //count dollars in a sentence
    if (GPS.newNMEAreceived()) //if newline has been found
    {
      continuousRead = false; //process the current sentence before reading further
      if (GPSDEBUG) 
      { 
        Serial.print('\n');
        if ( dollars >1 ) Serial.println("==> Multiple dollars in sentence were detected");
        Serial.println("new NMEA: " + rtc.now().timestamp()+" millis="+ String(millis()) + "' loopStart='" + String(loopStart)+ "' loopStartOld='" + String(loopStartOld)+ "' loopGapMsec='" + String(loopStart-loopStartOld) +"'\n");
      }
      if (GPSFITLOG && activityFile) 
      {
        activityFile.println("<new_NMEA rtc='" + rtc.now().timestamp() +  "'  millis='" + String(millis()) + "' loopStart='" + String(loopStart) +  "' loopGapMsec='" + String(loopStart-loopStartOld)+ "'>" + GPS.lastNMEA() + "</new_NMEA>");
        if (dollars > 1)
          activityFile.println("<multipleNMEAdetected count='" + String(dollars) + "'/>");
      }

      bool parsed = GPS.parse(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
      if (!parsed)
      {
        if (activityFile && GPSFITLOG)
          activityFile.println("<couldntParseNMEA rtc='" + rtc.now().timestamp() + "'  millis='" + String(millis()) +"'/>");
      }
      else //parsed
      {
        processGpsUpdate(); //this process the NMEA, but not the GUI update
      } //if newNMEA succesfully parsed
    }   // if newNMEA
    if (c == '\n')
      dollars = 0; //reset dollars at end of line
  }                //while GPS.read
  
  //---- check ble commands -------------------------------------------------------
  if (bleuart.available())
  {
    //Serial.printf("bleuart available. blecommand before='%s'\n", blecommand.c_str());
    blecommand = bleuart.readString();
    if (blecommand != "")
    {
      processCommand(blecommand);
      blecommand = "";
    }
  } //check ble commands

  //---- checking display buttons -----------------------------------
  // approximately every 120msec or so, check buttons
  // note: too frequent check will f*k up Serial1, hence GPS
  if ((xTaskGetTickCount() - buttontimer >= pdMS_TO_TICKS(120)) || (xTaskGetTickCount() < buttontimer)) //also take care of overflow
  {
    buttontimer = xTaskGetTickCount(); // reset the buttontimer
    checkDisplayButtons();
  } // chek display buttons

  //check RTC
  if (!RTC_checked && GPS.fix)
  {
    adjustRTC();
  } //if (!checked)

  if (!fix_detected && GPS.fix)
  {
    Serial.printf("GPS Fix detected. 20%02d-%02d-%02dT%02d:%02d:%02dZ\n", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
    fix_detected = true;
  }

  //---- refresh sensors DataFields at every 1000msec
  if ((xTaskGetTickCount() - displayTimer >= pdMS_TO_TICKS(1000)) || (displayTimer > xTaskGetTickCount()))
  {
    // if millis() or displayTimer wraps around, we'll just reset it
    // approximately every X seconds or so, print out the current stats
    displayTimer = xTaskGetTickCount(); // reset the displayTimer

    refreshSensorDataFields();
    measAndDisplayBattery();
  }
  if (gpsUpdated && activityRecording)
  {
    addTrackPoint();
    gpsUpdated = false;
  } //writing trkpts
} //loop
