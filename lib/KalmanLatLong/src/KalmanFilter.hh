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

#include <cmath>
#include <stdint.h>

/// <summary>
/// Kalman filter processing for lattitude and longitude
/// </summary>
/// <param name="lat_measurement_degrees">new measurement of lattidude</param>
/// <param name="lng_measurement">new measurement of longitude</param>
/// <param name="accuracy">measurement of 1 standard deviation error in metres</param>
/// <param name="TimeStamp_milliseconds">time of measurement</param>
/// <returns>new state</returns>
//See https://stackoverflow.com/questions/1134579/smooth-gps-data

class KalmanLatLong
{
public:
    KalmanLatLong(float p_Q_metres_per_second)
    {
        Q_metres_per_second = p_Q_metres_per_second;
        variance = -1;
    }

    void SetState(double p_lat, double p_lng, float p_accuracy, long p_TimeStamp_milliseconds)
    {
        lat = p_lat;
        lng = p_lng;
        variance = p_accuracy * p_accuracy;
        TimeStamp_milliseconds = p_TimeStamp_milliseconds;
    }

    long get_TimeStamp() { return TimeStamp_milliseconds; }
    double get_lat() { return lat; }
    double get_lng() { return lng; }
    float get_accuracy() {
        return sqrt(variance); 
        }

private:
    const float MinAccuracy = 1.0;

    float Q_metres_per_second;
    uint32_t TimeStamp_milliseconds;
    double lat;
    double lng;
    float variance; // P matrix.  Negative means object uninitialised.  NB: units irrelevant, as long as same units used throughout

public:
    void Process(double lat_measurement, double lng_measurement, float accuracy, long TimeStamp_milliseconds);
};