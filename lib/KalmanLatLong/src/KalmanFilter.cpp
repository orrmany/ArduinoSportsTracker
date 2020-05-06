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

#include "KalmanFilter.hh"
//#include <stdint.h>

/// <summary>
/// Kalman filter processing for lattitude and longitude
/// </summary>
/// <param name="lat_measurement_degrees">new measurement of lattidude</param>
/// <param name="lng_measurement">new measurement of longitude</param>
/// <param name="accuracy">measurement of 1 standard deviation error in metres</param>
/// <param name="TimeStamp_milliseconds">time of measurement</param>
/// <returns>new state</returns>
//See https://stackoverflow.com/questions/1134579/smooth-gps-data

void KalmanLatLong::Process(double p_lat_measurement, double p_lng_measurement, float p_accuracy, long p_TimeStamp_milliseconds)
{
    if (p_accuracy < MinAccuracy)
        p_accuracy = MinAccuracy;
    if (variance < 0)
    {
        // if variance < 0, object is unitialised, so initialise with current values
        TimeStamp_milliseconds = p_TimeStamp_milliseconds;
        lat = p_lat_measurement;
        lng = p_lng_measurement;
        variance = p_accuracy * p_accuracy;
    }
    else
    {
        // else apply Kalman filter methodology

        long TimeInc_milliseconds = p_TimeStamp_milliseconds - TimeStamp_milliseconds;
        if (TimeInc_milliseconds > 0)
        {
            // time has moved on, so the uncertainty in the current position increases
            variance += TimeInc_milliseconds * Q_metres_per_second * Q_metres_per_second / 1000;
            TimeStamp_milliseconds = p_TimeStamp_milliseconds;
            // TO DO: USE VELOCITY INFORMATION HERE TO GET A BETTER ESTIMATE OF CURRENT POSITION
        }

        // Kalman gain matrix K = Covarariance * Inverse(Covariance + MeasurementVariance)
        // NB: because K is dimensionless, it doesn't matter that variance has different units to lat and lng
        float K = variance / (variance + p_accuracy * p_accuracy);
        // apply K
        lat += K * (p_lat_measurement - lat);
        lng += K * (p_lng_measurement - lng);
        // new Covarariance  matrix is (IdentityMatrix - K) * Covarariance
        variance = (1 - K) * variance;
    }
};