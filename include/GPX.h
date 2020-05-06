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

#ifndef GPX_H
#define GPX_H
#include <Arduino.h>

//C++11 raw string literals, see https://stackoverflow.com/questions/1135841/c-multiline-string-literal
const char *gpx_intro_start()
{
    return R"XML(<?xml version="1.0" encoding="UTF-8"?>
<gpx creator="Garmin Connect" version="1.1"
  xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/11.xsd"
  xmlns:ns3="http://www.garmin.com/xmlschemas/TrackPointExtension/v1"
  xmlns="http://www.topografix.com/GPX/1/1"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:ns2="http://www.garmin.com/xmlschemas/GpxExtensions/v3">
  <metadata>
    <link href="connect.garmin.com">
      <text>Garmin Connect</text>
    </link>)XML";
} ;

const char * gpx_time(String timestamp) 
{
    String tmp=String("<time>" + timestamp + "Z</time>");
    return tmp.c_str();
};

const char* gpx_intro_end() {return "  </metadata>";}

const char* gpx_trk_start(String trkname)
{
    String  tmp=String("");
    tmp +="  <trk>\n";
    tmp +="    <name>" + trkname+ "</name>\n";
    tmp +="    <type>running</type>\n";
    tmp +="    <trkseg>";
    return tmp.c_str();
}

String gpx_trkpt_extensions(String ATEMP, String HR, String CAD) 
{
    String tmp= String("");

    if (ATEMP+HR+CAD == "") //if all extensions are missing
      ; //return empty 
    else //there is at least 1 extension
    {
      tmp += "      <extensions>\n";
      tmp += "        <ns3:TrackPointExtension>\n";
      if (ATEMP != "")
        tmp += "          <ns3:atemp>"+ATEMP+"</ns3:atemp>\n";
      if (HR != "")
        tmp += "          <ns3:hr>"+HR+"</ns3:hr>\n";
      if ( CAD!="") 
        tmp += "          <ns3:cad>"+CAD+"</ns3:cad>\n";
      tmp += "        </ns3:TrackPointExtension>\n";
      tmp += "      </extensions>\n";
    } //there is at least 1 extension
    return tmp;
}

const char*  gpx_trkpt(String LAT, String LON, String ELE, String TIME, String ATEMP, String HR, String CAD) 
{
    String tmp= String("");
    tmp += "    <trkpt lat=\"" + LAT +"\" lon=\""+LON+"\">\n";
    tmp += "      <ele>"+ELE+"</ele>\n";
    tmp += "      <time>"+TIME+"</time>\n";
    tmp += gpx_trkpt_extensions(ATEMP, HR, CAD);
    tmp += "    </trkpt>";
    return tmp.c_str();
}

const char* gpx_trk_end() 
{
    return R"XML(
    </trkseg>
  </trk>
</gpx>)XML"; 
}
#endif //GPX_H