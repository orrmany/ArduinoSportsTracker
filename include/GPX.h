#ifndef GPX_H
#define GPX_H
#include <Arduino.h>

//C++11 raw string literals, see https://stackoverflow.com/questions/1135841/c-multiline-string-literal
String gpx_intro_start()
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

String  gpx_time(String timestamp) 
{
    String tmp=String("<time>" + timestamp + ".000Z</time>");
    return tmp;
};

String  gpx_intro_end() {return "  </metadata>";}

String  gpx_trk_start(String trkname)
{
    String  tmp=String("");
    tmp +="  <trk>\n";
    tmp +="    <name>" + trkname+ "</name>\n";
    tmp +="    <type>running</type>\n";
    tmp +="    <trkseg>";
    return tmp;
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

String   gpx_trkpt(String LAT, String LON, String ELE, String TIME, String ATEMP, String HR, String CAD) 
{
    String tmp= String("");
    tmp += "    <trkpt lat=\"" + LAT +"\" lon=\""+LON+"\">\n";
    tmp += "      <ele>"+ELE+"</ele>\n";
    tmp += "      <time>"+TIME+"</time>\n";
    tmp += gpx_trkpt_extensions(ATEMP, HR, CAD);
    tmp += "    </trkpt>";
    return tmp;
}

String  gpx_trk_end() 
{
    return R"XML(
    </trkseg>
  </trk>
</gpx>)XML"; 
}
#endif //GPX_H