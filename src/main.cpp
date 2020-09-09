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

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
#include "Adafruit_miniTFTWing.h"

//seesaw 
Adafruit_miniTFTWing ss;
#define TFT_RST  -1 // we use the seesaw for resetting to save a pin
#define TFT_CS   5 //Fix for miniTFT
#define TFT_DC   6 //Fix for miniTFT
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

void setup()   {
  Serial.begin(115200);

  /*
  while (!Serial)  delay(10);  // Wait until serial console is opened
  */
}

void loop() {
  }
