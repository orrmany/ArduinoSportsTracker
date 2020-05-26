/*
MIT License

Copyright (c) 2020 Gábor Ziegler 

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
/*
** Classes for handling various Adafruit_GFX compatible displays. Currently,
** the Adafruit miniTFT with joystick Header Wing and  the Nokia 5110 LCD-s
*/
#ifndef DATAFIELD_H
#define DATAFIELD_H
#include <stdint.h>
#include <Arduino.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_PCD8544.h>
#include <Adafruit_GFX.h>

#define GFX_FONT_X 6
#define GFX_FONT_Y 8

/// Class to represent a generic field printable via Adafruit_GFX
/// to some display.
class PrintField
{
public:
    PrintField(
        uint16_t p_cursorX,   //< Upper left corner X
        uint16_t p_cursorY,   //< Upper left corner Y
        uint8_t p_columns,    //< Noof. char columns
        uint8_t p_xtextSize,   //< text size (pixel mulitplier)
        uint8_t p_ytextSize,   //< text size (pixel mulitplier)
        uint16_t p_fgColor,   //< Foreground color for text ('565' packed encoding)
        uint16_t p_bgColor,   //< Background color for text ('565' packed encoding)
        Adafruit_GFX *ptr,    //< Print object to print to
        String p_content = "" //< initial content
        ) : cursorX(p_cursorX),
            cursorY(p_cursorY),
            xtextSize(p_xtextSize),
            ytextSize(p_ytextSize),
            fgColor(p_fgColor),
            bgColor(p_bgColor),
            columns(p_columns),
               _prt(ptr)
    {
        if (p_content.length() > columns)
            p_content.remove(columns);
        setContentLeft(p_content);
    }
    uint16_t getOrigoX(void) { return cursorX; }
    uint16_t getOrigoY(void) { return cursorY; }
    uint16_t getWidthInPx(void) { return columns * xtextSize * GFX_FONT_X; }
    uint16_t getHeightInPx(void) { return ytextSize * GFX_FONT_Y; }
    PrintField() : cursorX(0),
            cursorY(0),
            xtextSize(0),
            ytextSize(0),
            fgColor(0),
            bgColor(0),
            columns(0),
            content(0),
            _prt(NULL) {}
    void display(void);
    void setContentLeft(String p_content) {setContentPadRight(p_content, ' ');}
    void setContentRight(String p_content) {setContentPadLeft(p_content, ' ');}
    void setContentPadRight(String p_content, const char p_padchar=' ');
    void setContentPadLeft(String p_content, const char p_padchar=' ') ;
    void invertColor(void) {
        uint16_t tmp = fgColor;
        fgColor = bgColor;
        bgColor = tmp;
        display();
    }
    uint16_t getFgColor(void) {return fgColor;}

protected:
    uint16_t cursorX; ///< the X coord. for the graphical cursor to set at
    uint16_t cursorY; ///< the X coord. for the graphical cursor to set at
    uint8_t xtextSize; ///< the text size to use for printing. 
    uint8_t ytextSize; ///< the text size to use for printing. 
                            //Adafruit GFX
                            // uses 6x8 pixel default font, this param multiples
                            // pixel size, i.e, size=2 means 12x16 pixel font
    uint16_t fgColor;       ///< The foreground text color to use, in '565' bits
                            // format of packed 16bit RGB data
    uint16_t bgColor;       ///< The backgroumd text color to use, in '565' bits
                            // format of packed 16bit RGB data
    uint16_t columns; //< the number of columns
    String content;         //< content
    Adafruit_GFX* _prt;     //< the Print object
};


/// Struct for attributes of a two lines data field. Bakcground color and
/// is uniform across the field, but desc. and data subfields stretch independently
/// Beware, that height is assumed determined by the data sub-field, so do not
/// set bigger vertical stretching for desc. than for the data
typedef struct
{
    uint16_t dataColor;        //< txt. color of the data field
    uint16_t descriptionColor; //< txt. color for desc. field
    uint16_t bgColor;          //< shared bg. color of both fields
    uint8_t displayRow = 0;    //< display row# assuming identical fields everywhere on display
    uint8_t displayCol = 0;    //< dislay column# assuming identical fields everywhere on display
    uint8_t xFieldScale = 3;   //< horizontal pixel stretching factor for data (ADafruit_GFX default font is 6x8 px) 
    uint8_t yFieldScale = 3;   //< vertical pixel stretching factor for data
    uint8_t xDescScale = 1;    //< vertical pixel stretching factor for description
    uint8_t yDescScale = 1;    //< horizontal pixel stretching factor for description
} TwoLineFieldDescriptor;

/// Struct for attributes of a one line data field. Bakcground color and
/// horizontal px stretching is uniform across the field 
typedef struct
{
    uint16_t dataColor;        //< txt. color of the data field
    uint16_t descriptionColor; //< txt. color for desc. field
    uint16_t bgColor;          //< shared bg. color of both fields
    uint8_t displayRow = 0;    //< display row# assuming identical fields everywhere on display
    uint8_t displayCol = 0;    //< dislay column# assuming identical fields everywhere on display
    uint8_t xScale = 2;        //< horizontal pixel stretching factor (ADafruit_GFX default font is 6x8 px) 
    uint8_t yFieldScale = 2;   //< vertical pixel stretching factor for the data field part 
    uint8_t yDescScale = 1;    //< vertical pixel stretching factor for the desc.  field part 
} OneLineFieldDescriptor;

/// Abstract base class for DataFields
class DataField
{
protected:
    PrintField fieldDescription;
    PrintField fieldContent;
    bool hasFrame;
    Adafruit_GFX* _disp;

public:
     uint16_t getOrigoX(void) { return fieldDescription.getOrigoX(); }
     uint16_t getOrigoY(void) { return fieldDescription.getOrigoY(); }
     virtual uint16_t getWidthInPx(void) =0; 
     virtual uint16_t getHeightInPx(void) =0; 
    //draw frame only at bottom & right 
    virtual void drawFrame() =0;
    void reprint(void); //reprints the entire datafield
    virtual void refreshContent(void); //reprints only the content
    virtual void updateContentLeft(String p_content, const char p_padding = ' ') {setContentLeft(p_content, p_padding); refreshContent();}
    virtual void updateContentRight(String p_content, const char p_padding = ' ') {setContentRight(p_content, p_padding); refreshContent();}
    virtual void setContentLeft(String p_content, const char p_padding = ' ') {fieldContent.setContentPadRight(p_content, p_padding);}
    virtual void setContentRight(String p_content, const char p_padding = ' ') {fieldContent.setContentPadLeft(p_content,  p_padding);}
    virtual void setDescriptionLeft(String p_content, const char p_padding = ' ') {fieldDescription.setContentPadRight(p_content, p_padding);}
    virtual void setDescriptionRight(String p_content, const char p_padding = ' ') {fieldDescription.setContentPadLeft(p_content, p_padding);}
    virtual void invertColors(void); 

    //~DataField();
};

/// Two lines data field: description field comes above the datafield.
/// Description field is usually 1/2, or 1/3 tall compared to datafield. 
/// Height is the heights of the two fields summed up. Width is determined
/// by the data field. Too long desc. labels are truncated sliently.
class TwoLineDataField : public DataField
{
public:
    TwoLineDataField(
        String p_desc,                  //< the description of the data field
        String p_field,                 //< the initial content of the datafield.
                                        //  Note: it will determine the width of the field
        TwoLineFieldDescriptor p_dispField, //< the type of the dataField
        Adafruit_GFX *p_disp,            //pointer to display
        bool p_HasFrame =false
    );
    //draw frame only at bottom & right 
    virtual void drawFrame();
    virtual uint16_t getWidthInPx(void) { return fieldContent.getWidthInPx(); }
    virtual uint16_t getHeightInPx(void) { return fieldContent.getHeightInPx() + fieldDescription.getHeightInPx(); }
    
    //~TwoLineDataField();
};


/// One line data field: description field comes in front of the datafield
/// Description field is same height compared to datafield, but usually 
/// much shorter. Width=description+data, height is determined by the datafield.
class OneLineDataField : public DataField
{
public:
    OneLineDataField(
        String p_desc,                  //< the description of the data field
        String p_field,                 //< the initial content of the datafield.
                                        //  Note: it will determine the width of the field
        OneLineFieldDescriptor p_dispField, //< the type of the dataField
        Adafruit_GFX *p_disp,            //pointer to display
        bool p_HasFrame =false
    );
    OneLineDataField() {}
    //draw frame only at bottom & right 
    virtual void drawFrame();
    virtual uint16_t getWidthInPx(void) { return fieldContent.getWidthInPx() + fieldDescription.getWidthInPx(); }
    virtual uint16_t getHeightInPx(void) { return fieldContent.getHeightInPx() ; }

    //~OneLineDataField();
};

/// Five element DataField combo:
/// [desc]+--+[unit]
/// [sign]+--+[frct]
/// xStretch is the same. Data field has double yStretch of small fields
/// Struct for attributes of a one line data field. Bakcground color and
/// horizontal px stretching is uniform across the field.
/// Small subfields assumed 1 char wide
typedef struct
{
    uint16_t dataColor;        //< txt. color of the data field
    uint16_t descriptionColor; //< txt. color for desc. field
    uint16_t bgColor;          //< shared bg. color of both fields
    uint8_t displayRow = 0;    //< display row# assuming identical fields everywhere on display
    uint8_t displayCol = 0;    //< dislay column# assuming identical fields everywhere on display
    uint8_t xScale = 2;        //< horizontal pixel stretching factor (ADafruit_GFX default font is 6x8 px) 
    uint8_t yScale = 2;        //< vertical base pixel stretching factor for the left and rigth small field stretch
    uint8_t xScale2 = 2;        //< horizontal pixel stretching factor (ADafruit_GFX default font is 6x8 px) 
    uint8_t yScale2 = 3;       //< Height multiply factor for the  DataField in the middle 
} Combo5FieldDescriptor;

class Combo5DataField : public DataField 
{
protected:
    PrintField fieldSign;
    PrintField fieldUnit;
    PrintField fieldFract;
public:
    Combo5DataField(
        String p_desc,                  //< the description of the data field. Upper left
        String p_sign,                  //< a lower left field, e.g., for sign.  The longer of [desc] and [sign] will set width for both  
        String p_field,                 //< the initial content of the datafield.
        String p_unit,                  //< the unit field of the data field. Upper right 
        String p_fract,                  //< a lower right field, e.g., for fract  The longer of [sign] and [fract] will set width for both  
        
        Combo5FieldDescriptor p_dispField, //< the type of the dataField
        Adafruit_GFX *p_disp,            //pointer to display
        bool p_HasFrame =false
    );
    uint16_t getWidthInPx(void) { return fieldContent.getWidthInPx() + fieldDescription.getWidthInPx() + fieldFract.getWidthInPx(); }
    uint16_t getHeightInPx(void) { return fieldContent.getHeightInPx() ; }
    //draw frame only at bottom & right 
    void drawFrame();
    void reprint(void); //reprints the entire datafield
    void refreshContent(void); //reprints only the content + fractional
    void invertColors(void); 
    void setSignLeft(String p_content, const char p_padding = ' ') {fieldSign.setContentPadRight(p_content, p_padding);}
    void setSignRight(String p_content, const char p_padding = ' ') {fieldSign.setContentPadLeft(p_content,  p_padding);}
    void setUnitLeft(String p_content, const char p_padding = ' ') {fieldUnit.setContentPadRight(p_content, p_padding);}
    void setUnitRight(String p_content, const char p_padding = ' ') {fieldUnit.setContentPadLeft(p_content, p_padding);}
    void setFractLeft(String p_content, const char p_padding = ' ') {fieldFract.setContentPadRight(p_content, p_padding);}
    void setFractRight(String p_content, const char p_padding = ' ') {fieldFract.setContentPadLeft(p_content, p_padding);}
    
};

#endif //DATAFIELD_H