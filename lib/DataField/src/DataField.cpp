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
#include "DataField.hh"

void PrintField::setContentPadRight(String p_content, const char p_padchar) {
        if (p_content.length() > columns)
            p_content.remove(columns);
        content=p_content;
        //pad content
        for (uint8_t i = content.length(); i < columns; i++)
        {
            content += p_padchar;
        }
    }
void PrintField::setContentPadLeft(String p_content, const char p_padchar) {
        if (p_content.length() > columns)
            p_content.remove(columns);
        content = "";
        //pad content
        for (uint8_t i = 0; i < columns - p_content.length();  i++)
        {
            content += p_padchar;
        }
        content +=p_content;
    }

void PrintField::display(void)
{
    _prt->setTextSize(xtextSize, ytextSize);
    _prt->setCursor(cursorX, cursorY);
    _prt->setTextColor(fgColor, bgColor);
    _prt->print(content);
}



void DataField::reprint(void) //reprints the entire datafield
        {
            fieldDescription.display();
            fieldContent.display(); 
            if (hasFrame) drawFrame();
        }
void DataField::refreshContent(void) //reprints only the content
        {
            fieldContent.display(); 
            if (hasFrame) drawFrame();
        }

void DataField::invertColors(void)
    { 
        fieldDescription.invertColor(); 
        fieldContent.invertColor();
        if (hasFrame) drawFrame();
    }


TwoLineDataField::TwoLineDataField(
    String p_desc,                  //< the description of the data field
    String p_field,                 //< the initial content of the datafield.
                                    //  Note: it will determine the width of the field
    TwoLineFieldDescriptor p_dispField, //< the type of the dataField
    Adafruit_GFX *p_disp,            //pointer to display
        bool p_hasFrame
        ) 
{
    uint16_t origoX= p_dispField.displayCol * GFX_FONT_X * p_dispField.xFieldScale;
    uint16_t origoY= p_dispField.displayRow * (p_dispField.yDescScale + p_dispField.yFieldScale) * GFX_FONT_Y;
    if (p_desc.length()*p_dispField.xDescScale > p_field.length()*p_dispField.xFieldScale) p_desc.remove(p_field.length()*p_dispField.xFieldScale/p_dispField.xDescScale);

    fieldDescription = PrintField(origoX //cursorX
                                , origoY //cursorY
                                , p_field.length() * p_dispField.xFieldScale  /  p_dispField.xDescScale //columns are 1/3 for desc than that of data
                                , p_dispField.xDescScale //textsize
                                , p_dispField.yDescScale
                                , p_dispField.descriptionColor //fgColor, almost black on TFT, black on LCD
                                , p_dispField.bgColor //bgColor
                                , p_disp //the display
                                , p_desc //description
                                );
    fieldContent = PrintField(origoX //cursorX
                                , origoY + p_dispField.yDescScale * GFX_FONT_Y //cursorY
                                , p_field.length()  //columns are 1/3 for desc than that of data
                                , p_dispField.xFieldScale //textsize
                                , p_dispField.yFieldScale
                                , p_dispField.dataColor //fgColor
                                , p_dispField.bgColor //bgColor
                                , p_disp //the display
                                , p_field //field content
                                );
    hasFrame = p_hasFrame;
    _disp = p_disp;
}

void TwoLineDataField::drawFrame()
    {
        uint16_t
            //lower left corner 1px up
            xLL = getOrigoX(),
            yLL = getOrigoY() + getHeightInPx() - 2,
            //lower right corner 1px left 1px up
            xLR = getOrigoX() + getWidthInPx() - 2,
            yLR = getOrigoY() + getHeightInPx() - 2,
            //upper right corner 1px left
            xUR = getOrigoX() + getWidthInPx() - 2,
            yUR = getOrigoY();
        _disp->drawLine(xLL, yLL, xLR, yLR, fieldDescription.getFgColor());
        _disp->drawLine(xUR, yUR, xLR, yLR, fieldDescription.getFgColor());
    }

    //draw frame only at bottom & right 
void OneLineDataField::drawFrame()
    {
        uint16_t
            //lower left corner 1px up
            xLL = getOrigoX(),
            yLL = getOrigoY() + getHeightInPx() - 1,
            //lower right corner 1px left 1px up
            xLR = getOrigoX() + getWidthInPx() - 1,
            yLR = getOrigoY() + getHeightInPx() - 1,
            //upper right corner 1px left
            xUR = getOrigoX() + getWidthInPx() - 1,
            yUR = getOrigoY();
        _disp->drawLine(xLL, yLL, xLR, yLR, fieldDescription.getFgColor());
        _disp->drawLine(xUR, yUR, xLR, yLR, fieldDescription.getFgColor());
    }



OneLineDataField::OneLineDataField(
    String p_desc,                  //< the description of the data field
    String p_field,                 //< the initial content of the datafield.
                                    //  Note: it will determine the height of the field
    OneLineFieldDescriptor p_dispField, //< the type of the dataField
    Adafruit_GFX *p_disp,            //pointer to display
        bool p_hasFrame
        )  
{
    uint16_t origoX= p_dispField.displayCol * GFX_FONT_X * p_dispField.xScale;
    uint16_t origoY= p_dispField.displayRow * p_dispField.yFieldScale * GFX_FONT_Y;
    if (p_dispField.yDescScale> p_dispField.yFieldScale) p_dispField.yDescScale = p_dispField.yFieldScale;
    
    fieldDescription = PrintField(origoX //cursorX
                                , origoY //cursorY
                                , p_desc.length()  //columns 
                                , p_dispField.xScale //textsize
                                , p_dispField.yDescScale
                                , p_dispField.descriptionColor //fgColor, almost black on TFT, black on LCD
                                , p_dispField.bgColor //bgColor
                                , p_disp //the display
                                , p_desc //description
                                );
    fieldContent = PrintField(origoX + p_desc.length() * GFX_FONT_X * p_dispField.xScale  //cursorX
                                , origoY  //cursorY
                                , p_field.length()  //columns are 1/3 for desc than that of data
                                , p_dispField.xScale //textsize
                                , p_dispField.yFieldScale
                                , p_dispField.dataColor //fgColor
                                , p_dispField.bgColor //bgColor
                                , p_disp //the display
                                , p_field //field content
                                );
    
    hasFrame = p_hasFrame;
    _disp = p_disp;
}


Combo5DataField::Combo5DataField(
        String p_desc,                  //< the description of the data field. Upper left
        String p_sign,                  //< a lower left field, e.g., for sign.  The longer of [desc] and [sign] will set width for both  
        String p_field,                 //< the initial content of the datafield.
        String p_unit,                  //< the unit field of the data field. Upper right 
        String p_fract,                  //< a lower right field, e.g., for fract  The longer of [sign] and [fract] will set width for both  
        
        Combo5FieldDescriptor p_dispField, //< the type of the dataField
        Adafruit_GFX *p_disp,            //pointer to display
        bool p_hasFrame 
    )
{
    if (p_dispField.yScale2< p_dispField.yScale*2) p_dispField.yScale2 =p_dispField.yScale*2;
    uint16_t origoX= p_dispField.displayCol * GFX_FONT_X * p_dispField.xScale2;
    uint16_t origoY= p_dispField.displayRow * GFX_FONT_Y * p_dispField.yScale2;

    int diff = (int)p_sign.length() - (int)p_desc.length();
    //if [sign] is longer than [desc] then pad [desc] from right
    for (int i = 0; i <  diff; i++) p_desc += " " ;
    //if [sign] is shorter than [desc] then pad [sign] from left
    for (int i = 0; i < -diff ; i++) p_sign = " "+p_sign ;
    
    diff = (int)p_unit.length() - (int)p_fract.length();
    for (int i = 0; i <  diff; i++) p_fract += " " ;
    for (int i = 0; i < -diff ; i++) p_unit += " " ;
    
    
        
    fieldDescription = PrintField(origoX //cursorX
                                , origoY //cursorY
                                , p_desc.length()  //columns 
                                , p_dispField.xScale //textsize
                                , p_dispField.yScale
                                , p_dispField.descriptionColor //fgColor, almost black on TFT, black on LCD
                                , p_dispField.bgColor //bgColor
                                , p_disp //the display
                                , p_desc //description
                                );
    fieldSign = PrintField(origoX //cursorX
                                , origoY + GFX_FONT_Y * (p_dispField.yScale2-p_dispField.yScale) //cursorY
                                , p_sign.length()  //columns 
                                , p_dispField.xScale //textsize
                                , p_dispField.yScale
                                , p_dispField.dataColor //fgColor, almost black on TFT, black on LCD
                                , p_dispField.bgColor //bgColor
                                , p_disp //the display
                                , p_sign //description
                                );
    fieldContent = PrintField(origoX + p_desc.length() * GFX_FONT_X * p_dispField.xScale  //cursorX
                                , origoY  //cursorY
                                , p_field.length()  //columns are 1/3 for desc than that of data
                                , p_dispField.xScale2 //textsize
                                , p_dispField.yScale2
                                , p_dispField.dataColor //fgColor
                                , p_dispField.bgColor //bgColor
                                , p_disp //the display
                                , p_field //field content
                                );
    fieldUnit = PrintField(origoX + GFX_FONT_X * (p_desc.length() * p_dispField.xScale  + p_field.length() *  p_dispField.xScale2) //cursorX
                                , origoY  //cursorY
                                , p_unit.length()  //columns
                                , p_dispField.xScale //textsize
                                , p_dispField.yScale
                                , p_dispField.dataColor //fgColor
                                , p_dispField.bgColor //bgColor
                                , p_disp //the display
                                , p_unit //field content
                                );
    fieldFract = PrintField(origoX +  GFX_FONT_X * (p_desc.length() * p_dispField.xScale  + p_field.length() *  p_dispField.xScale2)  //cursorX
                                , origoY + GFX_FONT_Y * (p_dispField.yScale2-p_dispField.yScale)  //cursorY
                                , p_unit.length()  //columns
                                , p_dispField.xScale //textsize
                                , p_dispField.yScale
                                , p_dispField.dataColor //fgColor
                                , p_dispField.bgColor //bgColor
                                , p_disp //the display
                                , p_fract //field content
                                );
    hasFrame = p_hasFrame;
    _disp = p_disp;
}

    //draw frame only at bottom & right 
void Combo5DataField::drawFrame()
    {
        uint16_t
            //lower left corner 1px up
            xLL = getOrigoX(),
            yLL = getOrigoY() + getHeightInPx() - 1,
            //lower right corner 1px left 1px up
            xLR = getOrigoX() + getWidthInPx() - 1,
            yLR = getOrigoY() + getHeightInPx() - 1,
            //upper right corner 1px left
            xUR = getOrigoX() + getWidthInPx() - 1,
            yUR = getOrigoY();
        _disp->drawLine(xLL, yLL, xLR, yLR, fieldDescription.getFgColor());
        _disp->drawLine(xUR, yUR, xLR, yLR, fieldDescription.getFgColor());
    }

void Combo5DataField ::reprint(void) //reprints the entire datafield
        {
            fieldDescription.display();
            fieldContent.display(); 
            fieldSign.display();
            fieldUnit.display();
            fieldFract.display();
            if (hasFrame) drawFrame();
        }
void Combo5DataField::refreshContent(void) //reprints only the content w. sign, unit and fractionals
        {
            fieldContent.display();
            fieldSign.display(); 
            fieldFract.display();
            fieldUnit.display();
            if (hasFrame) drawFrame();
        }

void Combo5DataField::invertColors(void)
    { 
        fieldDescription.invertColor(); 
        fieldContent.invertColor();
        fieldSign.invertColor();
        fieldUnit.invertColor();
        fieldFract.invertColor();
        if (hasFrame) drawFrame();
    }
