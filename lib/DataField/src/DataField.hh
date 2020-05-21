#ifndef DATAFIELD_H
#define DATAFIELD_H
#include <stdint.h>
#include <Arduino.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_PCD8544.h>
#include <Adafruit_GFX.h>

#define GFX_FONT_X 6
#define GFX_FONT_Y 8

// Class to represent a field printable via Adafruit_GFX
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
    void setContentPadRight(String p_content, const char p_padchar=' ') {
        if (p_content.length() > columns)
            p_content.remove(columns);
        content=p_content;
        //pad content
        for (uint8_t i = content.length(); i < columns; i++)
        {
            content += p_padchar;
        }
    }
    void setContentPadLeft(String p_content, const char p_padchar=' ') {
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
void PrintField::display(void)
{
    _prt->setTextSize(xtextSize, ytextSize);
    _prt->setCursor(cursorX, cursorY);
    _prt->setTextColor(fgColor, bgColor);
    _prt->print(content);
}


typedef struct
{
    uint16_t dataColor ;     //< fg. color. 
    uint16_t descriptionColor ;     //< fg. color. 
    uint16_t bgColor;
    uint8_t displayRow = 0;     //< display row: 1, or 2
    uint8_t displayCol = 0; //< dislay col, default 0;
    uint8_t xFieldScale =3;  //< only for 
    uint8_t yFieldScale = 3;  //< only for 
    uint8_t xDescScale = 1;  //< only for 
    uint8_t yDescScale = 1;  //< only for 
} TwoLineFieldDescriptor;
typedef struct
{
    uint16_t dataColor ;     //< fg. color. 
    uint16_t descriptionColor ;     //< fg. color. 
    uint16_t bgColor;
    uint8_t displayRow = 0;     //< display row: 1, or 2
    uint8_t displayCol = 0; //< dislay col, default 0;
    uint8_t xScale =2;  //< only for 
    uint8_t yFieldScale = 2;  //< only for 
    uint8_t yDescScale = 1;  //< only for 
} OneLineFieldDescriptor;


class TwoLineDataField
{
private:
    PrintField fieldDescription;
    PrintField fieldContent;
    bool hasFrame;
    Adafruit_GFX* _disp;

public:
    TwoLineDataField(
        String p_desc,                  //< the description of the data field
        String p_field,                 //< the initial content of the datafield.
                                        //  Note: it will determine the width of the field
        TwoLineFieldDescriptor p_dispField, //< the type of the dataField
        Adafruit_GFX *p_disp,            //pointer to display
        bool p_HasFrame =false
    );
    uint16_t getOrigoX(void) { return fieldDescription.getOrigoX(); }
    uint16_t getOrigoY(void) { return fieldDescription.getOrigoY(); }
    uint16_t getWidthInPx(void) { return fieldContent.getWidthInPx(); }
    uint16_t getHeightInPx(void) { return fieldContent.getHeightInPx() + fieldDescription.getHeightInPx(); }
    //draw frame only at bottom & right 
     void drawFrame()
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

  void reprint(void) //reprints the entire datafield
        {
            fieldDescription.display();
            fieldContent.display(); 
            if (hasFrame) drawFrame();
        }
    void refreshContent(void) //reprints only the content
        {
            fieldContent.display(); 
            if (hasFrame) drawFrame();
        }
    void setContentLeft(String p_content, const char p_padding = ' ') {fieldContent.setContentPadRight(p_content, p_padding);}
    void setContentRight(String p_content, const char p_padding = ' ') {fieldContent.setContentPadLeft(p_content,  p_padding);}
    void setDescriptionLeft(String p_content, const char p_padding = ' ') {fieldDescription.setContentPadRight(p_content, p_padding);}
    void setDescriptionRight(String p_content, const char p_padding = ' ') {fieldDescription.setContentPadLeft(p_content, p_padding);}

    //~TwoLineDataField();
};


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

class OneLineDataField
{
private:
    PrintField fieldDescription;
    PrintField fieldContent;
    bool hasFrame;
    Adafruit_GFX* _disp;

public:
    OneLineDataField(
        String p_desc,                  //< the description of the data field
        String p_field,                 //< the initial content of the datafield.
                                        //  Note: it will determine the width of the field
        OneLineFieldDescriptor p_dispField, //< the type of the dataField
        Adafruit_GFX *p_disp,            //pointer to display
        bool p_HasFrame =false
    );
    uint16_t getOrigoX(void) { return fieldDescription.getOrigoX(); }
    uint16_t getOrigoY(void) { return fieldDescription.getOrigoY(); }
    uint16_t getWidthInPx(void) { return fieldContent.getWidthInPx() + fieldDescription.getWidthInPx(); }
    uint16_t getHeightInPx(void) { return fieldContent.getHeightInPx(); }
    //draw frame only at bottom & right 
     void drawFrame()
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

  void reprint(void) //reprints the entire datafield
        {
            fieldDescription.display();
            fieldContent.display(); 
            if (hasFrame) drawFrame();
        }
    void refreshContent(void) //reprints only the content
        {
            fieldContent.display(); 
            if (hasFrame) drawFrame();
        }
    void setContentLeft(String p_content, const char p_padding = ' ') {fieldContent.setContentPadRight(p_content, p_padding);}
    void setContentRight(String p_content, const char p_padding = ' ') {fieldContent.setContentPadLeft(p_content,  p_padding);}
    void setDescriptionLeft(String p_content, const char p_padding = ' ') {fieldDescription.setContentPadRight(p_content, p_padding);}
    void setDescriptionRight(String p_content, const char p_padding = ' ') {fieldDescription.setContentPadLeft(p_content, p_padding);}

    //~OneLineDataField();
};


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


#endif //DATAFIELD_H