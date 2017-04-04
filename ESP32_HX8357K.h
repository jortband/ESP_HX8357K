/* Kolbans version of SPI HX8357 setup and stuff
 *
*/
#include <stdint.h>
#include <stdio.h>


#define WIDTH 320
#define HEIGHT 480
#define HX8357D 0xD
#define HX8357B 0xB

#define HX8357_TFTWIDTH  320
#define HX8357_TFTHEIGHT 480

#define HX8357_NOP     0x00
#define HX8357_SWRESET 0x01
#define HX8357_RDDID   0x04
#define HX8357_RDDST   0x09

#define HX8357_RDPOWMODE  0x0A
#define HX8357_RDMADCTL  0x0B
#define HX8357_RDCOLMOD  0x0C
#define HX8357_RDDIM  0x0D
#define HX8357_RDDSDR  0x0F

#define HX8357_SLPIN   0x10
#define HX8357_SLPOUT  0x11
#define HX8357B_PTLON   0x12
#define HX8357B_NORON   0x13

#define HX8357_INVOFF  0x20
#define HX8357_INVON   0x21
#define HX8357_DISPOFF 0x28
#define HX8357_DISPON  0x29

#define HX8357_CASET   0x2A
#define HX8357_PASET   0x2B
#define HX8357_RAMWR   0x2C
#define HX8357_RAMRD   0x2E

#define HX8357B_PTLAR   0x30
#define HX8357_TEON  0x35
#define HX8357_TEARLINE  0x44
#define HX8357_MADCTL  0x36
#define HX8357_COLMOD  0x3A

#define HX8357_SETOSC 0xB0
#define HX8357_SETPWR1 0xB1
#define HX8357B_SETDISPLAY 0xB2
#define HX8357_SETRGB 0xB3
#define HX8357D_SETCOM  0xB6

#define HX8357B_SETDISPMODE  0xB4
#define HX8357D_SETCYC  0xB4
#define HX8357B_SETOTP 0xB7
#define HX8357D_SETC 0xB9

#define HX8357B_SET_PANEL_DRIVING 0xC0
#define HX8357D_SETSTBA 0xC0
#define HX8357B_SETDGC  0xC1
#define HX8357B_SETID  0xC3
#define HX8357B_SETDDB  0xC4
#define HX8357B_SETDISPLAYFRAME 0xC5
#define HX8357B_GAMMASET 0xC8
#define HX8357B_SETCABC  0xC9
#define HX8357_SETPANEL  0xCC


#define HX8357B_SETPOWER 0xD0
#define HX8357B_SETVCOM 0xD1
#define HX8357B_SETPWRNORMAL 0xD2

#define HX8357B_RDID1   0xDA
#define HX8357B_RDID2   0xDB
#define HX8357B_RDID3   0xDC
#define HX8357B_RDID4   0xDD

#define HX8357D_SETGAMMA 0xE0

#define HX8357B_SETGAMMA 0xC8
#define HX8357B_SETPANELRELATED  0xE9



// Color definitions
#define	HX8357_BLACK   0x0000
#define	HX8357_BLUE    0xF800
#define	HX8357_RED     0x001F
#define	HX8357_GREEN   0x07E0
#define HX8357_CYAN    0xFFE0
#define HX8357_MAGENTA 0xF81F
#define HX8357_YELLOW  0x07FF
#define HX8357_WHITE   0xFFFF



#ifndef ESP32_HX8357K_H
#define ESP32_HX8357K_H
#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif

typedef struct {
    const unsigned char *index;
    const unsigned char *unicode;
    const unsigned char *data;
    unsigned char version;
    unsigned char reserved;
    unsigned char index1_first;
    unsigned char index1_last;
    unsigned char index2_first;
    unsigned char index2_last;
    unsigned char bits_index;
    unsigned char bits_width;
    unsigned char bits_height;
    unsigned char bits_xoffset;
    unsigned char bits_yoffset;
    unsigned char bits_delta;
    unsigned char line_space;
    unsigned char cap_height;
} TFT_font_t;



 class ESP32_HX8357K : public Print{
   public:
     ESP32_HX8357K();
     void begin();
     void drawPixel(uint16_t x, uint16_t y, uint16_t color);
     //void drawLine(uint16_t x, uint16_t y, uint16_t xp, uint16_t yp, uint16_t color);
     void drawFastVLine(uint16_t x, uint16_t y, uint16_t l, uint16_t color);
     void drawFastHLine(uint16_t x, uint16_t y, uint16_t l, uint16_t color);
		 void drawLine(int16_t x0, int16_t y0,	int16_t x1, int16_t y1, uint16_t color);
     void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
     void drawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *bmap[]);
		 void drawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *bmap);
     void fillScreen(uint16_t color);
		 void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

     void setRotation(uint8_t m);
		 void invertDisplay(boolean i);
		 void fillRectVGradient(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color1, uint16_t color2);
		 void fillRectHGradient(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color1, uint16_t color2);
		 void fillScreenVGradient(uint16_t color1, uint16_t color2);
		 void fillScreenHGradient(uint16_t color1, uint16_t color2);

     uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

		 // from Adafruit_GFX.h
		void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
		void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
		void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
		void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
		void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
		void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
		void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
		void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
		// void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
		void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
		void setCursor(int16_t x, int16_t y);
	  void getCursor(int16_t *x, int16_t *y);
		void setTextColor(uint16_t c);
		void setTextColor(uint16_t c, uint16_t bg);
		void setTextSize(uint8_t s);
		uint8_t getTextSize();
		void setTextWrap(boolean w);
		boolean getTextWrap();
		virtual size_t write(uint8_t);
		//virtual size_t write(char);
		// size_t write(const uint8_t *buffer, size_t size);
		// size_t write(const char *buffer, size_t size);
		uint8_t getRotation(void);
		int16_t getCursorX(void) const { return cursor_x; }
		int16_t getCursorY(void) const { return cursor_y; }
		void setFont(const TFT_font_t &f) { font = &f; }
		void setFontAdafruit(void) { font = NULL; }
		void drawFontChar(unsigned int c);
		// int16_t strPixelLen(char * str);
		int16_t width(void)  { return _width; }
		int16_t height(void) { return _height; }

		// //FOR PRINTING TAKEN FROM print.h
		// size_t write(const char *str) {
    //   if (str == NULL) return 0;
    //   return write((const uint8_t *)str, strlen(str));
    // }
    // size_t write(const char *buffer, size_t size) {
    //   return write((const uint8_t *)buffer, size);
    // }
		// size_t write(const uint8_t *str) {
    //   if (str == NULL) return 0;
    //   return write((const uint8_t *)str, strlen(str));
    // }
    // size_t write(const uint8_t *buffer, size_t size) {
    //   return write((const uint8_t *)buffer, size);
    // }
		//
		//
		// size_t print(const __FlashStringHelper *);
		// size_t print(const String &);
		// size_t print(const char[]);
		// size_t print(char);
		// size_t print(unsigned char, int = DEC);
		// size_t print(int, int = DEC);
		// size_t print(unsigned int, int = DEC);
		// size_t print(long, int = DEC);
		// size_t print(unsigned long, int = DEC);
		// size_t print(double, int = 2);
		// size_t print(const Printable&);
		//
		// size_t println(const __FlashStringHelper *);
		// size_t println(const String &s);
		// size_t println(const char[]);
		// size_t println(char);
		// size_t println(unsigned char, int = DEC);
		// size_t println(int, int = DEC);
		// size_t println(unsigned int, int = DEC);
		// size_t println(long, int = DEC);
		// size_t println(unsigned long, int = DEC);
		// size_t println(double, int = 2);
		// size_t println(const Printable&);
		// size_t println(void);

   private:
     void initLCD();
     void initSPI();
     void initGPIO();
     void setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
		 void writeData(uint32_t *data, uint8_t length);
     void writeCommand(uint8_t command);
     void writeByte(uint8_t command);
		 void drawFontBits(uint32_t bits, uint32_t numbits, uint32_t x, uint32_t y, uint32_t repeat);
		 uint16_t colorR(uint16_t color);
     uint16_t colorG(uint16_t color);
     uint16_t colorB(uint16_t color);
     uint16_t _width;
     uint16_t _height;
     spi_t *spi;
     uint32_t to32(uint16_t color);
     byte rotation  = 0;
     uint16_t cursor_y,cursor_x;
     uint16_t textsize;
     uint16_t textcolor, textbgcolor;
     boolean wrap;
     const TFT_font_t *font;
		 uint16_t spiTransferShortNL(spi_t * spi, uint16_t data);
		 void spiWriteShortNL(spi_t * spi, uint16_t data);
		 void setAddrNL(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
		 void spiSimpleTransaction(spi_t * spi);
		 void spiEndTransaction(spi_t * spi);
		 void drawHLine(uint16_t x, uint16_t y, int16_t l, uint16_t color);
		 void drawVLine(uint16_t x, uint16_t y, int16_t l, uint16_t color);
		 void drawPixelNL(uint16_t x, uint16_t y, uint16_t color);

 };
#ifndef swap
	#define swap(a, b) { typeof(a) t = a; a = b; b = t; }
#endif

#endif
