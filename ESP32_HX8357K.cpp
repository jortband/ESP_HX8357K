//Library for ESP32 fast SPI
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "esp32-hal-spi.h"
#include "esp32-hal.h"
//#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "Arduino.h"
#include "SPI.h"
#include "ESP32_HX8357K.h"

#define LCD_MISO 19
#define LCD_MOSI 23
#define LCD_CLK  18
#define LCD_CS   4
//TODO: CHECK IF CS IS WORKING
#define LCD_DC   17
#define LCD_RST  5
#define LCD_BCKL 25

#define SPI_MUTEX_LOCK()    do {} while (xSemaphoreTake(spi->lock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK()  xSemaphoreGive(spi->lock)
struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};

#define MSB_32_SET(var, val) { uint8_t * d = (uint8_t *)&(val); (var) = d[3] | (d[2] << 8) | (d[1] << 16) | (d[0] << 24); }
#define MSB_24_SET(var, val) { uint8_t * d = (uint8_t *)&(val); (var) = d[2] | (d[1] << 8) | (d[0] << 16); }
#define MSB_16_SET(var, val) { (var) = (((val) & 0xFF00) >> 8) | (((val) & 0xFF) << 8); }


ESP32_HX8357K::ESP32_HX8357K(){
  //do nothing here for now
  _width    = WIDTH;
	_height   = HEIGHT;
	rotation  = 0;
	cursor_y  = cursor_x    = 0;
	textsize  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;

}


void ESP32_HX8357K::begin(){
  //do initialisation here
  initGPIO();
  initSPI();
  initLCD();
  invertDisplay(false);
  //setRotation(0);
}

void ESP32_HX8357K::initGPIO(){
  pinMode(LCD_MISO,OUTPUT);
  pinMode(LCD_MOSI,OUTPUT);
  pinMode(LCD_CLK,OUTPUT);
  pinMode(LCD_CS,OUTPUT);
  pinMode(LCD_DC,OUTPUT);
  pinMode(LCD_RST,OUTPUT);
  digitalWrite(LCD_CS,LOW);
}

void ESP32_HX8357K::initSPI(){
  spi = spiStartBus(VSPI, 1000000, SPI_MODE0, SPI_MSBFIRST);
  spiAttachSCK(spi, LCD_CLK);
  spiAttachMISO(spi, LCD_MISO);
  spiAttachMOSI(spi, LCD_MOSI);
  spiAttachSS(spi, 0, LCD_CS);//if you want hardware SS
  spiEnableSSPins(spi, 1 << 0);//activate SS for CS0
  spiSSEnable(spi);
  spiSetClockDiv(spi, spiFrequencyToClockDiv(40000000));


}


void ESP32_HX8357K::initLCD(){
  //code for initialisation of LCD
  writeCommand(HX8357B_SETPOWER);
  writeByte(0x44);
  writeByte(0x41);
  writeByte(0x06);
  // seq_vcom
  writeCommand(HX8357B_SETVCOM);
  writeByte(0x40);
  writeByte(0x10);
  // seq_power_normal
  writeCommand(HX8357B_SETPWRNORMAL);
  writeByte(0x05);
  writeByte(0x12);
  // seq_panel_driving
  writeCommand(HX8357B_SET_PANEL_DRIVING);
  writeByte(0x14);
  writeByte(0x3b);
  writeByte(0x00);
  writeByte(0x02);
  writeByte(0x11);
  // seq_display_frame
  writeCommand(HX8357B_SETDISPLAYFRAME);
  writeByte(0x0c);  // 6.8mhz
  // seq_panel_related
  writeCommand(HX8357B_SETPANELRELATED);
  writeByte(0x01);  // BGR
  // seq_undefined1
  writeCommand(0xEA);
  writeByte(0x03);
  writeByte(0x00);
  writeByte(0x00);
  // undef2
  writeCommand(0xEB);
  writeByte(0x40);
  writeByte(0x54);
  writeByte(0x26);
  writeByte(0xdb);
  // seq_gamma
  writeCommand(HX8357B_SETGAMMA); // 0xC8
  writeByte(0x00);
  writeByte(0x15);
  writeByte(0x00);
  writeByte(0x22);
  writeByte(0x00);
  writeByte(0x08);
  writeByte(0x77);
  writeByte(0x26);
  writeByte(0x66);
  writeByte(0x22);
  writeByte(0x04);
  writeByte(0x00);

  // seq_addr mode
  writeCommand(HX8357_MADCTL);
  writeByte(0xC0);
  // pixel format
  writeCommand(HX8357_COLMOD);
  writeByte(0x55);

  // set up whole address box
  // paddr
  writeCommand(HX8357_PASET);
  writeByte(0x00);
  writeByte(0x00);
  writeByte(0x01);
  writeByte(0xDF);
  // caddr
  writeCommand(HX8357_CASET);
  writeByte(0x00);
  writeByte(0x00);
  writeByte(0x01);
  writeByte(0x3F);

  // display mode
  writeCommand(HX8357B_SETDISPMODE);
  writeByte(0x00); // CPU (DBI) and internal oscillation ??
  // exit sleep
  writeCommand(HX8357_SLPOUT);

  delay(120);
  // main screen turn on
  writeCommand(HX8357_DISPON);
  delay(10);

}

void ESP32_HX8357K::writeCommand(uint8_t command){
  spiWaitReady(spi);
  digitalWrite(LCD_DC, LOW);
  digitalWrite(LCD_CS, LOW);
  spiWriteByte(spi, command);
  //spiWrite(spi, uint32_t *data, uint8_t len);
  spiWaitReady(spi);
  digitalWrite(LCD_DC, HIGH);
}

void ESP32_HX8357K::writeData(uint32_t *data, uint8_t length){
  spiWrite(spi, data, length);
}

void ESP32_HX8357K::writeByte(uint8_t command){
  spiWriteByte(spi,command);
}

void ESP32_HX8357K::setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){

  writeCommand(HX8357_CASET); // Column addr set
  //TODO: package data and send in one go

  uint32_t xAddr = x0<<16;
  xAddr |= x1;
  spiWriteLong(spi, xAddr);
  //writeData(uint32_t *data, uint8_t length)
  // spiWriteByte(spi, x0 >> 8);
  // spiWriteByte(spi, x0 & 0xFF);     // XSTART
  // spiWriteByte(spi, x1 >> 8);
  // spiWriteByte(spi, x1 & 0xFF);     // XEND

  writeCommand(HX8357_PASET); // Row addr set

  uint32_t yAddr = y0<<16;
  yAddr |=y1;
  spiWriteLong(spi, yAddr);
  // spiWriteByte(spi,y0>>8);
  // spiWriteByte(spi,y0);     // YSTART
  // spiWriteByte(spi,y1>>8);
  // spiWriteByte(spi,y1);     // YEND

  writeCommand(HX8357_RAMWR); // write to RAM
}

void ESP32_HX8357K::fillScreen(uint16_t color){
  //create a data set for filling the screen
  //this works bitches. 102ms
  uint32_t data[16];
  uint32_t compactColor = to32(color);
  setAddr(0, 0, _width -1, _height-1);
  //setup data
  for(int i = 0; i<16; i ++){
    data[i] = compactColor;
  }

  for(int i = 0; i<4800; i ++){
    spiWrite(spi, data,16);
    //writeData(data, 16);
  }
}

void ESP32_HX8357K::drawPixel(uint16_t x, uint16_t y, uint16_t color){
  if((x >= _width) || (y >= _height)) return;
  setAddr(x, y, x, y);
  spiWriteWord(spi, color);
}

void ESP32_HX8357K::drawFastVLine(uint16_t x, uint16_t y, uint16_t l, uint16_t color){
  if((x >= _width) || (y >= _height) || l ==0 ) return;
	if((y + l - 1) >= _width)  l = _width  - y;

  uint32_t data[16];
  uint32_t compactColor = to32(color);
  setAddr(x,y,x,y+l-1);
  //setup data
  for(int i = 0; i<16; i ++){
    data[i] = compactColor;
  }


  uint32_t itt = l>>5;
  for(uint32_t i = 0; i<itt; i ++ ){
      spiWrite(spi, data,16);
  }
  uint32_t rest = l%32;
  while (rest-- > 0) {
    spiWriteWord(spi, color);
  }

}

void ESP32_HX8357K::drawFastHLine(uint16_t x, uint16_t y, uint16_t l, uint16_t color){
  if((x >= _width) || (y >= _height)) return;
	if((x + l ) >= _width)  l = _width  - x;
  uint32_t data[16];
  uint32_t compactColor = to32(color);
  setAddr(x,y,x+l-1,y);
  //setup data
  for(int i = 0; i<16; i ++){
    data[i] = compactColor;
  }
  uint32_t itt = l>>5;
  for(uint32_t i = 0; i<itt; i ++ ){
    spiWrite(spi, data,16);
  }
  uint32_t rest = l%32;
  while (rest-- > 0) {
    spiWriteWord(spi, color);
  }

}

void ESP32_HX8357K::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
  // if((x >= _width) || (y >= _height)) return;
	// if((x + w - 1) >= _width)  w = _width  - x;
	// if((y + h - 1) >= _height) h = _height - y;
  setAddr(x,y,x+w-1,y+h-1);
  //needs a faster way
  uint32_t data[16];
  uint32_t compactColor = to32(color);
  uint32_t itt = w*h>>5; // divide by 32 to account for number of itterations
  for(int i = 0 ; i<16; i ++){
    data[i] = compactColor;
  }
  if(itt>0){
    for(uint32_t i = 0 ; i < itt; i++){
      spiWrite(spi, data,16);
    }
  }

  //rest data
  uint32_t rItt = (w*h)&0x1F -1;
  for(int i = 0; i<rItt; i++){
    spiWriteWord(spi,color);
  }
}


void ESP32_HX8357K::drawLine(int16_t x0, int16_t y0,
	int16_t x1, int16_t y1, uint16_t color)
{
	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		} else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		} else {
			drawPixel(x0, y0, color);
		}
		return;
	} else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		} else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	int16_t xbegin = x0;
	if (steep) {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastVLine(y0, xbegin, len + 1, color);
				} else {
					drawPixel(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			drawFastVLine(y0, xbegin, x0 - xbegin, color);
		}

	} else {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastHLine(xbegin, y0, len + 1, color);
				} else {
					drawPixel(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			drawFastHLine(xbegin, y0, x0 - xbegin, color);
		}
	}
}

void ESP32_HX8357K::drawHLine(uint16_t x, uint16_t y, int16_t l, uint16_t color){

}

void ESP32_HX8357K::drawVLine(uint16_t x, uint16_t y, int16_t l, uint16_t color){

}

void ESP32_HX8357K::drawPixelNL(uint16_t x, uint16_t y, uint16_t color){
  if((x >= _width) || (y >= _height)) return;
  setAddrNL(x, x, y, y);
  spiWriteShortNL(spi, color);
}


void ESP32_HX8357K::fillRectVGradient(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color1, uint16_t color2){
  if((x >= _width) || (y >= _height) || w==0 || h == 0) return;
	if((x + w - 1) >= _width)  w = _width - x;
	if((y + h - 1) >= _height) h = _height- y;

  int32_t dr =(int16_t(colorR(color2)<<8) - int16_t(colorR(color1)<<8))/h;
  int32_t dg =(int16_t(colorG(color2)<<8) - int16_t(colorG(color1)<<8))/h;
  int32_t db =(int16_t(colorB(color2)<<8) - int16_t(colorB(color1)<<8))/h;

  uint16_t br = colorR(color1);
  uint16_t bg = colorG(color1);
  uint16_t bb = colorB(color1);

  for(uint32_t i = 0; i < h; i ++){
    uint16_t r = br + ((dr*i)>>8);
    uint16_t g = bg + ((dg*i)>>8);
    uint16_t b = bb + ((db*i)>>8);


    uint16_t finalColor = r<<11;
    finalColor |=g<<5;
    finalColor |= b;
    drawFastHLine(x,y+i,w, finalColor);
  }

}

void ESP32_HX8357K::fillRectHGradient(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color1, uint16_t color2){
  if((x >= _width) || (y >= _height) || w==0 || h== 0) return;
  if((x + w - 1) >= _width)  w = _width - x;
  if((y + h - 1) >= _height) h = _height - y;

  int32_t dr =(int16_t(colorR(color2)<<8) - int16_t(colorR(color1)<<8))/w;
  int32_t dg =(int16_t(colorG(color2)<<8) - int16_t(colorG(color1)<<8))/w;
  int32_t db =(int16_t(colorB(color2)<<8) - int16_t(colorB(color1)<<8))/w;

  uint16_t br = colorR(color1);
  uint16_t bg = colorG(color1);
  uint16_t bb = colorB(color1);

  for(uint32_t i = 0; i < w; i ++){
    uint32_t r = br + ((dr*i)>>8);
    uint32_t g = bg + ((dg*i)>>8);
    uint32_t b = bb + ((db*i)>>8);


    uint16_t finalColor = r<<11;
    finalColor |=g<<5;
    finalColor |= b;
    drawFastVLine(x + i,y,h , finalColor);
  }
}

void ESP32_HX8357K::fillScreenVGradient(uint16_t color1, uint16_t color2){
  fillRectVGradient(0, 0, _width, _height, color1, color2);
}
void ESP32_HX8357K::fillScreenHGradient(uint16_t color1, uint16_t color2){
  fillRectHGradient(0, 0, _width, _height, color1, color2);
}

/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).
Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!
Copyright (c) 2013 Adafruit Industries.  All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

//#include "glcdfont.c"
extern "C" const unsigned char glcdfont[];


void ESP32_HX8357K::drawCircle(int16_t x0, int16_t y0, int16_t r,
    uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  //start drawing
  //spiSimpleTransaction(spi);

  drawPixel(x0  , y0+r, color);
  drawPixel(x0  , y0-r, color);
  drawPixel(x0+r, y0  , color);
  drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
  yield();
  //spiEndTransaction(spi);
}

void ESP32_HX8357K::drawCircleHelper( int16_t x0, int16_t y0,
               int16_t r, uint8_t cornername, uint16_t color) {
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - y, y0 - x, color);
      drawPixel(x0 - x, y0 - y, color);
    }
  }
}

void ESP32_HX8357K::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  drawFastVLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}


// Used to do circles and roundrects
void ESP32_HX8357K::fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
    uint8_t cornername, int16_t delta, uint16_t color) {

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}

void ESP32_HX8357K::drawTriangle(int16_t x0, int16_t y0,
				int16_t x1, int16_t y1,
				int16_t x2, int16_t y2, uint16_t color) {
  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);
}

void ESP32_HX8357K::fillTriangle ( int16_t x0, int16_t y0,
				  int16_t x1, int16_t y1,
				  int16_t x2, int16_t y2, uint16_t color) {

  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    drawFastHLine(a, y0, b-a+1, color);
    return;
  }

  int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1,
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }
}

void ESP32_HX8357K::drawRoundRect(int16_t x, int16_t y, int16_t w,
  int16_t h, int16_t r, uint16_t color) {
  // smarter version
  drawFastHLine(x+r  , y    , w-2*r, color); // Top
  drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  drawFastVLine(x    , y+r  , h-2*r, color); // Left
  drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1, color);
  drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

void ESP32_HX8357K::fillRoundRect(int16_t x, int16_t y, int16_t w,
				 int16_t h, int16_t r, uint16_t color) {
  // smarter version
  fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}


static uint32_t fetchbit(const uint8_t *p, uint32_t index)
{
	if (p[index >> 3] & (1 << (7 - (index & 7)))) return 1;
	return 0;
}

static uint32_t fetchbits_unsigned(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = 0;
	do {
		uint8_t b = p[index >> 3];
		uint32_t avail = 8 - (index & 7);
		if (avail <= required) {
			val <<= avail;
			val |= b & ((1 << avail) - 1);
			index += avail;
			required -= avail;
		} else {
			b >>= avail - required;
			val <<= required;
			val |= b & ((1 << required) - 1);
			break;
		}
	} while (required);
	return val;
}

static uint32_t fetchbits_signed(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = fetchbits_unsigned(p, index, required);
	if (val & (1 << (required - 1))) {
		return (int32_t)val - (1 << required);
	}
	return (int32_t)val;
}

void ESP32_HX8357K::drawChar(int16_t x, int16_t y, unsigned char c,
			    uint16_t fgcolor, uint16_t bgcolor, uint8_t size)
{
	if((x >= _width)            || // Clip right
	   (y >= _height)           || // Clip bottom
	   ((x + 6 * size - 1) < 0) || // Clip left  TODO: is this correct?
	   ((y + 8 * size - 1) < 0))   // Clip top   TODO: is this correct?
		return;

	if (fgcolor == bgcolor) {
		// This transparent approach is only about 20% faster
		if (size == 1) {
			uint8_t mask = 0x01;
			int16_t xoff, yoff;
			for (yoff=0; yoff < 8; yoff++) {
				uint8_t line = 0;
				for (xoff=0; xoff < 5; xoff++) {
					if (glcdfont[c * 5 + xoff] & mask) line |= 1;
					line <<= 1;
				}
				line >>= 1;
				xoff = 0;
				while (line) {
					if (line == 0x1F) {
						drawFastHLine(x + xoff, y + yoff, 5, fgcolor);
						break;
					} else if (line == 0x1E) {
						drawFastHLine(x + xoff, y + yoff, 4, fgcolor);
						break;
					} else if ((line & 0x1C) == 0x1C) {
						drawFastHLine(x + xoff, y + yoff, 3, fgcolor);
						line <<= 4;
						xoff += 4;
					} else if ((line & 0x18) == 0x18) {
						drawFastHLine(x + xoff, y + yoff, 2, fgcolor);
						line <<= 3;
						xoff += 3;
					} else if ((line & 0x10) == 0x10) {
						drawPixel(x + xoff, y + yoff, fgcolor);
						line <<= 2;
						xoff += 2;
					} else {
						line <<= 1;
						xoff += 1;
					}
				}
				mask = mask << 1;
			}
		} else {
			uint8_t mask = 0x01;
			int16_t xoff, yoff;
			for (yoff=0; yoff < 8; yoff++) {
				uint8_t line = 0;
				for (xoff=0; xoff < 5; xoff++) {
					if (glcdfont[c * 5 + xoff] & mask) line |= 1;
					line <<= 1;
				}
				line >>= 1;
				xoff = 0;
				while (line) {
					if (line == 0x1F) {
						fillRect(x + xoff * size, y + yoff * size,
							5 * size, size, fgcolor);
						break;
					} else if (line == 0x1E) {
						fillRect(x + xoff * size, y + yoff * size,
							4 * size, size, fgcolor);
						break;
					} else if ((line & 0x1C) == 0x1C) {
						fillRect(x + xoff * size, y + yoff * size,
							3 * size, size, fgcolor);
						line <<= 4;
						xoff += 4;
					} else if ((line & 0x18) == 0x18) {
						fillRect(x + xoff * size, y + yoff * size,
							2 * size, size, fgcolor);
						line <<= 3;
						xoff += 3;
					} else if ((line & 0x10) == 0x10) {
						fillRect(x + xoff * size, y + yoff * size,
							size, size, fgcolor);
						line <<= 2;
						xoff += 2;
					} else {
						line <<= 1;
						xoff += 1;
					}
				}
				mask = mask << 1;
			}
		}
	} else {
		// This solid background approach is about 5 time faster
		setAddr(x, y, x + 6 * size - 1, y + 8 * size - 1);
		uint8_t xr, yr;
		uint8_t mask = 0x01;
		uint16_t color;
		for (y=0; y < 8; y++) {
			for (yr=0; yr < size; yr++) {
				for (x=0; x < 5; x++) {
					if (glcdfont[c * 5 + x] & mask) {
						color = fgcolor;
					} else {
						color = bgcolor;
					}
					for (xr=0; xr < size; xr++) {
						spiWriteWord(spi, color);
					}
				}
				for (xr=0; xr < size; xr++) {
					spiWriteWord(spi, bgcolor);
				}
			}
			mask = mask << 1;
		}

	}
}

void ESP32_HX8357K::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	drawFastHLine(x, y, w, color);
	drawFastHLine(x, y+h-1, w, color);
	drawFastVLine(x, y, h, color);
	drawFastVLine(x+w-1, y, h, color);
}

void ESP32_HX8357K::setCursor(int16_t x, int16_t y) {
	if (x < 0) x = 0;
	else if (x >= _width) x = _width - 1;
	cursor_x = x;
	if (y < 0) y = 0;
	else if (y >= _height) y = _height - 1;
	cursor_y = y;
}

void ESP32_HX8357K::getCursor(int16_t *x, int16_t *y) {
  *x = cursor_x;
  *y = cursor_y;
}


void ESP32_HX8357K::setTextColor(uint16_t c) {
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

void ESP32_HX8357K::setTextColor(uint16_t c, uint16_t b) {
  textcolor   = c;
  textbgcolor = b;
}

void ESP32_HX8357K::setTextSize(uint8_t s) {
  textsize = (s > 0) ? s : 1;
}

uint8_t ESP32_HX8357K::getTextSize() {
	return textsize;
}

void ESP32_HX8357K::setTextWrap(boolean w) {
  wrap = w;
}

boolean ESP32_HX8357K::getTextWrap()
{
	return wrap;
}

uint8_t ESP32_HX8357K::getRotation(void){
  return rotation;
}
size_t ESP32_HX8357K::write(uint8_t c)
{
	if (font) {
		if (c == '\n') {
			cursor_y += font->line_space; // Fix linefeed. Added by T.T., SoftEgg
			cursor_x = 0;
		} else {
			drawFontChar(c);
		}
	} else {
		if (c == '\n') {
			cursor_y += textsize*8;
			cursor_x  = 0;
		} else if (c == '\r') {
			// skip em
		} else {
			drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
			cursor_x += textsize*6;
			if (wrap && (cursor_x > (_width - textsize*6))) {
				cursor_y += textsize*8;
				cursor_x = 0;
			}
		}
	}
	return 1;
}


void ESP32_HX8357K::drawFontChar(unsigned int c)
{
	uint32_t bitoffset;
	const uint8_t *data;

	//Serial.printf("drawFontChar %d\n", c);

	if (c >= font->index1_first && c <= font->index1_last) {
		bitoffset = c - font->index1_first;
		bitoffset *= font->bits_index;
	} else if (c >= font->index2_first && c <= font->index2_last) {
		bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
		bitoffset *= font->bits_index;
	} else if (font->unicode) {
		return; // TODO: implement sparse unicode
	} else {
		return;
	}
	//Serial.printf("  index =  %d\n", fetchbits_unsigned(font->index, bitoffset, font->bits_index));
	data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

	uint32_t encoding = fetchbits_unsigned(data, 0, 3);
	if (encoding != 0) return;
	uint32_t width = fetchbits_unsigned(data, 3, font->bits_width);
	bitoffset = font->bits_width + 3;
	uint32_t height = fetchbits_unsigned(data, bitoffset, font->bits_height);
	bitoffset += font->bits_height;
	//Serial.printf("  size =   %d,%d\n", width, height);

	int32_t xoffset = fetchbits_signed(data, bitoffset, font->bits_xoffset);
	bitoffset += font->bits_xoffset;
	int32_t yoffset = fetchbits_signed(data, bitoffset, font->bits_yoffset);
	bitoffset += font->bits_yoffset;
	//Serial.printf("  offset = %d,%d\n", xoffset, yoffset);

	uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
	bitoffset += font->bits_delta;
	//Serial.printf("  delta =  %d\n", delta);

	//Serial.printf("  cursor = %d,%d\n", cursor_x, cursor_y);

	// horizontally, we draw every pixel, or none at all
	if (cursor_x < 0) cursor_x = 0;
	int32_t origin_x = cursor_x + xoffset;
	if (origin_x < 0) {
		cursor_x -= xoffset;
		origin_x = 0;
	}
	if (origin_x + (int)width > _width) {
		if (!wrap) return;
		origin_x = 0;
		if (xoffset >= 0) {
			cursor_x = 0;
		} else {
			cursor_x = -xoffset;
		}
		cursor_y += font->line_space;
	}
	if (cursor_y >= _height) return;
	cursor_x += delta;

	// vertically, the top and/or bottom can be clipped
	int32_t origin_y = cursor_y + font->cap_height - height - yoffset;
	//Serial.printf("  origin = %d,%d\n", origin_x, origin_y);

	// TODO: compute top skip and number of lines
	int32_t linecount = height;
	//uint32_t loopcount = 0;
	uint32_t y = origin_y;
	while (linecount) {
		//Serial.printf("    linecount = %d\n", linecount);
		uint32_t b = fetchbit(data, bitoffset++);
		if (b == 0) {
			//Serial.println("    single line");
			uint32_t x = 0;
			do {
				uint32_t xsize = width - x;
				if (xsize > 32) xsize = 32;
				uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
				drawFontBits(bits, xsize, origin_x + x, y, 1);
				bitoffset += xsize;
				x += xsize;
			} while (x < width);
			y++;
			linecount--;
		} else {
			uint32_t n = fetchbits_unsigned(data, bitoffset, 3) + 2;
			bitoffset += 3;
			uint32_t x = 0;
			do {
				uint32_t xsize = width - x;
				if (xsize > 32) xsize = 32;
				//Serial.printf("    multi line %d\n", n);
				uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
				drawFontBits(bits, xsize, origin_x + x, y, n);
				bitoffset += xsize;
				x += xsize;
			} while (x < width);
			y += n;
			linecount -= n;
		}
		//if (++loopcount > 100) {
			//Serial.println("     abort draw loop");
			//break;
		//}
	}
}


void ESP32_HX8357K::drawFontBits(uint32_t bits, uint32_t numbits, uint32_t x, uint32_t y, uint32_t repeat){
  if (bits == 0) return;
  int w = 0;
  do {
    uint32_t x1 = x;
    uint32_t n = numbits;

    do {
      n--;
      if (bits & (1 << n)) {
        w++;
      }
      else if (w > 0) {
        setAddr(x1-w, y, x1,y);
        while (w-- > 1) { // draw line
          spiWriteWord(spi,textcolor);
        }
        spiWriteWord(spi,textcolor);
      }

      x1++;
    } while (n > 0);

    if (w > 0) {
        setAddr(x1-w, y, x1, y);
        while (w-- > 1) { //draw line
          spiWriteWord(spi,textcolor);
        }
        spiWriteWord(spi,textcolor);
    }

    y++;
    repeat--;
  } while (repeat);
}



#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ESP32_HX8357K::setRotation(uint8_t m) {

  writeCommand(HX8357_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writeByte(MADCTL_MY | MADCTL_RGB);
     _width  = WIDTH;
     _height = HEIGHT;
     break;
   case 1:
     writeByte(MADCTL_MV| MADCTL_MX | MADCTL_MY| MADCTL_RGB);
     _width  = HEIGHT;
     _height = WIDTH;
     break;
  case 2:
    writeByte(MADCTL_MX | MADCTL_RGB);

     _width  = WIDTH;
     _height = HEIGHT;
    break;
   case 3:
    writeByte( MADCTL_MV| MADCTL_RGB);
     _width  = HEIGHT;
     _height = WIDTH;
     break;
  }
}

void ESP32_HX8357K::invertDisplay(boolean i) {
  writeCommand(i ? HX8357_INVON : HX8357_INVOFF);
}

uint16_t ESP32_HX8357K::color565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t res = (b & 0xF8) << 8;
  res |=(g & 0xFC) << 3;
  res |= r>>3;
  return res;
}

uint16_t ESP32_HX8357K::colorR(uint16_t color){
  uint16_t res = color>>11;
  return res;
}
uint16_t ESP32_HX8357K::colorG(uint16_t color){
  uint16_t res = color>>5;
  res &=0x3F;
  return res;
}
uint16_t ESP32_HX8357K::colorB(uint16_t color){
  uint16_t res =  color&0x1F;
  return res;
}

uint32_t ESP32_HX8357K::to32(uint16_t color){
  uint16_t hi = color>>8;
  uint16_t lo = color&0xFF;
  uint16_t revColor = (lo<<8) + hi;

  uint32_t c = revColor<<16;
  c += revColor;
  return c;
}


//faster pixel write implementation
void ESP32_HX8357K::spiSimpleTransaction(spi_t * spi)
{
    if(!spi) {
        return;
    }
    SPI_MUTEX_LOCK();
}

void ESP32_HX8357K::spiEndTransaction(spi_t * spi)
{
    if(!spi) {
        return;
    }
    SPI_MUTEX_UNLOCK();
}

uint16_t ESP32_HX8357K::spiTransferShortNL(spi_t * spi, uint16_t data)
{
    if(!spi) {
        return 0;
    }
    if(!spi->dev->ctrl.wr_bit_order){
        MSB_16_SET(data, data);
    }
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 15;
    spi->dev->miso_dlen.usr_miso_dbitlen = 15;
    spi->dev->data_buf[0] = data;
    spi->dev->cmd.usr = 1;
    while(spi->dev->cmd.usr);
    data = spi->dev->data_buf[0] & 0xFFFF;
    if(!spi->dev->ctrl.rd_bit_order){
        MSB_16_SET(data, data);
    }
    return data;
}

void ESP32_HX8357K::spiWriteShortNL(spi_t * spi, uint16_t data)
{
    if(!spi) {
        return;
    }
    if(!spi->dev->ctrl.wr_bit_order){
        MSB_16_SET(data, data);
    }
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 15;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    spi->dev->data_buf[0] = data;
    spi->dev->cmd.usr = 1;
    while(spi->dev->cmd.usr);
}

void spiWriteByteNL(spi_t * spi, uint8_t data)
{
    if(!spi) {
        return;
    }
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    spi->dev->data_buf[0] = data;
    spi->dev->cmd.usr = 1;
    while(spi->dev->cmd.usr);
}

void ESP32_HX8357K::setAddrNL(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
  digitalWrite(LCD_DC, LOW);
  spiWriteByteNL(spi, HX8357_CASET);
  digitalWrite(LCD_DC, HIGH);

  //writeCommand(HX8357_CASET); // Column addr set
  //TODO: package data and send in one go

  //writeData(uint32_t *data, uint8_t length)
  spiWriteShortNL(spi,x0);
  spiWriteShortNL(spi,x1);

  digitalWrite(LCD_DC, LOW);
  spiWriteByteNL(spi, HX8357_PASET);
  digitalWrite(LCD_DC, HIGH);

  spiWriteShortNL(spi,y0);
  spiWriteShortNL(spi,y1);

  digitalWrite(LCD_DC, LOW);
  spiWriteByteNL(spi, HX8357_RAMWR);  //write to ram
  digitalWrite(LCD_DC, HIGH);

}


void ESP32_HX8357K::drawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *bmap){
  setAddr(x, y, x+w-1, y+h-1);
  //now calculate the amount of bits that can be transferred through fast writing
  uint16_t itt = (w*h)>>5;
  uint16_t rest = (w*h)%32;
  uint32_t dataSet[16];
  for(int i = 0 ; i < itt; i++){
    for(int j = 0; j<32; j+=2){
      uint16_t cLoc = (i*16) + (j>>1);

      dataSet[cLoc] = bmap[j+i*32]<<16;
      dataSet[cLoc] |= bmap[j+1 + i*32];
    }
    spiWrite(spi,dataSet,16);
  }

  for(int i = 0; i<rest; i ++){
    spiWriteWord(spi, bmap[i+itt*32]);
  }

}
// size_t ESP32_HX8357K::print(const __FlashStringHelper *ifsh)
// {
//   PGM_P p = reinterpret_cast<PGM_P>(ifsh);
//   size_t n = 0;
//   while (1) {
//     unsigned char c = pgm_read_byte(p++);
//     if (c == 0) break;
//     if (write(c)) n++;
//     else break;
//   }
//   return n;
// }
//
// size_t ESP32_HX8357K::print(const String &s)
// {
//   return write(s.c_str(), s.length());
// }
//
// size_t ESP32_HX8357K::print(const char str[])
// {
//   return write(str);
// }
//
// size_t ESP32_HX8357K::print(char c)
// {
//   return write(c);
// }
//
// size_t ESP32_HX8357K::print(unsigned char b, int base)
// {
//   return print((unsigned long) b, base);
// }
//
// size_t ESP32_HX8357K::print(int n, int base)
// {
//   return print((long) n, base);
// }
//
// size_t ESP32_HX8357K::print(unsigned int n, int base)
// {
//   return print((unsigned long) n, base);
// }
//
// size_t ESP32_HX8357K::print(long n, int base)
// {
//   if (base == 0) {
//     return write(n);
//   } else if (base == 10) {
//     if (n < 0) {
//       int t = print('-');
//       n = -n;
//       return printNumber(n, 10) + t;
//     }
//     return printNumber(n, 10);
//   } else {
//     return printNumber(n, base);
//   }
// }
//
// size_t ESP32_HX8357K::print(unsigned long n, int base)
// {
//   if (base == 0) return write(n);
//   else return printNumber(n, base);
// }
//
// size_t ESP32_HX8357K::print(double n, int digits)
// {
//   return printFloat(n, digits);
// }
//
// size_t ESP32_HX8357K::println(const __FlashStringHelper *ifsh)
// {
//   size_t n = print(ifsh);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::print(const Printable& x)
// {
//   return x.printTo(*this);
// }
//
// size_t ESP32_HX8357K::println(void)
// {
//   return write("\r\n");
// }
//
// size_t ESP32_HX8357K::println(const String &s)
// {
//   size_t n = print(s);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::println(const char c[])
// {
//   size_t n = print(c);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::println(char c)
// {
//   size_t n = print(c);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::println(unsigned char b, int base)
// {
//   size_t n = print(b, base);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::println(int num, int base)
// {
//   size_t n = print(num, base);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::println(unsigned int num, int base)
// {
//   size_t n = print(num, base);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::println(long num, int base)
// {
//   size_t n = print(num, base);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::println(unsigned long num, int base)
// {
//   size_t n = print(num, base);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::println(double num, int digits)
// {
//   size_t n = print(num, digits);
//   n += println();
//   return n;
// }
//
// size_t ESP32_HX8357K::println(const Printable& x)
// {
//   size_t n = print(x);
//   n += println();
//   return n;
// }
