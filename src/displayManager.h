
#include <u8x8lib.h>
#include "main.h"

extern unsigned char refreshMenuDisplay(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned char menuNb, PPUMP * myPump, unsigned char clearDisp);

extern void display_menu_1(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data);        // MANUAL MODE STOPPED
extern void display_menu_1_0(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2,unsigned int data3);  // MANUAL MODE RUNNING
extern void display_menu_2(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2);        // AUTO MODE STOPPED
extern void display_menu_2_0(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2, unsigned int data3, unsigned int data4);  // AUTO MODE RUNNING 
extern void display_menu_2_1_0(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2); // AUTO MODE CONFIG SET QTY
extern void display_menu_2_2_0(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2); // AUTO MODE CONFIG SET SPEED    
extern void display_menu_3(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx); 