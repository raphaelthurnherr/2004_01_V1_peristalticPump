#include "displayManager.h"
#include <u8x8lib.h>


unsigned char refreshMenuDisplay(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned char menuNb, PPUMP * myPump, unsigned char clearDisp){
  if(clearDisp){
    gfx->clearDisplay();
  }

  switch(menuNb){
    case 1 : display_menu_1(gfx, myPump->ManualFlowRate_percent); break;
    case 2 : display_menu_2(gfx, myPump->AutoVolumeSetpoint_ml, myPump->AutoFlowRate_ml_min); break;
    case 3 : display_menu_3(gfx); break;  // Error menu flow

    case 10 : display_menu_1_0(gfx, myPump->ManualFlowRate_percent, myPump->VolumeCounter_ml, myPump->MeasuredFlowRate_ml_min[0]); break;
    case 20 : display_menu_2_0(gfx, myPump->AutoVolumeSetpoint_ml, myPump->AutoFlowRate_ml_min, myPump->VolumeCounter_ml, myPump->MeasuredFlowRate_ml_min[0]); break;
    case 210: display_menu_2_1_0(gfx, myPump->AutoVolumeSetpoint_ml, myPump->AutoFlowRate_ml_min); break;
    case 220: display_menu_2_2_0(gfx, myPump->AutoVolumeSetpoint_ml, myPump->AutoFlowRate_ml_min); break;
    default : gfx->drawString(0, 0, "ERROR MENU L0");
  }
}


//-------------- MENU AND SUB MENU -------------------

// MENU 1 : MANUAL MODE STOPPED

void display_menu_1(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data){
  gfx->drawString(0, 0, "  MANUAL  MODE  ");
  gfx->drawString(0, 2, "Motor speed    %");
  gfx->setCursor(12, 2);
  gfx->print(data, DEC);
  gfx->drawString(0, 4, "    -STOPPED-   ");
  gfx->drawString(0, 6, "  click for run ");
}

// MENU 1.0 : MANUAL MODE RUNNING
void display_menu_1_0(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2, unsigned int data3){
  gfx->drawString(0, 0, "  MANUAL  MODE  ");
  gfx->drawString(0, 2, "Motor speed    %");
  gfx->setInverseFont(1);
  gfx->setCursor(12, 2);
  gfx->print(data, DEC);
  gfx->setInverseFont(0);

  gfx->drawString(0, 4, "   ml/min     ml");

  gfx->setCursor(0, 4);
  gfx->print(data3, DEC);
  
  gfx->setCursor(11, 4);
  gfx->print(data2, DEC);
  gfx->drawString(0, 6, " click for stop ");
}

// MENU 2 : AUTO MODE STOPPED
void display_menu_2(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2){
  gfx->drawString(0, 0, "    AUTO MODE   ");
  gfx->drawString(0, 2, "   ml     ml/min");
  gfx->setCursor(0, 2);
  gfx->print(data, DEC);
  gfx->setCursor(7, 2);
  gfx->print(data2, DEC);
  gfx->drawString(0, 4, "    -STOPPED-   ");
  gfx->drawString(0, 6, "long push config");
}

// MENU 2.0 : AUTO MODE RUNNING
void display_menu_2_0(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2, unsigned int data3, unsigned int data4){
  gfx->drawString(0, 0, "    AUTO MODE   ");
  gfx->drawString(0, 2, "   ml     ml/min");
  gfx->setCursor(0, 2);
  gfx->print(data, DEC);
  gfx->setCursor(7, 2);
  gfx->print(data2, DEC);

  //gfx->drawString(0, 4, "RUN     ml");
  gfx->drawString(0, 4, "   ml/min     ml");

  gfx->setCursor(0, 4);
  gfx->print(data4, DEC);
  
  gfx->setCursor(11, 4);
  gfx->print(data3, DEC);
  gfx->drawString(0, 6, " click for stop ");
}

// MENU 2.1.0 : AUTO MODE CONFIG SET QTY
void display_menu_2_1_0(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2){
  gfx->drawString(0, 0, "  AUTO CONFIG   ");
  gfx->drawString(0, 2, "          ml    ");
  gfx->setInverseFont(1);
  gfx->setCursor(6, 2);
  gfx->print(data, DEC);
  gfx->setInverseFont(0);
  gfx->drawString(0, 4, "          ml/min");  
  gfx->setCursor(6, 4);
  gfx->print(data2, DEC);
  gfx->drawString(0, 6, "long push return");
}

// MENU 2.2.0 : AUTO MODE CONFIG SELECT SPEED 
void display_menu_2_2_0(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx, unsigned int data, unsigned int data2){
  gfx->drawString(0, 0, "  AUTO CONFIG   ");
  gfx->drawString(0, 2, "          ml    ");
  gfx->setCursor(6, 2);
  gfx->print(data, DEC);
  gfx->drawString(0, 4, "          ml/min");
  gfx->setInverseFont(1);  
  gfx->setCursor(6, 4);
  gfx->print(data2, DEC);
  gfx->setInverseFont(0);
  gfx->drawString(0, 6, "long push return");
}

// MENU 3 : FLOW ERROR 
void display_menu_3(U8X8_SSD1306_128X64_NONAME_HW_I2C * gfx){
  gfx->drawString(0, 0, "      ERROR     ");
  gfx->drawString(0, 2, "  No liquid or  ");
  gfx->drawString(0, 4, "bubbles in tubes");
  gfx->drawString(0, 6, "long push return");
}