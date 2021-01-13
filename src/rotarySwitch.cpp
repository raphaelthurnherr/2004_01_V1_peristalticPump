/**
 * @file rotarySwitch.cpp
 * @author your name (you@domain.com)
 * @brief Rotary switch events detection library, must be used with a dual state rotary switch
 * @version 1.0
 * @date 2021-01-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */

  #include <Arduino.h>
  #include "rotarySwitch.h"
  

  /**
 * @brief Get the Rotary Switch Event object
 * 
 * @return unsigned char Rotary switch event
 */
signed char getRotarySwitchEvent(unsigned char rswitch_clk_pin, unsigned char rswitch_dat_pin, unsigned char rswitch_sw_pin, int * timerPress){
  static unsigned char longPressWaitForRelease =0 ;
  unsigned char encoder_pin = 0;
  static unsigned char old_encoder_pin = 1;
  unsigned char rotation_direction = 0;
  unsigned char sw_event=0;
  unsigned char rotary_sw_event =0;
  unsigned char old_rotary_sw_event=0;
  signed char encoderAction=ROTARY_NO_EVENT;

  //*****
  // GET ROTARY PIN STATE AND DETECT EVENT TYPE CW, CCW, PUSH or RELEASE
  //*****
  encoder_pin |= digitalRead(rswitch_clk_pin);
  encoder_pin |= digitalRead(rswitch_dat_pin)<<1;
  encoder_pin |= digitalRead(rswitch_sw_pin)<<2;

  // Detect rising or falling edge of CLK for get rotary direction
  if((old_encoder_pin & 0x01) != (encoder_pin & 0x01)){
    if(old_encoder_pin & 0x01){
      if((encoder_pin & 0x02) == 0){
        // Get data level for CCW
        rotation_direction = 2;
      }else {
      // Get data level for CW
        rotation_direction = 1;
      }
    }else{
      if((encoder_pin & 0x02) > 0){
        // Get data level for CCW
        rotation_direction = 2;
      }else {
      // Get data level for CW
        rotation_direction = 1;
      }
    }
  }

    // Encoder rotation pin CLK and DAT must be to the same level before get SW state
    if((encoder_pin & 0x03) == (old_encoder_pin & 0x03)) {
      // Detect falling edge of SW push
      if(((old_encoder_pin & 0x04) > 0) && ((encoder_pin & 0x04) == 0)){
        sw_event = 1;
      }

      // Detect rising edge of SW push
      if(((old_encoder_pin & 0x04) == 0) && ((encoder_pin & 0x04) > 0)){
        sw_event = 2;
      } 
    }


 //******
 // MULTI EVENT ASSIGNEMENT
 //******
  rotary_sw_event |= rotation_direction;
  rotary_sw_event |= sw_event <<2;

  if(rotary_sw_event != old_rotary_sw_event){
    switch(rotary_sw_event & 0x03){
      case 0x01 : encoderAction = ROTARY_DEC; break;
      case 0x02 : encoderAction = ROTARY_INC; break;
      default : break;
    }

    switch(rotary_sw_event & 0x0C){
      case 0x04 : encoderAction = ROTARY_SW_PUSH;
                  *timerPress=0;    // Start / reset timer for long press detection
                  break;
      case 0x08 : if(longPressWaitForRelease){
                    longPressWaitForRelease = 0;
                  }
                  else {
                    encoderAction = ROTARY_SW_RELEASE; 
                  }
                  break;
      default : break;
    }
  } else encoderAction = ROTARY_NO_EVENT;


  // Check timer for long push detection. Send event only one time
  // DAT and CLK must be to the same level
  if((encoder_pin == 0x03) || (encoder_pin == 0x00)){
    if(!longPressWaitForRelease){
      // Long press detection (>1000ms)
      if(*timerPress >= 1000){
        encoderAction = ROTARY_SW_LONG_PUSH;
        longPressWaitForRelease = 1;
        *timerPress=0;
      }
    }
  }

  old_encoder_pin = encoder_pin;
  old_rotary_sw_event = rotary_sw_event;
      
  return encoderAction;
}